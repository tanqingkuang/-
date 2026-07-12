"""尾迹控制 ViewModel。注意：本模块只含纯 Python 状态与规则，不依赖 Qt。"""

from __future__ import annotations

import math
from collections import deque
from collections.abc import Iterator, Sequence
from dataclasses import dataclass
from itertools import count, islice

from src.ui.gui.view_models import TrailPoint, trail_seconds_for_duration


# 主窗口以 10 FPS 采集尾迹；32768 点可完整覆盖内置最长配置的 2100 秒默认窗口，
# 同时为异常长航时或高刷新率提供明确内存上界。渲染缓存可据此推导最大块数。
MAX_TRAIL_POINTS = 32_768
# 每次新建或显式清空缓冲区都取得不同代号，让绘制端可靠识别重置与配置切换。
_TRAIL_GENERATION_IDS = count(1)


@dataclass(frozen=True)
class TrailSnapshot(Sequence[TrailPoint]):
    """某一帧不可变的尾迹序列。注意：历史点对象与共享队列一致，容器不随下一帧变化。"""

    _points: tuple[TrailPoint, ...]
    generation: int
    revision: int
    first_sequence: int
    end_sequence: int
    capacity: int

    def __len__(self) -> int:
        """返回当前快照点数。注意：复杂度为 O(1)。"""

        return len(self._points)

    def __iter__(self) -> Iterator[TrailPoint]:
        """按采样时间顺序迭代稳定点对象。注意：迭代期间快照不会变化。"""

        return iter(self._points)

    def __getitem__(self, index: int | slice) -> TrailPoint | list[TrailPoint]:
        """按逻辑位置读取点或切片。注意：切片返回列表以兼容既有绘制代码。"""

        if isinstance(index, slice):
            return list(self._points[index])
        return self._points[index]


class TrailBuffer(Sequence[TrailPoint]):
    """单架飞机的稳定有界 ENU 尾迹队列。注意：追加和容量弹头均为 O(1)。"""

    def __init__(self, capacity: int = MAX_TRAIL_POINTS) -> None:
        """创建空尾迹队列。注意：capacity 必须为正数。"""

        if capacity <= 0:
            raise ValueError("尾迹队列容量必须为正数")
        self._points: deque[TrailPoint] = deque()
        self._capacity = int(capacity)
        self._generation = next(_TRAIL_GENERATION_IDS)
        self._revision = 0
        self._next_point_id = 0

    @property
    def capacity(self) -> int:
        """返回硬容量上限。注意：达到上限后下一次追加会先弹出队首。"""

        return self._capacity

    @property
    def generation(self) -> int:
        """返回当前尾迹代号。注意：显式清空后代号改变。"""

        return self._generation

    @property
    def revision(self) -> int:
        """返回内容修订号。注意：每次追加、过期或清空最多递增一次。"""

        return self._revision

    @property
    def first_sequence(self) -> int:
        """返回当前首点逻辑序号。注意：空队列时等于下一待分配序号。"""

        return self._points[0].point_id if self._points else self._next_point_id

    @property
    def end_sequence(self) -> int:
        """返回尾后逻辑序号。注意：与 range 的 stop 一样采用 exclusive 语义。"""

        return self._points[-1].point_id + 1 if self._points else self._next_point_id

    def __len__(self) -> int:
        """返回当前队列点数。注意：复杂度为 O(1)。"""

        return len(self._points)

    def __iter__(self) -> Iterator[TrailPoint]:
        """按入队顺序迭代点。注意：调用方不得在迭代期间修改队列。"""

        return iter(self._points)

    def __getitem__(self, index: int | slice) -> TrailPoint | list[TrailPoint]:
        """读取单点或切片。注意：切片仅为旧绘制接口兼容路径。"""

        if isinstance(index, slice):
            start, stop, step = index.indices(len(self._points))
            if step > 0:
                if step == 1 and stop == len(self._points):
                    # 从队尾反向取新增段，避免为跳过稳定历史而扫描整条 deque。
                    tail = list(islice(reversed(self._points), stop - start))
                    tail.reverse()
                    return tail
                # 其他正向兼容切片使用 islice，不创建无关的中间完整列表。
                return list(islice(self._points, start, stop, step))
            # 反向切片只用于兼容普通 Sequence 语义，不进入每帧增量热路径。
            return list(self._points)[index]
        return self._points[index]

    def append_position(
        self,
        x: float,
        y: float,
        altitude: float,
        time: float,
    ) -> TrailPoint:
        """追加一个 ENU 位置点并返回它。注意：同一时刻重复快照不会重复入队。"""

        if self._points and self._points[-1].time == time:
            return self._points[-1]
        if self._points and time < self._points[-1].time:
            # 时间回拨意味着控制器进入新一轮运行；换代后才能继续保持按时间有序弹头。
            self.clear()

        path_distance = 0.0
        if self._points:
            previous = self._points[-1]
            path_distance = previous.path_distance + math.hypot(x - previous.x, y - previous.y)
        point = TrailPoint(x, y, altitude, time, path_distance, self._next_point_id)
        self._next_point_id += 1
        # 显式 popleft 让容量淘汰与时间淘汰采用完全相同的稳定弹头语义。
        if len(self._points) >= self._capacity:
            self._points.popleft()
        self._points.append(point)
        self._revision += 1
        return point

    def expire(self, current_time: float, trail_seconds: float) -> int:
        """从队首弹出时间窗外的点并返回数量。注意：边界点会被保留。"""

        if trail_seconds <= 0.0:
            removed = len(self._points)
            self.clear()
            return removed
        removed = 0
        while self._points and current_time - self._points[0].time > trail_seconds:
            self._points.popleft()
            removed += 1
        if removed:
            self._revision += 1
        return removed

    def clear(self) -> None:
        """清空所有点并开启新代。注意：即使原本为空也会通知渲染端重置。"""

        self._points.clear()
        self._generation = next(_TRAIL_GENERATION_IDS)
        self._next_point_id = 0
        self._revision += 1

    def snapshot(self) -> TrailSnapshot:
        """冻结当前点引用供一个 GUI 快照共用。注意：后续追加不会改变已返回容器。"""

        return TrailSnapshot(
            _points=tuple(self._points),
            generation=self._generation,
            revision=self._revision,
            first_sequence=self.first_sequence,
            end_sequence=self.end_sequence,
            capacity=self._capacity,
        )


@dataclass(frozen=True)
class TrailControlUpdate:
    """尾迹控件需要执行的一次更新。注意：seconds 为 None 表示保留用户手动值。"""

    seconds: float | None
    range_max: float
    refresh_features: bool


class TrailViewModel:
    """封装尾迹默认值、手动值和配置重载语义。注意：不读写任何 GUI 控件。"""

    def __init__(self) -> None:
        """初始化尾迹控制状态。注意：首次收到时长时必须同步默认尾迹。"""

        # 记录最近一次自动同步依据的仿真时长，防止每帧刷新覆盖用户手动输入。
        self._duration_basis: float | None = None
        # 记录当前控件范围上限；手动输入只放宽不收窄，避免长航时默认值无法调回。
        self._range_max = 600.0

    def on_duration_synced(self, duration_s: float) -> TrailControlUpdate:
        """按飞行时长生成默认尾迹更新。注意：同一时长跳过自动覆盖。"""

        if self._duration_basis == duration_s:
            return TrailControlUpdate(seconds=None, range_max=self._range_max, refresh_features=False)
        seconds = trail_seconds_for_duration(duration_s)
        self._duration_basis = duration_s
        # 长时长配置的一半可能超过旧 600s 上限，控件需先放宽范围再写入值。
        self._range_max = max(600.0, seconds)
        return TrailControlUpdate(seconds=seconds, range_max=self._range_max, refresh_features=False)

    def on_manual_seconds(self, seconds: float) -> TrailControlUpdate:
        """处理用户手动尾迹输入。注意：负数按关闭尾迹处理，范围上限只放宽不收窄。"""

        clipped_seconds = max(0.0, float(seconds))
        self._range_max = max(self._range_max, clipped_seconds)
        return TrailControlUpdate(
            seconds=clipped_seconds,
            range_max=self._range_max,
            refresh_features=True,
        )

    def on_reset(self) -> None:
        """清空自动同步依据。注意：用于配置切换后重新应用默认尾迹。"""

        self._duration_basis = None
