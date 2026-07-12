"""尾迹控制 ViewModel。注意：本模块只含纯 Python 状态与规则，不依赖 Qt。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable

from src.ui.gui.view_models import trail_seconds_for_duration


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


def prune_trail(trail: Iterable[Any], current_time: float, trail_seconds: float) -> list[Any]:
    """按尾迹时间窗裁剪采样点。注意：只依赖采样点存在 time 属性。"""

    if trail_seconds <= 0.0:
        return []
    # 不假设输入按时间排序，逐点按当前时间窗保留边界内采样。
    return [point for point in trail if current_time - point.time <= trail_seconds]


# 2D 视图每帧逐段绘制尾迹，绘制段数必须有上限：长航时尾迹可积累上万点，
# 逐点 QPen+drawLine 会把 100ms 的 GUI tick 拖到数百毫秒(表现为 3D 飞机"一卡卡")。
MAX_TRAIL_DRAW_POINTS = 360
# 尾迹头部保留的原始采样数：头部若参与抽样，最新绘制点会滞后机体最多数秒，
# 表现为轨迹头与飞机脱节；保留近段原始点让轨迹头始终贴住机体。
TRAIL_HEAD_RAW_POINTS = 48


def sample_trail_for_display(trail: list, max_points: int = MAX_TRAIL_DRAW_POINTS) -> list:
    """按显示上限抽样尾迹：近段保留原始点，旧段均匀抽样。注意：只供绘制端使用，不改数据缓存。"""

    # 上限过小时夹到 4，保证头部保留与旧段抽样两部分都至少有 2 个点。
    max_points = max(4, max_points)
    if len(trail) <= max_points:
        return trail
    head_keep = min(TRAIL_HEAD_RAW_POINTS, max_points // 2)
    older, head = trail[:-head_keep], trail[-head_keep:]
    budget = max_points - head_keep
    # 旧段按索引均匀取样并保留首点，旧段整体处于淡出区间，抽稀在视觉上无感。
    last = len(older) - 1
    sampled = [older[round(last * index / (budget - 1))] for index in range(budget)]
    return sampled + head
