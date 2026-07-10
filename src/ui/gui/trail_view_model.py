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
