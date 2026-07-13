"""仿真控制器内部常量。注意：集中放置避免主入口文件膨胀。"""

from __future__ import annotations

_DEFAULT_ALGORITHM_DECIMATION = 10
_COMM_DECIMATION = 2
_MIN_PLAYBACK_RATE = 0.1
_MAX_PLAYBACK_RATE = 50.0
_RUN_LOOP_SLEEP_SLICE_S = 0.005
_MAX_RUN_LOOP_BATCH_TICKS = 100
_CPU_UTILIZATION_SAMPLE_PERIOD_S = 1.0
_DEFAULT_TRIANGLE_WING_SLOTS: tuple[tuple[float, float, float], ...] = (
    (-54.0, 0.0, -58.0),
    (-54.0, 0.0, 58.0),
)
_FORMATION_COORDINATE_SYSTEM = "x_forward_y_up_z_right"
_LOG_SAMPLE_PERIOD_S = 0.1
_TIME_EPSILON_S = 1e-9
