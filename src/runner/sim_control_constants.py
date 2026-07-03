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
# route（mission_route）首段方向与 rally_route 首段方向（A→A1，即 mission_heading_rad）允许的最大夹角。
# 超过此值时 JOINING(EXITED，沿 rally 方向飞)→CATCHUP(沿 mission_route 方向飞) 切换瞬间会有真实的
# 指令航向突变，只校验起点位置不够，还要校验方向一致，见 sim_control_modules.py::_ConfigLoader.validate。
_MAX_MISSION_RALLY_HEADING_MISMATCH_DEG = 10.0
