"""T1 不变量检查。注意：物理限幅从场景配置读取，任务阈值从 thresholds 读取。"""

from __future__ import annotations

import math
from typing import Any

from tests.st.support import thresholds
from tests.st.support.metrics import _distance3d, _leader_id, _node_by_id, _obstacle_margin, _obstacles, _route_end
from tests.st.support.runner import ScenarioRun
from tests.st.support.types import CheckIssue

_NUMERIC_NODE_FIELDS = (
    "x_m", "y_m", "altitude_m", "psi_v_deg", "speed_mps", "vx_mps", "vy_mps", "vz_mps",
    "phi_deg", "psi_dot_deg_s", "nx", "nz",
)


def run_invariants(run: ScenarioRun) -> list[CheckIssue]:
    """执行 UT-01~UT-07。注意：只返回结构化问题，不直接打印。"""

    checks = (
        check_run_completed,
        check_numeric_health,
        check_dynamic_limits,
        check_collision_distance,
        check_obstacle_clearance,
        check_task_completion,
        check_formation_convergence,
    )
    issues: list[CheckIssue] = []
    for check in checks:
        issues.extend(check(run))
    return issues


def check_run_completed(run: ScenarioRun) -> list[CheckIssue]:
    """UT-01 运行完成性。注意：帧数按当前日志 10Hz 采样校验。"""

    issues: list[CheckIssue] = []
    scenario = run.scenario
    if run.result.code != "OK":
        issues.append(CheckIssue(scenario, "UT-01", "运行入口返回失败", actual=run.result.code, limit="OK"))
        return issues
    for event in run.events:
        level = str(event.get("level", ""))
        message = str(event.get("message", ""))
        if level == "ERROR" or (level == "WARN" and not _warn_allowed(message)):
            issues.append(CheckIssue(scenario, "UT-01", "事件日志含错误或非白名单告警", time_s=_time(event), field=level, actual=message, limit="无 ERROR/WARN"))
    if not run.snapshots:
        issues.append(CheckIssue(scenario, "UT-01", "未生成 snapshots.jsonl", actual=0, limit=">0"))
        return issues
    duration = float(run.config.get("duration_s", 0.0))
    final_time = float(run.snapshots[-1].get("time_s", 0.0))
    if abs(final_time - duration) > thresholds.LOG_SAMPLE_PERIOD_S + thresholds.LIMIT_EPS:
        issues.append(CheckIssue(scenario, "UT-01", "仿真末帧未推进到配置时长", time_s=final_time, field="time_s", actual=final_time, limit=duration))
    expected_frames = round(duration / thresholds.LOG_SAMPLE_PERIOD_S)
    actual_frames = len(run.snapshots)
    if abs(actual_frames - expected_frames) > 1:
        issues.append(CheckIssue(scenario, "UT-01", "快照帧数与日志采样周期不匹配", field="snapshots", actual=actual_frames, limit=f"{expected_frames}±1"))
    return issues


def check_numeric_health(run: ScenarioRun) -> list[CheckIssue]:
    """UT-02 数值健康。注意：瞬移按相邻日志帧的真实 dt 检查。"""

    issues: list[CheckIssue] = []
    scenario = run.scenario
    speed_max = _forward_max(run.config)
    accel = _accel_limit(run.config)
    climb = _climb_limit(run.config)
    previous_by_id: dict[str, dict[str, Any]] = {}
    previous_time: float | None = None
    for snapshot in run.snapshots:
        time_s = _time(snapshot)
        for node in _nodes(snapshot):
            node_id = str(node.get("node_id", ""))
            for field in _NUMERIC_NODE_FIELDS:
                if field in node and not _finite(node.get(field)):
                    issues.append(CheckIssue(scenario, "UT-02", "节点字段出现 NaN/Inf", time_s=time_s, node=node_id, field=field, actual=node.get(field), limit="finite"))
            previous = previous_by_id.get(node_id)
            if previous is not None and previous_time is not None:
                dt = max(time_s - previous_time, 0.0)
                observed_speed = max(float(previous.get("speed_mps", 0.0)), float(node.get("speed_mps", 0.0)), speed_max)
                jump_limit = observed_speed * dt * thresholds.TELEPORT_MARGIN_FACTOR + 1.0
                distance = _distance3d(previous, node)
                if distance > jump_limit:
                    issues.append(CheckIssue(scenario, "UT-02", "相邻帧位移疑似瞬移", time_s=time_s, node=node_id, field="position", actual=round(distance, 3), limit=round(jump_limit, 3)))
                altitude_jump = abs(float(node.get("altitude_m", 0.0)) - float(previous.get("altitude_m", 0.0)))
                altitude_limit = climb * dt * thresholds.ALTITUDE_JUMP_MARGIN_FACTOR + 0.5
                if altitude_jump > altitude_limit:
                    issues.append(CheckIssue(scenario, "UT-02", "高度单帧跳变过大", time_s=time_s, node=node_id, field="altitude_m", actual=round(altitude_jump, 3), limit=round(altitude_limit, 3)))
                speed_jump = abs(float(node.get("speed_mps", 0.0)) - float(previous.get("speed_mps", 0.0)))
                speed_limit = accel * dt * thresholds.SPEED_JUMP_MARGIN_FACTOR + 0.5
                if speed_jump > speed_limit:
                    issues.append(CheckIssue(scenario, "UT-02", "速度单帧跳变过大", time_s=time_s, node=node_id, field="speed_mps", actual=round(speed_jump, 3), limit=round(speed_limit, 3)))
            previous_by_id[node_id] = node
        previous_time = time_s
    return issues


def check_dynamic_limits(run: ScenarioRun) -> list[CheckIssue]:
    """UT-03 动力学限幅。注意：speed/phi/nx/nz/psi_dot 的限值均由配置派生。"""

    issues: list[CheckIssue] = []
    scenario = run.scenario
    limits = run.config.get("model", {}).get("limits", {}) if isinstance(run.config.get("model"), dict) else {}
    control_limits = run.config.get("control", {}).get("velocity_command_limits", {}) if isinstance(run.config.get("control"), dict) else {}
    speed_min = float(control_limits.get("forward_min_mps", run.config.get("model", {}).get("min_speed_mps", 0.0)))
    speed_max = float(control_limits.get("forward_max_mps", float("inf")))
    phi_min = float(limits.get("phi_min_deg", -float("inf")))
    phi_max = float(limits.get("phi_max_deg", float("inf")))
    nx_min = float(limits.get("nx_min", -float("inf")))
    nx_max = float(limits.get("nx_max", float("inf")))
    nz_min = float(limits.get("nz_min", -float("inf")))
    nz_max = float(limits.get("nz_max", float("inf")))
    gravity = float(run.config.get("model", {}).get("gravity_mps2", 9.80665)) if isinstance(run.config.get("model"), dict) else 9.80665
    yaw_limit = math.degrees(gravity * math.tan(math.radians(max(abs(phi_min), abs(phi_max)))) / max(speed_min, 1.0))
    for snapshot in run.snapshots:
        time_s = _time(snapshot)
        for node in _nodes(snapshot):
            node_id = str(node.get("node_id", ""))
            _range_issue(issues, scenario, "UT-03", time_s, node_id, "speed_mps", node.get("speed_mps"), speed_min, speed_max)
            _range_issue(issues, scenario, "UT-03", time_s, node_id, "phi_deg", node.get("phi_deg"), phi_min, phi_max)
            _range_issue(issues, scenario, "UT-03", time_s, node_id, "nx", node.get("nx"), nx_min, nx_max)
            _range_issue(issues, scenario, "UT-03", time_s, node_id, "nz", node.get("nz"), nz_min, nz_max)
            psi_dot = abs(float(node.get("psi_dot_deg_s", 0.0)))
            if psi_dot > yaw_limit + thresholds.LIMIT_EPS:
                issues.append(CheckIssue(scenario, "UT-03", "航向角速率超过滚转/速度派生限幅", time_s=time_s, node=node_id, field="psi_dot_deg_s", actual=round(psi_dot, 3), limit=round(yaw_limit, 3)))
    return issues


def check_collision_distance(run: ScenarioRun) -> list[CheckIssue]:
    """UT-04 机间防撞。注意：单机场景自动跳过。"""

    issues: list[CheckIssue] = []
    for snapshot in run.snapshots:
        nodes = _nodes(snapshot)
        for i, left in enumerate(nodes):
            for right in nodes[i + 1:]:
                distance = _distance3d(left, right)
                if distance <= thresholds.COLLISION_DISTANCE_M:
                    issues.append(CheckIssue(run.scenario, "UT-04", "机间距低于防撞阈值", time_s=_time(snapshot), node=f"{left.get('node_id')}-{right.get('node_id')}", field="distance_m", actual=round(distance, 3), limit=thresholds.COLLISION_DISTANCE_M))
    return issues


def check_obstacle_clearance(run: ScenarioRun) -> list[CheckIssue]:
    """UT-05 避障有效性。注意：仅启用 avoidance 且有障碍时检查。"""

    avoidance = run.config.get("avoidance")
    if not isinstance(avoidance, dict) or not avoidance.get("enabled"):
        return []
    obstacles = _obstacles(run.config)
    clearance = float(avoidance.get("clearance_m", 0.0))
    issues: list[CheckIssue] = []
    for snapshot in run.snapshots:
        for node in _nodes(snapshot):
            for obstacle in obstacles:
                margin = _obstacle_margin(node, obstacle, clearance)
                if margin is not None and margin < -0.5:
                    issues.append(CheckIssue(run.scenario, "UT-05", "进入障碍膨胀区", time_s=_time(snapshot), node=str(node.get("node_id")), field=str(obstacle.get("id", "obstacle")), actual=round(margin, 3), limit=">=0"))
    return issues


def check_task_completion(run: ScenarioRun) -> list[CheckIssue]:
    """UT-06 任务完成。注意：按长机末端位置和速度方向检查。"""

    end = _route_end(run.config)
    leader_id = _leader_id(run.config)
    if end is None or leader_id is None or not run.snapshots:
        return []
    final = run.snapshots[-1]
    node = _node_by_id(final, leader_id)
    if node is None:
        return [CheckIssue(run.scenario, "UT-06", "末帧缺少长机节点", time_s=_time(final), node=leader_id)]
    issues: list[CheckIssue] = []
    error = math.hypot(float(node.get("x_m", 0.0)) - end[0], float(node.get("y_m", 0.0)) - end[1])
    if error > thresholds.TERMINAL_ERROR_M:
        issues.append(CheckIssue(run.scenario, "UT-06", "末端未到达航线终点", time_s=_time(final), node=leader_id, field="terminal_error_m", actual=round(error, 3), limit=thresholds.TERMINAL_ERROR_M))
    heading_error = _terminal_heading_error(run.config, node)
    if heading_error is not None and heading_error > thresholds.TERMINAL_HEADING_DEG:
        issues.append(CheckIssue(run.scenario, "UT-06", "末端航向偏离末航段", time_s=_time(final), node=leader_id, field="heading_error_deg", actual=round(heading_error, 3), limit=thresholds.TERMINAL_HEADING_DEG))
    return issues


def check_formation_convergence(run: ScenarioRun) -> list[CheckIssue]:
    """UT-07 编队收敛与保持。注意：仅 formation 场景检查后半段轨迹误差。"""

    if not isinstance(run.config.get("formation"), dict):
        return []
    if not run.snapshots:
        return []
    start_time = float(run.config.get("duration_s", 0.0)) * 0.5
    errors: list[float] = []
    final_errors: list[tuple[str, float]] = []
    for snapshot in run.snapshots:
        if _time(snapshot) < start_time:
            continue
        for node in _nodes(snapshot):
            if str(node.get("role", "")) == "leader":
                continue
            error = math.sqrt(
                float(node.get("track_pos_err_x_m", 0.0)) ** 2
                + float(node.get("track_pos_err_y_m", 0.0)) ** 2
                + float(node.get("track_pos_err_z_m", 0.0)) ** 2
            )
            errors.append(error)
    for node in _nodes(run.snapshots[-1]):
        if str(node.get("role", "")) == "leader":
            continue
        error = math.sqrt(
            float(node.get("track_pos_err_x_m", 0.0)) ** 2
            + float(node.get("track_pos_err_y_m", 0.0)) ** 2
            + float(node.get("track_pos_err_z_m", 0.0)) ** 2
        )
        final_errors.append((str(node.get("node_id", "")), error))
    issues: list[CheckIssue] = []
    if errors:
        mean_error = sum(errors) / len(errors)
        if mean_error > thresholds.FORMATION_ERROR_M:
            issues.append(CheckIssue(run.scenario, "UT-07", "后半段队形均值误差过大", field="formation_mean_error_m", actual=round(mean_error, 3), limit=thresholds.FORMATION_ERROR_M))
    for node_id, error in final_errors:
        if error > thresholds.FORMATION_ERROR_M:
            issues.append(CheckIssue(run.scenario, "UT-07", "末端队形误差过大", time_s=_time(run.snapshots[-1]), node=node_id, field="formation_error_m", actual=round(error, 3), limit=thresholds.FORMATION_ERROR_M))
    return issues


def _warn_allowed(message: str) -> bool:
    """判断 WARN 是否在白名单。注意：白名单集中在 thresholds。"""

    return any(part in message for part in thresholds.ALLOWED_WARN_MESSAGE_PARTS)


def _nodes(snapshot: dict[str, Any]) -> list[dict[str, Any]]:
    """读取快照节点列表。注意：过滤掉非字典项。"""

    return [node for node in snapshot.get("nodes", []) if isinstance(node, dict)]


def _time(record: dict[str, Any]) -> float:
    """读取记录时间。注意：缺失时返回 0。"""

    return float(record.get("time_s", 0.0))


def _finite(value: Any) -> bool:
    """判断值是否为有限数。注意：非数值字段按健康处理。"""

    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return True


def _forward_max(config: dict[str, Any]) -> float:
    """读取前向速度上限。注意：缺失时从航线速度兜底。"""

    control = config.get("control", {}) if isinstance(config.get("control"), dict) else {}
    limits = control.get("velocity_command_limits", {}) if isinstance(control.get("velocity_command_limits"), dict) else {}
    if "forward_max_mps" in limits:
        return float(limits["forward_max_mps"])
    route = config.get("route", {}) if isinstance(config.get("route"), dict) else {}
    return float(route.get("speed_mps", 100.0))


def _accel_limit(config: dict[str, Any]) -> float:
    """读取加速度限幅。注意：缺失时用当前配置常见值兜底。"""

    model = config.get("model", {}) if isinstance(config.get("model"), dict) else {}
    limits = model.get("limits", {}) if isinstance(model.get("limits"), dict) else {}
    return float(limits.get("acceleration_command_mps2", 20.0))


def _climb_limit(config: dict[str, Any]) -> float:
    """读取最大升降率。注意：上升和下降取较大值。"""

    model = config.get("model", {}) if isinstance(config.get("model"), dict) else {}
    limits = model.get("limits", {}) if isinstance(model.get("limits"), dict) else {}
    return max(float(limits.get("max_climb_rate_mps", 0.0)), float(limits.get("max_descent_rate_mps", 0.0)), 1.0)


def _range_issue(
    issues: list[CheckIssue], scenario: str, ut: str, time_s: float, node_id: str, field: str, value: Any, low: float, high: float
) -> None:
    """追加范围检查问题。注意：无穷边界自然跳过。"""

    number = float(value)
    if number < low - thresholds.LIMIT_EPS or number > high + thresholds.LIMIT_EPS:
        issues.append(CheckIssue(scenario, ut, "动力学字段超限", time_s=time_s, node=node_id, field=field, actual=round(number, 3), limit=f"[{low}, {high}]"))


def _terminal_heading_error(config: dict[str, Any], node: dict[str, Any]) -> float | None:
    """计算末端速度方向与末航段夹角。注意：航点不足时返回 None。"""

    route = config.get("route")
    if not isinstance(route, dict):
        return None
    waypoints = route.get("waypoints")
    if not isinstance(waypoints, list) or len(waypoints) < 2:
        return None
    start = waypoints[-2]
    end = waypoints[-1]
    if not isinstance(start, dict) or not isinstance(end, dict):
        return None
    dx = float(end.get("x_m", 0.0)) - float(start.get("x_m", 0.0))
    dy = float(end.get("y_m", 0.0)) - float(start.get("y_m", 0.0))
    vx = float(node.get("vx_mps", 0.0))
    vy = float(node.get("vy_mps", 0.0))
    route_norm = math.hypot(dx, dy)
    vel_norm = math.hypot(vx, vy)
    if route_norm <= 1e-9 or vel_norm <= 1e-9:
        return None
    cos_value = max(-1.0, min(1.0, (dx * vx + dy * vy) / (route_norm * vel_norm)))
    return math.degrees(math.acos(cos_value))
