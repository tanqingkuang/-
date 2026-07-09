"""T2 指标提取与回归比对。注意：只按恶化方向失败。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from tests.st.support import thresholds
from tests.st.support.types import CheckIssue


@dataclass(frozen=True)
class MetricComparison:
    """一条指标比对结果。注意：hint 不影响退出码。"""

    issue: CheckIssue | None = None
    hint: str | None = None


def extract_metrics(scenario: str, snapshots: list[dict[str, Any]], config: dict[str, Any]) -> dict[str, float | None]:
    """提取 ST 指标。注意：单机场景的机间距返回 None。"""

    return {
        "completion_time_s": _completion_time(snapshots, config),
        "min_inter_aircraft_distance_m": _min_inter_aircraft_distance(snapshots),
        "min_obstacle_margin_m": _min_obstacle_margin(snapshots, config) if _avoidance_enabled(config) else None,
    }


def compare_metrics(scenario: str, baseline: dict[str, Any], current: dict[str, Any]) -> list[MetricComparison]:
    """比较 T2 指标。注意：变好时返回提示，变差超容差才失败。"""

    results: list[MetricComparison] = []
    _compare_larger_is_worse(
        results,
        scenario,
        "UT-08",
        "完成时间劣化",
        baseline.get("completion_time_s"),
        current.get("completion_time_s"),
        thresholds.METRIC_COMPLETION_TIME_TOLERANCE_RATIO,
        field="completion_time_s",
    )
    _compare_smaller_is_worse(
        results,
        scenario,
        "UT-09",
        "最小机间距缩水",
        baseline.get("min_inter_aircraft_distance_m"),
        current.get("min_inter_aircraft_distance_m"),
        thresholds.METRIC_DISTANCE_SHRINK_TOLERANCE_RATIO,
        field="min_inter_aircraft_distance_m",
    )
    _compare_smaller_is_worse(
        results,
        scenario,
        "UT-09",
        "最小障碍裕度缩水",
        baseline.get("min_obstacle_margin_m"),
        current.get("min_obstacle_margin_m"),
        thresholds.METRIC_OBSTACLE_MARGIN_SHRINK_TOLERANCE_RATIO,
        field="min_obstacle_margin_m",
    )
    return results


def _compare_larger_is_worse(
    results: list[MetricComparison],
    scenario: str,
    ut: str,
    message: str,
    baseline: Any,
    current: Any,
    tolerance_ratio: float,
    *,
    field: str,
) -> None:
    """比较越大越坏的指标。注意：缺失指标跳过。"""

    if baseline is None or current is None:
        return
    base = float(baseline)
    now = float(current)
    limit = base * (1.0 + tolerance_ratio)
    if now > limit:
        results.append(MetricComparison(CheckIssue(scenario, ut, message, field=field, actual=round(now, 3), limit=round(limit, 3))))
    elif now < base:
        results.append(MetricComparison(hint=f"HINT [{scenario}][{ut}] {field} 变好: baseline={base:.3f} current={now:.3f}，可刷新基线"))


def _compare_smaller_is_worse(
    results: list[MetricComparison],
    scenario: str,
    ut: str,
    message: str,
    baseline: Any,
    current: Any,
    tolerance_ratio: float,
    *,
    field: str,
) -> None:
    """比较越小越坏的指标。注意：缺失指标跳过。"""

    if baseline is None or current is None:
        return
    base = float(baseline)
    now = float(current)
    limit = base - abs(base) * tolerance_ratio
    if now < limit:
        results.append(MetricComparison(CheckIssue(scenario, ut, message, field=field, actual=round(now, 3), limit=round(limit, 3))))
    elif now > base:
        results.append(MetricComparison(hint=f"HINT [{scenario}][{ut}] {field} 变好: baseline={base:.3f} current={now:.3f}，可刷新基线"))


def _leader_id(config: dict[str, Any]) -> str | None:
    """从配置读取长机 ID。注意：找不到显式长机时回退第一个节点。"""

    nodes = config.get("nodes", [])
    if not isinstance(nodes, list) or not nodes:
        return None
    for index, node in enumerate(nodes):
        if isinstance(node, dict) and str(node.get("role", "")) in {"leader", "rally_leader"}:
            return str(node.get("node_id", f"N{index + 1:02d}"))
    first = nodes[0]
    return str(first.get("node_id", "N01")) if isinstance(first, dict) else None


def _completion_time(snapshots: list[dict[str, Any]], config: dict[str, Any]) -> float | None:
    """提取首次到达终点时间。注意：未到达时返回末帧时间作为保守指标。"""

    end = _route_end(config)
    leader_id = _leader_id(config)
    if end is None or leader_id is None or not snapshots:
        return None
    for snapshot in snapshots:
        node = _node_by_id(snapshot, leader_id)
        if node is None:
            continue
        error = math.hypot(float(node.get("x_m", 0.0)) - end[0], float(node.get("y_m", 0.0)) - end[1])
        if error <= thresholds.TERMINAL_ERROR_M:
            return float(snapshot.get("time_s", 0.0))
    return float(snapshots[-1].get("time_s", 0.0))


def _min_inter_aircraft_distance(snapshots: list[dict[str, Any]]) -> float | None:
    """计算全程最小三维机间距。注意：单机返回 None。"""

    best: float | None = None
    for snapshot in snapshots:
        nodes = [node for node in snapshot.get("nodes", []) if isinstance(node, dict)]
        for i, left in enumerate(nodes):
            for right in nodes[i + 1:]:
                distance = _distance3d(left, right)
                best = distance if best is None else min(best, distance)
    return best


def _min_obstacle_margin(snapshots: list[dict[str, Any]], config: dict[str, Any]) -> float | None:
    """计算全程最小障碍裕度。注意：圆障碍按 radius+clearance 膨胀。"""

    obstacles = _obstacles(config)
    if not obstacles:
        return None
    clearance = float(config.get("avoidance", {}).get("clearance_m", 0.0)) if isinstance(config.get("avoidance"), dict) else 0.0
    best: float | None = None
    for snapshot in snapshots:
        for node in snapshot.get("nodes", []):
            if not isinstance(node, dict):
                continue
            for obstacle in obstacles:
                margin = _obstacle_margin(node, obstacle, clearance)
                if margin is not None:
                    best = margin if best is None else min(best, margin)
    return best


def _route_end(config: dict[str, Any]) -> tuple[float, float] | None:
    """读取航线终点。注意：使用已解析日志配置中的内部 ENU 坐标。"""

    route = config.get("route")
    if not isinstance(route, dict):
        return None
    waypoints = route.get("waypoints")
    if isinstance(waypoints, list) and waypoints:
        last = waypoints[-1]
        if isinstance(last, dict):
            return (float(last.get("x_m", last.get("east", 0.0))), float(last.get("y_m", last.get("north", 0.0))))
    return None


def _node_by_id(snapshot: dict[str, Any], node_id: str) -> dict[str, Any] | None:
    """从快照读取指定节点。注意：节点缺失时返回 None。"""

    for node in snapshot.get("nodes", []):
        if isinstance(node, dict) and str(node.get("node_id")) == node_id:
            return node
    return None


def _distance3d(left: dict[str, Any], right: dict[str, Any]) -> float:
    """计算三维距离。注意：字段单位为米。"""

    return math.sqrt(
        (float(left.get("x_m", 0.0)) - float(right.get("x_m", 0.0))) ** 2
        + (float(left.get("y_m", 0.0)) - float(right.get("y_m", 0.0))) ** 2
        + (float(left.get("altitude_m", 0.0)) - float(right.get("altitude_m", 0.0))) ** 2
    )


def _avoidance_enabled(config: dict[str, Any]) -> bool:
    """判断是否启用避障。注意：只看配置开关和障碍列表。"""

    avoidance = config.get("avoidance")
    return isinstance(avoidance, dict) and bool(avoidance.get("enabled")) and bool(avoidance.get("obstacles"))


def _obstacles(config: dict[str, Any]) -> list[dict[str, Any]]:
    """读取障碍列表。注意：只返回启用的字典障碍。"""

    avoidance = config.get("avoidance")
    if not isinstance(avoidance, dict):
        return []
    return [item for item in avoidance.get("obstacles", []) if isinstance(item, dict) and item.get("enabled", True)]


def _obstacle_margin(node: dict[str, Any], obstacle: dict[str, Any], clearance: float) -> float | None:
    """计算节点到障碍的水平裕度。注意：首期圆障碍精确支持，多边形按边界距离近似。"""

    x = float(node.get("x_m", 0.0))
    y = float(node.get("y_m", 0.0))
    kind = str(obstacle.get("type", obstacle.get("kind", "")))
    if kind == "circle":
        center = obstacle.get("center", {})
        if not isinstance(center, dict):
            return None
        cx = float(center.get("east_m", center.get("x_m", 0.0)))
        cy = float(center.get("north_m", center.get("y_m", 0.0)))
        return math.hypot(x - cx, y - cy) - float(obstacle.get("radius_m", obstacle.get("radius", 0.0))) - clearance
    vertices = obstacle.get("vertices")
    if kind == "polygon" and isinstance(vertices, list) and vertices:
        points = [(float(p.get("east_m", 0.0)), float(p.get("north_m", 0.0))) for p in vertices if isinstance(p, dict)]
        if len(points) < 3:
            return None
        distance = min(_point_segment_distance(x, y, points[i], points[(i + 1) % len(points)]) for i in range(len(points)))
        return -distance - clearance if _point_in_polygon(x, y, points) else distance - clearance
    return None


def _point_segment_distance(x: float, y: float, a: tuple[float, float], b: tuple[float, float]) -> float:
    """计算点到线段距离。注意：用于多边形障碍裕度。"""

    ax, ay = a
    bx, by = b
    dx = bx - ax
    dy = by - ay
    if dx == 0.0 and dy == 0.0:
        return math.hypot(x - ax, y - ay)
    t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(x - (ax + t * dx), y - (ay + t * dy))


def _point_in_polygon(x: float, y: float, points: list[tuple[float, float]]) -> bool:
    """判断点是否在多边形内。注意：射线法只服务 ST 裕度估算。"""

    inside = False
    j = len(points) - 1
    for i, (xi, yi) in enumerate(points):
        xj, yj = points[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi:
            inside = not inside
        j = i
    return inside
