"""可飞性校验：判断去冗余后的拐点折线（写 r=R 后）能否真正飞出来。

A* 只决定“从障碍哪边绕”（拓扑），本模块兜底“绕得过来吗”（可飞性），见 docs/避障-A星-开发计划.md §3.2/§9：
1. 急拐点：|Δψ| 过大（近掉头）→ ERR_AVOID_TURN_TOO_SHARP。
2. 腿太短：相邻拐点之间的腿无法容下两端圆弧占用 + 直线余度 L，
   即 leg < R·tan(Δψ_i/2) + R·tan(Δψ_{i+1}/2) + L → ERR_AVOID_LEG_TOO_SHORT。
3. 圆弧触障：拐点圆弧（向外凸）采样后落入真实障碍 → ERR_AVOID_ARC_HITS_OBSTACLE。

校验对象是“拐点折线 + R”，与 path_to_route.points_to_route 用同一 corner_arc 几何，
因此校验通过即代表它产出的 RouteS 可飞。圆弧触障默认按真实障碍（clearance=0）复核兜底（§8）。
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, ceil, cos, degrees, hypot, radians, sin, tan

from src.algorithm.context.leaf_types import PosInEarthS, WayLineS, WayPointS
from src.algorithm.units.algo.arc_path import arc_swept_rad, corner_arc

from .obstacle import ObstacleS, inside
from .path_to_route import _default_sample_step

Point = tuple[float, float]

# 失败原因码（呼应 sim_control 的 ResultCode 风格，见计划 §9）。
ERR_TURN_TOO_SHARP = "ERR_AVOID_TURN_TOO_SHARP"
ERR_LEG_TOO_SHORT = "ERR_AVOID_LEG_TOO_SHORT"
ERR_ARC_HITS_OBSTACLE = "ERR_AVOID_ARC_HITS_OBSTACLE"


@dataclass
class FeasibilityResult:
    """可飞性校验结论。注意：ok=True 时 code='OK'；失败时带原因码与定位信息。"""

    ok: bool
    code: str = "OK"
    detail: str = ""
    waypoint_index: int | None = None  # 触发失败的拐点序号（沿 points）
    leg_index: int | None = None  # 触发失败的腿序号（points[i]→points[i+1]）
    obstacle_id: str | None = None  # 触障的障碍 id


def _unit(dx: float, dy: float) -> tuple[float, float] | None:
    length = hypot(dx, dy)
    if length <= 1e-9:
        return None
    return dx / length, dy / length


def _deflection(p_prev: Point, p_corner: Point, p_next: Point) -> float | None:
    """相邻三点在 p_corner 处的航向偏转角 Δψ（带符号，左正）。注意：某段退化为零向量返回 None。"""
    u_in = _unit(p_corner[0] - p_prev[0], p_corner[1] - p_prev[1])
    u_out = _unit(p_next[0] - p_corner[0], p_next[1] - p_corner[1])
    if u_in is None or u_out is None:
        return None
    cross = u_in[0] * u_out[1] - u_in[1] * u_out[0]
    dot = max(-1.0, min(1.0, u_in[0] * u_out[0] + u_in[1] * u_out[1]))
    return atan2(cross, dot)


def _tangent_distance(deflection: float, radius: float) -> float:
    """切点到拐点距离 d = R·tan(|Δψ|/2)。注意：调用方须保证 |Δψ| 远离 π。"""
    return radius * tan(abs(deflection) / 2.0)


def _arc_sample_points(arc: tuple, radius: float, step: float) -> list[Point]:
    """采样圆弧 T1→T2 上的点（含两端）。注意：复用 arc_swept_rad 求扫掠角，按弧长密采。"""
    t1, t2, center, turn_sign = arc
    line = WayLineS(
        start=WayPointS(pos=PosInEarthS(t1.east, t1.north, t1.h)),
        end=WayPointS(pos=PosInEarthS(t2.east, t2.north, t2.h)),
        radius=radius,
        center=PosInEarthS(center.east, center.north, center.h),
        turnSign=turn_sign,
    )
    swept = arc_swept_rad(line)
    arc_length = radius * abs(swept)
    segments = max(1, ceil(arc_length / step))
    a_start = atan2(t1.north - center.north, t1.east - center.east)
    points: list[Point] = []
    for k in range(segments + 1):
        angle = a_start + swept * (k / segments)
        points.append((center.east + radius * cos(angle), center.north + radius * sin(angle)))
    return points


def _hit_obstacle(obstacles: list[ObstacleS], east: float, north: float, clearance: float) -> str | None:
    """返回首个命中（膨胀 clearance 后）该点的障碍 id；都不中返回 None。"""
    for obstacle in obstacles:
        if inside(obstacle, east, north, clearance):
            return obstacle.id
    return None


def check_feasibility(
    points: list[Point],
    obstacles: list[ObstacleS],
    *,
    turn_radius_m: float,
    leg_margin_m: float,
    arc_clearance: float = 0.0,
    sample_step: float | None = None,
    max_turn_deg: float = 150.0,
) -> FeasibilityResult:
    """校验拐点折线（写 r=R 后）是否可飞，返回首个失败原因或 OK。

    参数：
        points：去冗余后的拐点折线（≥2 点）。
        obstacles：真实障碍集（圆弧触障默认按真实障碍 clearance=0 复核）。
        turn_radius_m：配置转弯半径 R（人工配置）。
        leg_margin_m：直线余度 L（两端圆弧占用之外仍须保留的最短直线）。
        arc_clearance：圆弧触障判定的膨胀量，默认 0（真实障碍）。
        sample_step：圆弧采样步长，None 时按障碍尺寸/clearance 自动推。
        max_turn_deg：允许的最大偏转角，超过即判急拐点不可飞。
    返回：FeasibilityResult（ok 或带 ERR_AVOID_* 原因码 + 定位）。
    """
    if len(points) < 2:
        raise ValueError("check_feasibility needs at least two points")
    if turn_radius_m < 0.0:
        raise ValueError("turn_radius_m must be >= 0")
    if leg_margin_m < 0.0:
        raise ValueError("leg_margin_m must be >= 0")
    # sample_step 必须为正：=0 会在圆弧采样里除零；<0 会被 max(1,...) 压成只采两端，漏检中段触障圆弧。
    if sample_step is not None and sample_step <= 0.0:
        raise ValueError("sample_step must be > 0")
    if sample_step is None:
        sample_step = _default_sample_step(obstacles, arc_clearance)

    n = len(points)
    max_turn_rad = radians(max_turn_deg)

    # 各拐点偏转角与切点占用 d_i：首末点无转弯，d=0。
    deflections: list[float] = [0.0] * n
    tangent_d: list[float] = [0.0] * n
    for i in range(1, n - 1):
        deflection = _deflection(points[i - 1], points[i], points[i + 1])
        if deflection is None:
            continue  # 退化（重合点）：视为无转弯。
        deflections[i] = deflection
        # 急拐点：偏转角过大（近掉头），圆弧装不出来。
        if abs(deflection) > max_turn_rad:
            return FeasibilityResult(
                ok=False,
                code=ERR_TURN_TOO_SHARP,
                detail=f"拐点 {i} 偏转角 {degrees(abs(deflection)):.1f}° 超过上限 {max_turn_deg:.1f}°",
                waypoint_index=i,
            )
        tangent_d[i] = _tangent_distance(deflection, turn_radius_m)

    # 腿长校验：leg(i,i+1) ≥ d_i + d_{i+1} + L。
    for i in range(n - 1):
        leg_len = hypot(points[i + 1][0] - points[i][0], points[i + 1][1] - points[i][1])
        required = tangent_d[i] + tangent_d[i + 1] + leg_margin_m
        if leg_len + 1e-6 < required:
            return FeasibilityResult(
                ok=False,
                code=ERR_LEG_TOO_SHORT,
                detail=(
                    f"腿 {i}（拐点 {i}→{i + 1}）长 {leg_len:.1f}m < 需 {required:.1f}m"
                    f"（缺 {required - leg_len:.1f}m）"
                ),
                leg_index=i,
            )

    # 圆弧触障校验：逐拐点重建圆弧并采样，落入真实障碍即不可飞。
    if turn_radius_m > 0.0 and obstacles:
        for i in range(1, n - 1):
            if deflections[i] == 0.0:
                continue
            p_prev = PosInEarthS(points[i - 1][0], points[i - 1][1], 0.0)
            p_corner = PosInEarthS(points[i][0], points[i][1], 0.0)
            p_next = PosInEarthS(points[i + 1][0], points[i + 1][1], 0.0)
            arc = corner_arc(p_prev, p_corner, p_next, turn_radius_m)
            if arc is None:
                continue
            for east, north in _arc_sample_points(arc, turn_radius_m, sample_step):
                obstacle_id = _hit_obstacle(obstacles, east, north, arc_clearance)
                if obstacle_id is not None:
                    return FeasibilityResult(
                        ok=False,
                        code=ERR_ARC_HITS_OBSTACLE,
                        detail=f"拐点 {i} 的圆弧触及障碍 {obstacle_id}",
                        waypoint_index=i,
                        obstacle_id=obstacle_id,
                    )

    return FeasibilityResult(ok=True)
