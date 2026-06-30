"""避障编排：把 A* → 去冗余 → 圆弧 → 可飞性 串成 plan_avoidance_route。

职责（见 docs/避障-A星-开发计划.md §6 步骤5、§6.1）：按长机原航线逐段腿规划绕障，
拼接成一条连续的去冗余拐点折线，校验可飞后翻译为 RouteS。供 UI“生成航线”按钮与控制器调用。

设计：A* 只对每条腿 start→goal 求拓扑路径（保留原航线形状，不抄近路跨越中间航点）；
去冗余拉直、拐点圆弧、可飞性校验分层；任一步失败返回带 ERR_AVOID_* 原因码的 PlanResult。
高度：逐腿按水平里程在原航点高度之间线性插值，保持原高度剖面（二维避障只改水平）。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import hypot

from src.algorithm.context.leaf_types import WayPointInputS

from .astar import plan_path
from .feasibility import FeasibilityResult, check_route_feasibility
from .obstacle import ObstacleS, blocked, obstacle_bounds
from .path_to_route import (
    assign_transition_radius,
    bake_obstacle_hug_arcs,
    bake_transition_arcs,
    line_of_sight_clear,
    points_to_route,
    simplify_path_with_causes,
)

Point = tuple[float, float]
# (east, north, altitude) 三元组的原始航点。
Waypoint3D = tuple[float, float, float]

# 失败原因码（见计划 §9.1）。可飞性相关码由 check_feasibility 透传（ERR_AVOID_*）。
ERR_NO_PATH = "ERR_AVOID_NO_PATH"
ERR_ENDPOINT_IN_OBSTACLE = "ERR_AVOID_ENDPOINT_IN_OBSTACLE"


def _bounds_intersect(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> bool:
    """判断两个轴对齐包围盒是否相交。注意：用于航段级障碍预筛。"""
    return not (a[2] < b[0] or b[2] < a[0] or a[3] < b[1] or b[3] < a[1])


def _leg_candidate_obstacles(
    start: Point,
    goal: Point,
    obstacles: list[ObstacleS],
    clearance_m: float,
    margin_m: float,
) -> list[ObstacleS]:
    """筛出航段附近障碍。注意：只剔除超出航段搜索外扩范围的远场障碍。"""
    pad = clearance_m + margin_m
    # pad 与 A* 自动 bounds 外扩一致，远离该盒子的障碍不会参与本腿绕行。
    leg_bounds = (
        min(start[0], goal[0]) - pad,
        min(start[1], goal[1]) - pad,
        max(start[0], goal[0]) + pad,
        max(start[1], goal[1]) + pad,
    )
    candidates: list[ObstacleS] = []
    for obstacle in obstacles:
        # 障碍本体按 clearance 外扩后再和航段搜索盒求交，避免漏掉安全边界。
        min_e, min_n, max_e, max_n = obstacle_bounds(obstacle)
        obstacle_bounds_expanded = (
            min_e - clearance_m,
            min_n - clearance_m,
            max_e + clearance_m,
            max_n + clearance_m,
        )
        if _bounds_intersect(leg_bounds, obstacle_bounds_expanded):
            candidates.append(obstacle)
    return candidates


@dataclass
class PlanResult:
    """避障规划结论。注意：ok=True 时 route 可用；失败时带 ERR_AVOID_* 原因码与定位/诊断。"""

    ok: bool
    route: list[WayPointInputS] | None = None
    code: str = "OK"
    detail: str = ""
    leg_index: int | None = None  # 触发失败的原航线腿序号（waypoints[i]→[i+1]）
    obstacle_id: str | None = None
    simplified_points: list[Point] = field(default_factory=list)  # 去冗余后的拐点折线（诊断/预览用）
    feasibility: FeasibilityResult | None = None


def _interp_altitudes(points: list[Point], alt_start: float, alt_end: float) -> list[float]:
    """按水平里程在 alt_start→alt_end 间线性插值各点高度。注意：保持原航线高度剖面。"""
    if alt_start == alt_end or len(points) <= 1:
        return [alt_start] * len(points)
    cumulative = [0.0]
    for a, b in zip(points, points[1:]):
        cumulative.append(cumulative[-1] + hypot(b[0] - a[0], b[1] - a[1]))
    total = cumulative[-1]
    if total <= 0.0:
        return [alt_start] * len(points)
    return [alt_start + (alt_end - alt_start) * (d / total) for d in cumulative]


def plan_avoidance_route(
    waypoints: list[Waypoint3D],
    obstacles: list[ObstacleS],
    *,
    turn_radius_m: float,
    leg_margin_m: float,
    clearance_m: float,
    speed_mps: float,
    resolution_m: float,
    simplify_clearance_m: float | None = None,
    turn_switch_penalty_m: float = 0.0,
    turn_angle_weight_m: float = 0.0,
    margin_m: float = 0.0,
    arc_clearance: float = 0.0,
    sample_step: float | None = None,
    max_turn_deg: float = 150.0,
    allow_arc: bool = True,
) -> PlanResult:
    """对长机原航线逐段腿规划绕障，返回可飞的 RouteS 或带原因码的失败。

    参数：
        waypoints：长机原航线航点 [(east, north, altitude), ...]，至少 2 个。
        obstacles：本次启用的障碍集（按 clearance_m 膨胀做栅格规划）。
        turn_radius_m / leg_margin_m：配置转弯半径 R 与直线余度 L（可飞性校验用）。
        clearance_m：A* 栅格膨胀安全距离；simplify_clearance_m：A* 后视线去冗余使用的膨胀距离。
        simplify_clearance_m=None 时回退到 clearance_m，保持旧调用方行为。
        arc_clearance：圆弧触障复核膨胀（默认 0 真实障碍）。
        speed_mps：输出航段地速；resolution_m / margin_m：A* 栅格分辨率与范围外扩。
        turn_switch_penalty_m / turn_angle_weight_m：A* 搜索中用于减少航迹角切换的等效米代价。
        allow_arc：拐点是否烘焙成圆弧航段。True=航段本身即圆弧(turnSign!=0)，显示画弧；
            False=直线骨架+交接半径 r(turnSign=0)，显示画尖角、飞行时长机按 r 平滑过弯。
            注意：两种取值长机都平滑过弯(可飞性一致)，差异仅在航段表示/显示；
            check_feasibility 始终按真实 R 校验转弯可飞性。
    返回：PlanResult（ok+route 或 ERR_AVOID_* 原因码 + 定位 + 诊断点）。
    """
    if len(waypoints) < 2:
        raise ValueError("waypoints must contain at least two points")
    if simplify_clearance_m is None:
        simplify_clearance_m = clearance_m

    full_xy: list[Point] = []
    full_alt: list[float] = []
    full_causes: list[ObstacleS | None] = []
    for leg in range(len(waypoints) - 1):
        a = waypoints[leg]
        b = waypoints[leg + 1]
        start = (a[0], a[1])
        goal = (b[0], b[1])
        # 绝大多数长航段不需要绕障，先做连续线段判定，避免对原直线重复跑大范围 A*。
        direct_candidates = _leg_candidate_obstacles(start, goal, obstacles, clearance_m, 0.0)
        if line_of_sight_clear(start, goal, direct_candidates, clearance=clearance_m):
            # 原腿全程在膨胀障碍外，直接保留端点；这与 A* 最短路结果等价但省掉网格搜索。
            raw = [start, goal]
            leg_obstacles = direct_candidates
        else:
            leg_obstacles = _leg_candidate_obstacles(start, goal, obstacles, clearance_m, margin_m)
            # 只有被挡住的腿才跑 A*，且只把本腿搜索盒附近的障碍交给它。
            raw = plan_path(
                start, goal, leg_obstacles,
                resolution_m=resolution_m, clearance_m=clearance_m, margin_m=margin_m,
                turn_switch_penalty_m=turn_switch_penalty_m, turn_angle_weight_m=turn_angle_weight_m,
            )
        if raw is None:
            # 区分“端点落在膨胀障碍内”与“通道被封死”两类，便于诊断（见 §9.1）。
            blocked_a = blocked(obstacles, a[0], a[1], clearance_m)
            blocked_b = blocked(obstacles, b[0], b[1], clearance_m)
            if blocked_a or blocked_b:
                which = "起点" if blocked_a else "终点"
                return PlanResult(
                    ok=False, code=ERR_ENDPOINT_IN_OBSTACLE,
                    detail=f"腿 {leg} 的{which}落在膨胀障碍内", leg_index=leg,
                )
            return PlanResult(
                ok=False, code=ERR_NO_PATH,
                detail=f"腿 {leg} 无可行通道（通道被封死或绕行超出栅格范围）", leg_index=leg,
            )
        # 去冗余只需考虑本腿候选障碍；最终可飞性仍对全障碍集复核。
        simplified, causes = simplify_path_with_causes(raw, leg_obstacles, clearance=simplify_clearance_m)
        altitudes = _interp_altitudes(simplified, a[2], b[2])
        # 拼接：除首腿外丢掉与上一腿重合的衔接点（连同其 cause 标签一起丢）。
        if full_xy:
            simplified = simplified[1:]
            altitudes = altitudes[1:]
            causes = causes[1:]
        full_xy.extend(simplified)
        full_alt.extend(altitudes)
        full_causes.extend(causes)

    # 去掉相邻重合点，避免退化航段（cause 标签随点一并去重）。
    full_xy, full_alt, full_causes = _dedup(full_xy, full_alt, full_causes)
    if len(full_xy) < 2:
        return PlanResult(ok=False, code=ERR_NO_PATH, detail="规划结果退化为单点")

    # 先摆点、折叠贴障弧，再对"折叠后真正要飞的航线"做可飞性校验：
    # allow_arc=True 时把"连续贴同一圆"的拐点串折叠成一段沿膨胀圆的大弧(turnSign!=0)，避免 R<膨胀半径
    # 时被一串固定 R 小弧切碎；这些被合并的拐点不再受固定 R 腿长约束（校验后置的关键意义）。
    route = points_to_route(full_xy, speed_mps=speed_mps, altitudes=full_alt)  # 先只摆点(r=0)
    if allow_arc:  # 仅在航段允许带弧时折叠贴障弧；关闭则保留直线骨架交给固定 R 倒角
        route = bake_obstacle_hug_arcs(
            route, full_causes, obstacles,
            turn_radius_m=turn_radius_m, hug_clearance=simplify_clearance_m, sample_step=sample_step,
        )
    feasibility = check_route_feasibility(
        route, obstacles,
        turn_radius_m=turn_radius_m, leg_margin_m=leg_margin_m,
        arc_clearance=arc_clearance, sample_step=sample_step, max_turn_deg=max_turn_deg,
    )
    if not feasibility.ok:
        return PlanResult(
            ok=False, code=feasibility.code, detail=feasibility.detail,
            obstacle_id=feasibility.obstacle_id, simplified_points=full_xy, feasibility=feasibility,
        )

    # 校验通过后补交接半径：直线段是圆弧的外切线、R 即配置值，此处补 R 必不触障。
    # 弧两端相邻拐点的 r 会被 assign_transition_radius 自动清零（交接交给弧自身）。
    assign_transition_radius(route, turn_radius_m)
    # allow_arc=True：把剩余直线-直线拐点烘焙成圆弧航段(turnSign!=0)，航段本身即曲线，显示画弧；
    # allow_arc=False：保留直线骨架+交接半径 r(显示画尖角、飞行时长机按 r 平滑过弯)。
    if allow_arc:
        route = bake_transition_arcs(route)
    return PlanResult(ok=True, route=route, simplified_points=full_xy, feasibility=feasibility)


def _dedup(
    points: list[Point], altitudes: list[float], causes: list[ObstacleS | None]
) -> tuple[list[Point], list[float], list[ObstacleS | None]]:
    """去掉相邻水平重合的点（保留其首个高度与 cause 标签）。"""
    out_xy: list[Point] = []
    out_alt: list[float] = []
    out_cause: list[ObstacleS | None] = []
    for point, alt, cause in zip(points, altitudes, causes):
        if out_xy and hypot(point[0] - out_xy[-1][0], point[1] - out_xy[-1][1]) <= 1e-9:
            continue
        out_xy.append(point)
        out_alt.append(alt)
        out_cause.append(cause)
    return out_xy, out_alt, out_cause
