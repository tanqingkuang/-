"""出口翻译：A* 锯齿格点 → 视线去冗余 → WayPointInputS 列表。

职责（见 docs/避障-A星-设计文档.md §4.3、§6）：
- simplify_path()：视线可达去冗余，把栅格锯齿拉直成尽量少的拐点（拉直段不得穿障碍）。
- points_to_route()：只摆点，返回 list[WayPointInputS]，r 一律 0（不决策交接半径）。
- assign_transition_radius()：可飞性校验后，给两侧均为直线段的内部拐点补 r=turn_radius_m。
  圆弧几何由 leader.init() 中的 waypoint_inputs_to_waylines() 统一计算。

圆弧是否真正可飞（腿长 ≥ d_in+d_out+L、圆弧不触障）留待步骤4 可飞性校验。
"""

from __future__ import annotations

from math import ceil, hypot

from src.algorithm.context.leaf_types import PosInEarthS, WayPointInputS
from src.algorithm.units.algo.arc_path import corner_arc

from .obstacle import ObstacleS, blocked, obstacle_bounds

Point = tuple[float, float]


def _default_sample_step(obstacles: list[ObstacleS], clearance: float) -> float:
    """推一个足够密的视线采样步长：远小于最小障碍尺寸与 clearance，避免线段在两采样点间穿过细障碍。"""
    features: list[float] = []
    for obstacle in obstacles:
        if obstacle.kind == "circle":
            features.append(obstacle.radius)
        else:
            bmin_e, bmin_n, bmax_e, bmax_n = obstacle_bounds(obstacle)
            features.append(min(bmax_e - bmin_e, bmax_n - bmin_n))
    if clearance > 0.0:
        features.append(clearance)
    smallest = min((f for f in features if f > 0.0), default=10.0)
    return max(0.5, smallest / 4.0)


def line_of_sight_clear(
    start: Point,
    end: Point,
    obstacles: list[ObstacleS],
    *,
    clearance: float = 0.0,
    sample_step: float | None = None,
) -> bool:
    """线段 start→end 是否全程在（膨胀后的）障碍之外。注意：按 sample_step 密集采样逐点判定。"""
    if sample_step is not None and sample_step <= 0.0:
        raise ValueError("sample_step must be > 0")
    if not obstacles:
        return True
    if sample_step is None:
        sample_step = _default_sample_step(obstacles, clearance)
    length = hypot(end[0] - start[0], end[1] - start[1])
    # 用 ceil 保证实际采样间距 length/steps <= sample_step（int 向下取整会让间距超限、漏检细障碍）。
    steps = max(1, ceil(length / sample_step))
    for k in range(steps + 1):
        t = k / steps
        east = start[0] + (end[0] - start[0]) * t
        north = start[1] + (end[1] - start[1]) * t
        if blocked(obstacles, east, north, clearance):
            return False
    return True


def simplify_path(
    points: list[Point],
    obstacles: list[ObstacleS],
    *,
    clearance: float = 0.0,
    sample_step: float | None = None,
) -> list[Point]:
    """视线可达去冗余：贪心地把栅格折线拉直为尽量少的拐点，拉直段不得穿障碍。

    保留首点，从锚点尽量往前看；锚点看不到下一点时，当前点成为必须保留的拐点；末点恒保留。
    """
    if len(points) <= 2:
        return list(points)
    if sample_step is None:
        sample_step = _default_sample_step(obstacles, clearance)
    result: list[Point] = [points[0]]
    anchor = 0
    for i in range(1, len(points) - 1):
        # 锚点若看不到下一点，则当前点 i 是必须保留的转折；否则可跳过 i 继续拉直。
        if not line_of_sight_clear(
            points[anchor], points[i + 1], obstacles, clearance=clearance, sample_step=sample_step
        ):
            result.append(points[i])
            anchor = i
    result.append(points[-1])
    return result


def points_to_route(
    points: list[Point],
    *,
    speed_mps: float,
    altitude_m: float = 0.0,
    altitudes: list[float] | None = None,
) -> list[WayPointInputS]:
    """把（去冗余后的）拐点折线转成 WayPointInputS 列表：只摆点，r 一律 0。

    交接半径 r 由 assign_transition_radius() 在可飞性校验后统一补充，本函数不决策 r。
    高度：默认整条用 altitude_m；传入 altitudes（与 points 等长）则逐点取值。
    """
    if len(points) < 2:
        raise ValueError("points_to_route needs at least two points")
    if speed_mps < 0.0:
        raise ValueError("speed_mps must be >= 0")
    if altitudes is not None and len(altitudes) != len(points):
        raise ValueError("altitudes length must match points")

    result: list[WayPointInputS] = []
    for i, (east, north) in enumerate(points):
        alt = altitudes[i] if altitudes is not None else altitude_m
        result.append(WayPointInputS(idx=i, pos=PosInEarthS(east, north, alt), vdCmd=speed_mps, r=0.0))
    return result


def assign_transition_radius(inputs: list[WayPointInputS], turn_radius_m: float) -> list[WayPointInputS]:
    """补充函数：可飞性校验后，给两侧均为直线段的内部拐点补交接半径 R。

    判定按航段关系（与 allow_arc 无关，allow_arc 只决定航段自身是否曲线）：
    - 入段(i-1→i)、出段(i→i+1)都为直线（其起点 turnSign==0）→ inputs[i].r=turn_radius_m；
    - 任一邻段为曲线（turnSign!=0，如将来的贴障弧线段）→ inputs[i].r=0，交接交给曲线自身；
    - 首末点不补。
    原地改写 inputs[*].r 并返回同一列表。注意：每点都显式赋值，避免残留旧 r。
    """
    if turn_radius_m < 0.0:
        raise ValueError("turn_radius_m must be >= 0")
    n = len(inputs)
    for i in range(n):
        in_straight = i > 0 and inputs[i - 1].turnSign == 0.0  # 入段 (i-1→i) 是否直线
        out_straight = i < n - 1 and inputs[i].turnSign == 0.0  # 出段 (i→i+1) 是否直线
        is_interior = 0 < i < n - 1
        inputs[i].r = turn_radius_m if (is_interior and in_straight and out_straight) else 0.0
    return inputs


def bake_transition_arcs(route: list[WayPointInputS]) -> list[WayPointInputS]:
    """把带交接半径 r 的直线-直线拐点烘焙成相切圆弧段（turnSign!=0），供 allow_arc=True 使用。

    使航段本身成为圆弧（可被显示/下游当作曲率航段画弧）；turnSign==0 且 r<=0 的航点原样保留。
    与 leader.waypoint_inputs_to_waylines 情况2 同源：求相切圆弧，切点须落在两条腿内才烘焙，
    半径过大越出腿长时保留尖角。注意：烘焙后该拐点 r 清零（转弯信息已变成航段曲率）。
    """
    n = len(route)
    out: list[WayPointInputS] = []
    for i, wpi in enumerate(route):
        baked = False
        # 仅烘焙"内部、直线-直线、带 r"的拐点；其余原样保留。
        if 0 < i < n - 1 and wpi.r > 0.0 and wpi.turnSign == 0.0 and route[i - 1].turnSign == 0.0:
            arc = corner_arc(route[i - 1].pos, wpi.pos, route[i + 1].pos, wpi.r)
            if arc is not None:
                t1, t2, center, turn_sign = arc
                in_leg = hypot(wpi.pos.east - route[i - 1].pos.east, wpi.pos.north - route[i - 1].pos.north)
                out_leg = hypot(route[i + 1].pos.east - wpi.pos.east, route[i + 1].pos.north - wpi.pos.north)
                tangent_in = hypot(wpi.pos.east - t1.east, wpi.pos.north - t1.north)
                tangent_out = hypot(wpi.pos.east - t2.east, wpi.pos.north - t2.north)
                if tangent_in <= in_leg + 1e-9 and tangent_out <= out_leg + 1e-9:
                    out.append(
                        WayPointInputS(idx=len(out), pos=t1, vdCmd=route[i - 1].vdCmd, turnSign=turn_sign, center=center)
                    )
                    out.append(WayPointInputS(idx=len(out), pos=t2, vdCmd=wpi.vdCmd))
                    baked = True
        if not baked:
            out.append(
                WayPointInputS(idx=len(out), pos=wpi.pos, vdCmd=wpi.vdCmd, r=wpi.r, turnSign=wpi.turnSign, center=wpi.center)
            )
    return out
