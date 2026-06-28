"""出口翻译：A* 锯齿格点 → 视线去冗余 → WayPointInputS 列表。

职责（见 docs/避障-A星-开发计划.md §6 步骤3）：
- simplify_path()：视线可达去冗余，把栅格锯齿拉直成尽量少的拐点（拉直段不得穿障碍）。
- points_to_route()：拐点设 r=turn_radius_m，返回 list[WayPointInputS]；
  圆弧几何由 leader.init() 中的 _waypoint_inputs_to_waylines() 统一计算。

圆弧是否真正可飞（腿长 ≥ d_in+d_out+L、圆弧不触障）留待步骤4 可飞性校验。
"""

from __future__ import annotations

from math import ceil, hypot

from src.algorithm.context.leaf_types import PosInEarthS, WayPointInputS

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
    turn_radius_m: float,
    speed_mps: float,
    altitude_m: float = 0.0,
    altitudes: list[float] | None = None,
    insert_arcs: bool = True,
) -> list[WayPointInputS]:
    """把（去冗余后的）拐点折线转成 WayPointInputS 列表。

    内部拐点设 r=turn_radius_m（insert_arcs=True），圆弧几何由 leader.init() 统一计算。
    insert_arcs=False：r=0，全部直线直连。
    高度：默认整条用 altitude_m；传入 altitudes（与 points 等长）则逐点取值。
    """
    if len(points) < 2:
        raise ValueError("points_to_route needs at least two points")
    if turn_radius_m < 0.0:
        raise ValueError("turn_radius_m must be >= 0")
    if speed_mps < 0.0:
        raise ValueError("speed_mps must be >= 0")
    if altitudes is not None and len(altitudes) != len(points):
        raise ValueError("altitudes length must match points")

    n = len(points)
    result: list[WayPointInputS] = []
    for i, (east, north) in enumerate(points):
        alt = altitudes[i] if altitudes is not None else altitude_m
        r = turn_radius_m if (insert_arcs and 0 < i < n - 1) else 0.0
        result.append(WayPointInputS(idx=i, pos=PosInEarthS(east, north, alt), vdCmd=speed_mps, r=r))
    return result
