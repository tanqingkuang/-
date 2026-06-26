"""出口翻译：A* 锯齿格点 → 视线去冗余 → 拐点圆弧 → RouteS。

职责（见 docs/避障-A星-开发计划.md §6 步骤3）：
- simplify_path()：视线可达去冗余，把栅格锯齿拉直成尽量少的拐点（拉直段不得穿障碍）。
- points_to_route()：拐点写 WayPointS.r=R，复用 arc_path.corner_arc 生成相切圆弧，
  并沿用现有“切点落两腿内则插弧、否则回退直线”的退化保护，产出可被现有跟踪环消费的 RouteS。

圆弧是否真正可飞（腿长 ≥ d_in+d_out+L、圆弧不触障）留待步骤4 可飞性校验。
"""

from __future__ import annotations

from math import ceil, hypot

from src.algorithm.context.leaf_types import PosInEarthS, RouteS, WayLineS, WayPointS
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


def _same_xy(a: PosInEarthS, b: PosInEarthS) -> bool:
    """水平面是否重合（仅比较东/北）。注意：用于退化段保护，与 sim_control 口径一致。"""
    return abs(a.east - b.east) <= 1e-9 and abs(a.north - b.north) <= 1e-9


def _straight_wayline(idx: int, start: PosInEarthS, end: PosInEarthS, speed: float) -> WayLineS:
    """构造直线航段。注意：首末点深拷贝，避免相邻段共享引用。"""
    return WayLineS(
        idx=idx,
        start=WayPointS(idx=idx, pos=PosInEarthS(start.east, start.north, start.h)),
        end=WayPointS(idx=idx + 1, pos=PosInEarthS(end.east, end.north, end.h)),
        vdCmd=speed,
        radius=0.0,
    )


def _arc_wayline(
    idx: int, t1: PosInEarthS, t2: PosInEarthS, speed: float, radius: float, center: PosInEarthS, turn_sign: float
) -> WayLineS:
    """构造圆弧航段。注意：start/end 为切点，center 为圆心，turnSign 为转向。"""
    return WayLineS(
        idx=idx,
        start=WayPointS(idx=idx, pos=PosInEarthS(t1.east, t1.north, t1.h)),
        end=WayPointS(idx=idx + 1, pos=PosInEarthS(t2.east, t2.north, t2.h)),
        vdCmd=speed,
        radius=radius,
        center=PosInEarthS(center.east, center.north, center.h),
        turnSign=turn_sign,
    )


def points_to_route(
    points: list[Point],
    *,
    turn_radius_m: float,
    speed_mps: float,
    altitude_m: float = 0.0,
    altitudes: list[float] | None = None,
    insert_arcs: bool = True,
) -> RouteS:
    """把（去冗余后的）拐点折线转成 RouteS。

    insert_arcs=True：内部拐点写 r=turn_radius_m，复用 corner_arc 生成相切圆弧；切点须落在相邻两腿内，
    否则该拐点回退为直线（退化保护，与 sim_control._waylines_from_waypoints 同口径）。
    insert_arcs=False：所有拐点直连（外切线交付），全部直线航段穿过原拐点，供不支持圆弧航段的下游使用；
    转弯可飞性由上游 check_feasibility 按真实 R 校验，与编码无关。
    高度：默认整条用 altitude_m；传入 altitudes（与 points 等长）则逐点取值，用于保持原航线高度剖面。
    """
    if len(points) < 2:
        raise ValueError("points_to_route needs at least two points")
    if turn_radius_m < 0.0:
        raise ValueError("turn_radius_m must be >= 0")
    if speed_mps < 0.0:
        raise ValueError("speed_mps must be >= 0")
    if altitudes is not None and len(altitudes) != len(points):
        raise ValueError("altitudes length must match points")

    pts = [
        PosInEarthS(east, north, altitudes[i] if altitudes is not None else altitude_m)
        for i, (east, north) in enumerate(points)
    ]
    n = len(pts)
    # 首末拐点不做圆弧；内部拐点用配置 R。
    radii = [0.0] + [turn_radius_m] * (n - 2) + [0.0]

    lines: list[WayLineS] = []
    cur_start = pts[0]
    for index in range(1, n):
        corner = pts[index]
        arc = None
        if insert_arcs and index < n - 1 and radii[index] > 0.0:
            arc = corner_arc(pts[index - 1], corner, pts[index + 1], radii[index])
        if arc is not None:
            t1, t2, center, turn_sign = arc
            tangent_d = hypot(corner.east - t1.east, corner.north - t1.north)
            in_leg = hypot(corner.east - cur_start.east, corner.north - cur_start.north)
            out_leg = hypot(pts[index + 1].east - corner.east, pts[index + 1].north - corner.north)
            # 切点须落在两条腿内，否则回退为普通拐点（直线）。
            if tangent_d <= in_leg + 1e-6 and tangent_d <= out_leg + 1e-6:
                if not _same_xy(cur_start, t1):
                    lines.append(_straight_wayline(len(lines), cur_start, t1, speed_mps))
                lines.append(_arc_wayline(len(lines), t1, t2, speed_mps, radii[index], center, turn_sign))
                cur_start = t2
                continue
        lines.append(_straight_wayline(len(lines), cur_start, corner, speed_mps))
        cur_start = corner
    return RouteS(lines=lines)
