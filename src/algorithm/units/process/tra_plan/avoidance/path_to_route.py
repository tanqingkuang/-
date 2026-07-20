"""出口翻译：A* 锯齿格点 → 视线去冗余 → WayPointInputS 列表。

职责（见 docs/避障算法设计文档/避障-A星-设计文档.md §4.3、§6）：
- simplify_path()：视线可达去冗余，把栅格锯齿拉直成尽量少的拐点（拉直段不得穿障碍）。
- points_to_route()：只摆点，返回 list[WayPointInputS]，r 一律 0（不决策交接半径）。
- assign_transition_radius()：可飞性校验后，给两侧均为直线段的内部拐点补 r=turn_radius_m。
  圆弧几何由 tra_plan.leader_route.waypoint_inputs_to_waylines() 统一计算。

圆弧是否真正可飞（腿长 ≥ d_in+d_out+L、圆弧不触障）留待步骤4 可飞性校验。
"""

from __future__ import annotations

from dataclasses import replace
from math import atan2, ceil, cos, hypot, pi, sin

from src.algorithm.context.leaf_types import PosInEarthS, WayLineS, WayPointInputS
from src.algorithm.units.algo.arc_path import arc_swept_rad, common_tangent, corner_arc, tangent_point

from .obstacle import ObstacleS, blocked, inside, obstacle_bounds

Point = tuple[float, float]


def _bounds_intersect(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> bool:
    """判断两个轴对齐包围盒是否相交。注意：边界接触视为相交。"""
    return not (a[2] < b[0] or b[2] < a[0] or a[3] < b[1] or b[3] < a[1])


def _segment_candidate_obstacles(
    start: Point,
    end: Point,
    obstacles: list[ObstacleS],
    clearance: float,
) -> list[ObstacleS]:
    """按线段扩展包围盒预筛障碍。注意：只剔除几何上不可能命中的远场障碍。"""
    if not obstacles:
        return []
    segment_bounds = (
        min(start[0], end[0]) - clearance,
        min(start[1], end[1]) - clearance,
        max(start[0], end[0]) + clearance,
        max(start[1], end[1]) + clearance,
    )
    candidates: list[ObstacleS] = []
    for obstacle in obstacles:
        # 障碍 expanded bbox 与线段 expanded bbox 不相交时，逐点采样也不可能命中。
        min_e, min_n, max_e, max_n = obstacle_bounds(obstacle)
        obstacle_expanded = (min_e - clearance, min_n - clearance, max_e + clearance, max_n + clearance)
        if _bounds_intersect(segment_bounds, obstacle_expanded):
            candidates.append(obstacle)
    return candidates


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


def _first_blocker(
    start: Point,
    end: Point,
    obstacles: list[ObstacleS],
    *,
    clearance: float = 0.0,
    sample_step: float | None = None,
) -> ObstacleS | None:
    """线段 start→end 沿密采样首个命中（膨胀 clearance 后）的障碍；全程在外返回 None。

    返回的障碍即"逼停这条腿的那个"，供 simplify_path_with_causes 标记拐点来源。
    """
    if sample_step is not None and sample_step <= 0.0:
        raise ValueError("sample_step must be > 0")
    if not obstacles:
        return None
    candidates = _segment_candidate_obstacles(start, end, obstacles, clearance)
    if not candidates:
        # 没有候选障碍时整条线段必然清空，可直接跳过采样循环。
        return None
    if sample_step is None:
        sample_step = _default_sample_step(candidates, clearance)
    length = hypot(end[0] - start[0], end[1] - start[1])
    # 用 ceil 保证实际采样间距 length/steps <= sample_step（int 向下取整会让间距超限、漏检细障碍）。
    steps = max(1, ceil(length / sample_step))
    for k in range(steps + 1):
        t = k / steps
        east = start[0] + (end[0] - start[0]) * t
        north = start[1] + (end[1] - start[1]) * t
        for obstacle in candidates:
            # 只检查线段附近候选障碍，避免长航段对全图障碍重复做 inside 判定。
            if inside(obstacle, east, north, clearance):
                return obstacle  # 沿线首个命中的障碍即"逼停"这条腿者，作为拐点来源
    return None


def line_of_sight_clear(
    start: Point,
    end: Point,
    obstacles: list[ObstacleS],
    *,
    clearance: float = 0.0,
    sample_step: float | None = None,
) -> bool:
    """线段 start→end 是否全程在（膨胀后的）障碍之外。注意：按 sample_step 密集采样逐点判定。"""
    return _first_blocker(start, end, obstacles, clearance=clearance, sample_step=sample_step) is None


def simplify_path_with_causes(
    points: list[Point],
    obstacles: list[ObstacleS],
    *,
    clearance: float = 0.0,
    sample_step: float | None = None,
) -> tuple[list[Point], list[ObstacleS | None]]:
    """同 simplify_path，并额外返回每个保留拐点"被哪个障碍逼出"的来源标签 cause（与返回点等长）。

    cause[i]=障碍：拉直 anchor→i+1 被该障碍挡住、从而把 i 留作拐点；首末点与无障碍点为 None。
    供贴障弧识别：连续 cause 同一圆即"正在绕该圆"，无需用到圆心距离容差。
    """
    if len(points) <= 2:
        return list(points), [None] * len(points)
    if sample_step is None:
        sample_step = _default_sample_step(obstacles, clearance)
    result: list[Point] = [points[0]]
    causes: list[ObstacleS | None] = [None]
    anchor = 0
    for i in range(1, len(points) - 1):
        # 锚点若看不到下一点，则当前点 i 是必须保留的转折；否则可跳过 i 继续拉直。
        blocker = _first_blocker(
            points[anchor], points[i + 1], obstacles, clearance=clearance, sample_step=sample_step
        )
        if blocker is not None:
            result.append(points[i])
            causes.append(blocker)  # i 被保留，记下逼出它的障碍
            anchor = i  # 锚点前移到新拐点，从这里继续往前拉直
    result.append(points[-1])
    causes.append(None)  # 末点恒保留、无来源
    return result, causes


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
    points_only, _ = simplify_path_with_causes(
        points, obstacles, clearance=clearance, sample_step=sample_step
    )
    return points_only


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
        result.append(WayPointInputS(pos=PosInEarthS(east, north, alt), vdCmd=speed_mps, r=0.0))
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
    与 leader_route.waypoint_inputs_to_waylines 情况2 同源：求相切圆弧，切点须落在两条腿内才烘焙，
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
                        WayPointInputS(pos=t1, vdCmd=route[i - 1].vdCmd, turnSign=turn_sign, center=center)
                    )
                    out.append(WayPointInputS(pos=t2, vdCmd=wpi.vdCmd))
                    baked = True
        if not baked:
            out.append(
                WayPointInputS(pos=wpi.pos, vdCmd=wpi.vdCmd, r=wpi.r, turnSign=wpi.turnSign, center=wpi.center)
            )
    return out


def _hug_run(
    route: list[WayPointInputS],
    causes: list[ObstacleS | None],
    k: int,
    *,
    turn_radius_m: float,
    hug_clearance: float,
) -> tuple[int, int, ObstacleS, float] | None:
    """从 k 起找"连续贴同一圆同一侧"的极大顶点串；返回 (i0, j0, obstacle, turn_sign) 或 None。

    条件：k 为内部点、cause 为圆形障碍、膨胀半径≥R（贴得住）；沿圆心极角单调同向推进。
    串长须≥2（单顶点擦边交给固定 R 倒角），turn_sign 由角度推进方向给出。
    """
    n = len(route)
    if not (1 <= k <= n - 2):
        return None
    obstacle = causes[k]
    if obstacle is None or obstacle.kind != "circle":
        return None
    radius = obstacle.radius + hug_clearance
    if radius + 1e-9 < turn_radius_m:  # 膨胀圆比最小转弯半径还紧，贴不住，退回固定 R
        return None
    center = obstacle.center

    def ang(idx: int) -> float:
        """求 route[idx] 相对障碍圆心的极角(弧度)，用于判断贴障串的角度推进方向。"""
        return atan2(route[idx].pos.north - center.north, route[idx].pos.east - center.east)

    j = k
    sign = 0.0
    while j < n - 2 and causes[j + 1] is not None and causes[j + 1].id == obstacle.id:
        delta = atan2(sin(ang(j + 1) - ang(j)), cos(ang(j + 1) - ang(j)))  # wrap 到 (-pi,pi]
        if abs(delta) < 1e-9:  # 退化重合，停止
            break
        step_sign = 1.0 if delta > 0.0 else -1.0  # 本步极角增减向 = 绕圆心方向
        if sign == 0.0:
            sign = step_sign  # 首步定下整串转向(turnSign)
        elif step_sign != sign:  # 转向反号：贴一下→绕回，断成两段
            break
        j += 1
    if j == k:  # 只有单个顶点，不合弧
        return None
    return k, j, obstacle, sign


def _sample_hug_arc(
    t1: PosInEarthS,
    t2: PosInEarthS,
    center: PosInEarthS,
    turn_sign: float,
    radius: float,
    step: float,
) -> list[Point]:
    """采样贴障弧 T1→T2 上的点（含两端），用于触障复核。复用 arc_swept_rad 求扫掠角后按弧长密采。"""
    line = WayLineS(
        start=PosInEarthS(t1.east, t1.north, 0.0),
        end=PosInEarthS(t2.east, t2.north, 0.0),
        turnSign=turn_sign,
        center=PosInEarthS(center.east, center.north, 0.0),
    )
    swept = arc_swept_rad(line)  # 带符号扫掠角，决定采样沿弧推进方向
    segments = max(1, ceil(radius * abs(swept) / step))  # 按弧长/步长定段数，至少 1 段
    a_start = atan2(t1.north - center.north, t1.east - center.east)  # 切入点极角作采样起点
    points: list[Point] = []
    for m in range(segments + 1):
        angle = a_start + swept * (m / segments)  # 自起点极角按比例推进到切出点
        points.append((center.east + radius * cos(angle), center.north + radius * sin(angle)))
    return points


def _hug_arc_span_sane(
    t_in: PosInEarthS,
    t_out: PosInEarthS,
    center: PosInEarthS,
    turn_sign: float,
    vertices: list[PosInEarthS],
) -> bool:
    """护栏：贴障弧的扫掠角不应明显超过原贴障顶点的角度跨度。

    重选进/出切点后，若两者落在原贴障侧的反面，弧会沿长边绕大半圈(扫掠可达 ~322°)，却仍贴在
    膨胀圆上躲过触障复核。对比扫掠角与"首顶点→尾顶点沿转向的跨度"：欠扫(提前离开贴障走切线、
    切角)是允许的；明显超扫(超过跨度 + 90° 余量)即判定切点选反、绕了长边，拒绝折叠。
    """
    line = WayLineS(
        start=PosInEarthS(t_in.east, t_in.north, 0.0),
        end=PosInEarthS(t_out.east, t_out.north, 0.0),
        turnSign=turn_sign,
        center=PosInEarthS(center.east, center.north, 0.0),
    )
    swept = abs(arc_swept_rad(line))  # 弧扫掠角(绝对值)
    two_pi = 2.0 * pi
    a_first = atan2(vertices[0].north - center.north, vertices[0].east - center.east)
    a_last = atan2(vertices[-1].north - center.north, vertices[-1].east - center.east)
    span = ((a_last - a_first) * turn_sign) % two_pi  # 首→尾顶点沿转向的角度跨度
    return swept <= span + pi / 2.0  # 允许欠扫/切角与小幅超扫，拒绝长边绕圈


def _arc_clear(
    t1: PosInEarthS,
    t2: PosInEarthS,
    center: PosInEarthS,
    turn_sign: float,
    radius: float,
    obstacles: list[ObstacleS],
    sample_step: float,
) -> bool:
    """贴障弧采样点对真实障碍(clearance=0)的触障复核。贴的那个圆因半径=r+间距，自身不会误命中。"""
    for east, north in _sample_hug_arc(t1, t2, center, turn_sign, radius, sample_step):
        if blocked(obstacles, east, north, 0.0):
            return False
    return True


def _collect_hug_runs(
    route: list[WayPointInputS],
    causes: list[ObstacleS | None],
    *,
    turn_radius_m: float,
    hug_clearance: float,
) -> list[tuple[int, int, ObstacleS, float]]:
    """从左到右收集所有"连续贴同一圆同一侧"的极大顶点串(互不重叠)。"""
    runs: list[tuple[int, int, ObstacleS, float]] = []
    n = len(route)
    k = 0
    while k < n:
        run = _hug_run(route, causes, k, turn_radius_m=turn_radius_m, hug_clearance=hug_clearance)
        if run is not None:
            runs.append(run)
            k = run[1] + 1
        else:
            k += 1
    return runs


def _hug_endpoints(
    route: list[WayPointInputS],
    runs: list[tuple[int, int, ObstacleS, float]],
    hug_clearance: float,
    *,
    use_common_tangent: bool,
) -> list[tuple[PosInEarthS | None, PosInEarthS | None]]:
    """为每段贴障弧算入/出切点。相邻两段(同串无自由顶点间隔)用两圆公切线衔接，否则对相邻自由点作点-圆切线。

    use_common_tangent=False 时退化为全部按点-圆切线(各圆独立，不做公切线)。
    任一切点求解失败置 None，交由调用方决定跳过或整体回退。
    """
    m_count = len(runs)
    endpoints: list[tuple[PosInEarthS | None, PosInEarthS | None]] = []
    for m in range(m_count):
        i0, j0, obstacle, sign = runs[m]
        center, radius = obstacle.center, obstacle.radius + hug_clearance
        # 相邻段=同串里上/下一段贴障弧紧挨本段(中间无自由顶点)，此时衔接直线应是两圆公切线。
        left_adj = use_common_tangent and m > 0 and runs[m - 1][1] + 1 == i0
        right_adj = use_common_tangent and m < m_count - 1 and j0 + 1 == runs[m + 1][0]
        if left_adj:
            # 入切点取与左邻圆的公切线在本圆一端(ct[1])。
            p_obs, p_sign = runs[m - 1][2], runs[m - 1][3]
            ct = common_tangent(p_obs.center, p_obs.radius + hug_clearance, p_sign, center, radius, sign)
            t_in = PosInEarthS(ct[1][0], ct[1][1], route[i0].pos.h) if ct else None
        else:
            # 无左邻：对前一个自由点作点-圆切线(进入侧)。
            tp = tangent_point(route[i0 - 1].pos, center, radius, sign, leaving=False)
            t_in = PosInEarthS(tp[0], tp[1], route[i0].pos.h) if tp else None
        if right_adj:
            n_obs, n_sign = runs[m + 1][2], runs[m + 1][3]
            ct = common_tangent(center, radius, sign, n_obs.center, n_obs.radius + hug_clearance, n_sign)
            t_out = PosInEarthS(ct[0][0], ct[0][1], route[j0].pos.h) if ct else None
        else:
            tp = tangent_point(route[j0 + 1].pos, center, radius, sign, leaving=True)
            t_out = PosInEarthS(tp[0], tp[1], route[j0].pos.h) if tp else None
        endpoints.append((t_in, t_out))
    return endpoints


def _assemble_hug_arcs(
    route: list[WayPointInputS],
    runs: list[tuple[int, int, ObstacleS, float]],
    obstacles: list[ObstacleS],
    *,
    hug_clearance: float,
    sample_step: float,
    use_common_tangent: bool,
) -> list[WayPointInputS] | None:
    """按 runs 折叠贴障弧并组装新航点序列。每段弧+其衔接直线做真实障碍触障复核。

    use_common_tangent=True：相邻段用公切线，要求所有段都通过校验，否则返回 None(让上层退化重试)；
    use_common_tangent=False：各圆独立，逐段校验，失败的段保留原顶点(部分折叠)，恒返回一条航线。
    """
    n = len(route)
    endpoints = _hug_endpoints(route, runs, hug_clearance, use_common_tangent=use_common_tangent)
    collapse = [False] * len(runs)
    for m in range(len(runs)):
        i0, j0, obstacle, sign = runs[m]
        t_in, t_out = endpoints[m]
        if t_in is None or t_out is None:
            if use_common_tangent:
                return None
            continue
        center, radius = obstacle.center, obstacle.radius + hug_clearance
        # 入弧直线起点：左邻为相邻段则取其出切点(公切线另一端)，否则取相邻自由点。
        left_adj = use_common_tangent and m > 0 and runs[m - 1][1] + 1 == i0
        right_adj = use_common_tangent and m < len(runs) - 1 and j0 + 1 == runs[m + 1][0]
        src = endpoints[m - 1][1] if left_adj else route[i0 - 1].pos
        vertices = [route[idx].pos for idx in range(i0, j0 + 1)]  # 原贴障顶点
        ok = (
            src is not None
            # 护栏：弧扫掠角不应明显超过顶点跨度，否则切点选反、绕长边，拒绝折叠。
            and _hug_arc_span_sane(t_in, t_out, center, sign, vertices)
            and _arc_clear(t_in, t_out, center, sign, radius, obstacles, sample_step)
            and _first_blocker((src.east, src.north), (t_in.east, t_in.north), obstacles, sample_step=sample_step) is None
        )
        # 出弧直线仅在"自由尾"时校验(相邻段那条衔接线由下一段的入弧直线负责)。
        if ok and not right_adj:
            tail = route[j0 + 1].pos
            ok = _first_blocker((t_out.east, t_out.north), (tail.east, tail.north), obstacles, sample_step=sample_step) is None
        if not ok:
            if use_common_tangent:
                return None
            continue
        collapse[m] = True

    start_to_run = {runs[m][0]: m for m in range(len(runs))}
    out: list[WayPointInputS] = []
    k = 0
    while k < n:
        m = start_to_run.get(k, -1)
        if m >= 0 and collapse[m]:
            i0, j0, obstacle, sign = runs[m]
            t_in, t_out = endpoints[m]
            arc_center = PosInEarthS(obstacle.center.east, obstacle.center.north, route[i0].pos.h)
            # 弧用两航点表示：切入点(带 turnSign/圆心) + 切出点(直线)，整串 i0..j0 被这两点取代。
            out.append(WayPointInputS(pos=t_in, vdCmd=route[i0 - 1].vdCmd, turnSign=sign, center=arc_center))
            out.append(WayPointInputS(pos=t_out, vdCmd=route[j0].vdCmd))
            k = j0 + 1  # 跳过被折叠的整段贴障顶点
        else:
            out.append(replace(route[k]))  # 非折叠点复制保留，避免复用可变航点对象
            k += 1
    return out


def bake_obstacle_hug_arcs(
    route: list[WayPointInputS],
    causes: list[ObstacleS | None],
    obstacles: list[ObstacleS],
    *,
    turn_radius_m: float,
    hug_clearance: float,
    sample_step: float | None = None,
) -> list[WayPointInputS]:
    """把"连续贴同一圆同一侧"的拐点串折叠成一段真圆弧航段(turnSign!=0、圆心=障碍中心、半径=膨胀半径)。

    解决"最小转弯半径 R < 障碍膨胀半径"时，贴障路径被一串固定 R 小圆弧切碎的问题：
    既然 R 只是最小转弯半径，飞机可沿更大的膨胀圆贴飞，整段绕障归一为一条干净大弧。
    仅圆形障碍；膨胀半径<R 时贴不住，跳过。生成弧与衔接直线做真实障碍触障复核。
    相邻两段贴障弧之间优先用两圆公切线衔接(无拐点)；公切线方案整体不可飞时退化为各圆独立的部分折叠。
    供 allow_arc=True 在 assign_transition_radius 之前调用：弧两端 turnSign!=0，相邻交接半径会被自动清零。
    """
    n = len(route)
    if n < 3 or len(causes) != n:
        return route  # 不足三点无内部拐点可折叠，或 cause 标签未对齐，原样返回
    if sample_step is None:
        sample_step = _default_sample_step(obstacles, 0.0)  # 触障复核按真实障碍(clearance=0)定步长
    runs = _collect_hug_runs(route, causes, turn_radius_m=turn_radius_m, hug_clearance=hug_clearance)
    if not runs:
        return route  # 没有连续贴同一圆的串，无弧可折叠
    smooth = _assemble_hug_arcs(
        route, runs, obstacles, hug_clearance=hug_clearance, sample_step=sample_step, use_common_tangent=True
    )
    if smooth is not None:
        return smooth
    # 公切线方案整体不可飞：退化为各圆独立、逐段折叠（恒返回一条航线）。
    return _assemble_hug_arcs(
        route, runs, obstacles, hug_clearance=hug_clearance, sample_step=sample_step, use_common_tangent=False
    )
