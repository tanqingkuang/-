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

from src.algorithm.context.leaf_types import PosInEarthS, WayLineS, WayPointInputS, WayPointS  # noqa: F401  WayPointS via WayLineS
from src.algorithm.units.algo.arc_path import arc_swept_rad, corner_arc

from .obstacle import ObstacleS, inside
from .path_to_route import _default_sample_step

Point = tuple[float, float]

# 失败原因码（呼应 sim_control 的 ResultCode 风格，见计划 §9）。
ERR_TURN_TOO_SHARP = "ERR_AVOID_TURN_TOO_SHARP"
ERR_LEG_TOO_SHORT = "ERR_AVOID_LEG_TOO_SHORT"
ERR_ARC_HITS_OBSTACLE = "ERR_AVOID_ARC_HITS_OBSTACLE"

# 可飞性模块的输入已经是去冗余折线，因此这里不负责删除重复点或平滑路径。
# 本模块只回答“这条折线按指定转弯半径写成圆弧后能不能飞”，不重新选择绕行拓扑。
# 与 A* 不同，圆弧触障默认使用真实障碍 clearance=0，这是最后一道安全复核。
# arc_clearance 仅用于需要额外保守校验的场景，默认不重复叠加 A* 的膨胀量。
# max_turn_deg 用来拦截近似掉头拐点，避免 tan(|Δψ|/2) 在 π 附近发散。
# tangent_d[i] 表示第 i 个拐点两侧圆弧各自需要占用的切线距离。
# 腿长约束把相邻两个拐点的切线占用和直线余度一起纳入，防止圆弧互相重叠。
# leg_margin_m 是用户配置的额外直线余量，不是障碍安全距离。
# sample_step 为 None 时从障碍尺寸推导，保证圆弧触障不会因为采样过稀漏掉小障碍。
# 显式 sample_step 必须大于 0；否则圆弧采样会除零或只采端点，造成假通过。
# _deflection 返回带符号偏转角，正负只影响圆弧方向；可飞性阈值使用绝对值。
# 退化短腿会使 _unit 返回 None，此时偏转角视为无效，由腿长约束继续兜底。
# corner_arc 与 path_to_route 使用同一几何工具，保证校验和实际写 RouteS 的圆弧一致。
# _arc_sample_points 会包含圆弧两端，端点触障也会被识别为不可飞。
# _hit_obstacle 返回首个障碍 id，便于 UI 或日志把失败点定位到具体障碍。
# FeasibilityResult 只返回首个失败原因，调用方可修正后再次规划，不在这里累计全部错误。
# waypoint_index 对应原始 points 下标；leg_index 对应 points[i] 到 points[i+1]。
# 当 turn_radius_m 为 0 时圆弧触障阶段跳过，折线按直线腿处理。
# 入参合法性错误抛 ValueError，几何不可飞则返回 ok=False，二者语义不同。
# 成功返回 OK 不代表全局最优，只代表当前折线满足转弯、腿长和圆弧触障约束。
# 本模块不检查直线腿是否穿越障碍，因为 A* 阶段已经在膨胀网格上规避了线段拓扑。
# 如果未来允许手工输入任意折线，应在这里或上层增加直线腿连续碰撞检测。
# 可飞性校验先做角度和腿长，再做圆弧采样，顺序上先拦截廉价几何错误。
# 圆弧采样只在存在障碍且转弯半径为正时运行，避免普通无障碍路径付出额外成本。
# detail 文案面向中文 GUI/日志，不作为机器可解析接口；机器判断应使用 code 字段。
# code 字符串保持稳定，便于测试、日志分析和后续 UI 过滤。
# 浮点比较使用 1e-6 余量，避免边界等式因二进制误差被误判失败。
# degrees 只用于错误提示，内部判断始终使用弧度。
# corner_arc 返回 None 时代表该局部几何无法构造圆弧，当前逻辑按不可采样处理并继续兜底。
# 采样点高度固定为 0，因为障碍规划当前只在 east/north 平面做二维避障。
# WayLineS 只是复用 arc_swept_rad 的几何载体，不表示这里真的生成航路段。
# 障碍命中使用首个 id 返回，列表顺序由配置决定，便于复现同一失败提示。
# turn_radius_m 为负属于配置错误；为 0 则表示无需圆弧化的特殊直线折线模式。
# len(points)<2 没有可验证的路径语义，因此抛错而不是返回不可飞。
# 退化点不会立即失败，主要是为了兼容上游去冗余还未完全覆盖的历史数据。
# 但退化点通常会导致腿长不足或后续路径质量下降，上层仍应尽量清理。
# 本文件不导入 sim_control，避免避障底层工具反向依赖控制器。
# 若新增失败类型，应同时补充测试和 UI/日志展示文案，保持诊断闭环。


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
    """返回二维向量单位方向；零长度向量返回 None 交由调用方处理退化段。"""
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
        start=WayPointS(
            pos=PosInEarthS(t1.east, t1.north, t1.h),
            turnSign=turn_sign,
            center=PosInEarthS(center.east, center.north, center.h),
        ),
        end=WayPointS(pos=PosInEarthS(t2.east, t2.north, t2.h)),
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


def _split_route_segments(
    route: list[WayPointInputS],
) -> tuple[list[list[Point]], list[tuple[PosInEarthS, PosInEarthS, PosInEarthS, float]]]:
    """把贴障折叠后的航点序列拆成"直线骨架子折线"与"贴障弧段"。

    turnSign!=0 的航点是弧起点（其 pos=切入点、下一航点 pos=切出点）；切点同时是相邻骨架子折线的端点。
    直线骨架交给 check_feasibility 按固定 R 校验；贴障弧另做触障复核。
    """
    subs: list[list[Point]] = []
    arcs: list[tuple[PosInEarthS, PosInEarthS, PosInEarthS, float]] = []
    cur: list[Point] = []
    i = 0
    n = len(route)
    while i < n:
        w = route[i]
        if w.turnSign != 0.0 and i + 1 < n:  # 弧段：本点=切入、下一点=切出
            cur.append((w.pos.east, w.pos.north))  # 切入点闭合当前骨架
            subs.append(cur)
            arcs.append((w.pos, route[i + 1].pos, w.center, w.turnSign))  # 收下这段弧另行触障复核
            cur = [(route[i + 1].pos.east, route[i + 1].pos.north)]  # 新骨架从切出点起
            i += 2  # 跳过弧的两个端点
        else:
            cur.append((w.pos.east, w.pos.north))  # 直线点累加进当前骨架
            i += 1
    subs.append(cur)
    return subs, arcs


def check_route_feasibility(
    route: list[WayPointInputS],
    obstacles: list[ObstacleS],
    *,
    turn_radius_m: float,
    leg_margin_m: float,
    arc_clearance: float = 0.0,
    sample_step: float | None = None,
    max_turn_deg: float = 150.0,
) -> FeasibilityResult:
    """校验贴障折叠后真正要飞的航点序列是否可飞（取代"折叠前对固定 R 折线"的校验）。

    把航线在贴障弧处切成若干直线骨架子折线：每段骨架照旧 check_feasibility（转角/腿长/圆弧触障，
    固定 R）；贴障弧段另做触障复核（半径=膨胀半径>=R，由折叠阶段保证，这里只复核触障）。
    被贴障弧合并掉的拐点不再进入骨架，故不再受固定 R 腿长约束——这正是把校验后置的意义。
    无贴障弧时（如 allow_arc=false，整条都是直线）退化为对整条折线跑一次，与旧行为一致。
    """
    if len(route) < 2:
        raise ValueError("check_route_feasibility needs at least two waypoints")
    if sample_step is None:
        sample_step = _default_sample_step(obstacles, arc_clearance)
    subs, arcs = _split_route_segments(route)
    for sub in subs:
        if len(sub) >= 2:  # 单点骨架(相邻弧之间退化)无腿可校，跳过
            result = check_feasibility(
                sub, obstacles,
                turn_radius_m=turn_radius_m, leg_margin_m=leg_margin_m,
                arc_clearance=arc_clearance, sample_step=sample_step, max_turn_deg=max_turn_deg,
            )
            if not result.ok:
                return result  # 任一骨架不可飞即整体不可飞，透传原因码
    if turn_radius_m > 0.0 and obstacles:
        for t1, t2, center, sign in arcs:
            radius = hypot(t1.east - center.east, t1.north - center.north)  # 弧半径=切入点到圆心距
            arc = (
                PosInEarthS(t1.east, t1.north, 0.0),
                PosInEarthS(t2.east, t2.north, 0.0),
                PosInEarthS(center.east, center.north, 0.0),
                sign,
            )
            for east, north in _arc_sample_points(arc, radius, sample_step):  # 沿弧密采逐点判障
                obstacle_id = _hit_obstacle(obstacles, east, north, arc_clearance)
                if obstacle_id is not None:
                    return FeasibilityResult(
                        ok=False, code=ERR_ARC_HITS_OBSTACLE,
                        detail=f"贴障弧触及障碍 {obstacle_id}", obstacle_id=obstacle_id,
                    )
    return FeasibilityResult(ok=True)
