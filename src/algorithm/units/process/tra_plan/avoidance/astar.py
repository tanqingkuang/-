"""栅格化 + A* 内核（纯函数）。

职责（见 docs/避障-A星-开发计划.md §6 步骤2）：在膨胀后的栅格上求起点→终点的最短路，
输出格点折线（list of (east, north)）；无解返回 None。A* 只决定“从障碍哪边绕”（拓扑），
不含动力学；圆弧 / 可飞性留给后续步骤。格子是否可通行复用 obstacle.inside()。
"""

from __future__ import annotations

import heapq
from math import ceil, hypot, sqrt

from .obstacle import ObstacleS, blocked, obstacle_bounds

Point = tuple[float, float]

# 栅格单元总数上限：防止分辨率过小 / 范围过大导致内存与耗时失控。超限视为配置错误。
MAX_GRID_CELLS = 4_000_000

# 8 邻域：(di, dj, 步长系数)，正交 1、对角 sqrt(2)。
_SQRT2 = sqrt(2.0)
_NEIGHBORS = (
    (1, 0, 1.0),
    (-1, 0, 1.0),
    (0, 1, 1.0),
    (0, -1, 1.0),
    (1, 1, _SQRT2),
    (1, -1, _SQRT2),
    (-1, 1, _SQRT2),
    (-1, -1, _SQRT2),
)

# A* 模块只承担离散拓扑规划，不在这里混入航迹圆弧或动力学约束。
# 可飞性约束由 feasibility.py 负责，RouteS 转换由 path_to_route.py 负责。
# 这里的网格坐标全部使用 east/north 平面坐标，避免和机体系 x/y/z 混淆。
# bounds 是闭合采样范围：nx/ny 通过 ceil 后再 +1，确保 max 边界附近仍有格心。
# to_cell 使用 round 贴近最近格心，因此端点判障必须先用精确坐标做一次。
# is_blocked 按格心判定障碍；这意味着分辨率越粗，窄缝和小障碍的表示越保守。
# blocked_cache 是局部缓存，生命周期仅限一次规划，避免跨配置复用旧障碍结果。
# 对角移动采用“不能切角”规则：两个正交邻格任一被占，就禁止斜穿过去。
# open_heap 的 tie 计数器用于稳定排序，避免同代价节点因 tuple 比较细节变化而抖动。
# heuristic 使用欧氏距离，和 8 邻域步长一致，保持可采纳性。
# g_score 只存已经发现的最短格点代价，未出现过的格点视为 inf。
# closed 仅表示已经出堆扩展过的格点；更优路径出现前不会提前关闭节点。
# start_cell == goal_cell 时仍返回 [start, goal]，保留精确端点，方便上层保持输入语义。
# 无解返回 None，不抛异常；只有配置错误（分辨率、范围、网格规模）才抛 ValueError。
# MAX_GRID_CELLS 是保护阈值，不是算法精度参数；需要更细分辨率时应缩小 bounds。
# 自动 bounds 会纳入障碍膨胀和 margin，保证绕行空间不会被起终点包围框裁掉。
# clearance_m 统一传给 blocked/obstacle_bounds，语义是障碍外扩安全距离。
# _reconstruct 只做格点回溯，不做路径简化；后续去冗余由上层流程决定。
# A* 输出允许包含共线中间点，因为这些点有助于上层调试和复现搜索过程。
# 若调用方需要更短折线，应在可飞性校验前使用路径简化流程处理。
# 起点或终点位于膨胀障碍内时直接无解，不尝试把端点投影到障碍外。
# 自动 bounds 不会根据失败结果自扩张；如果无解，上层可用更大 margin 重新规划。
# 对障碍边界点的 inside 语义由 obstacle.py 统一定义，本模块不重复实现几何判定。
# resolution_m 既影响搜索精度，也影响贴障风险；配置层应让它小于障碍尺度和转弯半径。
# 这里不对相邻格之间的线段做连续碰撞检测，障碍安全性主要依赖膨胀和切角规则。
# 若未来需要连续检测，应优先加在邻接扩展处，而不是事后修补整条路径。
# 所有浮点距离都用米，避免和 UI 像素或经纬度角度混用。
# 本模块保持纯函数风格，便于 LLT 构造小障碍场景稳定回归。
# 异常消息保留英文参数名，方便直接对应 JSON 配置字段和函数入参。
# 返回路径首尾使用原始 start/goal，不使用吸附格心，保证控制器不会改写任务端点。


def compute_bounds(
    start: Point,
    goal: Point,
    obstacles: list[ObstacleS],
    clearance: float,
    pad: float,
) -> tuple[float, float, float, float]:
    """由起点/终点/障碍（含膨胀）外扩 pad 推出栅格范围。注意：保证起终点与绕行空间都在内。"""
    es = [start[0], goal[0]]
    ns = [start[1], goal[1]]
    for obstacle in obstacles:
        bmin_e, bmin_n, bmax_e, bmax_n = obstacle_bounds(obstacle)
        es.extend([bmin_e - clearance, bmax_e + clearance])
        ns.extend([bmin_n - clearance, bmax_n + clearance])
    return (min(es) - pad, min(ns) - pad, max(es) + pad, max(ns) + pad)


def plan_path(
    start: Point,
    goal: Point,
    obstacles: list[ObstacleS],
    *,
    resolution_m: float,
    clearance_m: float = 0.0,
    bounds: tuple[float, float, float, float] | None = None,
    margin_m: float = 0.0,
) -> list[Point] | None:
    """在栅格上用 A* 求避障路径。

    参数：
        start, goal：世界坐标 (east, north)。
        obstacles：障碍集；按 clearance_m 膨胀后视为不可通行。
        resolution_m：栅格分辨率（米），应远小于转弯半径 R。
        clearance_m：障碍膨胀安全距离。
        bounds：(min_e, min_n, max_e, max_n) 栅格范围；None 时由 compute_bounds 推出。
        margin_m：自动推范围时的额外外扩。
    返回：
        起点→终点的格点折线（首尾为精确 start/goal）；无解返回 None。
    异常：
        resolution_m<=0 / 范围非法 / 栅格超过 MAX_GRID_CELLS 时抛 ValueError（配置错误）。
    """
    if resolution_m <= 0.0:
        raise ValueError("resolution_m must be > 0")
    if bounds is None:
        bounds = compute_bounds(start, goal, obstacles, clearance_m, margin_m + resolution_m)
    min_e, min_n, max_e, max_n = bounds
    if max_e <= min_e or max_n <= min_n:
        raise ValueError(f"invalid bounds: {bounds}")

    nx = int(ceil((max_e - min_e) / resolution_m)) + 1
    ny = int(ceil((max_n - min_n) / resolution_m)) + 1
    if nx * ny > MAX_GRID_CELLS:
        raise ValueError(f"grid too large: {nx}x{ny} > {MAX_GRID_CELLS}; increase resolution_m or shrink bounds")

    def cell_center(i: int, j: int) -> Point:
        """把格点索引转换为 east/north 平面坐标。"""
        return (min_e + i * resolution_m, min_n + j * resolution_m)

    def to_cell(point: Point) -> tuple[int, int]:
        """把世界坐标吸附到最近格点，并裁剪到当前网格范围内。"""
        i = round((point[0] - min_e) / resolution_m)
        j = round((point[1] - min_n) / resolution_m)
        return (min(max(i, 0), nx - 1), min(max(j, 0), ny - 1))

    blocked_cache: dict[tuple[int, int], bool] = {}

    def is_blocked(i: int, j: int) -> bool:
        """查询格点是否被膨胀障碍占据，并缓存本次规划内的判定结果。"""
        key = (i, j)
        cached = blocked_cache.get(key)
        if cached is None:
            east, north = cell_center(i, j)
            cached = blocked(obstacles, east, north, clearance_m)
            blocked_cache[key] = cached
        return cached

    # 先按“精确”起点/终点判碰：to_cell 用 round 吸附到最近格心，小障碍可能漏检，
    # 而 _reconstruct 会把首尾换回精确坐标，必须在此用精确坐标拦截，否则会返回端点落在障碍内的路径。
    if blocked(obstacles, start[0], start[1], clearance_m) or blocked(obstacles, goal[0], goal[1], clearance_m):
        return None

    start_cell = to_cell(start)
    goal_cell = to_cell(goal)
    # 吸附后的格心若仍被占（如起点紧贴大障碍）→ 拓扑上无从规划。
    if is_blocked(*start_cell) or is_blocked(*goal_cell):
        return None
    if start_cell == goal_cell:
        return [start, goal]

    goal_e, goal_n = cell_center(*goal_cell)

    def heuristic(i: int, j: int) -> float:
        """A* 启发函数：当前格心到目标格心的欧氏距离。"""
        east, north = cell_center(i, j)
        return hypot(east - goal_e, north - goal_n)

    # A*：open 堆元素 (f, tie, cell)；tie 计数器保证可复现的稳定出栈顺序。
    counter = 0
    open_heap: list[tuple[float, int, tuple[int, int]]] = [(heuristic(*start_cell), counter, start_cell)]
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    g_score: dict[tuple[int, int], float] = {start_cell: 0.0}
    closed: set[tuple[int, int]] = set()

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current == goal_cell:
            return _reconstruct(came_from, current, start, goal, cell_center)
        if current in closed:
            continue
        closed.add(current)
        ci, cj = current
        for di, dj, step in _NEIGHBORS:
            ni, nj = ci + di, cj + dj
            if ni < 0 or ni >= nx or nj < 0 or nj >= ny:
                continue
            if is_blocked(ni, nj) or (ni, nj) in closed:
                continue
            # 防止贴障碍“抄对角”：对角移动时，两个正交相邻格任一被占就禁止穿过。
            if di != 0 and dj != 0 and (is_blocked(ci + di, cj) or is_blocked(ci, cj + dj)):
                continue
            tentative = g_score[current] + step * resolution_m
            if tentative < g_score.get((ni, nj), float("inf")):
                came_from[(ni, nj)] = current
                g_score[(ni, nj)] = tentative
                counter += 1
                heapq.heappush(open_heap, (tentative + heuristic(ni, nj), counter, (ni, nj)))

    return None


def _reconstruct(
    came_from: dict[tuple[int, int], tuple[int, int]],
    goal_cell: tuple[int, int],
    start: Point,
    goal: Point,
    cell_center,
) -> list[Point]:
    """回溯格点路径并把首尾替换为精确的 start/goal。"""
    cells = [goal_cell]
    node = goal_cell
    while node in came_from:
        node = came_from[node]
        cells.append(node)
    cells.reverse()
    points = [cell_center(i, j) for i, j in cells]
    points[0] = start
    points[-1] = goal
    return points
