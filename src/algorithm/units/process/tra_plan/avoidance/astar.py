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
        return (min_e + i * resolution_m, min_n + j * resolution_m)

    def to_cell(point: Point) -> tuple[int, int]:
        i = round((point[0] - min_e) / resolution_m)
        j = round((point[1] - min_n) / resolution_m)
        return (min(max(i, 0), nx - 1), min(max(j, 0), ny - 1))

    blocked_cache: dict[tuple[int, int], bool] = {}

    def is_blocked(i: int, j: int) -> bool:
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
