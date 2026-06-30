"""避障障碍数据结构与唯一形状基元。

整套避障的形状相关逻辑收敛到本模块的 inside()：A* 格子判定与可飞性校验都复用它，
新增形状只需扩展这一个函数（见 docs/避障-A星-开发计划.md §4.1）。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import hypot

from src.algorithm.context.leaf_types import PosInEarthS


@dataclass
class ObstacleS:
    """二维水平障碍（无限高柱体）。圆与矩形均原生保留，不互相转换。"""

    id: str  # 障碍唯一标识
    kind: str  # "circle" | "rect" | "polygon"
    center: PosInEarthS = field(default_factory=PosInEarthS)  # 圆心（kind=circle，h 忽略）
    radius: float = 0.0  # 半径，米（kind=circle）
    min_e: float = 0.0  # 矩形 east 下界（kind=rect）
    min_n: float = 0.0  # 矩形 north 下界
    max_e: float = 0.0  # 矩形 east 上界
    max_n: float = 0.0  # 矩形 north 上界
    vertices: list[PosInEarthS] = field(default_factory=list)  # 多边形顶点（kind=polygon，按顺/逆时针）


def make_circle(obstacle_id: str, east: float, north: float, radius: float) -> ObstacleS:
    """构造圆形障碍。注意：radius 应为正。"""
    return ObstacleS(id=obstacle_id, kind="circle", center=PosInEarthS(east=east, north=north), radius=radius)


def make_rect(obstacle_id: str, min_e: float, min_n: float, max_e: float, max_n: float) -> ObstacleS:
    """构造轴对齐矩形障碍。注意：自动规整使 min<=max。"""
    lo_e, hi_e = (min_e, max_e) if min_e <= max_e else (max_e, min_e)
    lo_n, hi_n = (min_n, max_n) if min_n <= max_n else (max_n, min_n)
    return ObstacleS(id=obstacle_id, kind="rect", min_e=lo_e, min_n=lo_n, max_e=hi_e, max_n=hi_n)


def make_polygon(obstacle_id: str, vertices: list[tuple[float, float]]) -> ObstacleS:
    """构造多边形障碍。注意：当前用于四点旋转矩形，也兼容凸多边形。"""
    if len(vertices) < 3:
        raise ValueError("polygon obstacle needs at least three vertices")
    # 后端统一用 PosInEarthS 表达水平点，h 对二维避障无意义。
    points = [PosInEarthS(east=east, north=north) for east, north in vertices]
    return ObstacleS(id=obstacle_id, kind="polygon", vertices=points)


def _point_segment_distance(east: float, north: float, a: PosInEarthS, b: PosInEarthS) -> float:
    """计算点到线段的水平距离。注意：用于 polygon clearance 外扩判定。"""
    dx = b.east - a.east
    dy = b.north - a.north
    length2 = dx * dx + dy * dy
    if length2 <= 1e-12:
        return hypot(east - a.east, north - a.north)
    # 投影参数裁剪到线段范围内，避免使用无限直线距离。
    t = ((east - a.east) * dx + (north - a.north) * dy) / length2
    t = max(0.0, min(1.0, t))
    closest_e = a.east + t * dx
    closest_n = a.north + t * dy
    return hypot(east - closest_e, north - closest_n)


def _inside_polygon(vertices: list[PosInEarthS], east: float, north: float) -> bool:
    """判断点是否在多边形内。注意：边界点视为在障碍内。"""
    if len(vertices) < 3:
        return False
    # 先做边界判断，保证点落在边上时也算触障。
    for start, end in zip(vertices, vertices[1:] + vertices[:1]):
        if _point_segment_distance(east, north, start, end) <= 1e-9:
            return True
    inside_flag = False
    j = len(vertices) - 1
    for i, vi in enumerate(vertices):
        vj = vertices[j]
        # 射线法：只在边跨过水平射线时切换 inside 状态，避免水平边重复计数。
        crosses = (vi.north > north) != (vj.north > north)
        if crosses:
            # 计算当前边和水平射线的交点 east 坐标。
            x_intersect = (vj.east - vi.east) * (north - vi.north) / (vj.north - vi.north) + vi.east
            if east <= x_intersect:
                inside_flag = not inside_flag
        j = i
    return inside_flag


def inside(obstacle: ObstacleS, east: float, north: float, clearance: float = 0.0) -> bool:
    """点 (east, north) 是否落在膨胀 clearance 后的障碍内（边界算内）。

    这是唯一的形状判定基元：A* 把它当“格子是否被占”，可飞性校验把圆弧采样后逐点调它。
    矩形按方角外扩近似（圆角误差由 clearance 吸收，见 §8）。
    """
    if obstacle.kind == "circle":
        return hypot(east - obstacle.center.east, north - obstacle.center.north) <= obstacle.radius + clearance
    if obstacle.kind == "polygon":
        if _inside_polygon(obstacle.vertices, east, north):
            return True
        # clearance 按点到各边距离膨胀，支持旋转矩形而不变成轴对齐包围盒。
        return any(
            _point_segment_distance(east, north, start, end) <= clearance
            for start, end in zip(obstacle.vertices, obstacle.vertices[1:] + obstacle.vertices[:1])
        )
    return (
        obstacle.min_e - clearance <= east <= obstacle.max_e + clearance
        and obstacle.min_n - clearance <= north <= obstacle.max_n + clearance
    )


def blocked(obstacles: list[ObstacleS], east: float, north: float, clearance: float = 0.0) -> bool:
    """点是否被任一障碍（膨胀 clearance 后）占据。注意：空障碍集恒为 False。"""
    return any(inside(obstacle, east, north, clearance) for obstacle in obstacles)


def obstacle_bounds(obstacle: ObstacleS) -> tuple[float, float, float, float]:
    """返回障碍未膨胀的轴对齐包围盒 (min_e, min_n, max_e, max_n)。"""
    if obstacle.kind == "circle":
        return (
            obstacle.center.east - obstacle.radius,
            obstacle.center.north - obstacle.radius,
            obstacle.center.east + obstacle.radius,
            obstacle.center.north + obstacle.radius,
        )
    if obstacle.kind == "polygon":
        # A* 搜索范围仍使用轴对齐 bounds，但碰撞判定用 polygon 原形。
        east_values = [point.east for point in obstacle.vertices]
        north_values = [point.north for point in obstacle.vertices]
        return (min(east_values), min(north_values), max(east_values), max(north_values))
    return (obstacle.min_e, obstacle.min_n, obstacle.max_e, obstacle.max_n)
