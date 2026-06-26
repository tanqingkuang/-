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
    kind: str  # "circle" | "rect"
    center: PosInEarthS = field(default_factory=PosInEarthS)  # 圆心（kind=circle，h 忽略）
    radius: float = 0.0  # 半径，米（kind=circle）
    min_e: float = 0.0  # 矩形 east 下界（kind=rect）
    min_n: float = 0.0  # 矩形 north 下界
    max_e: float = 0.0  # 矩形 east 上界
    max_n: float = 0.0  # 矩形 north 上界


def make_circle(obstacle_id: str, east: float, north: float, radius: float) -> ObstacleS:
    """构造圆形障碍。注意：radius 应为正。"""
    return ObstacleS(id=obstacle_id, kind="circle", center=PosInEarthS(east=east, north=north), radius=radius)


def make_rect(obstacle_id: str, min_e: float, min_n: float, max_e: float, max_n: float) -> ObstacleS:
    """构造轴对齐矩形障碍。注意：自动规整使 min<=max。"""
    lo_e, hi_e = (min_e, max_e) if min_e <= max_e else (max_e, min_e)
    lo_n, hi_n = (min_n, max_n) if min_n <= max_n else (max_n, min_n)
    return ObstacleS(id=obstacle_id, kind="rect", min_e=lo_e, min_n=lo_n, max_e=hi_e, max_n=hi_n)


def inside(obstacle: ObstacleS, east: float, north: float, clearance: float = 0.0) -> bool:
    """点 (east, north) 是否落在膨胀 clearance 后的障碍内（边界算内）。

    这是唯一的形状判定基元：A* 把它当“格子是否被占”，可飞性校验把圆弧采样后逐点调它。
    矩形按方角外扩近似（圆角误差由 clearance 吸收，见 §8）。
    """
    if obstacle.kind == "circle":
        return hypot(east - obstacle.center.east, north - obstacle.center.north) <= obstacle.radius + clearance
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
    return (obstacle.min_e, obstacle.min_n, obstacle.max_e, obstacle.max_n)
