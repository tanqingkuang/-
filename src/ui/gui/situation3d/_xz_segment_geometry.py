"""3D 线带在 XZ 平面的共享线段几何。"""

from __future__ import annotations

import math

Point3D = tuple[float, float, float]
Direction2D = tuple[float, float]


def segment_direction(
    start: Point3D,
    end: Point3D,
    *,
    epsilon: float = 1e-6,
    fallback: Direction2D = (0.0, 1.0),
) -> Direction2D:
    """返回 XZ 平面的单位方向；退化段使用调用方明确给出的有限兜底。"""

    delta_x = end[0] - start[0]
    delta_z = end[2] - start[2]
    length = math.hypot(delta_x, delta_z)
    if length <= epsilon:
        return fallback
    return delta_x / length, delta_z / length


def segment_normal(
    start: Point3D,
    end: Point3D,
    *,
    epsilon: float = 1e-6,
    fallback: Direction2D = (-1.0, 0.0),
) -> Direction2D:
    """返回 XZ 平面的左法向；退化段不猜测方向，直接返回显式兜底。"""

    delta_x = end[0] - start[0]
    delta_z = end[2] - start[2]
    length = math.hypot(delta_x, delta_z)
    if length <= epsilon:
        return fallback
    return -delta_z / length, delta_x / length


def edge_positions(
    point: Point3D,
    normal: Direction2D,
    scale: float,
) -> tuple[Point3D, Point3D]:
    """按统一左法向和半宽返回中心线两侧边缘点。"""

    offset_x = normal[0] * scale
    offset_z = normal[1] * scale
    return (
        (point[0] - offset_x, point[1], point[2] - offset_z),
        (point[0] + offset_x, point[1], point[2] + offset_z),
    )
