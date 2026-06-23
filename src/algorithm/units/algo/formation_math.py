"""编队算法无状态数学工具。注意：函数不应保存跨帧状态。"""

from __future__ import annotations

import math

from src.algorithm.context.leaf_types import MotionProfS


def clamp(value: float, lower: float, upper: float) -> float:
    """将数值限制在闭区间内。注意：调用方需保证上下界顺序正确。"""

    if lower > upper:
        raise ValueError("lower must be <= upper")
    return max(lower, min(upper, value))


def enu_to_track(vector: tuple[float, float, float], state: MotionProfS) -> tuple[float, float, float]:
    """把 ENU 向量转换到航迹坐标系。注意：航段退化时基向量可能不可用。"""

    forward, lateral, vertical = _track_basis(state)
    return (
        _dot(vector, forward),
        _dot(vector, lateral),
        _dot(vector, vertical),
    )


def track_to_enu(vector: tuple[float, float, float], state: MotionProfS) -> tuple[float, float, float]:
    """把航迹坐标向量转换回 ENU 坐标。注意：输入分量应与 forward/lateral/up 约定一致。"""

    forward, lateral, vertical = _track_basis(state)
    return (
        vector[0] * forward[0] + vector[1] * lateral[0] + vector[2] * vertical[0],
        vector[0] * forward[1] + vector[1] * lateral[1] + vector[2] * vertical[1],
        vector[0] * forward[2] + vector[1] * lateral[2] + vector[2] * vertical[2],
    )


def horizontal_track_basis(state: MotionProfS) -> tuple[float, float]:
    """计算水平航迹方向单位向量。注意：仅使用水平分量，垂向不参与方向计算。"""

    vx = state.v.vEast
    vy = state.v.vNorth
    ground = math.hypot(vx, vy)
    if ground <= 0.0:
        raise ValueError("horizontal track frame requires non-zero horizontal velocity")
    return vx / ground, vy / ground


def horizontal_track_to_enu(vector: tuple[float, float], state: MotionProfS) -> tuple[float, float]:
    """把水平航迹坐标点转换为 ENU 点。注意：高度由调用方显式给出。"""

    # 队形槽位只按水平航迹旋转，不把长机爬升/下降角耦合进平面偏移。
    return horizontal_track_vector_to_enu(vector, horizontal_track_basis(state))


def horizontal_track_vector_to_enu(vector: tuple[float, float], track: tuple[float, float]) -> tuple[float, float]:
    """用预先计算的水平基向量转换航迹向量。注意：适合批量计算槽位偏移。"""

    track_x, track_y = track
    return (
        vector[0] * track_x - vector[1] * track_y,
        vector[0] * track_y + vector[1] * track_x,
    )


def _track_basis(state: MotionProfS) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    """计算完整航迹坐标基。注意：航段长度过小时会返回默认水平基。"""
    vx = state.v.vEast
    vy = state.v.vNorth
    vz = state.v.vUp
    ground = math.hypot(vx, vy)
    speed = math.sqrt(vx * vx + vy * vy + vz * vz)
    if speed <= 0.0 or ground <= 0.0:
        raise ValueError("track frame requires non-zero horizontal velocity")

    cos_theta = ground / speed
    sin_theta = vz / speed
    cos_psi = vx / ground
    sin_psi = vy / ground
    forward = (cos_theta * cos_psi, cos_theta * sin_psi, sin_theta)
    lateral = (-sin_psi, cos_psi, 0.0)
    vertical = (-sin_theta * cos_psi, -sin_theta * sin_psi, cos_theta)
    return forward, lateral, vertical


def _dot(left: tuple[float, float, float], right: tuple[float, float, float]) -> float:
    """计算三维向量点积。注意：仅用于本模块的小型几何运算。"""
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2]
