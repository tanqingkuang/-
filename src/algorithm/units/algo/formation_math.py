"""编队算法无状态数学工具。注意：函数不应保存跨帧状态。"""

from __future__ import annotations

import math

from src.algorithm.context.leaf_types import MotionProfS
from src.common.coordinates import enu_to_fur, fur_basis_from_velocity, fur_to_enu


def clamp(value: float, lower: float, upper: float) -> float:
    """将数值限制在闭区间内。注意：调用方需保证上下界顺序正确。"""

    if lower > upper:
        raise ValueError("lower must be <= upper")
    return max(lower, min(upper, value))


def enu_to_track(vector: tuple[float, float, float], state: MotionProfS) -> tuple[float, float, float]:
    """把 ENU 向量转换到苏联式航迹系。注意：输出轴序为前向、法向/上向、侧向右。"""

    # 算法层只负责从运动状态取地速，基向量和投影公式统一交给公共坐标模块。
    return enu_to_fur(
        vector,
        fur_basis_from_velocity((state.v.vEast, state.v.vNorth, state.v.vUp)),
    )


def track_to_enu(vector: tuple[float, float, float], state: MotionProfS) -> tuple[float, float, float]:
    """把苏联式航迹系向量转换回 ENU 坐标。注意：输入轴序为前向、法向/上向、侧向右。"""

    # 与 enu_to_track 复用同一建基入口，避免往返变换因公式副本漂移而不互逆。
    return fur_to_enu(
        vector,
        fur_basis_from_velocity((state.v.vEast, state.v.vNorth, state.v.vUp)),
    )


def horizontal_track_basis(state: MotionProfS) -> tuple[float, float]:
    """计算水平航迹方向单位向量。注意：仅使用水平分量，垂向不参与方向计算。"""

    vx = state.v.vEast
    vy = state.v.vNorth
    # 该二维基有意忽略 vUp，只能用于明确声明为 ENU 水平平面的几何。
    ground = math.hypot(vx, vy)
    if ground <= 0.0:
        raise ValueError("horizontal track frame requires non-zero horizontal velocity")
    return vx / ground, vy / ground


def horizontal_track_to_enu(vector: tuple[float, float], state: MotionProfS) -> tuple[float, float]:
    """把水平航迹坐标点转换为 ENU 点。注意：轴序为前向、侧向右，高度由调用方显式给出。"""

    # 该二维辅助仅供明确采用水平平面的几何（如集结松散点）；三维槽位使用公共 FUR 变换。
    return horizontal_track_vector_to_enu(vector, horizontal_track_basis(state))


def horizontal_track_vector_to_enu(vector: tuple[float, float], track: tuple[float, float]) -> tuple[float, float]:
    """在 ENU 水平面用预先计算的航向基转换向量。注意：二维第二轴为侧向右。"""

    track_x, track_y = track
    # 水平第二轴为右轴，所以正东航向下正值映射到负北向，不能套用左法向公式。
    return (
        vector[0] * track_x + vector[1] * track_y,
        vector[0] * track_y - vector[1] * track_x,
    )
