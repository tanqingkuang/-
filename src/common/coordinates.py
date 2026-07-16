"""东北天 ENU 与前-上-右 FUR 坐标变换。

统一约定：ENU 为东-北-天右手系；FUR 为前向-上法向-右侧向右手系，满足 ``F × U = R``。
航迹倾角 ``theta`` 爬升为正，水平航向角 ``psi`` 从东向北逆时针为正。
"""

from __future__ import annotations

import math


Vector3 = tuple[float, float, float]
FurBasis = tuple[Vector3, Vector3, Vector3]

# ENU 的三个分量依次对应东、北、天，变换函数不得混入经纬高或机体系量。
# FUR 的三个分量依次对应前、上法向、右，固定轴序用于保证跨模块符号一致。
# 本项目航向从东向北为正，因而正东飞行时右轴指向正南，而不是正北。


def fur_basis_from_angles(theta_rad: float, psi_rad: float) -> FurBasis:
    """按航迹倾角和水平航向角生成 FUR 在 ENU 中的三个单位基向量。"""

    if not math.isfinite(theta_rad) or not math.isfinite(psi_rad):
        raise ValueError("FUR basis angles must be finite")
    cos_theta = math.cos(theta_rad)
    sin_theta = math.sin(theta_rad)
    cos_psi = math.cos(psi_rad)
    sin_psi = math.sin(psi_rad)
    # 前轴同时包含水平航向和航迹倾角，爬升对应正天向分量。
    # U 位于航迹竖直平面内并指向航迹倾角增大方向；R 始终为水平右侧向。
    # 右轴不随倾角抬升，确保侧向正值始终落在飞机右手边。
    # 三个公式共同满足 F×U=R，这是苏联系前上右坐标的右手性约束。
    forward = (cos_theta * cos_psi, cos_theta * sin_psi, sin_theta)
    up_normal = (-sin_theta * cos_psi, -sin_theta * sin_psi, cos_theta)
    right = (sin_psi, -cos_psi, 0.0)
    return forward, up_normal, right


def fur_basis_from_velocity(velocity_enu: Vector3) -> FurBasis:
    """按 ENU 速度方向生成 FUR 基；水平速度为零时航向及右轴无定义，显式报错。"""

    east, north, up = velocity_enu
    # 航迹基取自地速方向；空速或姿态角若需建基，应由调用方显式传入对应角度。
    if not all(math.isfinite(value) for value in velocity_enu):
        raise ValueError("FUR basis velocity must be finite")
    horizontal_speed = math.hypot(east, north)
    speed = math.sqrt(east * east + north * north + up * up)
    if speed <= 0.0 or horizontal_speed <= 0.0:
        raise ValueError("FUR basis requires non-zero horizontal velocity")
    return fur_basis_from_angles(
        math.atan2(up, horizontal_speed),
        math.atan2(north, east),
    )


def enu_to_fur(vector_enu: Vector3, basis: FurBasis) -> Vector3:
    """把一个 ENU 向量投影到给定 FUR 正交基，返回轴序 ``(前、上、右)``。"""

    forward, up_normal, right = basis
    # 正交单位基下，世界向量与各基轴点积就是对应的 FUR 分量。
    return (
        _dot(vector_enu, forward),
        _dot(vector_enu, up_normal),
        _dot(vector_enu, right),
    )


def fur_to_enu(vector_fur: Vector3, basis: FurBasis) -> Vector3:
    """把轴序为 ``(前、上、右)`` 的 FUR 向量还原到 ENU。"""

    forward, up_normal, right = basis
    # 逆变换按三个 FUR 分量线性合成 ENU 向量，不附加任何位置平移。
    return (
        vector_fur[0] * forward[0] + vector_fur[1] * up_normal[0] + vector_fur[2] * right[0],
        vector_fur[0] * forward[1] + vector_fur[1] * up_normal[1] + vector_fur[2] * right[1],
        vector_fur[0] * forward[2] + vector_fur[1] * up_normal[2] + vector_fur[2] * right[2],
    )


def _dot(left: Vector3, right: Vector3) -> float:
    """计算三维向量点积。"""

    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2]
