"""长机运动状态到广播载荷的转换。注意：`RallyLeaderBroadcast` 是唯一的出站实现，字段名需与入站解析保持一致。"""

from __future__ import annotations

from src.algorithm.context.leaf_types import MotionProfS


def _motion_payload(motion: MotionProfS) -> dict[str, dict[str, float]]:
    """把运动状态转换为通信载荷。注意：字段名需与入站解析保持一致。"""
    return {
        "pos": {
            "east": motion.pos.east,
            "north": motion.pos.north,
            "h": motion.pos.h,
        },
        "vd": {
            "vEast": motion.v.vEast,
            "vNorth": motion.v.vNorth,
            "vUp": motion.v.vUp,
            "vTheta": motion.v.vTheta,
            "vPsi": motion.v.vPsi,
            "vd": motion.v.vd,
            "dVPsi": motion.v.dVPsi,
        },
    }
