"""编队通信协议公共定义。注意：入站与出站必须共同引用本模块，禁止重复定义协议字段。"""

from __future__ import annotations

from src.algorithm.context.leaf_types import MotionProfS


# topic 是通信层与算法层共同识别报文类型的稳定协议键。
LEADER_BROADCAST_TOPIC = "formation.leader"
FOLLOWER_STATUS_TOPIC = "formation.follower_status"


def motion_payload(motion: MotionProfS) -> dict[str, dict[str, float]]:
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
            "vPsi": motion.v.vPsi,
            "vd": motion.v.vd,
            "dVPsi": motion.v.dVPsi,
        },
    }
