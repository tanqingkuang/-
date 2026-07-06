"""解析长机广播消息的共享工具。注意：`RallyLeaderFollower` 是唯一的入站实现，长机/僚机是否携带集结字段（scale=1.0/t_ref 无效）由发送方决定，这里统一解析。"""

from __future__ import annotations

from src.algorithm.context.leaf_types import FormSnapshotS, FormStageE


LEADER_BROADCAST_TOPIC = "formation.leader"


def _write_motion_from_payload(payload: dict[str, object], dst: object) -> None:
    """把收到的长机运动载荷写入输出端口。注意：消息字段缺失时保持目标对象默认值。"""
    pos = payload.get("pos")
    vd = payload.get("vd")
    # 位置或速度子结构缺失则整体放弃，保留目标对象原值
    if not isinstance(pos, dict) or not isinstance(vd, dict):
        return
    # 逐字段以 float 还原，缺省补 0；字段名须与出站 _motion_payload 一致
    dst.pos.east = float(pos.get("east", 0.0))
    dst.pos.north = float(pos.get("north", 0.0))
    dst.pos.h = float(pos.get("h", 0.0))
    dst.v.vEast = float(vd.get("vEast", 0.0))
    dst.v.vNorth = float(vd.get("vNorth", 0.0))
    dst.v.vUp = float(vd.get("vUp", 0.0))
    dst.v.vTheta = float(vd.get("vTheta", 0.0))
    dst.v.vPsi = float(vd.get("vPsi", 0.0))
    dst.v.vd = float(vd.get("vd", 0.0))
    dst.v.dVPsi = float(vd.get("dVPsi", 0.0))


def _write_cmd_from_payload(payload: dict[str, object], dst: FormSnapshotS) -> None:
    """把收到的编队指令载荷写入输出端口。注意：stage 转回枚举，pattern 为纯整型队形索引，字段缺省回退到 0。"""
    dst.stage = FormStageE(int(payload.get("stage", FormStageE.NONE)))
    dst.pattern = int(payload.get("pattern", 0))
    dst.step = int(payload.get("step", 0))
