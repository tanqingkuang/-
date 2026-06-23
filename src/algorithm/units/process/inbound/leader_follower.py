"""解析长机广播消息供僚机实体使用。注意：只消费领航跟随 topic。"""

from __future__ import annotations

from src.algorithm.context.leaf_types import FormPatE, FormStageE
from src.algorithm.units.process.inbound.base import InboundBase, InboundInitS, InboundInputS, InboundOutputS


LEADER_BROADCAST_TOPIC = "formation.leader"


class LeaderFollower(InboundBase):
    def init(self, cfg: InboundInitS) -> None:
        """按配置初始化 LeaderFollower。注意：调用方需先准备好必要依赖和输入数据。"""
        del cfg

    def step(self, u: InboundInputS, y: InboundOutputS) -> None:
        """推进 LeaderFollower 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        if y.leaderState is None or y.cmd is None:
            raise ValueError("LeaderFollower output ports must be bound")
        for msg in u.inbox:
            if msg.topic != LEADER_BROADCAST_TOPIC or not isinstance(msg.payload, dict):
                continue
            payload = msg.payload
            state = payload.get("leader_state")
            cmd = payload.get("cmd")
            if not isinstance(state, dict) or not isinstance(cmd, dict):
                continue
            _write_motion_from_payload(state, y.leaderState)
            y.cmd.stage = FormStageE(int(cmd.get("stage", FormStageE.NONE)))
            y.cmd.pattern = FormPatE(int(cmd.get("pattern", FormPatE.NONE)))
            y.cmd.step = int(cmd.get("step", 0))

    def reset(self) -> None:
        """复位 LeaderFollower 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        return None


def _write_motion_from_payload(payload: dict[str, object], dst: object) -> None:
    """把收到的长机运动载荷写入输出端口。注意：消息字段缺失时保持目标对象默认值。"""
    pos = payload.get("pos")
    vd = payload.get("vd")
    if not isinstance(pos, dict) or not isinstance(vd, dict):
        return
    dst.pos.east = float(pos.get("east", 0.0))
    dst.pos.north = float(pos.get("north", 0.0))
    dst.pos.h = float(pos.get("h", 0.0))
    dst.vd.vEast = float(vd.get("vEast", 0.0))
    dst.vd.vNorth = float(vd.get("vNorth", 0.0))
    dst.vd.vUp = float(vd.get("vUp", 0.0))
    dst.vd.vTheta = float(vd.get("vTheta", 0.0))
    dst.vd.vPsi = float(vd.get("vPsi", 0.0))
    dst.vd.vd = float(vd.get("vd", 0.0))
