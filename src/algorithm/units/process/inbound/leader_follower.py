"""解析长机广播消息供僚机实体使用。注意：只消费领航跟随 topic。"""

from __future__ import annotations

from src.algorithm.context.leaf_types import FormPatE, FormStageE
from src.algorithm.units.process.inbound.base import InboundBase, InboundInitS, InboundInputS, InboundOutputS


LEADER_BROADCAST_TOPIC = "formation.leader"


class LeaderFollower(InboundBase):
    """僚机入站单元：从收件箱中筛出长机广播，解析出长机状态与编队指令写入黑板。注意：同帧多条时后到的覆盖先到的。"""

    def init(self, cfg: InboundInitS) -> None:
        """按配置初始化 LeaderFollower。注意：调用方需先准备好必要依赖和输入数据。"""
        del cfg

    def step(self, u: InboundInputS, y: InboundOutputS) -> None:
        """推进 LeaderFollower 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        if y.leaderState is None or y.cmd is None:
            raise ValueError("LeaderFollower output ports must be bound")
        for msg in u.inbox:
            # 仅消费长机广播 topic，载荷须为 dict，其余消息跳过
            if msg.topic != LEADER_BROADCAST_TOPIC or not isinstance(msg.payload, dict):
                continue
            payload = msg.payload
            state = payload.get("leader_state")
            cmd = payload.get("cmd")
            # 关键字段缺失或类型不符则丢弃该条，避免写入半截数据
            if not isinstance(state, dict) or not isinstance(cmd, dict):
                continue
            _write_motion_from_payload(state, y.leaderState)  # 还原长机运动状态
            # 还原编队指令：int 转回枚举，缺省回退到 NONE/0
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
    # 位置或速度子结构缺失则整体放弃，保留目标对象原值
    if not isinstance(pos, dict) or not isinstance(vd, dict):
        return
    # 逐字段以 float 还原，缺省补 0；字段名须与出站 _motion_payload 一致
    dst.pos.east = float(pos.get("east", 0.0))
    dst.pos.north = float(pos.get("north", 0.0))
    dst.pos.h = float(pos.get("h", 0.0))
    dst.vd.vEast = float(vd.get("vEast", 0.0))
    dst.vd.vNorth = float(vd.get("vNorth", 0.0))
    dst.vd.vUp = float(vd.get("vUp", 0.0))
    dst.vd.vTheta = float(vd.get("vTheta", 0.0))
    dst.vd.vPsi = float(vd.get("vPsi", 0.0))
    dst.vd.vd = float(vd.get("vd", 0.0))
