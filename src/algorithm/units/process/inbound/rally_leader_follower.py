"""僚机解析长机广播：唯一的入站实现，额外解析 slot_scale/t_ref 字段。注意：hold 场景长机若不广播这些字段（旧格式），按 scale=1.0/t_ref_valid=False 兜底；多消息同帧后到覆盖先到，各字段来自同一条消息保证一致性。"""

from __future__ import annotations

from dataclasses import dataclass

from src.algorithm.context.leaf_types import FormSnapshotS, FormStageE, RallySlotScaleS
from src.algorithm.units.process.inbound.base import InboundBase, InboundInitS, InboundInputS, InboundOutputS

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


@dataclass
class RallyLeaderFollowerOutputS(InboundOutputS):
    """集结僚机入站输出端口。"""

    # 继承 leaderState: MotionProfS, cmd: FormSnapshotS
    slotScale: RallySlotScaleS | None = None  # 端口 → Context.slotScale
    t_ref: float = 0.0  # 长机广播的集结基准时刻（秒）；由实体每帧复制到 cxt.rally_t_ref
    t_ref_valid: bool = False  # 旧格式或非法 t_ref 默认 False，禁止冷启动误切出


class RallyLeaderFollower(InboundBase):
    """集结僚机入站单元：解析长机广播，同时写入 leaderState/cmd/slotScale。注意：字段来自同一条消息，保证一致性。"""

    def init(self, cfg: InboundInitS) -> None:
        """按配置初始化 RallyLeaderFollower。注意：调用方需先准备好必要依赖和输入数据。"""
        del cfg

    def step(self, u: InboundInputS, y: RallyLeaderFollowerOutputS) -> None:
        """推进 RallyLeaderFollower 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        if y.leaderState is None or y.cmd is None or y.slotScale is None:
            raise ValueError("RallyLeaderFollower output ports must be bound")
        # leaderState、cmd、slotScale 必须来自同一条 formation.leader 消息，不能跨消息拼接。
        # 这样僚机不会在“新阶段 + 旧缩放”或“旧阶段 + 新长机位置”的组合状态下解算槽位。
        # 未携带 slot_scale 的旧格式广播仍可解析，默认 scale=1，保证与普通保持编队兼容。
        # 非目标 topic 或非 dict payload 直接跳过，避免其它业务消息污染编队黑板。
        for msg in u.inbox:
            if msg.topic != LEADER_BROADCAST_TOPIC or not isinstance(msg.payload, dict):
                continue
            payload = msg.payload
            state = payload.get("leader_state")
            cmd = payload.get("cmd")
            if not isinstance(state, dict) or not isinstance(cmd, dict):
                continue
            # t_ref 先解析：非法则整条消息丢弃，避免「新阶段 + 无效 T_ref」半截状态提交到黑板。
            try:
                t_ref_parsed = float(payload.get("t_ref", 0.0))
            except (TypeError, ValueError):
                continue
            raw_t_ref_valid = payload.get("t_ref_valid", False)
            t_ref_valid_parsed = raw_t_ref_valid if isinstance(raw_t_ref_valid, bool) else False
            # 解析 slot_scale，任何异常均 fallback 到默认值（scale=1.0, scaleRate=0.0）
            try:
                ss = payload.get("slot_scale", {})
                if not isinstance(ss, dict):
                    raise TypeError
                scale_parsed = float(ss.get("scale", 1.0))
                scale_rate_parsed = float(ss.get("scale_rate", 0.0))
            except (TypeError, ValueError):
                scale_parsed, scale_rate_parsed = 1.0, 0.0
            # 全部字段解析成功后，一次性写入输出端口，保证多字段一致性。
            _write_motion_from_payload(state, y.leaderState)
            _write_cmd_from_payload(cmd, y.cmd)
            y.slotScale.scale = scale_parsed
            y.slotScale.scaleRate = scale_rate_parsed
            y.t_ref = t_ref_parsed
            y.t_ref_valid = t_ref_valid_parsed

    def reset(self) -> None:
        """复位 RallyLeaderFollower 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        return None
