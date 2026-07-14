"""僚机解析长机广播：完整校验单条消息后原子更新所有入站输出。"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from src.algorithm.context.leaf_types import (
    FormSnapshotS,
    FormStageE,
    MotionProfS,
    RallySlotScaleS,
    copy_motion,
    copy_rally_slot_scale,
    copy_snapshot,
)
from src.algorithm.units.process.inbound.base import InboundBase, InboundInitS, InboundInputS, InboundOutputS

LEADER_BROADCAST_TOPIC = "formation.leader"


@dataclass
class _ParsedLeaderBroadcast:
    """完整解析后的单条长机广播临时快照。"""

    leader_state: MotionProfS  # 长机实际运动状态
    leader_cmd: MotionProfS  # 长机跟踪指令，旧格式回退为实际状态
    cmd: FormSnapshotS  # 已通过枚举校验的编队命令
    slot_scale: RallySlotScaleS  # 已通过有限性校验的槽位缩放
    t_ref: float  # 固定公共到达时刻
    t_ref_valid: bool  # 固定计划有效位
    loop_counts: dict[str, int]  # 每个节点的非负完整圈数


def _finite_number(value: object) -> float:
    """把非布尔数值转换为有限浮点数，类型或有限性非法时抛出 ValueError。"""

    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise ValueError("数值字段类型非法")
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError("数值字段必须有限")
    return parsed


def _parse_motion_payload(payload: object) -> MotionProfS:
    """完整解析运动载荷到新对象，避免校验失败时污染绑定输出。"""

    if not isinstance(payload, dict):
        raise ValueError("运动载荷必须为映射")
    pos = payload.get("pos")
    vd = payload.get("vd")
    if not isinstance(pos, dict) or not isinstance(vd, dict):
        raise ValueError("运动载荷缺少位置或速度映射")
    parsed = MotionProfS()
    # 位置三轴必须作为同一运动快照通过校验，不能逐字段写入外部对象。
    parsed.pos.east = _finite_number(pos.get("east", 0.0))
    parsed.pos.north = _finite_number(pos.get("north", 0.0))
    parsed.pos.h = _finite_number(pos.get("h", 0.0))
    # 速度、姿态和角速率同样先落临时对象，任一非有限值都会废弃整条消息。
    parsed.v.vEast = _finite_number(vd.get("vEast", 0.0))
    parsed.v.vNorth = _finite_number(vd.get("vNorth", 0.0))
    parsed.v.vUp = _finite_number(vd.get("vUp", 0.0))
    parsed.v.vTheta = _finite_number(vd.get("vTheta", 0.0))
    parsed.v.vPsi = _finite_number(vd.get("vPsi", 0.0))
    parsed.v.vd = _finite_number(vd.get("vd", 0.0))
    parsed.v.dVPsi = _finite_number(vd.get("dVPsi", 0.0))
    return parsed


def _parse_cmd_payload(payload: object) -> FormSnapshotS:
    """严格解析编队命令的整数类型与阶段枚举。"""

    if not isinstance(payload, dict):
        raise ValueError("命令载荷必须为映射")
    # 缺省值仅兼容字段缺失；显式提供的浮点、字符串或 bool 都不做静默转换。
    stage = payload.get("stage", FormStageE.NONE)
    pattern = payload.get("pattern", 0)
    step = payload.get("step", 0)
    if any(not isinstance(value, int) or isinstance(value, bool) for value in (stage, pattern, step)):
        raise ValueError("命令字段必须为非布尔整数")
    return FormSnapshotS(stage=FormStageE(stage), pattern=pattern, step=step)


def _parse_leader_broadcast(payload: dict[str, object]) -> _ParsedLeaderBroadcast:
    """把一条长机广播完整解析到临时对象，任何非法字段都拒绝整条消息。"""

    leader_state = _parse_motion_payload(payload.get("leader_state"))
    raw_cmd = payload.get("cmd")
    cmd = _parse_cmd_payload(raw_cmd)
    assert isinstance(raw_cmd, dict)
    # 旧格式没有 leader 指令时使用同一条消息内已校验的实际状态，禁止跨消息拼装。
    raw_leader_cmd = raw_cmd.get("leader")
    leader_cmd = leader_state if raw_leader_cmd is None else _parse_motion_payload(raw_leader_cmd)

    # 缺少整个 slot_scale 时兼容为单位缩放；显式载荷则必须是有效映射。
    raw_slot_scale = payload.get("slot_scale", {})
    if not isinstance(raw_slot_scale, dict):
        raise ValueError("槽位缩放载荷必须为映射")
    slot_scale = RallySlotScaleS(
        scale=_finite_number(raw_slot_scale.get("scale", 1.0)),
        scaleRate=_finite_number(raw_slot_scale.get("scale_rate", 0.0)),
    )
    # 时间与有效位共同属于固定计划，不能把非法有效位降级成 False 后部分提交。
    t_ref = _finite_number(payload.get("t_ref", 0.0))
    t_ref_valid = payload.get("t_ref_valid", False)
    if not isinstance(t_ref_valid, bool):
        raise ValueError("T_ref 有效位必须为布尔值")

    # 圈数映射整体校验；bool 虽是 int 子类，也不能作为合法圈数。
    raw_loop_counts = payload.get("loop_counts", {})
    if not isinstance(raw_loop_counts, dict) or any(
        not isinstance(node_id, str)
        or not isinstance(count, int)
        or isinstance(count, bool)
        or count < 0
        for node_id, count in raw_loop_counts.items()
    ):
        raise ValueError("圈数计划必须由字符串节点 ID 映射到非负整数")
    return _ParsedLeaderBroadcast(
        leader_state=leader_state,
        leader_cmd=leader_cmd,
        cmd=cmd,
        slot_scale=slot_scale,
        t_ref=t_ref,
        t_ref_valid=t_ref_valid,
        loop_counts=dict(raw_loop_counts),
    )


@dataclass
class RallyLeaderFollowerOutputS(InboundOutputS):
    """集结僚机入站输出端口。"""

    # 继承 leaderState: MotionProfS, cmd: FormSnapshotS
    leaderCmd: MotionProfS | None = None  # 长机跟踪指令；None 时接收端可沿用 leaderState。
    slotScale: RallySlotScaleS | None = None  # 端口 → Context.slotScale
    t_ref: float = 0.0  # 长机广播的集结基准时刻（秒）；由实体每帧复制到 cxt.rally_t_ref
    t_ref_valid: bool = False  # 旧格式或非法 t_ref 默认 False，禁止冷启动误切出
    loopCounts: dict[str, int] = field(default_factory=dict)  # 长机固定计划的节点整数圈数映射


class RallyLeaderFollower(InboundBase):
    """集结僚机入站单元：解析长机广播，同时写入 leaderState/cmd/slotScale。注意：字段来自同一条消息，保证一致性。"""

    def init(self, cfg: InboundInitS) -> None:
        """按配置初始化 RallyLeaderFollower。注意：调用方需先准备好必要依赖和输入数据。"""
        del cfg
        # 输出端口由 step 的 y 参数绑定；首次 step 前 reset 明确定义为无输出可清理的 no-op。
        self._latched_output: RallyLeaderFollowerOutputS | None = None

    def step(self, u: InboundInputS, y: RallyLeaderFollowerOutputS) -> None:
        """推进 RallyLeaderFollower 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        if y.leaderState is None or y.cmd is None or y.slotScale is None:
            raise ValueError("RallyLeaderFollower output ports must be bound")
        # 每拍更新为最近一次绑定端口；reset 只负责清理该对象，不假设端口终身不变。
        self._latched_output = y
        # leaderState、cmd、slotScale 必须来自同一条 formation.leader 消息，不能跨消息拼接。
        # 这样僚机不会在“新阶段 + 旧缩放”或“旧阶段 + 新长机位置”的组合状态下解算槽位。
        # 未携带 slot_scale 的旧格式广播仍可解析，默认 scale=1，保证与普通保持编队兼容。
        # 非目标 topic 或非 dict payload 直接跳过，避免其它业务消息污染编队黑板。
        for msg in u.inbox:
            if msg.topic != LEADER_BROADCAST_TOPIC or not isinstance(msg.payload, dict):
                continue
            try:
                parsed = _parse_leader_broadcast(msg.payload)
            except (TypeError, ValueError, OverflowError):
                # 通信边界上的畸形报文只丢弃本条，不能把解析异常传播到实体主循环。
                continue
            # 所有临时对象均已通过类型、枚举和有限性检查，此处才统一提交。
            copy_motion(parsed.leader_state, y.leaderState)
            copy_snapshot(parsed.cmd, y.cmd)
            if y.leaderCmd is not None:
                copy_motion(parsed.leader_cmd, y.leaderCmd)
            copy_rally_slot_scale(parsed.slot_scale, y.slotScale)
            y.t_ref = parsed.t_ref
            y.t_ref_valid = parsed.t_ref_valid
            y.loopCounts.clear()
            y.loopCounts.update(parsed.loop_counts)

    def reset(self) -> None:
        """复位最近一次 step 绑定的入站输出；尚未执行 step 时无操作。"""

        output = getattr(self, "_latched_output", None)
        if output is None:
            return
        if output.leaderState is not None:
            copy_motion(MotionProfS(), output.leaderState)
        if output.leaderCmd is not None:
            copy_motion(MotionProfS(), output.leaderCmd)
        if output.cmd is not None:
            copy_snapshot(FormSnapshotS(), output.cmd)
        if output.slotScale is not None:
            copy_rally_slot_scale(RallySlotScaleS(), output.slotScale)
        output.t_ref = 0.0
        output.t_ref_valid = False
        output.loopCounts.clear()
