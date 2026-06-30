"""集结长机广播：在 LeaderBroadcast 基础上追加 slot_scale 字段。注意：payload 格式需与 RallyLeaderFollower 解析约定一致。"""

from __future__ import annotations

from dataclasses import dataclass

from src.algorithm.context.leaf_types import RallySlotScaleS
from src.algorithm.units.process.inbound.leader_follower import LEADER_BROADCAST_TOPIC
from src.algorithm.units.process.outbound.base import OutboundBase, OutboundInitS, OutboundInputS, OutboundOutputS
from src.algorithm.units.process.outbound.leader_broadcast import LeaderBroadcast, _motion_payload
from src.common.envelope import MessageEnvelope


@dataclass
class RallyLeaderBroadcastInputS(OutboundInputS):
    """集结长机广播输入端口。"""

    # 继承 cmd: FormSnapshotS, selfState: MotionProfS
    slotScale: RallySlotScaleS | None = None  # 端口 → Context.slotScale
    t_ref: float = 0.0  # 集结基准时刻（Rally 任务计算，每帧注入）
    t_ref_valid: bool = False  # 是否允许接收方使用 t_ref 执行切出判定


class RallyLeaderBroadcast(OutboundBase):
    """集结长机广播单元：在长机广播的 payload 中追加 slot_scale 字段。注意：复用 LeaderBroadcast 的目标列表逻辑。"""

    def __init__(self) -> None:
        """初始化 RallyLeaderBroadcast 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        self._base = LeaderBroadcast()

    def init(self, cfg: OutboundInitS) -> None:
        """按配置初始化 RallyLeaderBroadcast。注意：调用方需先准备好必要依赖和输入数据。"""
        self._base.init(cfg)

    def step(self, u: RallyLeaderBroadcastInputS, y: OutboundOutputS) -> None:
        """推进 RallyLeaderBroadcast 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        if u.cmd is None or u.selfState is None or u.slotScale is None:
            raise ValueError("RallyLeaderBroadcast input ports must be bound")
        targets = self._base._targets()
        y.outbox.clear()
        if not targets:
            return
        y.outbox.append(
            MessageEnvelope(
                topic=LEADER_BROADCAST_TOPIC,
                source=self._base._self_id,
                target=targets,
                timestamp=0.0,
                payload={
                    "leader_state": _motion_payload(u.selfState),
                    "cmd": {
                        "stage": int(u.cmd.stage),
                        "pattern": int(u.cmd.pattern),
                        "step": int(u.cmd.step),
                    },
                    "slot_scale": {
                        "scale": u.slotScale.scale,
                        "scale_rate": u.slotScale.scaleRate,
                    },
                    "t_ref": u.t_ref,
                    "t_ref_valid": u.t_ref_valid,
                },
            )
        )

    def reset(self) -> None:
        """复位 RallyLeaderBroadcast 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        self._base.reset()
