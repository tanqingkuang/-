"""长机广播：唯一的出站实现，携带 slot_scale/t_ref 字段。注意：hold 场景写入 Context 默认集结字段
（scale=1.0/scaleRate=0.0/t_ref_valid=False），接收端按默认值解析，对 hold 无影响；
payload 格式需与 RallyLeaderFollower 解析约定一致。"""

from __future__ import annotations

from dataclasses import dataclass

from src.algorithm.context.leaf_types import CommDirE, MotionProfS, RallySlotScaleS
from src.algorithm.units.process.inbound.rally_leader_follower import LEADER_BROADCAST_TOPIC
from src.algorithm.units.process.outbound.base import OutboundBase, OutboundInitS, OutboundInputS, OutboundOutputS
from src.common.envelope import MessageEnvelope


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


@dataclass
class RallyLeaderBroadcastInputS(OutboundInputS):
    """集结长机广播输入端口。"""

    # 继承 cmd: FormSnapshotS, selfState: MotionProfS
    slotScale: RallySlotScaleS | None = None  # 端口 → Context.slotScale
    t_ref: float = 0.0  # 集结基准时刻（Rally 任务计算，每帧注入）
    t_ref_valid: bool = False  # 是否允许接收方使用 t_ref 执行切出判定


class RallyLeaderBroadcast(OutboundBase):
    """长机广播单元：把本机状态、编队指令、槽位缩放打包成一条多播消息发给所有跟随机。注意：目标列表由通信拓扑推导，不含自身。"""

    def __init__(self) -> None:
        """初始化 RallyLeaderBroadcast 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        self._self_id = ""
        self._net_work = []

    def init(self, cfg: OutboundInitS) -> None:
        """按配置初始化 RallyLeaderBroadcast。注意：调用方需先准备好必要依赖和输入数据。"""
        self._self_id = cfg.selfId
        self._net_work = list(cfg.netWork)

    def step(self, u: RallyLeaderBroadcastInputS, y: OutboundOutputS) -> None:
        """推进 RallyLeaderBroadcast 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        if u.cmd is None or u.selfState is None or u.slotScale is None:
            raise ValueError("RallyLeaderBroadcast input ports must be bound")
        targets = self._targets()  # 据拓扑解析需要接收广播的跟随机
        y.outbox.clear()  # 先清空，保证每帧只产出本帧消息
        if not targets:
            return  # 无接收者则不发消息
        y.outbox.append(
            MessageEnvelope(
                topic=LEADER_BROADCAST_TOPIC,
                source=self._self_id,
                target=targets,
                timestamp=0.0,  # 时间戳由外层通信层补，算法侧置 0
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
        return None

    def _targets(self) -> list[str]:
        """计算长机需要广播的跟随机目标列表。注意：不向自身发送消息。"""
        targets: list[str] = []
        for link in self._net_work:
            # 本机为链路起点：可直接发往终点
            if link.startId == self._self_id:
                targets.append(link.endId)
            # 本机为链路终点且链路双工：反向也可达，加入起点
            elif link.endId == self._self_id and link.dir == CommDirE.DUPLEX:
                targets.append(link.startId)
        # 用 dict.fromkeys 保序去重，并剔除空串与自身
        return list(dict.fromkeys(target for target in targets if target and target != self._self_id))
