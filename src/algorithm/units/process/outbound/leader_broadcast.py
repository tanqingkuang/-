"""把长机状态打包为多播编队消息。注意：目标列表来自通信拓扑。"""

from __future__ import annotations

from src.algorithm.context.leaf_types import CommDirE, MotionProfS
from src.algorithm.units.process.inbound.leader_follower import LEADER_BROADCAST_TOPIC
from src.algorithm.units.process.outbound.base import OutboundBase, OutboundInitS, OutboundInputS, OutboundOutputS
from src.common.envelope import MessageEnvelope


class LeaderBroadcast(OutboundBase):
    """长机广播单元：把本机状态与编队指令打包成一条多播消息发给所有跟随机。注意：目标列表由通信拓扑推导，不含自身。"""

    def __init__(self) -> None:
        """初始化 LeaderBroadcast 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        self._self_id = ""
        self._net_work = []

    def init(self, cfg: OutboundInitS) -> None:
        """按配置初始化 LeaderBroadcast。注意：调用方需先准备好必要依赖和输入数据。"""
        self._self_id = cfg.selfId
        self._net_work = list(cfg.netWork)

    def step(self, u: OutboundInputS, y: OutboundOutputS) -> None:
        """推进 LeaderBroadcast 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        if u.cmd is None or u.selfState is None:
            raise ValueError("LeaderBroadcast input ports must be bound")
        targets = self._targets()  # 据拓扑解析需要接收广播的跟随机
        y.outbox.clear()  # 先清空，保证每帧只产出本帧消息
        if not targets:
            return  # 无接收者则不发消息
        # 单条多播消息：枚举转 int 便于跨进程序列化，topic 与入站解析约定一致
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
                },
            )
        )

    def reset(self) -> None:
        """复位 LeaderBroadcast 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
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


def _motion_payload(motion: MotionProfS) -> dict[str, dict[str, float]]:
    """把运动状态转换为通信载荷。注意：字段名需与入站解析保持一致。"""
    return {
        "pos": {
            "east": motion.pos.east,
            "north": motion.pos.north,
            "h": motion.pos.h,
        },
        "vd": {
            "vEast": motion.vd.vEast,
            "vNorth": motion.vd.vNorth,
            "vUp": motion.vd.vUp,
            "vTheta": motion.vd.vTheta,
            "vPsi": motion.vd.vPsi,
            "vd": motion.vd.vd,
        },
    }
