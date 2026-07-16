"""长机广播：发送编队指令与固定协调计划，载荷格式与僚机原子入站解析一致。"""

from __future__ import annotations

from dataclasses import InitVar, dataclass, field

from src.algorithm.context.leaf_types import CommDirE, MotionProfS, RallyPlanS
from src.algorithm.units.process.formation_protocol import LEADER_BROADCAST_TOPIC, motion_payload
from src.algorithm.units.process.outbound.base import OutboundBase, OutboundInitS, OutboundInputS, OutboundOutputS
from src.common.envelope import MessageEnvelope


_motion_payload = motion_payload  # 兼容既有测试和外部调用，协议实现统一由 formation_protocol 提供。


@dataclass
class RallyLeaderBroadcastInputS(OutboundInputS):
    """集结长机广播输入端口。"""

    # 继承 cmd: FormSnapshotS, selfState: MotionProfS
    leaderCmd: MotionProfS | None = None  # 长机跟踪指令，供僚机建立槽位坐标系。
    rallyPlan: RallyPlanS = field(default_factory=RallyPlanS)  # 端口 → Context.rallyPlan
    t_ref: InitVar[float | None] = None  # 兼容旧 Hold 调用；Rally 应直接绑定 rallyPlan
    t_ref_valid: InitVar[bool | None] = None  # 兼容旧 Hold 调用；Rally 应直接绑定 rallyPlan

    def __post_init__(
        self,
        t_ref: float | None,
        t_ref_valid: bool | None,
    ) -> None:
        """接收旧标量参数。注意：仅在构造期写入计划对象，不参与运行期同步。"""
        if t_ref is not None:
            self.rallyPlan.t_ref = t_ref
        if t_ref_valid is not None:
            self.rallyPlan.valid = t_ref_valid


class RallyLeaderBroadcast(OutboundBase):
    """长机广播单元：把本机状态、编队指令和协调计划打包成一条多播消息。注意：目标列表由通信拓扑推导，不含自身。"""

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
        if u.cmd is None or u.selfState is None:
            raise ValueError("RallyLeaderBroadcast input ports must be bound")
        y.outbox.clear()  # 先清空，异常输入也不能遗留可发送消息
        if any(
            not isinstance(node_id, str)
            or not isinstance(count, int)
            or isinstance(count, bool)
            or count < 0
            for node_id, count in u.rallyPlan.loop_counts.items()
        ):
            raise ValueError("RallyLeaderBroadcast loop_counts 必须由字符串节点 ID 映射到非负整数")
        leader_cmd = u.leaderCmd or u.selfState
        targets = self._targets()  # 据拓扑解析需要接收广播的跟随机
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
                        "leader": _motion_payload(leader_cmd),
                    },
                    "t_ref": u.rallyPlan.t_ref,
                    "t_ref_valid": u.rallyPlan.valid,
                    "loop_counts": dict(u.rallyPlan.loop_counts),
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
