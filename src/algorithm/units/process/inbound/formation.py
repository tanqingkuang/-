"""统一编队入站处理。注意：只按消息 topic 解析并原子更新黑板。"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from src.algorithm.context.leaf_types import (
    AlgorithmClockS,
    FollowerStateS,
    FormSnapshotS,
    MotionProfS,
    RallyPlanS,
    copy_follower_state,
    copy_motion,
    copy_snapshot,
)
from src.algorithm.units.process.formation_protocol import FOLLOWER_STATUS_TOPIC, LEADER_BROADCAST_TOPIC
from src.algorithm.units.process.inbound.base import InboundBase, InboundInitS
from src.algorithm.units.process.inbound.follower_status import _parse_follower_status
from src.algorithm.units.process.inbound.rally_leader_follower import (
    _parse_leader_broadcast,
)

if TYPE_CHECKING:
    from src.algorithm.entity.types import EntityRuntimeS
    from src.common.envelope import MessageEnvelope


@dataclass
class FormationInboundInitS(InboundInitS):
    """统一入站初始化配置。注意：selfId 只用于确认本机是否取得圈数分配。"""

    selfId: str = ""  # 本机节点 ID


@dataclass
class FormationInboundInputS:
    """统一入站私有输入端口。"""

    inbox: list[MessageEnvelope] = field(default_factory=list)
    clock: AlgorithmClockS | None = None


@dataclass
class FormationInboundOutputS:
    """统一入站私有输出端口。注意：只暴露协议解析需要更新的黑板字段。"""

    leaderState: MotionProfS | None = None
    leaderCmd: MotionProfS | None = None
    cmd: FormSnapshotS | None = None
    rallyPlan: RallyPlanS | None = None
    followerStates: list[FollowerStateS] | None = None


class FormationInbound(InboundBase):
    """按 topic 解析所有编队报文。注意：不感知实体角色和任务阶段。"""

    def __init__(self) -> None:
        """建立未绑定的私有端口。"""
        self._u = FormationInboundInputS()
        self._y = FormationInboundOutputS()
        self._bound = False

    def bind(self, runtime: EntityRuntimeS) -> None:
        """绑定入站所需字段。注意：后续 step 不再访问完整运行环境。"""
        cxt = runtime.context
        self._u = FormationInboundInputS(inbox=runtime.inbox, clock=cxt.clock)
        self._y = FormationInboundOutputS(
            leaderState=cxt.leaderState,
            leaderCmd=cxt.leaderCmd,
            cmd=cxt.cmd,
            rallyPlan=cxt.rallyPlan,
            followerStates=cxt.followerStates,
        )
        self._bound = True

    def init(self, cfg: FormationInboundInitS) -> None:
        """记录本机身份。注意：所有飞机使用同一套消息路由。"""
        # selfId 只用于验证公共计划是否包含本机，不参与消息类型选择。
        # 入站单元不配置角色，因此长机和僚机可以使用同一个实现。
        if not cfg.selfId:
            raise ValueError("FormationInbound selfId must be non-empty")
        self._self_id = cfg.selfId

    def step(self) -> None:
        """处理本帧邮箱。注意：空邮箱不清空黑板，未知 topic 直接忽略。"""
        if not self._bound:
            raise ValueError("FormationInbound 尚未绑定端口")
        self._process(self._u, self._y)

    def _process(self, u: FormationInboundInputS, y: FormationInboundOutputS) -> None:
        """解析本拍邮箱并直接更新绑定输出。"""
        if y.followerStates is None or u.clock is None:
            raise ValueError("FormationInbound ports must be bound")
        # 空邮箱表示没有新数据，不能清除上一拍仍有效的长机状态和公共计划。
        # 先建立索引，使同一批消息对既有节点执行原地更新。
        # 原地更新保证任务单元持有的 followerStates 列表引用始终有效。
        # 同一节点一拍出现多条消息时按邮箱顺序处理，最后一条有效消息生效。
        # 非法消息不得创建空状态，也不得刷新既有状态的超时时间。
        state_lookup = {state.id: state for state in y.followerStates}
        for message in u.inbox:
            # topic 是唯一解析路由依据，角色和任务阶段不参与通信解析。
            # 未知 topic 属于其他业务流，统一入站应直接忽略而不是报错。
            if message.topic == LEADER_BROADCAST_TOPIC:
                self._apply_leader_message(message.payload, y)
            elif message.topic == FOLLOWER_STATUS_TOPIC:
                parsed = _parse_follower_status(message, u.clock.now_s)
                if parsed is None:
                    continue
                entry = state_lookup.get(parsed.id)
                # 新节点追加，既有节点逐字段覆盖，不能替换整个状态列表。
                if entry is None:
                    y.followerStates.append(parsed)
                    state_lookup[parsed.id] = parsed
                else:
                    copy_follower_state(parsed, entry)

    def reset(self) -> None:
        """复位入站单元。注意：运行期数据由 Context 所有者统一清理。"""
        return None

    def _apply_leader_message(self, payload: object, y: FormationInboundOutputS) -> None:
        """解析并提交长机广播。注意：任何字段非法时整条消息丢弃。"""
        # 解析函数先构造完整临时结果，失败时黑板保持上一份有效快照。
        # 只有全部字段通过校验后，状态、指令和计划才一起提交。
        if not isinstance(payload, dict):
            return
        try:
            parsed = _parse_leader_broadcast(payload)
        except (TypeError, ValueError, OverflowError):
            return
        if y.leaderState is None or y.leaderCmd is None or y.cmd is None or y.rallyPlan is None:
            raise ValueError("FormationInbound leader ports must be bound")
        # 嵌套对象按字段复制，维持其他单元在 init 时绑定的引用。
        # 状态、指令和协调计划来自同一报文，禁止跨报文拼接半份快照。
        # 圈数映射先清后写，避免新计划缺少的旧节点继续保留分配。
        copy_motion(parsed.leader_state, y.leaderState)
        copy_motion(parsed.leader_cmd, y.leaderCmd)
        copy_snapshot(parsed.cmd, y.cmd)
        y.rallyPlan.t_ref = parsed.t_ref
        y.rallyPlan.loop_counts.clear()
        y.rallyPlan.loop_counts.update(parsed.loop_counts)
        # 即使报文声明有效，缺少本机圈数时也不得执行公共计划。
        y.rallyPlan.valid = parsed.t_ref_valid and self._self_id in parsed.loop_counts
