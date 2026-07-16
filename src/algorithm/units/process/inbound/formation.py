"""统一编队入站处理。注意：只按消息 topic 解析并原子更新黑板。"""

from __future__ import annotations

from dataclasses import dataclass

from src.algorithm.context.context import FormContextS
from src.algorithm.context.leaf_types import copy_follower_state, copy_motion, copy_snapshot
from src.algorithm.units.process.inbound.base import InboundBase, InboundInitS, InboundInputS, InboundOutputS
from src.algorithm.units.process.inbound.follower_status import _parse_follower_status
from src.algorithm.units.process.inbound.rally_leader_follower import (
    LEADER_BROADCAST_TOPIC,
    _parse_leader_broadcast,
)
from src.algorithm.units.process.outbound.follower_broadcast import FOLLOWER_STATUS_TOPIC


@dataclass
class FormationInboundInitS(InboundInitS):
    """统一入站初始化配置。注意：selfId 只用于确认本机是否取得圈数分配。"""

    selfId: str = ""  # 本机节点 ID


@dataclass
class FormationInboundOutputS(InboundOutputS):
    """统一入站输出端口。注意：context 绑定实体持有的完整黑板。"""

    context: FormContextS | None = None  # 所有解析结果的原地写入目标


class FormationInbound(InboundBase):
    """按 topic 解析所有编队报文。注意：不感知实体角色和任务阶段。"""

    def init(self, cfg: FormationInboundInitS) -> None:
        """记录本机身份。注意：所有飞机使用同一套消息路由。"""
        if not cfg.selfId:
            raise ValueError("FormationInbound selfId must be non-empty")
        self._self_id = cfg.selfId

    def step(self, u: InboundInputS, y: FormationInboundOutputS) -> None:
        """处理本帧邮箱。注意：空邮箱不清空黑板，未知 topic 直接忽略。"""
        cxt = y.context
        if cxt is None:
            raise ValueError("FormationInbound context port must be bound")
        state_lookup = {state.id: state for state in cxt.followerStates}
        for message in u.inbox:
            if message.topic == LEADER_BROADCAST_TOPIC:
                self._apply_leader_message(message.payload, cxt)
            elif message.topic == FOLLOWER_STATUS_TOPIC:
                parsed = _parse_follower_status(message, cxt.clock.now_s)
                if parsed is None:
                    continue
                entry = state_lookup.get(parsed.id)
                if entry is None:
                    cxt.followerStates.append(parsed)
                    state_lookup[parsed.id] = parsed
                else:
                    copy_follower_state(parsed, entry)

    def reset(self) -> None:
        """复位入站单元。注意：运行期数据由 Context 所有者统一清理。"""
        return None

    def _apply_leader_message(self, payload: object, cxt: FormContextS) -> None:
        """解析并提交长机广播。注意：任何字段非法时整条消息丢弃。"""
        if not isinstance(payload, dict):
            return
        try:
            parsed = _parse_leader_broadcast(payload)
        except (TypeError, ValueError, OverflowError):
            return
        copy_motion(parsed.leader_state, cxt.leaderState)
        copy_motion(parsed.leader_cmd, cxt.leaderCmd)
        copy_snapshot(parsed.cmd, cxt.cmd)
        cxt.rallyPlan.t_ref = parsed.t_ref
        cxt.rallyPlan.loop_counts.clear()
        cxt.rallyPlan.loop_counts.update(parsed.loop_counts)
        cxt.rallyPlan.valid = parsed.t_ref_valid and self._self_id in parsed.loop_counts
