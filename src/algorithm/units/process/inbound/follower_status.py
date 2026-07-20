"""长机解析僚机回报消息，写入 Context.followerStates。注意：以 envelope.source 作为节点 ID，不信任 payload.id。"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from src.algorithm.context.leaf_types import AlgorithmClockS, FollowerStateS, copy_follower_state
from src.algorithm.units.algo.pos_calc.rally_join_pos import (
    RALLY_STATE_EXITED,
    RALLY_STATE_FLYING,
    RALLY_STATE_LOITERING,
    RALLY_STATE_STANDBY,
)
from src.algorithm.units.process.formation_protocol import FOLLOWER_STATUS_TOPIC
from src.algorithm.units.process.inbound.base import InboundInitS
from src.common.envelope import MessageEnvelope


@dataclass
class FollowerStatusInitS(InboundInitS):
    """长机入站初始化配置基类。注意：当前无扩展字段，预留派生扩展。"""

    pass


@dataclass
class FollowerStatusInputS:
    """长机入站输入端口。注意：时钟绑定到 Context 黑板。"""

    inbox: list[MessageEnvelope] = field(default_factory=list)
    clock: AlgorithmClockS | None = None  # 端口 → Context.clock，提供状态更新时间


@dataclass
class FollowerStatusOutputS:
    """长机入站输出端口。注意：followerStates 绑到 Context.followerStates。"""

    followerStates: list[FollowerStateS] | None = None  # 端口 → Context.followerStates


def update_follower_states(
    inbox: list[MessageEnvelope],
    follower_states: list[FollowerStateS],
    now_s: float,
) -> None:
    """解析僚机状态报文并原地更新列表。注意：非法报文不得部分覆盖已有状态。"""
    state_lookup = {state.id: state for state in follower_states}
    for msg in inbox:
        parsed = _parse_follower_status(msg, now_s)
        if parsed is None:
            continue
        entry = state_lookup.get(parsed.id)
        if entry is None:
            follower_states.append(parsed)
            state_lookup[parsed.id] = parsed
        else:
            copy_follower_state(parsed, entry)


def _parse_follower_status(msg: MessageEnvelope, now_s: float) -> FollowerStateS | None:
    """把单条僚机报文解析为完整快照。注意：身份以 envelope.source 为准。"""
    if msg.topic != FOLLOWER_STATUS_TOPIC or not isinstance(msg.payload, dict):
        return None
    payload = msg.payload
    required = ("pos_err_m", "heading_err_rad", "rally_state", "planned_path_length_m")
    if not all(key in payload for key in required):
        return None
    try:
        pos_err_m = float(payload["pos_err_m"])
        heading_err_rad = float(payload["heading_err_rad"])
        planned_path_length_m = float(payload["planned_path_length_m"])
        rally_state = str(payload["rally_state"])
    except (TypeError, ValueError):
        return None
    numeric_fields = (pos_err_m, heading_err_rad, planned_path_length_m)
    if not all(math.isfinite(value) for value in numeric_fields):
        return None
    if planned_path_length_m < 0.0 and planned_path_length_m != -1.0:
        return None
    valid_states = {
        RALLY_STATE_STANDBY,
        RALLY_STATE_FLYING,
        RALLY_STATE_LOITERING,
        RALLY_STATE_EXITED,
    }
    if rally_state not in valid_states:
        return None
    return FollowerStateS(
        id=msg.source,
        posErr_m=pos_err_m,
        headingErr_rad=heading_err_rad,
        lastUpdate_s=now_s,
        plannedPathLength_m=planned_path_length_m,
        rally_state=rally_state,
    )


class FollowerStatus:
    """长机入站单元：从收件箱筛出僚机回报，原地更新 followerStates 列表。注意：断链帧不更新 lastUpdate_s，超时由 Rally 侧处理。"""

    def init(self, cfg: FollowerStatusInitS) -> None:
        """按配置初始化 FollowerStatus。注意：调用方需先准备好必要依赖和输入数据。"""
        del cfg

    def step(self, u: FollowerStatusInputS, y: FollowerStatusOutputS) -> None:
        """推进 FollowerStatus 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        if y.followerStates is None or u.clock is None:
            raise ValueError("FollowerStatus ports must be bound")
        update_follower_states(u.inbox, y.followerStates, u.clock.now_s)

    def reset(self) -> None:
        """复位 FollowerStatus 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        return None
