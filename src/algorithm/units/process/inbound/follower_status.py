"""长机解析僚机回报消息，写入 Context.followerStates。注意：以 envelope.source 作为节点 ID，不信任 payload.id。"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from src.algorithm.context.leaf_types import AlgorithmClockS, FollowerStateS, copy_follower_state
from src.algorithm.units.algo.pos_calc.rally_join_pos import RALLY_STATE_FLYING
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
    if not all(key in payload for key in ("pos_east", "pos_north", "pos_h", "pos_err_m")):
        return None
    try:
        pos_east = float(payload["pos_east"])
        pos_north = float(payload["pos_north"])
        pos_h = float(payload["pos_h"])
        pos_err_m = float(payload["pos_err_m"])
        heading_err_rad = float(payload.get("heading_err_rad", 0.0))
        arrived = int(payload.get("arrived", 0))
        planned_path_length_m = float(payload.get("planned_path_length_m", -1.0))
        rally_state = str(payload.get("rally_state", RALLY_STATE_FLYING))
        reached_slot_once = bool(payload.get("reached_slot_once", False))
    except (TypeError, ValueError):
        return None
    numeric_fields = (pos_east, pos_north, pos_h, pos_err_m, heading_err_rad, planned_path_length_m)
    if not all(math.isfinite(value) for value in numeric_fields):
        return None
    if planned_path_length_m < 0.0 and planned_path_length_m != -1.0:
        return None
    parsed = FollowerStateS(id=msg.source)
    parsed.pos.east = pos_east
    parsed.pos.north = pos_north
    parsed.pos.h = pos_h
    parsed.posErr_m = pos_err_m
    parsed.headingErr_rad = heading_err_rad
    parsed.arrived = arrived
    parsed.valid = True
    parsed.lastUpdate_s = now_s
    parsed.plannedPathLength_m = planned_path_length_m
    parsed.rally_state = rally_state
    parsed.reachedSlotOnce = reached_slot_once
    return parsed


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
