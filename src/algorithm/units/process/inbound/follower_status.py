"""长机解析僚机回报消息，写入 Context.followerStates。注意：以 envelope.source 作为节点 ID，不信任 payload.id。"""

from __future__ import annotations

import math
from dataclasses import dataclass

from src.algorithm.context.leaf_types import FollowerStateS
from src.algorithm.units.process.inbound.base import InboundBase, InboundInitS, InboundInputS, InboundOutputS
from src.algorithm.units.process.outbound.follower_broadcast import FOLLOWER_STATUS_TOPIC


@dataclass
class FollowerStatusInitS(InboundInitS):
    """长机入站初始化配置基类。注意：当前无扩展字段，预留派生扩展。"""

    pass


@dataclass
class FollowerStatusInputS(InboundInputS):
    """长机入站输入端口。注意：now_s 由实体从边界注入，写入 lastUpdate_s。"""

    # 继承 inbox: list[MessageEnvelope]
    now_s: float = 0.0  # 当前仿真时间，写入 FollowerStateS.lastUpdate_s


@dataclass
class FollowerStatusOutputS(InboundOutputS):
    """长机入站输出端口。注意：followerStates 绑到 Context.followerStates。"""

    followerStates: list[FollowerStateS] | None = None  # 端口 → Context.followerStates


class FollowerStatus(InboundBase):
    """长机入站单元：从收件箱筛出僚机回报，原地更新 followerStates 列表。注意：断链帧不更新 lastUpdate_s，超时由 Rally 侧处理。"""

    def init(self, cfg: FollowerStatusInitS) -> None:
        """按配置初始化 FollowerStatus。注意：调用方需先准备好必要依赖和输入数据。"""
        del cfg

    def step(self, u: FollowerStatusInputS, y: FollowerStatusOutputS) -> None:
        """推进 FollowerStatus 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        if y.followerStates is None:
            raise ValueError("FollowerStatus output port must be bound")
        # followerStates 是 Context 中被 Rally 任务单元共享的列表，必须原地更新，不能整体替换。
        # 空 inbox 表示本帧未收到回报，不清 valid，也不刷新 lastUpdate_s，由 Rally 按超时判失效。
        # 同一帧多条同源消息按遍历顺序覆盖，保留最后一条，匹配通信层“后到为准”的语义。
        # payload.id 只用于诊断展示，不用于身份判定，避免伪造载荷污染长机状态表。
        # 关键字段先做完整性校验，防止缺字段消息创建半初始化的 FollowerStateS。
        _state_lookup: dict[str, FollowerStateS] = {s.id: s for s in y.followerStates}
        for msg in u.inbox:
            if msg.topic != FOLLOWER_STATUS_TOPIC or not isinstance(msg.payload, dict):
                continue
            payload = msg.payload
            # 关键字段缺失则丢弃，避免写入半截状态
            if not all(k in payload for k in ("pos_east", "pos_north", "pos_h", "pos_err_m")):
                continue
            # 先把全部字段转换到局部变量；任何字段非法时不得部分覆盖已有状态。
            try:
                pos_east = float(payload["pos_east"])
                pos_north = float(payload["pos_north"])
                pos_h = float(payload["pos_h"])
                pos_err_m = float(payload["pos_err_m"])
                heading_err_rad = float(payload.get("heading_err_rad", 0.0))
                arrived = int(payload.get("arrived", 0))
                eta_s = float(payload.get("eta_s", 0.0))
                rally_state = str(payload.get("rally_state", "FLYING"))
                reached_slot_once = bool(payload.get("reached_slot_once", False))
            except (TypeError, ValueError):
                continue
            numeric_fields = (pos_east, pos_north, pos_h, pos_err_m, heading_err_rad, eta_s)
            if not all(math.isfinite(value) for value in numeric_fields):
                continue
            # 以 envelope.source 作为节点 ID，不信任 payload 中的 id 字段。
            node_id = msg.source
            entry = _state_lookup.get(node_id)
            if entry is None:
                entry = FollowerStateS(id=node_id)
                y.followerStates.append(entry)
                _state_lookup[node_id] = entry
            entry.pos.east = pos_east
            entry.pos.north = pos_north
            entry.pos.h = pos_h
            entry.posErr_m = pos_err_m
            entry.headingErr_rad = heading_err_rad
            entry.arrived = arrived
            entry.eta_s = eta_s
            entry.rally_state = rally_state
            entry.reachedSlotOnce = reached_slot_once
            entry.id = node_id
            entry.valid = True
            entry.lastUpdate_s = u.now_s

    def reset(self) -> None:
        """复位 FollowerStatus 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        return None
