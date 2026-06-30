"""僚机将本机位置、槽位误差、到达状态打包广播给长机。注意：target 为显式配置的 leaderId，不依赖拓扑推断。"""

from __future__ import annotations

import math
from dataclasses import dataclass

from src.algorithm.context.leaf_types import MotionProfS, NetWorkS, dist3d
from src.algorithm.units.process.outbound.base import OutboundBase, OutboundInitS, OutboundInputS, OutboundOutputS
from src.common.envelope import MessageEnvelope

FOLLOWER_STATUS_TOPIC = "formation.follower_status"


@dataclass
class FollowerBroadcastInitS(OutboundInitS):
    """僚机广播初始化配置。注意：leaderId 必须非空，否则 init 抛 ValueError。"""

    # 继承 selfId: str, netWork: list[NetWorkS]
    leaderId: str = ""  # 长机节点 ID，明确指定发送目标，不依赖 netWork 推断角色


@dataclass
class FollowerBroadcastInputS(OutboundInputS):
    """僚机广播输入端口。"""

    # 继承 cmd: FormSnapshotS, selfState: MotionProfS
    selfCmd: MotionProfS | None = None  # 端口 → Context.selfCmd，当前目标（用于计算 posErr_m）
    selfArrived: int = 0  # 兼容旧协议；新协议使用 rally_state
    rally_state: str = "FLYING"  # 集结汇合状态：FLYING / LOITERING / EXITED
    eta_s: float = 0.0  # 预计到达松散点的仿真时刻（秒）
    pos_err_m_override: float | None = None  # 不为 None 时直接作为 posErr_m 广播（CATCHUP 阶段用 dist2d(self,slot)）


class FollowerBroadcast(OutboundBase):
    """僚机广播单元：把本机位置、到目标距离、到达标志打包为 formation.follower_status 消息。注意：每帧覆盖发送，不累积。"""

    def __init__(self) -> None:
        """初始化 FollowerBroadcast 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        self._self_id = ""
        self._leader_id = ""

    def init(self, cfg: FollowerBroadcastInitS) -> None:
        """按配置初始化 FollowerBroadcast。注意：leaderId 为空则抛 ValueError，防止消息无目标节点广播。"""
        if not cfg.leaderId:
            raise ValueError("FollowerBroadcast: leaderId must not be empty")
        self._self_id = cfg.selfId
        self._leader_id = cfg.leaderId

    def step(self, u: FollowerBroadcastInputS, y: OutboundOutputS) -> None:
        """推进 FollowerBroadcast 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        if u.selfState is None or u.selfCmd is None or y.outbox is None:
            raise ValueError("FollowerBroadcast input ports must be bound")
        # 僚机回报每帧只保留当前状态，先清 outbox 防止上帧消息重复发送。
        # target 使用显式 leaderId，而不是从拓扑反推，避免多长机/旁路链路场景误投递。
        # arrived 来自实体锁存值，不能由当前 pos_err_m 反算，否则越过 M_i 后会反复抖动。
        # pos_err_m 跟随 selfCmd：APPROACH 表示到 M_i，LOOSE/COMPRESS 表示到当前槽位。
        y.outbox.clear()
        pos_err_m = u.pos_err_m_override if u.pos_err_m_override is not None else dist3d(u.selfState.pos, u.selfCmd.pos)
        heading_err_rad = abs(math.remainder(u.selfState.v.vPsi - u.selfCmd.v.vPsi, 2.0 * math.pi))
        y.outbox.append(
            MessageEnvelope(
                topic=FOLLOWER_STATUS_TOPIC,
                source=self._self_id,
                target=self._leader_id,
                timestamp=0.0,
                payload={
                    "id": self._self_id,
                    "pos_east": u.selfState.pos.east,
                    "pos_north": u.selfState.pos.north,
                    "pos_h": u.selfState.pos.h,
                    "pos_err_m": pos_err_m,
                    "heading_err_rad": heading_err_rad,
                    "arrived": int(u.selfArrived),
                    "rally_state": u.rally_state,
                    "eta_s": float(u.eta_s),
                },
            )
        )

    def reset(self) -> None:
        """复位 FollowerBroadcast 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        return None
