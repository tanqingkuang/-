"""编队统一出站处理。注意：消息类型由初始化配置固定，运行期只读取黑板并组包。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable

from src.algorithm.context.context import FormContextS
from src.algorithm.context.leaf_types import CommDirE, dist3d
from src.algorithm.units.process.formation_protocol import (
    FOLLOWER_STATUS_TOPIC,
    LEADER_BROADCAST_TOPIC,
    motion_payload,
)
from src.algorithm.units.process.outbound.base import (
    OutboundBase,
    OutboundInitS,
    OutboundInputS,
    OutboundMessageE,
    OutboundOutputS,
)
from src.common.envelope import MessageEnvelope


@dataclass
class FormationOutboundInitS(OutboundInitS):
    """统一出站初始化配置。注意：僚机状态消息必须提供 leaderId。"""

    messageType: OutboundMessageE | None = None  # 本实例固定发送的消息类型
    leaderId: str = ""  # 僚机状态消息的明确接收方


@dataclass
class FormationOutboundInputS(OutboundInputS):
    """统一出站输入端口。注意：Rally 实体只绑定 context，继承字段仅为基类兼容。"""

    context: FormContextS | None = None  # 编队黑板，出站按固定消息类型读取所需字段


class FormationOutbound(OutboundBase):
    """统一出站单元。注意：初始化决定报文产品，step 不感知实体角色和任务阶段。"""

    def __init__(self) -> None:
        """建立空配置。注意：使用前必须调用 init。"""
        self._self_id = ""
        self._leader_id = ""
        self._net_work = []
        self._writer: Callable[[FormContextS, OutboundOutputS], None] | None = None

    def init(self, cfg: FormationOutboundInitS) -> None:
        """锁定出站消息类型和寻址配置。注意：不在 step 中重新选择消息类型。"""
        # 消息类型属于实体装配结果，而不是运行期任务状态。
        # 这里拒绝普通整数和缺省值，避免配置错误被静默解释成某种角色。
        # 僚机寻址依赖显式 leaderId，不能从可能包含旁路的网络拓扑反推。
        if not isinstance(cfg.messageType, OutboundMessageE):
            raise ValueError("FormationOutbound: messageType 必须显式配置")
        if cfg.messageType == OutboundMessageE.FOLLOWER_STATUS and not cfg.leaderId:
            raise ValueError("FormationOutbound: FOLLOWER_STATUS 必须配置 leaderId")
        self._self_id = cfg.selfId
        self._leader_id = cfg.leaderId
        self._net_work = list(cfg.netWork)
        # 初始化时绑定具体组包函数，后续每拍不再判断角色或任务阶段。
        # 函数引用在整个实体生命周期内保持不变，reset 只清运行期数据。
        # 这样 Entity 只看到统一 step，具体报文产品仍由配置决定。
        self._writer = {
            OutboundMessageE.LEADER_BROADCAST: self._write_leader_broadcast,
            OutboundMessageE.FOLLOWER_STATUS: self._write_follower_status,
        }[cfg.messageType]

    def step(self, u: FormationOutboundInputS, y: OutboundOutputS) -> None:
        """按固定消息类型生成本帧报文。注意：每帧先清空 outbox，避免重复发送。"""
        # context 是唯一业务输入，继承的旧端口字段仅服务兼容类。
        # 每帧覆盖 outbox，通信层拿到的始终是本拍完整快照。
        # 组包失败时也不能残留上一拍可发送消息。
        if u.context is None:
            raise ValueError("FormationOutbound: context 端口未绑定")
        y.outbox.clear()
        if self._writer is None:
            raise RuntimeError("FormationOutbound: 尚未初始化")
        self._writer(u.context, y)

    def reset(self) -> None:
        """复位运行期状态。注意：本单元无跨帧缓存，保留初始化配置。"""
        return None

    def _write_leader_broadcast(self, context: FormContextS, y: OutboundOutputS) -> None:
        """生成长机广播。注意：目标由通信拓扑推导，不向自身发送。"""
        # 圈数计划将被僚机直接执行，必须在发送前整体校验。
        # bool 虽是 int 子类，但不能充当圈数；负数同样没有协议语义。
        # 校验先于组包，防止发送只含部分有效字段的计划。
        if any(
            not isinstance(node_id, str)
            or not isinstance(count, int)
            or isinstance(count, bool)
            or count < 0
            for node_id, count in context.rallyPlan.loop_counts.items()
        ):
            raise ValueError("FormationOutbound: loop_counts 必须由字符串节点 ID 映射到非负整数")
        targets = self._leader_targets()
        # 没有可达僚机时不生成无目标广播，保持旧通信层语义。
        if not targets:
            return
        # 状态、任务指令和公共计划必须来自同一拍黑板快照。
        # effectiveCmd 是位置跟踪限幅后的有效指令，僚机据此建立槽位坐标系。
        # timestamp 仍由外层通信系统填写，算法协议固定输出 0。
        y.outbox.append(
            MessageEnvelope(
                topic=LEADER_BROADCAST_TOPIC,
                source=self._self_id,
                target=targets,
                timestamp=0.0,
                payload={
                    "leader_state": motion_payload(context.selfState),
                    "cmd": {
                        "stage": int(context.cmd.stage),
                        "pattern": int(context.cmd.pattern),
                        "step": int(context.cmd.step),
                        "leader": motion_payload(context.effectiveCmd),
                    },
                    "t_ref": context.rallyPlan.t_ref,
                    "t_ref_valid": context.rallyPlan.valid,
                    "loop_counts": dict(context.rallyPlan.loop_counts),
                },
            )
        )

    def _write_follower_status(self, context: FormContextS, y: OutboundOutputS) -> None:
        """生成僚机状态回报。注意：集结状态直接取位置解算黑板，不在实体中重复同步。"""
        # 位置和航向误差均以本拍 selfCmd 为目标，避免实体重复保存派生量。
        # PosCalcStatus 已锁存越点和切出事件，不能按瞬时距离重新推导 arrived。
        pos_err_m = dist3d(context.selfState.pos, context.selfCmd.pos)
        heading_err_rad = abs(
            math.remainder(context.selfState.v.vPsi - context.selfCmd.v.vPsi, 2.0 * math.pi)
        )
        status = context.posCalcStatus
        # planned_path_length_m 的 -1 哨兵和 rally_state 原样进入协议。
        # STANDBY 的复位语义由 RallyJoinPos 发布，Outbound 不再感知阶段。
        # 显式 leaderId 保证多长机或旁路拓扑下不会误投递。
        y.outbox.append(
            MessageEnvelope(
                topic=FOLLOWER_STATUS_TOPIC,
                source=self._self_id,
                target=self._leader_id,
                timestamp=0.0,
                payload={
                    "id": self._self_id,
                    "pos_east": context.selfState.pos.east,
                    "pos_north": context.selfState.pos.north,
                    "pos_h": context.selfState.pos.h,
                    "pos_err_m": pos_err_m,
                    "heading_err_rad": heading_err_rad,
                    "arrived": int(status.join_exited),
                    "rally_state": status.rally_state,
                    "planned_path_length_m": float(status.planned_path_length_m),
                    "reached_slot_once": bool(status.reached_slot_once),
                },
            )
        )

    def _leader_targets(self) -> list[str]:
        """解析长机广播目标。注意：单工仅允许起点发往终点，双工允许反向发送。"""
        # 拓扑可能重复声明同一目标，最终需要保序去重。
        # 单工反向不可达；双工链路才允许本机作为 endId 时回发。
        targets: list[str] = []
        for link in self._net_work:
            if link.startId == self._self_id:
                targets.append(link.endId)
            elif link.endId == self._self_id and link.dir == CommDirE.DUPLEX:
                targets.append(link.startId)
        return list(dict.fromkeys(target for target in targets if target and target != self._self_id))
