"""带缩放的槽位几何：在 SlotGeometry 后处理中将槽位偏置乘以 scale，COMPRESS 阶段追加压缩速度前馈。注意：scale=1.0 且 scaleRate=0 时精确复现 SlotGeometry。"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from src.algorithm.context.leaf_types import FormCommInitS, RallySlotScaleS
from src.algorithm.units.algo.pos_calc.base import PosCalcOutputS
from src.algorithm.units.algo.pos_calc.slot_geometry import SlotGeometry, SlotGeometryInitS, SlotGeometryInputS


@dataclass
class ScaledSlotInitS:
    """ScaledSlotGeometry 初始化参数。注意：commInit 用于组装 SlotGeometryInitS，selfId 用于查找本机槽位。"""

    selfId: str = ""
    commInit: FormCommInitS = field(default_factory=FormCommInitS)


@dataclass
class ScaledSlotInputS(SlotGeometryInputS):
    """ScaledSlotGeometry 输入端口。注意：继承 SlotGeometryInputS 确保 super().step() 类型兼容。"""

    # 继承 selfState: MotionProfS → Context.selfState
    # 继承 leaderState: MotionProfS → Context.leaderState
    # 继承 cmd: FormSnapshotS → Context.cmd
    slotScale: RallySlotScaleS | None = None  # 端口 → Context.slotScale（新增字段）


class ScaledSlotGeometry(SlotGeometry):
    """带缩放的槽位几何。注意：先调 super().step() 得到 scale=1 结果，再按 scale/scaleRate 做后处理。"""

    def init(self, cfg: ScaledSlotInitS) -> None:
        """按配置初始化 ScaledSlotGeometry。注意：必须显式调用 super().init()，否则 step() 必抛 ValueError。"""
        super().init(SlotGeometryInitS(
            selfId=cfg.selfId,
            formPat=cfg.commInit.formPat,
            formPos=cfg.commInit.formPos,
        ))

    def step(self, u: ScaledSlotInputS, y: PosCalcOutputS) -> None:
        """推进 ScaledSlotGeometry 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        if u.slotScale is None:
            raise ValueError("ScaledSlotGeometry: slotScale port must be bound")
        super().step(u, y)  # 先按 scale=1 算标准槽位

        scale = u.slotScale.scale
        scaleRate = u.slotScale.scaleRate

        # 世界坐标系下的未缩放偏置（super 已算好）
        offset_e = y.selfCmd.pos.east - u.leaderState.pos.east
        offset_n = y.selfCmd.pos.north - u.leaderState.pos.north
        offset_h = y.selfCmd.pos.h - u.leaderState.pos.h

        # 位置缩放
        y.selfCmd.pos.east = u.leaderState.pos.east + scale * offset_e
        y.selfCmd.pos.north = u.leaderState.pos.north + scale * offset_n
        y.selfCmd.pos.h = u.leaderState.pos.h + scale * offset_h

        # 速度后处理：d/dt(scale·R·slot) = scale·dR/dt·slot + scaleRate·R·slot
        # super 给出 leaderVel + dR/dt·slot（旋转前馈）；提取旋转前馈再乘 scale，加 scaleRate 项
        ff_e = y.selfCmd.v.vEast - u.leaderState.v.vEast
        ff_n = y.selfCmd.v.vNorth - u.leaderState.v.vNorth
        ff_up = y.selfCmd.v.vUp - u.leaderState.v.vUp
        y.selfCmd.v.vEast = u.leaderState.v.vEast + scale * ff_e + scaleRate * offset_e
        y.selfCmd.v.vNorth = u.leaderState.v.vNorth + scale * ff_n + scaleRate * offset_n
        y.selfCmd.v.vUp = u.leaderState.v.vUp + scale * ff_up + scaleRate * offset_h
        y.selfCmd.v.vd = math.hypot(y.selfCmd.v.vEast, y.selfCmd.v.vNorth)
        y.selfCmd.v.vPsi = math.atan2(y.selfCmd.v.vNorth, y.selfCmd.v.vEast)
        # dVPsi（偏航角速率）不随 scale 变化，保持父类值不动
