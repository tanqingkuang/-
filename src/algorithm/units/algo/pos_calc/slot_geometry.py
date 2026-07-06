"""僚机实体的槽位几何目标计算。注意：支持普通槽位和集结槽位缩放，槽位随长机水平航迹旋转。"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from src.algorithm.context.leaf_types import (
    FormPosS,
    FormSnapshotS,
    MotionProfS,
    RallySlotScaleS,
    copy_velocity,
)
from src.algorithm.units.algo.formation_math import horizontal_track_basis, horizontal_track_vector_to_enu
from src.algorithm.units.algo.pos_calc.base import PosCalcBase, PosCalcInitS, PosCalcInputS, PosCalcOutputS
from src.algorithm.units.algo.td_han import TdHan, TdHanInitS

_GRAVITY_MPS2 = 9.80665
# 相对槽位 TD 的加速度上界默认 = 0.8×各轴加速度权限：前向/垂向按加速度指令上限 6.0；侧向按 g·tan(40°)≈8.2。
_DEFAULT_R_FORWARD = 0.8 * 6.0
_DEFAULT_R_VERTICAL = 0.8 * 6.0
_DEFAULT_R_LATERAL = 0.8 * _GRAVITY_MPS2 * math.tan(math.radians(40.0))


@dataclass
class SlotGeometryInitS(PosCalcInitS):
    """槽位几何初始化参数。注意：formPat(队形名) 和 formPos 按队形行一一对应，仅 formPos 参与解算。"""

    selfId: str = ""
    formPat: list[str] = field(default_factory=list)
    formPos: list[list[FormPosS]] = field(default_factory=list)
    # control_period_s>0 时对"长机航迹系相对槽位偏移"(slot.x/y/z)挂 Han TD 软化队形重构阶跃；<=0 关闭(旧行为)。
    control_period_s: float = 0.0
    rForward: float = _DEFAULT_R_FORWARD  # 前向 slot.x 的 TD 加速度上界
    rVertical: float = _DEFAULT_R_VERTICAL  # 垂向 slot.y 的 TD 加速度上界
    rLateral: float = _DEFAULT_R_LATERAL  # 侧向 slot.z 的 TD 加速度上界
    # 三轴 TD 参考速度上界(=各通道速度权限)，<=0 表示不限。防大阶跃参考跑飞、退化成阶跃使软化失效。
    vMaxForward: float = 0.0
    vMaxVertical: float = 0.0
    vMaxLateral: float = 0.0
    # 是否把 TD 的 x2(相对槽位切换速度)叠加进速度前馈。经真机场景验证：r=0.8×执行器时参考末端减速过急，
    # velFf 忠实驱动飞机按急减速走、ζ=0.65 回路跟不上反致过冲(比不加更差)；故默认关，仅用位置软化。
    # 后续把 r 按回路带宽调缓、或引入积分后再评估是否开启。见 docs/相对槽位TD。
    slotVelFf: bool = True


@dataclass
class SlotGeometryInputS(PosCalcInputS):
    """槽位几何输入端口。注意：slotScale 为可选端口，未绑定时按 scale=1.0/scaleRate=0.0 处理。"""

    leaderState: MotionProfS | None = None
    cmd: FormSnapshotS | None = None
    slotScale: RallySlotScaleS | None = None  # 端口 → Context.slotScale；保持场景默认 scale=1，集结压缩动态变化
    # selfState 继承自 PosCalcInputS：仅在启用 TD 时用于(重)挂载首拍播种，稳态几何目标解算不依赖本机状态。


class SlotGeometry(PosCalcBase):
    """僚机槽位目标计算器。注意：产出随长机水平航迹旋转和可选缩放的槽位目标，前向待飞距闭环交给 PidCompose。"""

    def __init__(self) -> None:
        """初始化 SlotGeometry 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        self._self_id = ""
        self._form_pat: list[str] = []
        self._form_pos: list[list[FormPosS]] = []
        # 相对槽位 TD：仅在 control_period_s>0 时启用。三路对应 slot.x(前向)/slot.y(上)/slot.z(右)。
        self._td_enabled = False
        self._ff_enabled = False
        self._td_x = TdHan()
        self._td_y = TdHan()
        self._td_z = TdHan()
        self._seeded = False

    def init(self, cfg: SlotGeometryInitS) -> None:
        """按配置初始化 SlotGeometry。注意：调用方需先准备好必要依赖和输入数据。"""
        self._self_id = cfg.selfId
        self._form_pat = list(cfg.formPat)
        self._form_pos = [list(row) for row in cfg.formPos]
        self._td_enabled = cfg.control_period_s > 0.0
        self._ff_enabled = cfg.slotVelFf
        if self._td_enabled:
            h = cfg.control_period_s
            self._td_x.init(TdHanInitS(r=cfg.rForward, h=h, vMax=cfg.vMaxForward))
            self._td_y.init(TdHanInitS(r=cfg.rVertical, h=h, vMax=cfg.vMaxVertical))
            self._td_z.init(TdHanInitS(r=cfg.rLateral, h=h, vMax=cfg.vMaxLateral))
        self._seeded = False

    def step(self, u: SlotGeometryInputS, y: PosCalcOutputS) -> None:
        """推进 SlotGeometry 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        if u.leaderState is None or u.cmd is None or y.selfCmd is None:
            raise ValueError("SlotGeometry ports must be bound")
        # cmd.pattern 是纯整型队形索引，直接作为 formPos 行号（0 起）。
        row_index = int(u.cmd.pattern)
        if row_index < 0 or row_index >= len(self._form_pos):
            raise ValueError(f"formation pattern index out of range: {row_index}")
        slot = next((item for item in self._form_pos[row_index] if item.id == self._self_id), None)
        if slot is None:
            raise ValueError(f"missing slot for selfId: {self._self_id}")

        track = _horizontal_track_or_none(u.leaderState)
        track_defined = track is not None

        # 相对槽位 TD 软化：只对长机航迹系下的 (x 前向, y 上, z 右) 三路做——绝对(长机)位置连续、不过 TD。
        # sx/sy/sz 为平滑后的相对偏移，vfx/vfy/vfz 为其导数(相对槽位切换速度前馈)。关闭或未播种时透传原始槽位。
        sx, sy, sz, vfx, vfy, vfz = self._smooth_slot(slot, track, u)

        if track is None:
            # 长机水平航迹未定义时按东向航迹兜底，避免起步/悬停首拍崩溃。
            slot_east, slot_north = sx, -sz
        else:
            # FormPosS 使用 x 前向、z 右侧向，与水平航迹转换函数的轴序一致。
            slot_east, slot_north = horizontal_track_vector_to_enu((sx, sz), track)
        y.selfCmd.pos.east = u.leaderState.pos.east + slot_east
        y.selfCmd.pos.north = u.leaderState.pos.north + slot_north
        y.selfCmd.pos.h = u.leaderState.pos.h + sy
        copy_velocity(u.leaderState.v, y.selfCmd.v)
        # 槽位随长机刚性旋转，僚机航迹偏航角速率即长机的(刚体绕同一瞬心，各点航向角速率相同)。
        y.selfCmd.v.dVPsi = u.leaderState.v.dVPsi
        # 相对槽位垂向速度前馈叠加(TD 的 x2)；巡航无重构时 vfy=0，等价旧行为。
        y.selfCmd.v.vUp = u.leaderState.v.vUp + vfy
        if track_defined:
            track_x, track_y = track
            # 槽位速度前馈：槽位随长机刚性旋转，其真实速度 v_S = (V + b·ω)·t̂ + (a·ω)·n̂，
            # 其中 a=slot.x(前向)、b=slot.z(右向)、ω=长机偏航角速率、n̂=左单位向量。用平滑后的 sx/sz。
            # 沿航迹分量按 b·ω 增减(对某一转向，外侧半径大加速、内侧减速)；a·ω 补后方槽位转弯时的横扫。
            omega = u.leaderState.v.dVPsi
            v_along = u.leaderState.v.vd + sz * omega
            v_swing = sx * omega
            left_x, left_y = -track_y, track_x
            # 刚体旋转速度前馈 + 相对槽位切换速度前馈(TD x2 由航迹系转 ENU 叠加)。
            rel_e, rel_n = horizontal_track_vector_to_enu((vfx, vfz), track)
            y.selfCmd.v.vEast = v_along * track_x + v_swing * left_x + rel_e
            y.selfCmd.v.vNorth = v_along * track_y + v_swing * left_y + rel_n
            # 前向待飞距闭环已下沉到 PidCompose 的前向位置环，这里只产出纯几何目标与速度前馈，不再读本机状态。
            y.selfCmd.v.vd = math.hypot(y.selfCmd.v.vEast, y.selfCmd.v.vNorth)
            y.selfCmd.v.vPsi = math.atan2(y.selfCmd.v.vNorth, y.selfCmd.v.vEast)
        if u.slotScale is not None:
            self._apply_slot_scale(u, y)
        return None

    def reset(self) -> None:
        """复位 SlotGeometry 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        self._seeded = False
        if self._td_enabled:
            self._td_x.reset()
            self._td_y.reset()
            self._td_z.reset()
        return None

    def _smooth_slot(
        self, slot: FormPosS, track: tuple[float, float] | None, u: SlotGeometryInputS
    ) -> tuple[float, float, float, float, float, float]:
        """对相对槽位 (x前向, y上, z右) 三路做 Han TD 软化，返回 (sx, sy, sz, vfx, vfy, vfz)。

        未启用 TD、或首拍尚不满足播种条件(长机航迹未定义/无本机状态)时，透传原始槽位、速度前馈置 0(旧行为)。
        """
        if not self._td_enabled:
            return slot.x, slot.y, slot.z, 0.0, 0.0, 0.0
        if not self._seeded:
            if track is None or u.selfState is None:
                # 播种条件不足，本拍先透传原始槽位、待下一拍航迹可用时再对齐当前位置播种。
                return slot.x, slot.y, slot.z, 0.0, 0.0, 0.0
            self._seed_from_current(slot, track, u)
            self._seeded = True
        sx, vfx = self._td_x.step(slot.x)
        sy, vfy = self._td_y.step(slot.y)
        sz, vfz = self._td_z.step(slot.z)
        if not self._ff_enabled:
            return sx, sy, sz, 0.0, 0.0, 0.0  # 仅位置软化；速度前馈默认关(见 slotVelFf 说明)
        return sx, sy, sz, vfx * 0.2, vfy * 0.2, vfz * 0.2

    def _seed_from_current(self, slot: FormPosS, track: tuple[float, float], u: SlotGeometryInputS) -> None:
        """把本机当前位置换算成长机航迹系相对偏移，作为三路 TD 的 x1 初值(x2=0)，避免起步大阶跃。"""
        assert u.selfState is not None and u.leaderState is not None
        track_x, track_y = track
        rel_e = u.selfState.pos.east - u.leaderState.pos.east
        rel_n = u.selfState.pos.north - u.leaderState.pos.north
        # 投影到长机航迹系：前向=dot(rel, 前向单位)，右向=dot(rel, 右向单位=(track_y,-track_x))。
        seed_fwd = rel_e * track_x + rel_n * track_y
        seed_right = rel_e * track_y - rel_n * track_x
        seed_up = u.selfState.pos.h - u.leaderState.pos.h
        # TD 工作在缩放前的原始槽位空间；当前相对位置是缩放后的物理量，水平需 /scale 对齐(高度不缩放)。
        if u.slotScale is not None and u.slotScale.scale > 0.0:
            seed_fwd /= u.slotScale.scale
            seed_right /= u.slotScale.scale
        self._td_x.seed(seed_fwd, 0.0)
        self._td_y.seed(seed_up, 0.0)
        self._td_z.seed(seed_right, 0.0)

    def _apply_slot_scale(self, u: SlotGeometryInputS, y: PosCalcOutputS) -> None:
        """按 slotScale 后处理槽位位置和速度。注意：高度偏置不缩放，垂向速度不加 scaleRate 项。"""
        assert u.leaderState is not None and u.slotScale is not None and y.selfCmd is not None
        scale = u.slotScale.scale
        scale_rate = u.slotScale.scaleRate
        # 世界坐标系下的未缩放水平偏置（上游刚按标准槽位写入）。
        offset_e = y.selfCmd.pos.east - u.leaderState.pos.east
        offset_n = y.selfCmd.pos.north - u.leaderState.pos.north
        # 水平位置缩放；高度直接保持 leader.h + slot.y，不随 scale 扩展。
        y.selfCmd.pos.east = u.leaderState.pos.east + scale * offset_e
        y.selfCmd.pos.north = u.leaderState.pos.north + scale * offset_n
        # 速度后处理：d/dt(scale·R·slot) = scale·dR/dt·slot + scaleRate·R·slot。
        # 先提取标准槽位相对长机的旋转前馈，再乘 scale 并叠加压缩速度前馈。
        ff_e = y.selfCmd.v.vEast - u.leaderState.v.vEast
        ff_n = y.selfCmd.v.vNorth - u.leaderState.v.vNorth
        y.selfCmd.v.vEast = u.leaderState.v.vEast + scale * ff_e + scale_rate * offset_e
        y.selfCmd.v.vNorth = u.leaderState.v.vNorth + scale * ff_n + scale_rate * offset_n
        y.selfCmd.v.vd = math.hypot(y.selfCmd.v.vEast, y.selfCmd.v.vNorth)
        y.selfCmd.v.vPsi = math.atan2(y.selfCmd.v.vNorth, y.selfCmd.v.vEast)


def _horizontal_track_or_none(state: MotionProfS) -> tuple[float, float] | None:
    """计算可用的水平航迹基向量。注意：航段退化时返回空值并由调用方兜底。"""
    try:
        return horizontal_track_basis(state)
    except ValueError:
        return None
