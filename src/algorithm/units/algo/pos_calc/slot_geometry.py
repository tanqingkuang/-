"""僚机实体的槽位几何目标计算。注意：槽位使用长机三维 FUR 航迹系。"""

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
from src.algorithm.units.algo.pos_calc.base import PosCalcBase, PosCalcInitS, PosCalcInputS, PosCalcOutputS
from src.algorithm.units.algo.td_han import TdHan, TdHanInitS
from src.common.coordinates import (
    FurBasis,
    enu_to_fur,
    fur_basis_from_angles,
    fur_basis_from_velocity,
    fur_to_enu,
)

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
    leaderCmd: MotionProfS | None = None  # 长机跟踪指令；槽位坐标系方向优先使用它，位置原点仍取 leaderState。
    cmd: FormSnapshotS | None = None
    slotScale: RallySlotScaleS | None = None  # 端口 → Context.slotScale；保持场景默认 scale=1，集结压缩动态变化
    # selfState 继承自 PosCalcInputS：仅在启用 TD 时用于(重)挂载首拍播种，稳态几何目标解算不依赖本机状态。


class SlotGeometry(PosCalcBase):
    """僚机槽位目标计算器。注意：槽位使用长机三维 FUR 航迹系，前向待飞距闭环交给 PidCompose。"""

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

        # 指令航迹可用时优先稳定队形朝向；否则退回实际航迹，位置原点始终取长机实际位置。
        frame, basis = _select_frame_and_basis(u.leaderCmd, u.leaderState)

        # 相对槽位 TD 软化：只对长机 FUR 航迹系下的 (x 前向, y 上法向, z 右) 三路做。
        # sx/sy/sz 为平滑后的相对偏移，vfx/vfy/vfz 为其导数(相对槽位切换速度前馈)。关闭或未播种时透传原始槽位。
        # TD 工作在缩放前的槽位坐标中，使集结比例变化不会反向污染平滑器内部状态。
        sx, sy, sz, vfx, vfy, vfz = self._smooth_slot(slot, basis, u)
        scale = u.slotScale.scale if u.slotScale is not None else 1.0
        scale_rate = u.slotScale.scaleRate if u.slotScale is not None else 0.0
        # 集结缩放只作用于队形平面尺寸 x/z；y 仍表示固定的上法向间隔。
        slot_fur = (scale * sx, sy, scale * sz)
        transform_basis = basis if basis is not None else fur_basis_from_angles(0.0, 0.0)
        # 长机航迹未定义时按“东向平飞”的 FUR 兜底，保持 x→东、y→天、z→南的旧行为。
        # 兜底基仍满足前上右手性，只是不声称能代表零水平速度时不存在的真实航向。
        slot_east, slot_north, slot_up = fur_to_enu(slot_fur, transform_basis)
        # 槽位是相对量，完成旋转后再叠加长机实际 ENU 位置，不能使用指令位置作原点。
        y.selfCmd.pos.east = u.leaderState.pos.east + slot_east
        y.selfCmd.pos.north = u.leaderState.pos.north + slot_north
        y.selfCmd.pos.h = u.leaderState.pos.h + slot_up
        copy_velocity(frame.v, y.selfCmd.v)
        # 槽位随长机指令航迹刚性旋转，避免长机实际速度受扰时把僚机坐标系带乱。
        y.selfCmd.v.dVPsi = frame.v.dVPsi
        if basis is not None:
            # 偏航角速率左转为正，而 FUR 的 z 轴向右；刚体偏航速度统一在右轴表达。
            # 因右轴与左转正方向相反，后方槽位的横扫项在水平飞行时表现为 -a·omega。
            # theta 固定时：v_F=b·ω·cosθ，v_U=-b·ω·sinθ，v_R=(-a·cosθ+y·sinθ)·ω。
            # 当前状态不提供 theta_dot，故这里只加入偏航刚体项；水平飞行时严格退化为 (b·ω, 0, -a·ω)。
            omega = frame.v.dVPsi
            # cos_theta 和 sin_theta 直接从单位前轴读取，避免再次由速度反解角度引入分支差异。
            cos_theta = math.hypot(basis[0][0], basis[0][1])
            sin_theta = basis[0][2]
            yaw_velocity_fur = (
                slot_fur[2] * omega * cos_theta,
                -slot_fur[2] * omega * sin_theta,
                (-slot_fur[0] * cos_theta + slot_fur[1] * sin_theta) * omega,
            )
            # TD 导数及 scaleRate 也在同一 FUR 基中叠加，避免先转世界系再缩放破坏三维轴义。
            # 缩放只作用 x/z，因此 y 通道仅保留 TD 自身导数，不叠加 scaleRate。
            transition_velocity_fur = (
                scale * vfx + scale_rate * sx,
                vfy,
                scale * vfz + scale_rate * sz,
            )
            relative_velocity_fur = (
                yaw_velocity_fur[0] + transition_velocity_fur[0],
                yaw_velocity_fur[1] + transition_velocity_fur[1],
                yaw_velocity_fur[2] + transition_velocity_fur[2],
            )
            rel_e, rel_n, rel_u = fur_to_enu(relative_velocity_fur, basis)
            y.selfCmd.v.vEast = frame.v.vEast + rel_e
            y.selfCmd.v.vNorth = frame.v.vNorth + rel_n
            y.selfCmd.v.vUp = frame.v.vUp + rel_u
        else:
            # 兜底帧无旋转语义，仅保留队形重构和缩放速度前馈。
            transition_velocity_fur = (
                scale * vfx + scale_rate * sx,
                vfy,
                scale * vfz + scale_rate * sz,
            )
            rel_e, rel_n, rel_u = fur_to_enu(transition_velocity_fur, transform_basis)
            y.selfCmd.v.vEast = frame.v.vEast + rel_e
            y.selfCmd.v.vNorth = frame.v.vNorth + rel_n
            y.selfCmd.v.vUp = frame.v.vUp + rel_u
        # 前向待飞距闭环已下沉到 PidCompose；本单元只产出几何目标与刚体/重构速度前馈。
        # 三个 ENU 速度分量更新后必须同步派生 vd、vPsi、vTheta，保证下游读到自洽指令。
        y.selfCmd.v.vd = math.hypot(y.selfCmd.v.vEast, y.selfCmd.v.vNorth)
        y.selfCmd.v.vPsi = (
            math.atan2(y.selfCmd.v.vNorth, y.selfCmd.v.vEast)
            if y.selfCmd.v.vd > 0.0
            else 0.0
        )
        y.selfCmd.v.vTheta = math.atan2(y.selfCmd.v.vUp, y.selfCmd.v.vd)
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
        self, slot: FormPosS, basis: FurBasis | None, u: SlotGeometryInputS
    ) -> tuple[float, float, float, float, float, float]:
        """对相对槽位 (x前向, y上, z右) 三路做 Han TD 软化，返回 (sx, sy, sz, vfx, vfy, vfz)。

        未启用 TD、或首拍尚不满足播种条件(长机航迹未定义/无本机状态)时，透传原始槽位、速度前馈置 0(旧行为)。
        """
        if not self._td_enabled:
            return slot.x, slot.y, slot.z, 0.0, 0.0, 0.0
        if not self._seeded:
            if basis is None or u.selfState is None:
                # 播种条件不足，本拍先透传原始槽位、待下一拍航迹可用时再对齐当前位置播种。
                return slot.x, slot.y, slot.z, 0.0, 0.0, 0.0
            self._seed_from_current(basis, u)
            self._seeded = True
        sx, vfx = self._td_x.step(slot.x)
        sy, vfy = self._td_y.step(slot.y)
        sz, vfz = self._td_z.step(slot.z)
        if not self._ff_enabled:
            return sx, sy, sz, 0.0, 0.0, 0.0  # 仅位置软化；速度前馈默认关(见 slotVelFf 说明)
        return sx, sy, sz, vfx * 0.2, vfy * 0.2, vfz * 0.2

    def _seed_from_current(self, basis: FurBasis, u: SlotGeometryInputS) -> None:
        """把本机当前位置换算成长机三维 FUR 相对偏移，作为三路 TD 初值，避免起步大阶跃。"""
        assert u.selfState is not None and u.leaderState is not None
        rel_e = u.selfState.pos.east - u.leaderState.pos.east
        rel_n = u.selfState.pos.north - u.leaderState.pos.north
        rel_u = u.selfState.pos.h - u.leaderState.pos.h
        # 播种反投影必须复用本拍相同的三维基，否则爬升时前向距离会被误记为纯水平距离。
        seed_fwd, seed_up, seed_right = enu_to_fur((rel_e, rel_n, rel_u), basis)
        # TD 工作在缩放前的原始槽位空间；当前相对位置是缩放后的物理量，水平需 /scale 对齐(高度不缩放)。
        if u.slotScale is not None and u.slotScale.scale > 0.0:
            seed_fwd /= u.slotScale.scale
            seed_right /= u.slotScale.scale
        self._td_x.seed(seed_fwd, 0.0)
        self._td_y.seed(seed_up, 0.0)
        self._td_z.seed(seed_right, 0.0)


def _fur_basis_or_none(state: MotionProfS) -> FurBasis | None:
    """计算可用的三维 FUR 航迹基。注意：水平航迹退化时返回空值并由调用方兜底。"""
    try:
        return fur_basis_from_velocity((state.v.vEast, state.v.vNorth, state.v.vUp))
    except ValueError:
        return None


def _select_frame_and_basis(
    leader_cmd: MotionProfS | None, leader_state: MotionProfS
) -> tuple[MotionProfS, FurBasis | None]:
    """选择槽位航迹系参考帧。注意：默认零值 leaderCmd 不能覆盖仍有效的 leaderState。"""
    if leader_cmd is not None:
        basis = _fur_basis_or_none(leader_cmd)
        if basis is not None:
            return leader_cmd, basis
    return leader_state, _fur_basis_or_none(leader_state)
