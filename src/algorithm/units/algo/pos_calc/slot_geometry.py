"""僚机实体的槽位几何目标计算。注意：槽位随长机水平航迹旋转。"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from src.algorithm.context.leaf_types import (
    FormPatE,
    FormPosS,
    FormSnapshotS,
    MotionProfS,
    copy_velocity,
)
from src.algorithm.units.algo.formation_math import horizontal_track_basis, horizontal_track_vector_to_enu
from src.algorithm.units.algo.pos_calc.base import PosCalcBase, PosCalcInitS, PosCalcInputS, PosCalcOutputS

_ALONG_SLOT_SPEED_GAIN = 0.08
_MAX_ALONG_SLOT_SPEED_CORRECTION = 2.0


@dataclass
class SlotGeometryInitS(PosCalcInitS):
    """槽位几何初始化参数。注意：formPat 和 formPos 需要按队形行一一对应。"""

    selfId: str = ""
    formPat: list[FormPatE] = field(default_factory=list)
    formPos: list[list[FormPosS]] = field(default_factory=list)


@dataclass
class SlotGeometryInputS(PosCalcInputS):
    """槽位几何输入端口。注意：leaderState 和 cmd 都需要在僚机每拍计算前写入。"""

    leaderState: MotionProfS | None = None
    cmd: FormSnapshotS | None = None


class SlotGeometry(PosCalcBase):
    """僚机槽位目标计算器。注意：槽位按长机水平航迹旋转，前向误差只转为速度修正。"""

    def __init__(self) -> None:
        """初始化 SlotGeometry 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        self._self_id = ""
        self._form_pat: list[FormPatE] = []
        self._form_pos: list[list[FormPosS]] = []

    def init(self, cfg: SlotGeometryInitS) -> None:
        """按配置初始化 SlotGeometry。注意：调用方需先准备好必要依赖和输入数据。"""
        self._self_id = cfg.selfId
        self._form_pat = list(cfg.formPat)
        self._form_pos = [list(row) for row in cfg.formPos]

    def step(self, u: SlotGeometryInputS, y: PosCalcOutputS) -> None:
        """推进 SlotGeometry 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        if u.leaderState is None or u.cmd is None or y.selfCmd is None:
            raise ValueError("SlotGeometry ports must be bound")
        pattern = FormPatE(u.cmd.pattern)
        try:
            row_index = self._form_pat.index(pattern)
        except ValueError as exc:
            raise ValueError(f"unknown formation pattern: {pattern!r}") from exc
        if row_index >= len(self._form_pos):
            raise ValueError("formPos does not contain row for pattern")
        slot = next((item for item in self._form_pos[row_index] if item.id == self._self_id), None)
        if slot is None:
            raise ValueError(f"missing slot for selfId: {self._self_id}")

        track = _horizontal_track_or_none(u.leaderState)
        if track is None:
            # 长机水平航迹未定义时按东向航迹兜底，避免起步/悬停首拍崩溃。
            slot_east, slot_north = slot.x, -slot.z
        else:
            # FormPosS 使用 x 前向、z 右侧向，与水平航迹转换函数的轴序一致。
            slot_east, slot_north = horizontal_track_vector_to_enu((slot.x, slot.z), track)
        y.selfCmd.pos.east = u.leaderState.pos.east + slot_east
        y.selfCmd.pos.north = u.leaderState.pos.north + slot_north
        y.selfCmd.pos.h = u.leaderState.pos.h + slot.y
        copy_velocity(u.leaderState.v, y.selfCmd.v)
        # 槽位随长机刚性旋转，僚机航迹偏航角速率即长机的(刚体绕同一瞬心，各点航向角速率相同)。
        y.selfCmd.v.dVPsi = u.leaderState.v.dVPsi
        if u.selfState is None or track is None:
            return None
        track_x, track_y = track
        # 槽位速度前馈：槽位随长机刚性旋转，其真实速度 v_S = (V + b·ω)·t̂ + (a·ω)·n̂，
        # 其中 a=slot.x(前向)、b=slot.z(右向)、ω=长机偏航角速率、n̂=左单位向量。
        # 沿航迹分量按 b·ω 增减(对某一转向，外侧半径大加速、内侧减速；某槽位是外/内侧由 b 与 ω 符号共定)；a·ω 补后方槽位转弯时的横扫。
        # 直接覆盖 copy_velocity 写入的长机速度——ω=0(直线)时二者相等，行为不变。
        omega = u.leaderState.v.dVPsi
        v_along = u.leaderState.v.vd + slot.z * omega
        v_swing = slot.x * omega
        left_x, left_y = -track_y, track_x
        y.selfCmd.v.vEast = v_along * track_x + v_swing * left_x
        y.selfCmd.v.vNorth = v_along * track_y + v_swing * left_y
        err_x = y.selfCmd.pos.east - u.selfState.pos.east
        err_y = y.selfCmd.pos.north - u.selfState.pos.north
        # 前向位置不由 PidCompose 控制，剩余待飞距误差只做小幅沿航迹 trim(前馈担主项，不触限)。
        along_error = err_x * track_x + err_y * track_y
        speed_correction = max(
            -_MAX_ALONG_SLOT_SPEED_CORRECTION,
            min(_MAX_ALONG_SLOT_SPEED_CORRECTION, _ALONG_SLOT_SPEED_GAIN * along_error),
        )
        y.selfCmd.v.vEast += speed_correction * track_x
        y.selfCmd.v.vNorth += speed_correction * track_y
        y.selfCmd.v.vd = math.hypot(y.selfCmd.v.vEast, y.selfCmd.v.vNorth)
        y.selfCmd.v.vPsi = math.atan2(y.selfCmd.v.vNorth, y.selfCmd.v.vEast)
        return None

    def reset(self) -> None:
        """复位 SlotGeometry 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        return None


def _horizontal_track_or_none(state: MotionProfS) -> tuple[float, float] | None:
    """计算可用的水平航迹基向量。注意：航段退化时返回空值并由调用方兜底。"""
    try:
        return horizontal_track_basis(state)
    except ValueError:
        return None
