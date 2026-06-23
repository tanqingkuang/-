"""Slot geometry target calculation for follower entities."""

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
from src.algorithm.units.algo.pos_calc.base import PosCalcBase, PosCalcInitS, PosCalcInputS, PosCalcOutputS

_ALONG_SLOT_SPEED_GAIN = 0.08
_MAX_ALONG_SLOT_SPEED_CORRECTION = 2.0
_LATERAL_SLOT_SPEED_GAIN = 0.08
_MAX_LATERAL_SLOT_SPEED_CORRECTION = 2.0


@dataclass
class SlotGeometryInitS(PosCalcInitS):
    selfId: str = ""
    formPat: list[FormPatE] = field(default_factory=list)
    formPos: list[list[FormPosS]] = field(default_factory=list)


@dataclass
class SlotGeometryInputS(PosCalcInputS):
    leaderState: MotionProfS | None = None
    cmd: FormSnapshotS | None = None


class SlotGeometry(PosCalcBase):
    def __init__(self) -> None:
        self._self_id = ""
        self._form_pat: list[FormPatE] = []
        self._form_pos: list[list[FormPosS]] = []

    def init(self, cfg: SlotGeometryInitS) -> None:
        self._self_id = cfg.selfId
        self._form_pat = list(cfg.formPat)
        self._form_pos = [list(row) for row in cfg.formPos]

    def step(self, u: SlotGeometryInputS, y: PosCalcOutputS) -> None:
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

        slot_east, slot_north = _horizontal_slot_to_enu(slot.x, slot.y, u.leaderState)
        y.selfCmd.pos.east = u.leaderState.pos.east + slot_east
        y.selfCmd.pos.north = u.leaderState.pos.north + slot_north
        y.selfCmd.pos.h = u.leaderState.pos.h + slot.z
        copy_velocity(u.leaderState.vd, y.selfCmd.vd)
        if u.selfState is None:
            return None

        ground_speed = math.hypot(u.leaderState.vd.vEast, u.leaderState.vd.vNorth)
        if ground_speed <= 1e-9:
            return None
        track_x = u.leaderState.vd.vEast / ground_speed
        track_y = u.leaderState.vd.vNorth / ground_speed
        err_x = y.selfCmd.pos.east - u.selfState.pos.east
        err_y = y.selfCmd.pos.north - u.selfState.pos.north
        along_error = err_x * track_x + err_y * track_y
        lateral_error = -err_x * track_y + err_y * track_x
        speed_correction = max(
            -_MAX_ALONG_SLOT_SPEED_CORRECTION,
            min(_MAX_ALONG_SLOT_SPEED_CORRECTION, _ALONG_SLOT_SPEED_GAIN * along_error),
        )
        lateral_speed_correction = max(
            -_MAX_LATERAL_SLOT_SPEED_CORRECTION,
            min(_MAX_LATERAL_SLOT_SPEED_CORRECTION, _LATERAL_SLOT_SPEED_GAIN * lateral_error),
        )
        y.selfCmd.vd.vEast += speed_correction * track_x
        y.selfCmd.vd.vNorth += speed_correction * track_y
        y.selfCmd.vd.vEast += -lateral_speed_correction * track_y
        y.selfCmd.vd.vNorth += lateral_speed_correction * track_x
        y.selfCmd.vd.vd = math.hypot(y.selfCmd.vd.vEast, y.selfCmd.vd.vNorth)
        y.selfCmd.vd.vPsi = math.atan2(y.selfCmd.vd.vNorth, y.selfCmd.vd.vEast)
        return None

    def reset(self) -> None:
        return None


def _horizontal_slot_to_enu(forward_m: float, lateral_m: float, leader_state: MotionProfS) -> tuple[float, float]:
    ground_speed = math.hypot(leader_state.vd.vEast, leader_state.vd.vNorth)
    if ground_speed <= 1e-9:
        raise ValueError("slot frame requires non-zero leader horizontal velocity")
    track_x = leader_state.vd.vEast / ground_speed
    track_y = leader_state.vd.vNorth / ground_speed
    return (
        forward_m * track_x - lateral_m * track_y,
        forward_m * track_y + lateral_m * track_x,
    )
