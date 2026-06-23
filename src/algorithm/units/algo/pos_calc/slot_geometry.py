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
from src.algorithm.units.algo.formation_math import horizontal_track_basis, horizontal_track_vector_to_enu
from src.algorithm.units.algo.pos_calc.base import PosCalcBase, PosCalcInitS, PosCalcInputS, PosCalcOutputS

_ALONG_SLOT_SPEED_GAIN = 0.08
_MAX_ALONG_SLOT_SPEED_CORRECTION = 2.0


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

        track = _horizontal_track_or_none(u.leaderState)
        if track is None:
            slot_east, slot_north = slot.x, slot.y
        else:
            slot_east, slot_north = horizontal_track_vector_to_enu((slot.x, slot.y), track)
        y.selfCmd.pos.east = u.leaderState.pos.east + slot_east
        y.selfCmd.pos.north = u.leaderState.pos.north + slot_north
        y.selfCmd.pos.h = u.leaderState.pos.h + slot.z
        copy_velocity(u.leaderState.vd, y.selfCmd.vd)
        if u.selfState is None or track is None:
            return None
        track_x, track_y = track
        err_x = y.selfCmd.pos.east - u.selfState.pos.east
        err_y = y.selfCmd.pos.north - u.selfState.pos.north
        along_error = err_x * track_x + err_y * track_y
        speed_correction = max(
            -_MAX_ALONG_SLOT_SPEED_CORRECTION,
            min(_MAX_ALONG_SLOT_SPEED_CORRECTION, _ALONG_SLOT_SPEED_GAIN * along_error),
        )
        y.selfCmd.vd.vEast += speed_correction * track_x
        y.selfCmd.vd.vNorth += speed_correction * track_y
        y.selfCmd.vd.vd = math.hypot(y.selfCmd.vd.vEast, y.selfCmd.vd.vNorth)
        y.selfCmd.vd.vPsi = math.atan2(y.selfCmd.vd.vNorth, y.selfCmd.vd.vEast)
        return None

    def reset(self) -> None:
        return None


def _horizontal_track_or_none(state: MotionProfS) -> tuple[float, float] | None:
    try:
        return horizontal_track_basis(state)
    except ValueError:
        return None
