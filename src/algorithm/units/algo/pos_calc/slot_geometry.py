"""Slot geometry target calculation for follower entities."""

from __future__ import annotations

from dataclasses import dataclass, field

from src.algorithm.context.leaf_types import (
    FormPatE,
    FormPosS,
    FormSnapshotS,
    MotionProfS,
    copy_velocity,
)
from src.algorithm.units.algo.pos_calc.base import PosCalcBase, PosCalcInitS, PosCalcInputS, PosCalcOutputS


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

        y.selfCmd.pos.east = u.leaderState.pos.east + slot.x
        y.selfCmd.pos.north = u.leaderState.pos.north + slot.y
        y.selfCmd.pos.h = u.leaderState.pos.h + slot.z
        copy_velocity(u.leaderState.vd, y.selfCmd.vd)

    def reset(self) -> None:
        return None
