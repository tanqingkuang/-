"""PID-composed position tracking."""

from __future__ import annotations

from dataclasses import dataclass

from src.algorithm.units.algo.ctrl.base import CtrlInitS
from src.algorithm.units.algo.ctrl.pid import Pid
from src.algorithm.units.algo.formation_math import enu_to_track, track_to_enu
from src.algorithm.units.algo.pos_track.base import PosTrackBase, PosTrackInitS, PosTrackInputS, PosTrackOutputS


@dataclass
class PidComposeInitS(PosTrackInitS):
    vMin: float = 3.0
    gainForward: CtrlInitS | None = None
    gainLateral: CtrlInitS | None = None
    gainVertical: CtrlInitS | None = None


class PidCompose(PosTrackBase):
    def __init__(self) -> None:
        self._v_min = 3.0
        self._forward = Pid()
        self._lateral = Pid()
        self._vertical = Pid()

    def init(self, cfg: PidComposeInitS) -> None:
        self._v_min = cfg.vMin
        self._forward.init(cfg.gainForward or CtrlInitS())
        self._lateral.init(cfg.gainLateral or CtrlInitS())
        self._vertical.init(cfg.gainVertical or CtrlInitS())

    def step(self, u: PosTrackInputS, y: PosTrackOutputS) -> None:
        if u.selfCmd is None or u.selfState is None or y.accCmd is None:
            raise ValueError("PidCompose ports must be bound")
        if u.selfState.vd.vd < self._v_min:
            raise ValueError(f"ground speed below vMin: {u.selfState.vd.vd} < {self._v_min}")

        pos_err_enu = (
            u.selfCmd.pos.east - u.selfState.pos.east,
            u.selfCmd.pos.north - u.selfState.pos.north,
            u.selfCmd.pos.h - u.selfState.pos.h,
        )
        vel_err_enu = (
            u.selfCmd.vd.vEast - u.selfState.vd.vEast,
            u.selfCmd.vd.vNorth - u.selfState.vd.vNorth,
            u.selfCmd.vd.vUp - u.selfState.vd.vUp,
        )
        pos_err = enu_to_track(pos_err_enu, u.selfState)
        vel_err = enu_to_track(vel_err_enu, u.selfState)

        acc_track = (
            self._forward.step(0.0, vel_err[0]),
            self._lateral.step(pos_err[1], vel_err[1]),
            self._vertical.step(pos_err[2], vel_err[2]),
        )
        acc_enu = track_to_enu(acc_track, u.selfState)
        y.accCmd.accEast = acc_enu[0]
        y.accCmd.accNorth = acc_enu[1]
        y.accCmd.accUp = acc_enu[2]

    def reset(self) -> None:
        self._forward.reset()
        self._lateral.reset()
        self._vertical.reset()
