"""Leader entity for the leader-following hold scenario."""

from __future__ import annotations

from src.algorithm.context.context import FormContextS
from src.algorithm.context.leaf_types import RemoteCmdS, copy_motion
from src.algorithm.entity.base import EntityBase
from src.algorithm.entity.types import EntityInitS, EntityInputS, EntityOutputS
from src.algorithm.units.algo.ctrl.base import CtrlInitS
from src.algorithm.units.algo.pos_calc.base import PosCalcOutputS
from src.algorithm.units.algo.pos_calc.route_interp import RouteInterp, RouteInterpInputS
from src.algorithm.units.algo.pos_track.base import PosTrackInputS, PosTrackOutputS
from src.algorithm.units.algo.pos_track.pid_compose import PidCompose, PidComposeInitS
from src.algorithm.units.process.formation_task.base import FormationTaskInputS, FormationTaskOutputS
from src.algorithm.units.process.formation_task.hold import Hold
from src.algorithm.units.process.outbound.base import OutboundInputS, OutboundOutputS
from src.algorithm.units.process.outbound.leader_broadcast import LeaderBroadcast, OutboundInitS
from src.algorithm.units.process.tra_plan.base import TraPlanInputS, TraPlanOutputS
from src.algorithm.units.process.tra_plan.leader_route import LeaderRoute


class LeaderEntity(EntityBase):
    def init(self, cfg: EntityInitS) -> None:
        self.cxt = FormContextS()
        self._remote = RemoteCmdS()
        self._outbox = []

        self._task = Hold()
        self._tra_plan = LeaderRoute()
        self._pos_calc = RouteInterp()
        self._pos_track = PidCompose()
        self._outbound = LeaderBroadcast()

        self._task.init(None)
        self._tra_plan.init(None)
        self._pos_calc.init(None)
        self._pos_track.init(_default_tracker_init())
        self._outbound.init(OutboundInitS(cfg.selfInit.id, cfg.commInit.netWork))

        self._task_u = FormationTaskInputS(remote=self._remote, cmd=self.cxt.cmd)
        self._task_y = FormationTaskOutputS(cmd=self.cxt.cmd)
        self._tra_plan_u = TraPlanInputS(cmd=self.cxt.cmd, wayLine=self.cxt.wayLine, selfState=self.cxt.selfState)
        self._tra_plan_y = TraPlanOutputS(wayLine=self.cxt.wayLine)
        self._pos_calc_u = RouteInterpInputS(selfState=self.cxt.selfState, wayLine=self.cxt.wayLine)
        self._pos_calc_y = PosCalcOutputS(selfCmd=self.cxt.selfCmd)
        self._pos_track_u = PosTrackInputS(selfCmd=self.cxt.selfCmd, selfState=self.cxt.selfState)
        self._pos_track_y = PosTrackOutputS(accCmd=self.cxt.selfAccCmd)
        self._outbound_u = OutboundInputS(cmd=self.cxt.cmd, selfState=self.cxt.selfState)
        self._outbound_y = OutboundOutputS(outbox=self._outbox)

    def step(self, u: EntityInputS, y: EntityOutputS) -> None:
        if u.selfState is not None:
            copy_motion(u.selfState, self.cxt.selfState)
        if u.remote is not None:
            self._remote.stage = u.remote.stage
        self._task.step(self._task_u, self._task_y)
        self._tra_plan.step(self._tra_plan_u, self._tra_plan_y)
        self._pos_calc.step(self._pos_calc_u, self._pos_calc_y)
        self._pos_track.step(self._pos_track_u, self._pos_track_y)
        self._outbound.step(self._outbound_u, self._outbound_y)
        if y.selfAccCmd is None:
            y.selfAccCmd = self.cxt.selfAccCmd
        else:
            y.selfAccCmd.accEast = self.cxt.selfAccCmd.accEast
            y.selfAccCmd.accNorth = self.cxt.selfAccCmd.accNorth
            y.selfAccCmd.accUp = self.cxt.selfAccCmd.accUp
        y.outbox.clear()
        y.outbox.extend(self._outbox)

    def reset(self) -> None:
        self._pos_track.reset()

    def close(self) -> None:
        return None


def _default_tracker_init() -> PidComposeInitS:
    gain_forward = CtrlInitS(kp=0.0, ki=0.0, kd=1.0, dt=0.1, outMax=6.0)
    gain_lateral = CtrlInitS(kp=0.2, ki=0.0, kd=0.6, dt=0.1, outMax=6.0)
    gain_vertical = CtrlInitS(kp=0.2, ki=0.0, kd=0.6, dt=0.1, outMax=6.0)
    return PidComposeInitS(3.0, gain_forward, gain_lateral, gain_vertical)
