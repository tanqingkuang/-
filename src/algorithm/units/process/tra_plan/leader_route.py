"""Single-segment route planner for the first leader-following use case."""

from __future__ import annotations

from src.algorithm.context.leaf_types import PosInEarthS, WayLineS, WayPointS
from src.algorithm.units.process.tra_plan.base import TraPlanBase, TraPlanInitS, TraPlanInputS, TraPlanOutputS


class LeaderRoute(TraPlanBase):
    def __init__(self) -> None:
        self._line = _default_line()

    def init(self, cfg: TraPlanInitS) -> None:
        del cfg
        self._line = _default_line()

    def step(self, u: TraPlanInputS, y: TraPlanOutputS) -> None:
        del u
        if y.wayLine is None:
            raise ValueError("LeaderRoute output port must be bound")
        _copy_wayline(self._line, y.wayLine)

    def reset(self) -> None:
        return None


def _default_line() -> WayLineS:
    return WayLineS(
        idx=0,
        start=WayPointS(idx=0, pos=PosInEarthS(0.0, 0.0, 1000.0)),
        end=WayPointS(idx=1, pos=PosInEarthS(1000.0, 0.0, 1000.0)),
        vdCmd=8.0,
        radius=0.0,
    )


def _copy_wayline(src: WayLineS, dst: WayLineS) -> None:
    dst.idx = src.idx
    dst.start.idx = src.start.idx
    dst.start.pos.east = src.start.pos.east
    dst.start.pos.north = src.start.pos.north
    dst.start.pos.h = src.start.pos.h
    dst.end.idx = src.end.idx
    dst.end.pos.east = src.end.pos.east
    dst.end.pos.north = src.end.pos.north
    dst.end.pos.h = src.end.pos.h
    dst.vdCmd = src.vdCmd
    dst.radius = src.radius
