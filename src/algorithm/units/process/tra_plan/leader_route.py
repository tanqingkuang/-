"""Route planner that selects the current segment from an injected route."""

from __future__ import annotations

from dataclasses import dataclass

from src.algorithm.context.leaf_types import MotionProfS, PosInEarthS, RouteS, WayLineS, WayPointS, copy_wayline
from src.algorithm.units.process.tra_plan.base import TraPlanBase, TraPlanInitS, TraPlanInputS, TraPlanOutputS


@dataclass
class LeaderRouteInitS(TraPlanInitS):
    route: RouteS | None = None


class LeaderRoute(TraPlanBase):
    def __init__(self) -> None:
        self._route = _default_route()
        self._current_index = 0

    def init(self, cfg: TraPlanInitS | None) -> None:
        if isinstance(cfg, LeaderRouteInitS) and cfg.route is not None and cfg.route.lines:
            self._route = _clone_route(cfg.route)
        else:
            self._route = _default_route()
        self._current_index = 0

    def step(self, u: TraPlanInputS, y: TraPlanOutputS) -> None:
        if y.wayLine is None:
            raise ValueError("LeaderRoute output port must be bound")
        line = self._select_current_line(u.selfState)
        copy_wayline(line, y.wayLine)

    def reset(self) -> None:
        self._current_index = 0

    def _select_current_line(self, self_state: MotionProfS | None) -> WayLineS:
        lines = self._route.lines
        if not lines:
            raise ValueError("route must contain at least one wayLine")
        if self_state is None:
            return lines[self._current_index]
        while self._current_index < len(lines) - 1 and _line_progress(
            lines[self._current_index],
            self_state,
        ) >= 1.0:
            self._current_index += 1
        return lines[self._current_index]


def _default_route() -> RouteS:
    return RouteS(lines=[_default_line()])


def _default_line() -> WayLineS:
    return WayLineS(
        idx=0,
        start=WayPointS(idx=0, pos=PosInEarthS(0.0, 0.0, 1000.0)),
        end=WayPointS(idx=1, pos=PosInEarthS(1000.0, 0.0, 1000.0)),
        vdCmd=8.0,
        radius=0.0,
    )


def _clone_wayline(src: WayLineS) -> WayLineS:
    dst = WayLineS()
    copy_wayline(src, dst)
    return dst


def _clone_route(src: RouteS) -> RouteS:
    return RouteS(lines=[_clone_wayline(line) for line in src.lines])


def _line_progress(line: WayLineS, self_state: MotionProfS) -> float:
    start = line.start.pos
    end = line.end.pos
    dx = end.east - start.east
    dy = end.north - start.north
    dz = end.h - start.h
    length2 = dx * dx + dy * dy + dz * dz
    if length2 <= 0.0:
        raise ValueError("wayLine start and end must be different")
    relx = self_state.pos.east - start.east
    rely = self_state.pos.north - start.north
    relz = self_state.pos.h - start.h
    return (relx * dx + rely * dy + relz * dz) / length2
