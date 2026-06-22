"""No-op trajectory planner for follower placeholder flow."""

from __future__ import annotations

from src.algorithm.units.process.tra_plan.base import TraPlanBase, TraPlanInitS, TraPlanInputS, TraPlanOutputS


class Noop(TraPlanBase):
    def init(self, cfg: TraPlanInitS) -> None:
        del cfg

    def step(self, u: TraPlanInputS, y: TraPlanOutputS) -> None:
        del u, y

    def reset(self) -> None:
        return None
