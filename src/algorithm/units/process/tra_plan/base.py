"""Base API for trajectory planning."""

from __future__ import annotations

from dataclasses import dataclass

from src.algorithm.context.leaf_types import FormSnapshotS, MotionProfS, WayLineS


@dataclass
class TraPlanInitS:
    pass


@dataclass
class TraPlanInputS:
    cmd: FormSnapshotS | None = None
    wayLine: WayLineS | None = None
    selfState: MotionProfS | None = None


@dataclass
class TraPlanOutputS:
    wayLine: WayLineS | None = None


class TraPlanBase:
    def init(self, cfg: TraPlanInitS) -> None:
        raise NotImplementedError

    def step(self, u: TraPlanInputS, y: TraPlanOutputS) -> None:
        raise NotImplementedError

    def reset(self) -> None:
        raise NotImplementedError
