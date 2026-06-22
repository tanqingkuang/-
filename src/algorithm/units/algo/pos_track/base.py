"""Base API for position tracking."""

from __future__ import annotations

from dataclasses import dataclass

from src.algorithm.context.leaf_types import AccInEarthS, MotionProfS


@dataclass
class PosTrackInitS:
    pass


@dataclass
class PosTrackInputS:
    selfCmd: MotionProfS | None = None
    selfState: MotionProfS | None = None


@dataclass
class PosTrackOutputS:
    accCmd: AccInEarthS | None = None


class PosTrackBase:
    def init(self, cfg: PosTrackInitS) -> None:
        raise NotImplementedError

    def step(self, u: PosTrackInputS, y: PosTrackOutputS) -> None:
        raise NotImplementedError

    def reset(self) -> None:
        raise NotImplementedError
