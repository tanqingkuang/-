"""Base API for target-position calculation."""

from __future__ import annotations

from dataclasses import dataclass

from src.algorithm.context.leaf_types import MotionProfS


@dataclass
class PosCalcInitS:
    pass


@dataclass
class PosCalcInputS:
    selfState: MotionProfS | None = None


@dataclass
class PosCalcOutputS:
    selfCmd: MotionProfS | None = None


class PosCalcBase:
    def init(self, cfg: PosCalcInitS) -> None:
        raise NotImplementedError

    def step(self, u: PosCalcInputS, y: PosCalcOutputS) -> None:
        raise NotImplementedError

    def reset(self) -> None:
        raise NotImplementedError
