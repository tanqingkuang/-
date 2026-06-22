"""Base API for atomic control laws."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class CtrlInitS:
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    dt: float = 0.0
    iMax: float = 0.0
    outMax: float = 0.0


class CtrlBase:
    def init(self, cfg: CtrlInitS) -> None:
        raise NotImplementedError

    def step(self, posErr: float, velErr: float) -> float:
        raise NotImplementedError

    def reset(self) -> None:
        raise NotImplementedError
