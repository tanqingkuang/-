"""PID atomic control law."""

from __future__ import annotations

from src.algorithm.units.algo.ctrl.base import CtrlBase, CtrlInitS
from src.algorithm.units.algo.formation_math import clamp


class Pid(CtrlBase):
    """Single-axis PID using velocity error as the derivative term."""

    def __init__(self) -> None:
        self._cfg = CtrlInitS()
        self._integral = 0.0

    def init(self, cfg: CtrlInitS) -> None:
        if cfg.dt < 0.0:
            raise ValueError("dt must be >= 0")
        self._cfg = cfg
        self.reset()

    def step(self, posErr: float, velErr: float) -> float:
        self._integral += posErr * self._cfg.dt
        if self._cfg.iMax > 0.0:
            self._integral = clamp(self._integral, -self._cfg.iMax, self._cfg.iMax)
        output = self._cfg.kp * posErr + self._cfg.ki * self._integral + self._cfg.kd * velErr
        if self._cfg.outMax > 0.0:
            output = clamp(output, -self._cfg.outMax, self._cfg.outMax)
        return output

    def reset(self) -> None:
        self._integral = 0.0
