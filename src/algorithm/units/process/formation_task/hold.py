"""Hold-mode placeholder task orchestration."""

from __future__ import annotations

from src.algorithm.context.leaf_types import FormPatE, FormStageE
from src.algorithm.units.process.formation_task.base import FormationTaskBase, FormationTaskInitS, FormationTaskInputS, FormationTaskOutputS


class Hold(FormationTaskBase):
    def init(self, cfg: FormationTaskInitS) -> None:
        del cfg

    def step(self, u: FormationTaskInputS, y: FormationTaskOutputS) -> None:
        del u
        if y.cmd is None:
            raise ValueError("Hold output port must be bound")
        y.cmd.stage = FormStageE.HOLD
        y.cmd.pattern = FormPatE.TRIANGLE

    def reset(self) -> None:
        return None
