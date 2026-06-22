"""Base API for formation task orchestration."""

from __future__ import annotations

from dataclasses import dataclass

from src.algorithm.context.leaf_types import FormSnapshotS, RemoteCmdS


@dataclass
class FormationTaskInitS:
    pass


@dataclass
class FormationTaskInputS:
    remote: RemoteCmdS | None = None
    cmd: FormSnapshotS | None = None


@dataclass
class FormationTaskOutputS:
    cmd: FormSnapshotS | None = None


class FormationTaskBase:
    def init(self, cfg: FormationTaskInitS) -> None:
        raise NotImplementedError

    def step(self, u: FormationTaskInputS, y: FormationTaskOutputS) -> None:
        raise NotImplementedError

    def reset(self) -> None:
        raise NotImplementedError
