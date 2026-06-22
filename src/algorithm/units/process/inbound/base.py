"""Base API for inbound message processing."""

from __future__ import annotations

from dataclasses import dataclass, field

from src.algorithm.context.leaf_types import FormSnapshotS, MotionProfS
from src.common.envelope import MessageEnvelope


@dataclass
class InboundInitS:
    pass


@dataclass
class InboundInputS:
    inbox: list[MessageEnvelope] = field(default_factory=list)


@dataclass
class InboundOutputS:
    leaderState: MotionProfS | None = None
    cmd: FormSnapshotS | None = None


class InboundBase:
    def init(self, cfg: InboundInitS) -> None:
        raise NotImplementedError

    def step(self, u: InboundInputS, y: InboundOutputS) -> None:
        raise NotImplementedError

    def reset(self) -> None:
        raise NotImplementedError
