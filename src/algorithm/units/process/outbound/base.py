"""Base API for outbound message processing."""

from __future__ import annotations

from dataclasses import dataclass, field

from src.algorithm.context.leaf_types import FormSnapshotS, MotionProfS, NetWorkS
from src.common.envelope import MessageEnvelope


@dataclass
class OutboundInitS:
    selfId: str = ""
    netWork: list[NetWorkS] = field(default_factory=list)


@dataclass
class OutboundInputS:
    cmd: FormSnapshotS | None = None
    selfState: MotionProfS | None = None


@dataclass
class OutboundOutputS:
    outbox: list[MessageEnvelope] = field(default_factory=list)


class OutboundBase:
    def init(self, cfg: OutboundInitS) -> None:
        raise NotImplementedError

    def step(self, u: OutboundInputS, y: OutboundOutputS) -> None:
        raise NotImplementedError

    def reset(self) -> None:
        raise NotImplementedError
