"""Entity boundary types."""

from __future__ import annotations

from dataclasses import dataclass, field

from src.algorithm.context.leaf_types import AccInEarthS, FormCommInitS, FormSelfInitS, MotionProfS, RemoteCmdS
from src.common.envelope import MessageEnvelope


@dataclass
class EntityInitS:
    selfInit: FormSelfInitS = field(default_factory=FormSelfInitS)
    commInit: FormCommInitS = field(default_factory=FormCommInitS)


@dataclass
class EntityInputS:
    selfState: MotionProfS | None = None
    inbox: list[MessageEnvelope] = field(default_factory=list)
    remote: RemoteCmdS | None = None


@dataclass
class EntityOutputS:
    selfAccCmd: AccInEarthS | None = None
    outbox: list[MessageEnvelope] = field(default_factory=list)
