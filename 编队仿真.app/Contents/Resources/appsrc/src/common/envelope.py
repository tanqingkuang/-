"""Generic communication message envelope."""

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class MessageEnvelope:
    """Transport-level message wrapper independent of algorithm payloads."""

    topic: str
    source: str
    target: str
    timestamp: float
    payload: Any

