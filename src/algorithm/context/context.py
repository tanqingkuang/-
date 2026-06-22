"""Formation algorithm blackboard."""

from __future__ import annotations

from dataclasses import dataclass, field

from src.algorithm.context.leaf_types import (
    AccInEarthS,
    FormSnapshotS,
    MotionProfS,
    WayLineS,
)


@dataclass
class FormContextS:
    """Cross-frame formation state owned by one entity."""

    cmd: FormSnapshotS = field(default_factory=FormSnapshotS)
    state: list[FormSnapshotS] = field(default_factory=list)
    wayLine: WayLineS = field(default_factory=WayLineS)
    leaderState: MotionProfS = field(default_factory=MotionProfS)
    selfCmd: MotionProfS = field(default_factory=MotionProfS)
    selfState: MotionProfS = field(default_factory=MotionProfS)
    selfAccCmd: AccInEarthS = field(default_factory=AccInEarthS)
