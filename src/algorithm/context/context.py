"""Formation algorithm blackboard."""

from __future__ import annotations

from dataclasses import dataclass, field

from src.algorithm.context.leaf_types import (
    AccInEarthS,
    FormSnapshotS,
    MotionProfS,
    WayLineS,
    copy_motion,
    copy_snapshot,
    copy_wayline,
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


def reset_context(dst: FormContextS) -> None:
    fresh = FormContextS()
    copy_snapshot(fresh.cmd, dst.cmd)
    dst.state.clear()
    copy_wayline(fresh.wayLine, dst.wayLine)
    copy_motion(fresh.leaderState, dst.leaderState)
    copy_motion(fresh.selfCmd, dst.selfCmd)
    copy_motion(fresh.selfState, dst.selfState)
    dst.selfAccCmd.accEast = fresh.selfAccCmd.accEast
    dst.selfAccCmd.accNorth = fresh.selfAccCmd.accNorth
    dst.selfAccCmd.accUp = fresh.selfAccCmd.accUp
