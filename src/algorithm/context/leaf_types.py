"""C-friendly leaf types for formation algorithms."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum


class FormStageE(IntEnum):
    """Formation stage used by both commands and per-node state."""

    NONE = 0
    RALLY = 1
    HOLD = 2
    RECONFIG = 3


class FormPatE(IntEnum):
    """Formation pattern."""

    NONE = 0
    TRIANGLE = 1


class CommDirE(IntEnum):
    """Communication direction."""

    DUPLEX = 0
    SIMPLEX = 1


@dataclass
class FormSelfInitS:
    id: str = ""


@dataclass
class NetWorkS:
    startId: str = ""
    endId: str = ""
    dir: CommDirE = CommDirE.DUPLEX


@dataclass
class FormPosS:
    id: str = ""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class FormCommInitS:
    netWork: list[NetWorkS] = field(default_factory=list)
    formPat: list[FormPatE] = field(default_factory=list)
    formPos: list[list[FormPosS]] = field(default_factory=list)


@dataclass
class PosInEarthS:
    east: float = 0.0
    north: float = 0.0
    h: float = 0.0


@dataclass
class VdInEarthS:
    vEast: float = 0.0
    vNorth: float = 0.0
    vUp: float = 0.0
    vTheta: float = 0.0
    vPsi: float = 0.0
    vd: float = 0.0


@dataclass
class AccInEarthS:
    accEast: float = 0.0
    accNorth: float = 0.0
    accUp: float = 0.0


@dataclass
class MotionProfS:
    pos: PosInEarthS = field(default_factory=PosInEarthS)
    vd: VdInEarthS = field(default_factory=VdInEarthS)


@dataclass
class WayPointS:
    idx: int = 0
    pos: PosInEarthS = field(default_factory=PosInEarthS)


@dataclass
class WayLineS:
    idx: int = 0
    start: WayPointS = field(default_factory=WayPointS)
    end: WayPointS = field(default_factory=WayPointS)
    vdCmd: float = 0.0
    radius: float = 0.0


@dataclass
class RouteS:
    lines: list[WayLineS] = field(default_factory=list)


@dataclass
class FormSnapshotS:
    stage: FormStageE = FormStageE.NONE
    pattern: FormPatE = FormPatE.NONE
    step: int = 0


@dataclass
class RemoteCmdS:
    stage: FormStageE = FormStageE.NONE


def copy_position(src: PosInEarthS, dst: PosInEarthS) -> None:
    dst.east = src.east
    dst.north = src.north
    dst.h = src.h


def copy_velocity(src: VdInEarthS, dst: VdInEarthS) -> None:
    dst.vEast = src.vEast
    dst.vNorth = src.vNorth
    dst.vUp = src.vUp
    dst.vTheta = src.vTheta
    dst.vPsi = src.vPsi
    dst.vd = src.vd


def copy_motion(src: MotionProfS, dst: MotionProfS) -> None:
    copy_position(src.pos, dst.pos)
    copy_velocity(src.vd, dst.vd)


def copy_wayline(src: WayLineS, dst: WayLineS) -> None:
    dst.idx = src.idx
    dst.start.idx = src.start.idx
    copy_position(src.start.pos, dst.start.pos)
    dst.end.idx = src.end.idx
    copy_position(src.end.pos, dst.end.pos)
    dst.vdCmd = src.vdCmd
    dst.radius = src.radius


def copy_snapshot(src: FormSnapshotS, dst: FormSnapshotS) -> None:
    dst.stage = FormStageE(src.stage)
    dst.pattern = FormPatE(src.pattern)
    dst.step = int(src.step)
