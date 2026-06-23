"""面向 C 风格结构的编队算法叶类型。注意：字段尽量保持简单可序列化。"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum


class FormStageE(IntEnum):
    """编队指令和节点状态共用的阶段枚举。注意：新增阶段需同步控制器回报。"""

    NONE = 0
    RALLY = 1
    HOLD = 2
    RECONFIG = 3


class FormPatE(IntEnum):
    """编队队形枚举。注意：枚举值需与配置中的队形名称兼容。"""

    NONE = 0
    TRIANGLE = 1


class CommDirE(IntEnum):
    """通信方向枚举。注意：方向含义需与通信链路配置一致。"""

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
    """复制位置对象，避免调用方持有原始可变引用。注意：新增坐标字段时需同步补齐。"""
    dst.east = src.east
    dst.north = src.north
    dst.h = src.h


def copy_velocity(src: VdInEarthS, dst: VdInEarthS) -> None:
    """复制速度对象，避免速度状态被外部误改。注意：单位保持为米每秒。"""
    dst.vEast = src.vEast
    dst.vNorth = src.vNorth
    dst.vUp = src.vUp
    dst.vTheta = src.vTheta
    dst.vPsi = src.vPsi
    dst.vd = src.vd


def copy_motion(src: MotionProfS, dst: MotionProfS) -> None:
    """复制运动状态对象，包含位置、速度和姿态信息。注意：嵌套对象需要逐层复制。"""
    copy_position(src.pos, dst.pos)
    copy_velocity(src.vd, dst.vd)


def copy_wayline(src: WayLineS, dst: WayLineS) -> None:
    """复制单段航线数据，供算法模块安全读写。注意：起终点对象不能复用原引用。"""
    dst.idx = src.idx
    dst.start.idx = src.start.idx
    copy_position(src.start.pos, dst.start.pos)
    dst.end.idx = src.end.idx
    copy_position(src.end.pos, dst.end.pos)
    dst.vdCmd = src.vdCmd
    dst.radius = src.radius


def copy_snapshot(src: FormSnapshotS, dst: FormSnapshotS) -> None:
    """复制上下文快照，隔离算法输入和显示输出。注意：新增快照字段时需同步复制。"""
    dst.stage = FormStageE(src.stage)
    dst.pattern = FormPatE(src.pattern)
    dst.step = int(src.step)
