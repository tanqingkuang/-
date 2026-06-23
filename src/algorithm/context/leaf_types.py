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
    """实体自身初始化标识。注意：id 用于在通信拓扑中唯一定位本机。"""

    id: str = ""  # 本机唯一编号，与通信链路 startId/endId 对应


@dataclass
class NetWorkS:
    """单条通信链路描述。注意：方向决定消息能否反向传播。"""

    startId: str = ""  # 链路起点节点 id
    endId: str = ""  # 链路终点节点 id
    dir: CommDirE = CommDirE.DUPLEX  # 通信方向：双工可双向，单工仅 start->end


@dataclass
class FormPosS:
    """队形中单个槽位的相对坐标。注意：坐标为相对长机的机体/编队系偏移。"""

    id: str = ""  # 占据该槽位的机号
    x: float = 0.0  # 纵向偏移，单位米
    y: float = 0.0  # 横向偏移，单位米
    z: float = 0.0  # 垂向偏移，单位米


@dataclass
class FormCommInitS:
    """编队通信与队形初始化配置。注意：三者按队形索引对齐。"""

    netWork: list[NetWorkS] = field(default_factory=list)  # 通信拓扑链路集合
    formPat: list[FormPatE] = field(default_factory=list)  # 各阶段/步可选的队形枚举列表
    formPos: list[list[FormPosS]] = field(default_factory=list)  # 与 formPat 对应的各队形槽位坐标表


@dataclass
class PosInEarthS:
    """地理系位置。注意：东北天右手系，单位米。"""

    east: float = 0.0  # 东向坐标，单位米
    north: float = 0.0  # 北向坐标，单位米
    h: float = 0.0  # 高度，单位米


@dataclass
class VdInEarthS:
    """地理系速度与姿态角分量。注意：线速度单位米每秒，角度单位弧度。"""

    vEast: float = 0.0  # 东向速度，米每秒
    vNorth: float = 0.0  # 北向速度，米每秒
    vUp: float = 0.0  # 天向速度，米每秒
    vTheta: float = 0.0  # 俯仰角，弧度
    vPsi: float = 0.0  # 航向角，弧度
    vd: float = 0.0  # 地速标量，米每秒


@dataclass
class AccInEarthS:
    """地理系加速度指令。注意：作为位置跟踪环的最终输出。"""

    accEast: float = 0.0  # 东向加速度，米每二次方秒
    accNorth: float = 0.0  # 北向加速度，米每二次方秒
    accUp: float = 0.0  # 天向加速度，米每二次方秒


@dataclass
class MotionProfS:
    """完整运动状态剖面，聚合位置与速度。注意：算法各环节以此为统一状态载体。"""

    pos: PosInEarthS = field(default_factory=PosInEarthS)  # 地理系位置
    v: VdInEarthS = field(default_factory=VdInEarthS)  # 地理系速度与姿态


@dataclass
class WayPointS:
    """单个航路点。注意：idx 用于航段内首尾点的稳定标识。"""

    idx: int = 0  # 航路点序号
    pos: PosInEarthS = field(default_factory=PosInEarthS)  # 航路点地理系位置


@dataclass
class WayLineS:
    """单段直线航段。注意：算法按 start->end 跟踪并据 radius 决定转弯提前量。"""

    idx: int = 0  # 航段序号
    start: WayPointS = field(default_factory=WayPointS)  # 航段起点
    end: WayPointS = field(default_factory=WayPointS)  # 航段终点
    vdCmd: float = 0.0  # 该航段地速指令，米每秒
    radius: float = 0.0  # 转弯半径，米；0 表示直线无转弯


@dataclass
class RouteS:
    """整条航线，由若干有序航段拼成。注意：航段顺序即飞行顺序。"""

    lines: list[WayLineS] = field(default_factory=list)  # 有序航段列表


@dataclass
class FormSnapshotS:
    """编队指令/状态快照。注意：在算法各单元间传递当前阶段与队形。"""

    stage: FormStageE = FormStageE.NONE  # 当前编队阶段
    pattern: FormPatE = FormPatE.NONE  # 当前队形
    step: int = 0  # 阶段内步进计数


@dataclass
class RemoteCmdS:
    """外部下发的遥控指令。注意：当前仅承载目标阶段，由任务单元解释。"""

    stage: FormStageE = FormStageE.NONE  # 期望进入的编队阶段


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
    copy_velocity(src.v, dst.v)


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
