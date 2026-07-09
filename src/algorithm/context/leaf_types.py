"""面向 C 风格结构的编队算法叶类型。注意：字段尽量保持简单可序列化。"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, fields, replace
from enum import IntEnum


class FormStageE(IntEnum):
    """编队指令和节点状态共用的阶段枚举。注意：新增阶段需同步控制器回报。"""

    NONE = 0
    RALLY = 1
    HOLD = 2
    RECONFIG = 3
    STANDBY = 4


class CommDirE(IntEnum):
    """通信方向枚举。注意：方向含义需与通信链路配置一致。"""

    DUPLEX = 0
    SIMPLEX = 1


class RallyPhaseE(IntEnum):
    """集结子阶段枚举，cmd.step 的类型安全替代；值与历史整数协议完全兼容。"""

    JOINING = 0
    CATCHUP = 1
    LOOSE = 2
    COMPRESS = 3


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
    """队形中单个槽位的相对坐标。注意：沿长机航迹 x 前向、y 上向、z 右向。"""

    id: str = ""  # 占据该槽位的机号
    x: float = 0.0  # 沿长机航迹前向偏移，单位米
    y: float = 0.0  # 沿天向偏移，单位米
    z: float = 0.0  # 沿长机右向偏移，单位米


@dataclass
class FormCommInitS:
    """编队通信与队形初始化配置。注意：三者按队形索引对齐。"""

    netWork: list[NetWorkS] = field(default_factory=list)  # 通信拓扑链路集合
    formPat: list[str] = field(default_factory=list)  # 各队形名字（仅供显示；索引=队形序号，与 formPos 逐行对齐）
    formPos: list[list[FormPosS]] = field(default_factory=list)  # 各队形槽位坐标表；cmd.pattern(整型索引)直接取第几行
    initialPattern: int = 0  # 初始队形索引（cmd.pattern 初值），对应 formPos 的行号


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
    dVPsi: float = 0.0  # 航迹偏航角速率(水平航向角变化率)，弧度每秒


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
class PosTrackDiagS:
    """位置跟踪诊断量。注意：仅作为输出快照，不写入算法 Context。"""

    cmd_pos_east_m: float = 0.0  # 位置指令东向分量，单位米
    cmd_pos_north_m: float = 0.0  # 位置指令北向分量，单位米
    cmd_pos_h_m: float = 0.0  # 位置指令高度分量，单位米
    cmd_vel_east_mps: float = 0.0  # 速度指令东向分量，单位米每秒
    cmd_vel_north_mps: float = 0.0  # 速度指令北向分量，单位米每秒
    cmd_vel_up_mps: float = 0.0  # 速度指令天向分量，单位米每秒
    pos_err_east_m: float = 0.0  # 位置误差东向分量，单位米
    pos_err_north_m: float = 0.0  # 位置误差北向分量，单位米
    pos_err_h_m: float = 0.0  # 位置误差高度分量，单位米
    vel_err_east_mps: float = 0.0  # 速度误差东向分量，单位米每秒
    vel_err_north_mps: float = 0.0  # 速度误差北向分量，单位米每秒
    vel_err_up_mps: float = 0.0  # 速度误差天向分量，单位米每秒
    track_pos_err_x_m: float = 0.0  # 航迹系位置误差 x 分量，单位米
    track_pos_err_y_m: float = 0.0  # 航迹系位置误差 y 分量，单位米
    track_pos_err_z_m: float = 0.0  # 航迹系位置误差 z 分量，单位米
    track_vel_err_x_mps: float = 0.0  # 航迹系速度误差 x 分量，单位米每秒
    track_vel_err_y_mps: float = 0.0  # 航迹系速度误差 y 分量，单位米每秒
    track_vel_err_z_mps: float = 0.0  # 航迹系速度误差 z 分量，单位米每秒


@dataclass
class WayPointS:
    """单个航路点，携带该点之后一段的航段属性。注意：turnSign!=0 表示该点起始处为圆弧段。"""

    idx: int = 0  # 航路点序号
    pos: PosInEarthS = field(default_factory=PosInEarthS)  # 航路点地理系位置
    vdCmd: float = 0.0  # 该点之后一段的地速指令，米每秒
    turnSign: float = 0.0  # 该点之后一段的转向：+1 左转/逆时针、-1 右转/顺时针、0 直线
    center: PosInEarthS = field(default_factory=PosInEarthS)  # 圆弧圆心(仅 turnSign!=0 有意义)


@dataclass
class WayLineS:
    """单段航段。注意：start.* 描述本段属性，end.* 描述下一段属性(供前瞻)。"""

    idx: int = 0  # 航段序号
    start: WayPointS = field(default_factory=WayPointS)  # 航段起点；start.turnSign!=0 表示本段为圆弧
    end: WayPointS = field(default_factory=WayPointS)  # 航段终点；end.turnSign/vdCmd 描述下一段


@dataclass
class WayPointInputS:
    """用户输入的原始航点，供长机 init 转换为内部 WayLineS 序列。"""

    idx: int = 0  # 航点编号，从 0 开始
    pos: PosInEarthS = field(default_factory=PosInEarthS)  # 航点位置
    vdCmd: float = 0.0  # 该航点之后一段的地速指令，米每秒
    r: float = 0.0  # 该拐点处的期望圆弧半径(米)；0=不做圆弧；首末点无意义
    turnSign: float = 0.0  # 该点之后一段的转向(已知圆弧时填入)；0 表示直线或待按 r 计算
    center: PosInEarthS = field(default_factory=PosInEarthS)  # 圆弧圆心(turnSign!=0 时有意义)


def to_display_inputs(route: list[WayPointInputS]) -> list[WayPointInputS]:
    """生成显示用航点：去掉转弯信息（交接半径 r），保留航段曲率（turnSign）。

    显示只画"航段几何"——直线航段画直线、曲率航段画曲线；拐点的交接圆弧属于"转弯信息"，
    由飞行时按 r 平滑，不进入显示。配置航线/避障预览/避障采用三处显示统一走此规则。
    注意：浅拷贝，pos/center 仍共享引用，仅供只读渲染使用。
    """
    return [replace(wpi, r=0.0) for wpi in route]


@dataclass
class FormSnapshotS:
    """编队指令/状态快照。注意：在算法各单元间传递当前阶段与队形。"""

    stage: FormStageE = FormStageE.NONE  # 当前编队阶段
    pattern: int = 0  # 当前队形索引（formPos 行号），0 起；语义由配置队形列表顺序决定
    step: int = 0  # 阶段内步进计数


@dataclass
class RemoteCmdS:
    """外部下发的遥控指令。注意：当前仅承载目标阶段，由实体或任务单元解释。"""

    stage: FormStageE = FormStageE.NONE  # 期望进入的编队阶段；集结待命使用 STANDBY


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
    dst.dVPsi = src.dVPsi


def copy_motion(src: MotionProfS, dst: MotionProfS) -> None:
    """复制运动状态对象，包含位置、速度和姿态信息。注意：嵌套对象需要逐层复制。"""
    copy_position(src.pos, dst.pos)
    copy_velocity(src.v, dst.v)


def copy_pos_track_diag(src: PosTrackDiagS, dst: PosTrackDiagS) -> None:
    """复制位置跟踪诊断量。注意：新增字段时通过 dataclass 字段表自动覆盖。"""
    for item in fields(PosTrackDiagS):
        setattr(dst, item.name, getattr(src, item.name))


def copy_wayline(src: WayLineS, dst: WayLineS) -> None:
    """复制单段航线数据，供算法模块安全读写。注意：起终点对象不能复用原引用。"""
    dst.idx = src.idx
    dst.start.idx = src.start.idx
    copy_position(src.start.pos, dst.start.pos)
    dst.start.vdCmd = src.start.vdCmd
    dst.start.turnSign = src.start.turnSign
    copy_position(src.start.center, dst.start.center)
    dst.end.idx = src.end.idx
    copy_position(src.end.pos, dst.end.pos)
    dst.end.vdCmd = src.end.vdCmd
    dst.end.turnSign = src.end.turnSign
    copy_position(src.end.center, dst.end.center)


def copy_snapshot(src: FormSnapshotS, dst: FormSnapshotS) -> None:
    """复制上下文快照，隔离算法输入和显示输出。注意：新增快照字段时需同步复制。"""
    dst.stage = FormStageE(src.stage)
    dst.pattern = int(src.pattern)
    dst.step = int(src.step)


def zero_velocity(v: VdInEarthS) -> None:
    """将速度对象所有分量原地清零。注意：与 copy_velocity 配套，NONE 分支输出零速时调用。"""
    v.vEast = 0.0
    v.vNorth = 0.0
    v.vUp = 0.0
    v.vTheta = 0.0
    v.vPsi = 0.0
    v.vd = 0.0
    v.dVPsi = 0.0


def zero_acceleration(a: AccInEarthS) -> None:
    """将加速度对象所有分量原地清零。注意：NONE 分支防止上一帧残留时调用。"""
    a.accEast = 0.0
    a.accNorth = 0.0
    a.accUp = 0.0


@dataclass
class RallySlotScaleS:
    """集结阶段的槽位偏置缩放因子。注意：需跨拍保留，进 Context；scaleRate 供 SlotGeometry 计算压缩速度前馈。"""

    scale: float = 1.0  # 槽位偏置放大倍数；1.0 为最终队形，>1.0 为松散放大
    scaleRate: float = 0.0  # scale 对时间的导数（1/s）；LOOSE 为 0，COMPRESS 为负值


@dataclass
class FollowerStateS:
    """单架僚机向长机回报的集结状态快照。注意：id 与节点 ID 对应；posErr 为到当前目标的合距离。"""

    id: str = ""  # 节点 ID，与 envelope.source 对应
    pos: PosInEarthS = field(default_factory=PosInEarthS)  # 实际位置
    posErr_m: float = 0.0  # 到当前目标的三维距离，米；CATCHUP 阶段为 dist3d(self, slot)
    headingErr_rad: float = 0.0  # 当前航向与目标航向之差的绝对值，弧度
    arrived: int = 0  # 兼容旧协议；新协议以 rally_state == EXITED 为准
    valid: bool = False  # 本帧数据是否有效（收到最新报文则置 True）
    lastUpdate_s: float = 0.0  # 最近一次收到该机报文的仿真时间戳，秒
    eta_s: float = 0.0  # 预计到达松散点的仿真时刻（秒）；LOITERING/EXITED 时为当前时刻
    rally_state: str = "STANDBY"  # 集结汇合状态：STANDBY / FLYING / LOITERING / EXITED
    reachedSlotOnce: bool = False  # 是否已至少一次路过松散点 M_i（LOITERING 但尚未路过时仍应计入 T_ref 聚合）


@dataclass
class FormationAnalysisS:
    """集结完成后的一次性编队质量分析。注意：仅作边界诊断量输出，不进 Context。"""

    posErrMax_m: float = 0.0  # 期望僚机中的最大位置偏差，米（仅统计 expectedFollowerIds 里有效节点）
    posErrRms_m: float = 0.0  # 期望僚机位置误差 RMSE，米
    inPositionCount: int = 0  # 期望僚机中满足精度要求的机数
    totalCount: int = 0  # 期望参与集结的总机数（= len(expectedFollowerIds)，不受断链影响）


def copy_rally_slot_scale(src: RallySlotScaleS, dst: RallySlotScaleS) -> None:
    """复制槽位缩放因子，避免 Context 字段被外部别名引用。注意：新增字段时同步补齐。"""
    dst.scale = src.scale
    dst.scaleRate = src.scaleRate


def dist3d(a: "PosInEarthS", b: "PosInEarthS") -> float:
    """两点 3D 欧氏距离，单位米。"""
    de = a.east - b.east
    dn = a.north - b.north
    dh = a.h - b.h
    return math.sqrt(de * de + dn * dn + dh * dh)


def copy_follower_state(src: FollowerStateS, dst: FollowerStateS) -> None:
    """复制单架僚机状态快照。注意：pos 为嵌套对象，须逐字段复制。"""
    dst.id = src.id
    copy_position(src.pos, dst.pos)
    dst.posErr_m = src.posErr_m
    dst.headingErr_rad = src.headingErr_rad
    dst.arrived = src.arrived
    dst.valid = src.valid
    dst.lastUpdate_s = src.lastUpdate_s
    dst.eta_s = src.eta_s
    dst.rally_state = src.rally_state
    dst.reachedSlotOnce = src.reachedSlotOnce


def copy_formation_analysis(src: FormationAnalysisS, dst: FormationAnalysisS) -> None:
    """复制编队分析快照，供仿真层安全持有诊断结果。注意：新增字段时同步补齐。"""
    dst.posErrMax_m = src.posErrMax_m
    dst.posErrRms_m = src.posErrRms_m
    dst.inPositionCount = src.inPositionCount
    dst.totalCount = src.totalCount
