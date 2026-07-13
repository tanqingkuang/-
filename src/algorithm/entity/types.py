"""实体边界类型。注意：用于控制器和算法实体之间传递数据。"""

from __future__ import annotations

from dataclasses import dataclass, field

from src.algorithm.context.leaf_types import (
    AccInEarthS,
    FormCommInitS,
    FormationAnalysisS,
    FormSelfInitS,
    MotionProfS,
    PosInEarthS,
    PosTrackDiagS,
    RemoteCmdS,
    WayPointInputS,
)
from src.common.envelope import MessageEnvelope


DEFAULT_CONTROL_PERIOD_S = 0.05


@dataclass
class VelCmdLimitS:
    """前向/垂向速度指令限幅(串级 P+PI 外环输出)。注意：非对称，默认 ±inf 表示不限；侧向不限速。"""

    forwardMin: float = float("-inf")  # 前向速度指令下限(前向恒正时设 >0)
    forwardMax: float = float("inf")  # 前向速度指令上限
    verticalMin: float = float("-inf")  # 垂向速度指令下限(下降速度上限取负)
    verticalMax: float = float("inf")  # 垂向速度指令上限(爬升速度上限)


@dataclass
class EntityInitS:
    """实体一次性初始化配置。注意：集结实体共用 route 前两点确定集结中心和航向。"""

    selfInit: FormSelfInitS = field(default_factory=FormSelfInitS)  # 本机标识
    commInit: FormCommInitS = field(default_factory=FormCommInitS)  # 通信拓扑与队形配置
    route: list[WayPointInputS] = field(default_factory=list)  # 任务航线；集结实体同时读取前两点计算集结几何
    control_period_s: float = DEFAULT_CONTROL_PERIOD_S  # 控制算法处理周期，单位 s
    velCmdLimit: VelCmdLimitS = field(default_factory=VelCmdLimitS)  # 前向/垂向速度指令限幅
    rally_cfg: object | None = None  # RallyTaskInitS；长机使用完整参数，僚机只取 convergenceRadius_m
    rally_approach_speed_mps: float = 20.0  # 僚机飞向 M_i 的速度
    rally_leader_id: str = ""  # 僚机回报消息的发送目标（来自节点配置 leader_id）
    rally_layer_altitude_m: float | None = None  # 待命/JOINING/CATCHUP 分层目标高度；None 表示沿用集结槽位高度


@dataclass
class EntityInputS:
    """实体每帧输入。注意：各字段可为空，缺省时沿用上一帧状态。"""

    selfState: MotionProfS | None = None  # 本机最新运动状态反馈
    inbox: list[MessageEnvelope] = field(default_factory=list)  # 本帧收到的消息
    remote: RemoteCmdS | None = None  # 外部遥控指令
    now_s: float = 0.0  # 当前仿真时间戳（秒）；由仿真框架每帧注入，用于僚机报文超时检测


@dataclass
class EntityOutputS:
    """实体每帧输出。注意：控制器只从该边界读取算法结果。"""

    selfAccCmd: AccInEarthS | None = None  # 本机加速度指令
    selfCmd: MotionProfS | None = None  # 本机位置/速度指令快照
    controlDiag: PosTrackDiagS | None = None  # 位置跟踪诊断快照
    outbox: list[MessageEnvelope] = field(default_factory=list)  # 本帧待发送的消息
    formationAnalysis: FormationAnalysisS | None = None  # 仅集结完成首帧非 None；仿真层须另行锁存
