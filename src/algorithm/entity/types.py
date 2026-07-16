"""实体边界类型。注意：用于控制器和算法实体之间传递数据。"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum

from src.algorithm.context.context import FormContextS
from src.algorithm.context.leaf_types import (
    AccInEarthS,
    FormCommInitS,
    FormationAnalysisS,
    FormSelfInitS,
    MotionProfS,
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


@dataclass(frozen=True)
class EntityProcessSpecS:
    """单个流程策略规格。注意：空规格表示固定流程没有可配置策略。"""

    default_strategy: object | None = None  # 常规阶段默认策略；无策略流程保持 None
    strategies: tuple[object, ...] = ()  # 本实例允许创建和选择的全部/附加策略


@dataclass(frozen=True)
class EntityProcessTableS:
    """实体固定流程装配表。注意：字段顺序就是 EntityBase 的标准执行顺序。"""

    inbound: EntityProcessSpecS = field(default_factory=EntityProcessSpecS)  # 收消息流程
    formation_task: EntityProcessSpecS = field(default_factory=EntityProcessSpecS)  # 任务编排流程
    tra_plan: EntityProcessSpecS = field(default_factory=EntityProcessSpecS)  # 轨迹规划流程
    pos_calc: EntityProcessSpecS = field(default_factory=EntityProcessSpecS)  # 位置解算流程
    pos_track: EntityProcessSpecS = field(default_factory=EntityProcessSpecS)  # 位置跟踪流程
    outbound: EntityProcessSpecS = field(default_factory=EntityProcessSpecS)  # 发消息流程


class EntityProfileE(IntEnum):
    """实体身份枚举。注意：外部只选择身份，不拼装流程策略。"""

    RALLY_LEADER = 1  # 集结长机：集结位置解算、任务航线和速度控制
    RALLY_FOLLOWER = 2  # 集结僚机：集结/槽位位置解算和速度/位置控制


@dataclass(frozen=True)
class EntityProfileS:
    """实体不可变身份证。注意：同一身份的实例共享配置，不共享运行状态。"""

    identity: EntityProfileE  # 工厂选择键
    processes: EntityProcessTableS  # 本身份固定启用的流程策略


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
    rally_enabled: bool = True  # 当前实例是否执行集结任务；直接 HOLD 时关闭集结专用槽位配置


@dataclass(frozen=True)
class EntityManagerInitS:
    """流程 Manager 内部初始化参数。注意：由 Entity 根据自身 Profile 生成。"""

    entity: EntityInitS  # 每架飞机不同的运行初始化参数
    process: EntityProcessSpecS  # 当前流程所属实体身份的固定策略规格


@dataclass
class EntityRuntimeS:
    """实体流程共享运行环境。注意：各流程自行绑定所需对象，Entity 不维护具体端口。"""

    context: FormContextS = field(default_factory=FormContextS)  # 算法共享黑板
    remote: RemoteCmdS = field(default_factory=RemoteCmdS)  # 外部任务指令
    inbox: list[MessageEnvelope] = field(default_factory=list)  # 本拍收件箱
    outbox: list[MessageEnvelope] = field(default_factory=list)  # 本拍发件箱
    posTrackDiag: PosTrackDiagS = field(default_factory=PosTrackDiagS)  # 控制诊断输出


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
