"""仿真控制器对外快照类型和内部轻量数据类型。注意：保持 sim_control 兼容导出。"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum
from typing import Callable, Literal

from src.algorithm.context.leaf_types import PosTrackDiagS
from src.common.envelope import MessageEnvelope
from src.environment.model import AccelerationCommand

ControlReport = Literal["待命", "集结", "保持", "重构"]
EventLevel = Literal["DEBUG", "INFO", "WARN", "ERROR"]
ResultCode = Literal[
    "OK",
    "ERR_NO_CONFIG",
    "ERR_CONFIG_NOT_FOUND",
    "ERR_CONFIG_INVALID",
    "ERR_INVALID_STATE",
    "ERR_INVALID_ARGUMENT",
    "ERR_BUSY",
    "ERR_MODULE_INIT_FAILED",
    "ERR_TICK_FAILED",
    "ERR_LOG_FAILED",
    "ERR_INTERNAL",
]


class RunState(StrEnum):
    """仿真运行状态。注意：枚举值保持既有快照与日志字符串契约。"""

    UNLOADED = "UNLOADED"
    READY = "READY"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    FINISHED = "FINISHED"


class DisturbanceType(StrEnum):
    """控制器支持的扰动类型。注意：字符串值也是跨层序列化契约。"""

    WIND = "wind"
    NODE_FAULT = "node_fault"
    LINK_LOSS = "link_loss"
    LINK_FAULT = "link_fault"
    CLEAR = "clear"

@dataclass(frozen=True)
class NodeState:
    """面向 UI/CLI 的单个飞机节点状态。注意：字段单位为界面展示契约。"""

    # 节点位置只采用东北天 ENU 地面系，禁止在对外快照中混入航迹系分量。
    # 航向与俯仰展示字段描述地面航迹，横风下不等同于机体的空速方向。
    # speed_mps 延续历史空速语义，新增代码应优先读取语义更明确的 airspeed_mps。
    # vx/vy/vz 是 ENU 地速分量，可与相邻位置快照直接进行差分校核。
    # nx/ny/nz 采用前上右 FUR 航迹系，三轴符号不能按 ENU 字段名推断。
    # phi_deg 由 ny/nz 派生，采用右倾为正的苏联系右手约定。
    node_id: str
    role: str
    health: str
    # ENU 位置：x 为东向，y 为北向，altitude 为天向。
    x_m: float
    y_m: float
    altitude_m: float
    psi_v_deg: float  # 地面航迹偏航角（度），自东向逆时针为正。
    theta_deg: float  # 地面航迹俯仰角（度），爬升为正。
    speed_mps: float  # 空速大小；保留历史字段名以兼容现有调用方。
    # ENU 三轴地速分量。
    vx_mps: float
    vy_mps: float
    vz_mps: float
    nx: float  # 航迹系 x 前向过载。
    nz: float  # 航迹系 z 右向过载。
    phi_deg: float  # 滚转角（度），右倾为正。
    psi_dot_deg_s: float  # 地面航迹偏航角速率（度/秒），左转为正。
    # 新增字段放在历史必填字段之后并提供默认值，旧日志仍可按原字段集离线读取。
    ground_speed_mps: float = 0.0  # 三维地速大小。
    ny: float = 0.0  # 航迹系 y 上向过载。
    n_normal: float = 0.0  # y-z 法向平面的合过载。
    # 显式空速字段用于区分空气动力学状态与受风影响的地面运动状态。
    airspeed_mps: float = 0.0  # 显式空速别名；数值与兼容字段 speed_mps 相同。
    air_psi_v_deg: float = 0.0  # 空速航向角（度），左转为正。
    air_theta_deg: float = 0.0  # 空速航迹倾角（度），爬升为正。
    air_psi_dot_deg_s: float = 0.0  # 空速航向角速率（度/秒），用于物理限幅。
    # ground_speed_mps 是三维地速模长，而算法 vd 另有“水平地速”契约，二者不可互换。
    # air_psi_v_deg 与 psi_v_deg 在无风时相同，横风时分别代表机头气流方向和地面航迹方向。
    # air_theta_deg 与 theta_deg 在垂向风存在时可能不同，日志需同时保留以便追因。
    # air_psi_dot_deg_s 服务动力学包线判断，psi_dot_deg_s 服务航线跟踪与画面展示。
    # 位置/速度指令，采用 ENU 命名。
    # cmd_pos_* 与 cmd_vel_* 均采用 ENU，不能直接与 FUR 槽位量逐分量相减。
    cmd_pos_east_m: float = 0.0
    cmd_pos_north_m: float = 0.0
    cmd_pos_h_m: float = 0.0
    cmd_vel_east_mps: float = 0.0
    cmd_vel_north_mps: float = 0.0
    cmd_vel_up_mps: float = 0.0
    # 位置/速度误差，采用 ENU 命名。
    pos_err_east_m: float = 0.0
    pos_err_north_m: float = 0.0
    pos_err_h_m: float = 0.0
    vel_err_east_mps: float = 0.0
    vel_err_north_mps: float = 0.0
    vel_err_up_mps: float = 0.0
    # 航迹坐标系误差，采用 x/y/z 命名。
    # track_pos_err_* 与 track_vel_err_* 采用前上右 FUR，z 正值表示目标在右侧。
    # ENU 误差用于世界坐标诊断，FUR 误差用于控制品质诊断，两套字段都保留物理出处。
    track_pos_err_x_m: float = 0.0
    track_pos_err_y_m: float = 0.0
    track_pos_err_z_m: float = 0.0
    track_vel_err_x_mps: float = 0.0
    track_vel_err_y_mps: float = 0.0
    track_vel_err_z_mps: float = 0.0
    # 相对当前航段的侧偏与待飞距，无航线时为 None。
    # cross_track_error_m 以航迹右侧为正，符号与 FUR 的 z 轴保持一致。
    # distance_to_go_m 是沿航段方向的投影距离，不是 ENU 三维直线距离。
    cross_track_error_m: float | None = None
    distance_to_go_m: float | None = None
    rally_phase: str = ""  # 集结阶段字符串，如 JOINING/FLYING、CATCHUP、LOOSE、HOLD
    # 评测补充字段：原始控制指令、饱和证据、算法耗时与槽位上下文，均为增量字段，
    # 旧日志缺失时离线分析工具按"通道不可用"处理，不得用 0 兜底。
    # cmd_acc_* 是位置跟踪器输出的 ENU 原始加速度指令（模型限幅前）。
    cmd_acc_east_mps2: float = 0.0
    cmd_acc_north_mps2: float = 0.0
    cmd_acc_up_mps2: float = 0.0
    # 任一 ENU 轴原始指令达到模型幅值上限即视为指令饱和。
    acc_saturated: bool = False
    # 横侧向串级变限幅是否触发饱和，来自 PosTrackDiagS，仅串级配置有意义。
    lateral_saturated: bool = False
    # 本节点最近一次算法链路单步耗时（毫秒），按算法分频节拍更新。
    algo_step_ms: float = 0.0
    # 当前队形下本机的标称槽位坐标（长机 FUR，前/上/右），未定义槽位时为 None。
    # 注意：这是配置标称值（scale=1），槽位缩放的动态变化不反映在此字段。
    slot_x_m: float | None = None
    slot_y_m: float | None = None
    slot_z_m: float | None = None


@dataclass(frozen=True)
class LinkState:
    """面向 UI/CLI 的单条通信链路状态。注意：双向链路会折叠为配置链路显示。"""

    link_id: str
    direction: str  # 单工/双工，决定快照是否折叠反向。
    latency_ms: float  # 折叠后时延（双工取两向最大）。
    loss_rate: float  # 折叠后丢包率（双工取两向最大）。
    status: str  # 折叠后状态（任一方向 lost 即 lost）。


@dataclass(frozen=True)
class RouteState:
    """面向 UI 的 ENU 参考航段。注意：只表示单个航段。"""

    start_x_m: float
    start_y_m: float
    start_altitude_m: float
    end_x_m: float
    end_y_m: float
    end_altitude_m: float
    radius_m: float = 0.0
    center_x_m: float = 0.0
    center_y_m: float = 0.0
    turn_sign: float = 0.0


@dataclass(frozen=True)
class RallyPlanGeometryState:
    """集结节点的待命圆和集结圆。注意：供 GUI 预览与运行期叠加显示。"""

    node_id: str
    local_center_east_m: float  # 本地待命盘旋圆圆心东向坐标。
    local_center_north_m: float
    local_radius_m: float  # 本地待命盘旋圆半径。
    rally_center_east_m: float  # 集结盘旋圆圆心东向坐标。
    rally_center_north_m: float
    rally_radius_m: float  # 集结盘旋圆半径。

    @property
    def loiter_center_east_m(self) -> float:
        """兼容旧 GUI 字段名，返回集结盘旋圆圆心东向坐标。"""
        return self.rally_center_east_m

    @property
    def loiter_center_north_m(self) -> float:
        """兼容旧 GUI 字段名，返回集结盘旋圆圆心北向坐标。"""
        return self.rally_center_north_m

    @property
    def loiter_radius_m(self) -> float:
        """兼容旧 GUI 字段名，返回集结盘旋圆半径。"""
        return self.rally_radius_m

# 保留公开类型别名，避免圆几何快照改名影响现有调用方。
RallyJoinGeometryState = RallyPlanGeometryState


@dataclass(frozen=True)
class SimulationSnapshot:
    """完整实时观测快照。注意：供 GUI、CLI 和订阅回调读取。"""

    time_s: float  # 当前仿真时间。
    duration_s: float  # 总时长。
    step_s: float  # 仿真步长。
    run_state: RunState  # 运行状态机当前态。
    control_report: ControlReport  # 控制回报文本（待命/集结/保持/重构）。
    nodes: list[NodeState]
    links: list[LinkState]
    active_disturbances: tuple[DisturbanceType, ...] = ()  # 当前仍生效的权威扰动类型。
    route: RouteState | None = None  # 当前航段。
    route_segments: list[RouteState] = field(default_factory=list)  # 全部航段。
    cpu_utilization: float = 0.0  # 后台调度忙碌时间占墙钟周期比例，范围 0..1。
    rally_geometry: dict[str, RallyPlanGeometryState] = field(default_factory=dict)  # 按 node_id 索引，非集结场景为空字典
    blocked_route_segments: list[RouteState] = field(default_factory=list)  # 被封锁的原始航线(仅避障覆盖生效时非空)。


@dataclass(frozen=True)
class TimedSnapshotCursor:
    """固定仿真时钟快照的读取游标。注意：索引只在同一运行代内有意义。"""

    run_generation: int  # 配置初始化或重置后递增，防止旧索引跳过新运行样本。
    next_index: int  # 下一次应读取的快照索引，采用左闭右开序列语义。


@dataclass(frozen=True)
class SimulationEvent:
    """近期事件记录。注意：用于 GUI 日志窗口和 CLI 诊断。"""

    time_s: float
    level: EventLevel
    source: str
    message: str


@dataclass(frozen=True)
class CommandResult:
    """应用层命令执行结果。注意：code 用于程序判断，message 用于显示。"""

    code: ResultCode
    message: str = ""


@dataclass(frozen=True)
class DisturbanceCommand:
    """inject_disturbance 接收的动态扰动命令。注意：params 必须可序列化。"""

    type: DisturbanceType  # 扰动种类。
    target: str | None = None  # 作用对象（节点/链路 ID），按类型解释。
    duration_s: float | None = None  # 持续时长；None 表示持续到显式 clear。
    params: dict[str, object] = field(default_factory=dict)  # 类型相关附加参数。


class Subscription:
    """subscribe_snapshot 返回的订阅句柄。注意：调用 unsubscribe 可取消回调。"""

    def __init__(self, unsubscribe: Callable[[], None]) -> None:
        """初始化 Subscription 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        self._unsubscribe = unsubscribe
        self._active = True

    def unsubscribe(self) -> None:
        """取消订阅回调。注意：回调不存在时应保持幂等。"""

        if self._active:
            self._unsubscribe()
            self._active = False


@dataclass
class _NodeAlgorithmOutput:
    """单个节点算法一步的输出。注意：聚合控制指令、待发消息与状态文本，供主循环分发。"""

    control: AccelerationCommand  # 该节点本步算出的加速度控制指令，喂给模型。
    outbox: list[MessageEnvelope]  # 该节点本步要广播/发送的消息，统一交给通信模块。
    status: str  # 算法运行态文本（如 "forming"/"reconfiguring"），用于推导控制回报。
    control_diag: PosTrackDiagS  # 该节点本步位置跟踪诊断，供快照和日志记录。


@dataclass(frozen=True)
class _ConfiguredLink:
    """配置中声明的一条链路（折叠前）。注意：用于把通信模块的双向状态归并为面向 UI 的单条链路。"""

    link_id: str
    direction: str  # "duplex" 时快照阶段需合并正反向状态。
