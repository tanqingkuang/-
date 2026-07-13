"""仿真控制器对外快照类型和内部轻量数据类型。注意：保持 sim_control 兼容导出。"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Literal

from src.algorithm.context.leaf_types import PosTrackDiagS
from src.common.envelope import MessageEnvelope
from src.environment.model import AccelerationCommand

RunState = Literal["UNLOADED", "READY", "RUNNING", "PAUSED", "FINISHED"]
ControlReport = Literal["待命", "集结", "保持", "重构"]
EventLevel = Literal["DEBUG", "INFO", "WARN", "ERROR"]
DisturbanceType = Literal["wind", "node_fault", "link_loss", "link_fault", "clear"]
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

@dataclass(frozen=True)
class NodeState:
    """面向 UI/CLI 的单个飞机节点状态。注意：字段单位为界面展示契约。"""

    node_id: str
    role: str
    health: str
    # ENU 位置：x 为东向，y 为北向，altitude 为天向。
    x_m: float
    y_m: float
    altitude_m: float
    psi_v_deg: float  # 航迹偏航角（度）。
    theta_deg: float  # 航迹俯仰角（度）。
    speed_mps: float  # 合速度大小。
    # ENU 三轴速度分量。
    vx_mps: float
    vy_mps: float
    vz_mps: float
    nx: float  # 切向过载。
    nz: float  # 法向过载。
    phi_deg: float  # 滚转角（度）。
    psi_dot_deg_s: float  # 航迹偏航角速率（度/秒）。
    # 位置/速度指令，采用 ENU 命名。
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
    track_pos_err_x_m: float = 0.0
    track_pos_err_y_m: float = 0.0
    track_pos_err_z_m: float = 0.0
    track_vel_err_x_mps: float = 0.0
    track_vel_err_y_mps: float = 0.0
    track_vel_err_z_mps: float = 0.0
    # 相对当前航段的侧偏与待飞距，无航线时为 None。
    cross_track_error_m: float | None = None
    distance_to_go_m: float | None = None
    rally_phase: str = ""  # 集结阶段字符串，如 JOINING/FLYING、CATCHUP、LOOSE、COMPRESS、HOLD


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
    """集结节点的待命圆、集结圆与切线关键点。注意：供 GUI 预览与运行期叠加显示。"""

    node_id: str
    local_center_east_m: float  # 本地待命盘旋圆圆心东向坐标。
    local_center_north_m: float
    local_radius_m: float  # 本地待命盘旋圆半径。
    rally_center_east_m: float  # 集结盘旋圆圆心东向坐标。
    rally_center_north_m: float
    rally_radius_m: float  # 集结盘旋圆半径。
    local_tangent_east_m: float  # 从本地圆切出的位置，东向坐标。
    local_tangent_north_m: float
    rally_tangent_east_m: float  # 切入集结圆的位置，东向坐标。
    rally_tangent_north_m: float
    slot_east_m: float  # 松散目标点 M_i（同时是集结圆切出点），东向坐标。
    slot_north_m: float
    fallback_used: bool = False  # True 表示几何退化时使用了直飞兜底点。

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

    @property
    def entry_east_m(self) -> float:
        """兼容旧 GUI 字段名，返回集结圆切入点东向坐标。"""
        return self.rally_tangent_east_m

    @property
    def entry_north_m(self) -> float:
        """兼容旧 GUI 字段名，返回集结圆切入点北向坐标。"""
        return self.rally_tangent_north_m


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
    route: RouteState | None = None  # 当前航段。
    route_segments: list[RouteState] = field(default_factory=list)  # 全部航段。
    cpu_utilization: float = 0.0  # 后台调度忙碌时间占墙钟周期比例，范围 0..1。
    rally_analysis: object | None = None  # FormationAnalysisS；集结完成首帧非 None，控制器锁存
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
    formation_analysis: object | None = None  # FormationAnalysisS；集结完成首帧非 None


@dataclass(frozen=True)
class _ConfiguredLink:
    """配置中声明的一条链路（折叠前）。注意：用于把通信模块的双向状态归并为面向 UI 的单条链路。"""

    link_id: str
    direction: str  # "duplex" 时快照阶段需合并正反向状态。
