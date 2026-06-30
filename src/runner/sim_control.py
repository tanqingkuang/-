"""Simulation control facade.

The controller implements the application contract described in
``docs/1-仿真控制HLD.md``. The UAV model is provided by
``src.environment.model``; communication, algorithm, disturbance, and logging
remain first-pass local implementations.
"""

from __future__ import annotations

import json
import math
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass, field, replace
from decimal import Decimal, ROUND_HALF_UP
from pathlib import Path
from typing import Any, Callable, Literal

from src.algorithm.context.leaf_types import (
    CommDirE,
    FormCommInitS,
    FormPatE,
    FormPosS,
    FormSelfInitS,
    FormStageE,
    FormationAnalysisS,
    MotionProfS,
    NetWorkS,
    PosInEarthS,
    PosTrackDiagS,
    RemoteCmdS,
    VdInEarthS,
    WayLineS,
    WayPointInputS,
    WayPointS,
    copy_motion,
    to_display_inputs,
)
from src.algorithm.entity.base import EntityBase
from src.algorithm.entity.leader_follower_hold.follower import FollowerEntity
from src.algorithm.entity.leader_follower_hold.leader import LeaderEntity, waypoint_inputs_to_waylines
from src.algorithm.entity.leader_follower_rally.leader import RallyLeaderEntity
from src.algorithm.entity.leader_follower_rally.follower import RallyFollowerEntity
from src.algorithm.entity.types import EntityInitS, EntityInputS, EntityOutputS, VelCmdLimitS
from src.algorithm.units.process.formation_task.rally import RallyTaskInitS
from src.common.envelope import MessageEnvelope
from src.environment.comm import CommunicationChannel
from src.environment.model import AccelerationCommand, AircraftState, ModelIterator, node_id_from_config


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

_DEFAULT_ALGORITHM_DECIMATION = 10
_COMM_DECIMATION = 2
_MIN_PLAYBACK_RATE = 0.1
_MAX_PLAYBACK_RATE = 20.0
_RUN_LOOP_SLEEP_SLICE_S = 0.005
_MAX_RUN_LOOP_BATCH_TICKS = 100
_CPU_UTILIZATION_SAMPLE_PERIOD_S = 1.0


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


_DEFAULT_TRIANGLE_WING_SLOTS: tuple[tuple[float, float, float], ...] = (
    (-54.0, 0.0, -58.0),
    (-54.0, 0.0, 58.0),
)
_FORMATION_COORDINATE_SYSTEM = "x_forward_y_up_z_right"
_LOG_SAMPLE_PERIOD_S = 0.1
_TIME_EPSILON_S = 1e-9


def _motion_from_aircraft_state(state: AircraftState) -> MotionProfS:
    """把环境模型状态转换为算法运动状态。注意：单位和坐标系必须保持一致。"""
    # 地速取水平面速度模长（不含垂向分量）。
    ground_speed = (state.vx_mps * state.vx_mps + state.vy_mps * state.vy_mps) ** 0.5
    return MotionProfS(
        # 位置直接映射 ENU 三轴。
        pos=PosInEarthS(state.x_m, state.y_m, state.altitude_m),
        # 速度含三轴分量、航迹角与地速，供算法做编队/航线解算。
        v=VdInEarthS(
            vEast=state.vx_mps,
            vNorth=state.vy_mps,
            vUp=state.vz_mps,
            vTheta=state.theta_rad,
            vPsi=state.psi_rad,
            vd=ground_speed,
            # 航迹偏航角速率：模型以 deg/s 输出(俯视图左偏为正)，与算法 vPsi(自东向逆时针)同向，转 rad/s。
            dVPsi=math.radians(state.psi_dot_deg_s),
        ),
    )


def _build_formation_comm_init(
    nodes: list[object],
    links: list[object],
    config: dict[str, object] | None = None,
) -> FormCommInitS:
    """根据配置生成编队通信初始化信息。注意：节点 ID 必须与模型配置一致。"""
    network: list[NetWorkS] = []
    for link in links:
        if not isinstance(link, dict):
            continue
        # 解析 "start-end" 链路 ID；格式不合法的条目跳过。
        link_id = str(link.get("link_id") or "")
        start_id, sep, end_id = link_id.partition("-")
        if not sep or not start_id or not end_id:
            continue
        # 默认双工，仅显式 "simplex" 时为单工。
        direction = CommDirE.SIMPLEX if link.get("direction") == "simplex" else CommDirE.DUPLEX
        network.append(NetWorkS(start_id, end_id, direction))

    # 队形（模式 + 槽位）与通信网络一并打包为编队初始化结构。
    pattern, slots = _build_formation_slots(nodes, config)
    return FormCommInitS(
        netWork=network,
        formPat=[pattern],
        formPos=[slots],
    )


def _build_formation_slots(
    nodes: list[object],
    config: dict[str, object] | None,
) -> tuple[FormPatE, list[FormPosS]]:
    """根据配置生成编队槽位定义。注意：槽位是队形定义，不应依赖飞机初始位置。"""
    # 未配置 formation 时回退到默认三角队形。
    formation_config = (config or {}).get("formation")
    if formation_config is None:
        return FormPatE.TRIANGLE, _default_formation_slots(nodes)
    if not isinstance(formation_config, dict):
        raise ValueError("formation must be an object")

    pattern = _formation_pattern_from_config(formation_config.get("pattern", "TRIANGLE"))
    # 给了 pattern 但未给 slots 时同样用默认槽位。
    slot_config = formation_config.get("slots")
    if slot_config is None:
        return pattern, _default_formation_slots(nodes)
    if not isinstance(slot_config, list) or not slot_config:
        raise ValueError("formation.slots must be a non-empty list")
    _validate_formation_coordinate_system(formation_config)

    # 收集已知节点 ID，用于校验槽位引用的节点是否存在。
    known_node_ids = {
        node_id_from_config(node, index)
        for index, node in enumerate(nodes)
        if isinstance(node, dict)
    }
    slots_by_id: dict[str, FormPosS] = {}
    for index, slot in enumerate(slot_config):
        if not isinstance(slot, dict):
            raise ValueError(f"formation.slots[{index}] must be an object")
        # 兼容 node_id / id 两种键名。
        node_id = str(slot.get("node_id", slot.get("id", "")))
        if not node_id:
            raise ValueError(f"formation.slots[{index}].node_id is required")
        # 槽位不得重复或引用未知节点。
        if node_id in slots_by_id:
            raise ValueError(f"formation.slots contains duplicate node_id {node_id!r}")
        if known_node_ids and node_id not in known_node_ids:
            raise ValueError(f"formation.slots contains unknown node_id {node_id!r}")
        slots_by_id[node_id] = FormPosS(
            node_id,
            _float_from_keys(slot, "formation.slots", index, ("x_m", "x")),
            _float_from_keys(slot, "formation.slots", index, ("y_m", "y")),
            _float_from_keys(slot, "formation.slots", index, ("z_m", "z")),
        )

    # 每个已知节点都必须有对应槽位，缺一即报错。
    missing = [node_id for node_id in known_node_ids if node_id not in slots_by_id]
    if missing:
        raise ValueError(f"formation.slots missing node_id {missing[0]!r}")

    # 按节点声明顺序输出槽位，保证与模型节点顺序对齐。
    ordered_slots: list[FormPosS] = []
    for index, node in enumerate(nodes):
        if not isinstance(node, dict):
            continue
        ordered_slots.append(slots_by_id[node_id_from_config(node, index)])
    return pattern, ordered_slots


def _validate_formation_coordinate_system(formation_config: dict[str, object]) -> None:
    """校验显式槽位的坐标轴声明。注意：用于阻止旧 y 侧向配置被新轴序静默解释。"""
    raw_value = formation_config.get("coordinate_system")
    if raw_value is None:
        raise ValueError(
            "formation.coordinate_system is required when formation.slots is configured; "
            f"use {_FORMATION_COORDINATE_SYSTEM!r}"
        )
    if str(raw_value).strip() != _FORMATION_COORDINATE_SYSTEM:
        raise ValueError(
            "formation.coordinate_system must be "
            f"{_FORMATION_COORDINATE_SYSTEM!r}; x=forward, y=up, z=right"
        )


def _default_formation_slots(nodes: list[object]) -> list[FormPosS]:
    """生成默认三机队形槽位。注意：仅在配置未给出队形时兜底。"""
    leader_id = _leader_id_from_nodes(nodes)
    slots: list[FormPosS] = []
    wing_slot_index = 0
    for index, node in enumerate(nodes):
        if not isinstance(node, dict):
            continue
        node_id = node_id_from_config(node, index)
        # 长机居于队形原点。
        if node_id == leader_id:
            slots.append(FormPosS(node_id, 0.0, 0.0, 0.0))
        else:
            # 僚机按预设左右两翼槽位依次分配；超过两机需显式配置。
            if wing_slot_index >= len(_DEFAULT_TRIANGLE_WING_SLOTS):
                raise ValueError("default triangle formation requires explicit slots for more than two wingmen")
            slot = _DEFAULT_TRIANGLE_WING_SLOTS[wing_slot_index]
            slots.append(FormPosS(node_id, slot[0], slot[1], slot[2]))
            wing_slot_index += 1
    return slots


def _formation_pattern_from_config(raw_pattern: object) -> FormPatE:
    """从配置中读取队形类型。注意：未知类型按默认队形处理。"""
    # 字符串按枚举名（大小写无关）解析。
    if isinstance(raw_pattern, str):
        try:
            return FormPatE[raw_pattern.strip().upper()]
        except KeyError as exc:
            raise ValueError(f"unknown formation.pattern {raw_pattern!r}") from exc
    # 否则按枚举整数值解析。
    try:
        return FormPatE(int(raw_pattern))
    except (TypeError, ValueError) as exc:
        raise ValueError(f"unknown formation.pattern {raw_pattern!r}") from exc


def _float_from_keys(
    config: dict[str, object],
    prefix: str,
    index: int,
    keys: tuple[str, str],
) -> float:
    """按候选键读取浮点配置。注意：用于兼容历史字段名。"""
    # 依次尝试候选键（如 ("x_m","x")），命中即返回，全缺则报错。
    for key in keys:
        if key in config:
            return float(config[key])
    raise ValueError(f"{prefix}[{index}].{keys[0]} is required")


def _build_leader_route(config: dict[str, object] | None = None, *, insert_arcs: bool = True) -> list[WayPointInputS]:
    """根据配置生成长机航线。注意：insert_arcs=False 时拐点 r=0，不插圆弧(用于显示原始航段)。"""
    # 无 route 配置时用默认直线航线兜底。
    route_config = (config or {}).get("route")
    if route_config is None:
        return _default_leader_wpi()
    if not isinstance(route_config, dict):
        raise ValueError("route must be an object")

    # 三种写法优先级：waypoints（航点序列）> segments/lines（航段列表）> 单段。
    waypoints = route_config.get("waypoints")
    if waypoints is not None:
        return _wpi_from_waypoints(waypoints, route_config, insert_arcs=insert_arcs)

    segments = route_config.get("segments", route_config.get("lines"))
    if segments is None:
        wl = _wayline_from_config(route_config, 0, "route")
        return [
            WayPointInputS(idx=0, pos=wl.start.pos, vdCmd=wl.start.vdCmd),
            WayPointInputS(idx=1, pos=wl.end.pos, vdCmd=wl.start.vdCmd),
        ]
    if not isinstance(segments, list) or not segments:
        raise ValueError("route.segments must be a non-empty list")
    wpi_list: list[WayPointInputS] = []
    for index, segment in enumerate(segments):
        wl = _wayline_from_config(segment, index, f"route.segments[{index}]", route_config)
        if not wpi_list:
            wpi_list.append(WayPointInputS(idx=0, pos=wl.start.pos, vdCmd=wl.start.vdCmd))
        else:
            # 衔接航点的 vdCmd 描述“该点之后一段”，因此取下一航段速度。
            wpi_list[-1].vdCmd = wl.start.vdCmd
        wpi_list.append(WayPointInputS(idx=len(wpi_list), pos=wl.end.pos, vdCmd=wl.start.vdCmd))
    return wpi_list


def _waypoint_radius(raw_point: object, default_radius: float, field_name: str) -> float:
    """读取航点的预期转弯半径 R。注意：兼容 R/radius_m 两种键名，缺省取全局默认。"""
    radius = default_radius
    if isinstance(raw_point, dict):
        radius = float(raw_point.get("R", raw_point.get("radius_m", default_radius)))
    if radius < 0.0:
        raise ValueError(f"{field_name}.R must be >= 0")
    return radius


def _same_xy(a: PosInEarthS, b: PosInEarthS) -> bool:
    """判断两点水平面是否重合。注意：仅比较东/北，用于退化段保护。"""
    return abs(a.east - b.east) <= 1e-9 and abs(a.north - b.north) <= 1e-9


def _wpi_from_waypoints(
    raw_waypoints: object, route_defaults: dict[str, object], *, insert_arcs: bool = True
) -> list[WayPointInputS]:
    """把航点序列转换为 WayPointInputS 列表。注意：insert_arcs=True 时内部拐点设 r=R，由 leader.init() 插圆弧。"""
    if not isinstance(raw_waypoints, list) or len(raw_waypoints) < 2:
        raise ValueError("route.waypoints must contain at least two points")
    speed = float(route_defaults.get("speed_mps", route_defaults.get("vdCmd", 8.0)))
    if speed < 0.0:
        raise ValueError("route.speed_mps must be non-negative")
    default_r = float(route_defaults.get("radius_m", route_defaults.get("radius", 0.0)))
    if default_r < 0.0:
        raise ValueError("route.radius_m must be >= 0")
    points = [
        _route_point_from_config(raw_point, f"route.waypoints[{index}]")
        for index, raw_point in enumerate(raw_waypoints)
    ]
    radii = [
        _waypoint_radius(raw_point, default_r, f"route.waypoints[{index}]")
        for index, raw_point in enumerate(raw_waypoints)
    ]
    for index in range(len(points) - 1):
        if _same_xy(points[index], points[index + 1]) and points[index].h == points[index + 1].h:
            raise ValueError(f"route.waypoints[{index}] and route.waypoints[{index + 1}] must be different")
    n = len(points)
    return [
        WayPointInputS(
            idx=index,
            pos=points[index],
            vdCmd=speed,
            r=radii[index] if (insert_arcs and 0 < index < n - 1) else 0.0,
        )
        for index in range(n)
    ]


def _wayline_from_config(
    segment_config: object,
    index: int,
    field_name: str,
    route_defaults: dict[str, object] | None = None,
) -> WayLineS:
    """从单段配置构造航段对象。注意：字段单位统一为米和米每秒。"""
    if not isinstance(segment_config, dict):
        raise ValueError(f"{field_name} must be an object")
    defaults = route_defaults or {}
    # 速度/半径按"段内字段 > 全局默认"逐级回退，并兼容历史键名 vdCmd/radius。
    speed = float(
        segment_config.get(
            "speed_mps",
            segment_config.get("vdCmd", defaults.get("speed_mps", defaults.get("vdCmd", 8.0))),
        )
    )
    radius = float(
        segment_config.get(
            "radius_m",
            segment_config.get("radius", defaults.get("radius_m", defaults.get("radius", 0.0))),
        )
    )
    if speed < 0.0:
        raise ValueError(f"{field_name}.speed_mps must be non-negative")
    # 当前仅支持直线段，半径必须为 0。
    if radius != 0.0:
        raise ValueError(f"{field_name}.radius_m must be 0 for straight route")
    start = _route_point_from_config(segment_config.get("start"), f"{field_name}.start")
    end = _route_point_from_config(segment_config.get("end"), f"{field_name}.end")
    # 起讫点重合的退化段非法。
    if start.east == end.east and start.north == end.north and start.h == end.h:
        raise ValueError(f"{field_name} start and end must be different")
    return WayLineS(
        idx=index,
        start=WayPointS(idx=index, pos=start, vdCmd=speed),
        end=WayPointS(idx=index + 1, pos=end),
    )


def _default_leader_wpi() -> list[WayPointInputS]:
    """生成默认长机航点输入。注意：只作为配置缺省兜底。"""
    return [
        WayPointInputS(idx=0, pos=PosInEarthS(0.0, 0.0, 1000.0), vdCmd=8.0),
        WayPointInputS(idx=1, pos=PosInEarthS(1000.0, 0.0, 1000.0), vdCmd=8.0),
    ]


def _route_point_from_config(raw: object, field_name: str) -> PosInEarthS:
    """从配置读取航点坐标。注意：兼容数组和对象两种写法。"""
    if not isinstance(raw, dict):
        raise ValueError(f"{field_name} must be an object")
    # 兼容 x_m/east、y_m/north、altitude_m/h 两套字段名，统一为 ENU 坐标。
    return PosInEarthS(
        float(raw.get("x_m", raw.get("east", 0.0))),
        float(raw.get("y_m", raw.get("north", 0.0))),
        float(raw.get("altitude_m", raw.get("h", 0.0))),
    )


def _leader_id_from_nodes(nodes: list[object]) -> str:
    """从节点配置中识别长机 ID。注意：找不到时使用默认长机。"""
    # 优先取显式 role=="leader" 或 "rally_leader" 的节点。
    for index, node in enumerate(nodes):
        if isinstance(node, dict) and str(node.get("role") or "") in {"leader", "rally_leader"}:
            return node_id_from_config(node, index)
    # 没有显式长机则回退到第一个节点。
    for index, node in enumerate(nodes):
        if isinstance(node, dict):
            return node_id_from_config(node, index)
    return ""


def _route_state_from_wayline(route: WayLineS) -> RouteState:
    """根据当前航段生成航线状态。注意：用于快照显示和航段跟踪。"""
    from src.algorithm.units.algo.arc_path import arc_radius as _arc_radius
    radius_m = _arc_radius(route) if route.start.turnSign != 0.0 else 0.0
    return RouteState(
        start_x_m=route.start.pos.east,
        start_y_m=route.start.pos.north,
        start_altitude_m=route.start.pos.h,
        end_x_m=route.end.pos.east,
        end_y_m=route.end.pos.north,
        end_altitude_m=route.end.pos.h,
        radius_m=radius_m,
        center_x_m=route.start.center.east,
        center_y_m=route.start.center.north,
        turn_sign=route.start.turnSign,
    )


def _build_vel_cmd_limit(config: dict[str, object] | None = None) -> VelCmdLimitS:
    """从配置 control.velocity_command_limits 解析前向/垂向速度指令限幅。注意：整块或单项缺省即不限(±inf)，侧向不参与。"""
    control = (config or {}).get("control")
    if control is None:
        return VelCmdLimitS()
    if not isinstance(control, dict):
        raise ValueError("control must be an object")
    limits = control.get("velocity_command_limits")
    if limits is None:
        return VelCmdLimitS()
    if not isinstance(limits, dict):
        raise ValueError("control.velocity_command_limits must be an object")
    limit = VelCmdLimitS(
        forwardMin=float(limits.get("forward_min_mps", float("-inf"))),
        forwardMax=float(limits.get("forward_max_mps", float("inf"))),
        verticalMin=float(limits.get("vertical_min_mps", float("-inf"))),
        verticalMax=float(limits.get("vertical_max_mps", float("inf"))),
    )
    if limit.forwardMin > limit.forwardMax:
        raise ValueError("control.velocity_command_limits: forward_min_mps must be <= forward_max_mps")
    if limit.verticalMin > limit.verticalMax:
        raise ValueError("control.velocity_command_limits: vertical_min_mps must be <= vertical_max_mps")
    return limit


def _build_rally_route(config: dict[str, object] | None = None, *, insert_arcs: bool = True) -> list[WayPointInputS] | None:
    """从配置 rally_route 段生成集结长机航线。注意：键不存在时返回 None。"""
    route_config = (config or {}).get("rally_route")
    if route_config is None:
        return None
    if not isinstance(route_config, dict):
        raise ValueError("rally_route must be an object")
    waypoints = route_config.get("waypoints")
    if waypoints is not None:
        return _wpi_from_waypoints(waypoints, route_config, insert_arcs=insert_arcs)
    segments = route_config.get("segments", route_config.get("lines"))
    if segments is None:
        wl = _wayline_from_config(route_config, 0, "rally_route")
        return [
            WayPointInputS(idx=0, pos=wl.start.pos, vdCmd=wl.start.vdCmd),
            WayPointInputS(idx=1, pos=wl.end.pos, vdCmd=wl.start.vdCmd),
        ]
    if not isinstance(segments, list) or not segments:
        raise ValueError("rally_route.segments must be a non-empty list")
    wpi_list: list[WayPointInputS] = []
    for index, segment in enumerate(segments):
        wl = _wayline_from_config(segment, index, f"rally_route.segments[{index}]", route_config)
        if index == 0:
            wpi_list.append(WayPointInputS(idx=0, pos=wl.start.pos, vdCmd=wl.start.vdCmd))
        wpi_list.append(WayPointInputS(idx=index + 1, pos=wl.end.pos, vdCmd=wl.start.vdCmd))
    return wpi_list


def _build_rally_task_init(
    config: dict[str, object] | None,
    algorithm_period_s: float,
    nodes: list[object],
) -> RallyTaskInitS | None:
    """从配置 rally_cfg 段构造集结任务初始化参数。注意：键不存在时返回 None。"""
    rally_cfg_raw = (config or {}).get("rally_cfg")
    if rally_cfg_raw is None:
        return None
    if not isinstance(rally_cfg_raw, dict):
        raise ValueError("rally_cfg must be an object")
    expected_ids = [
        node_id_from_config(node, i)
        for i, node in enumerate(nodes)
        if isinstance(node, dict) and str(node.get("role") or "") == "rally_follower"
    ]
    raw_pattern = rally_cfg_raw.get("target_pattern", "TRIANGLE")
    pattern = _formation_pattern_from_config(raw_pattern)
    return RallyTaskInitS(
        looseScale=float(rally_cfg_raw.get("loose_scale", 3.0)),
        convergenceRadius_m=float(rally_cfg_raw.get("convergence_radius_m", 5.0)),
        stableHold_s=float(rally_cfg_raw.get("stable_hold_s", 5.0)),
        compressTime_s=float(rally_cfg_raw.get("compress_time_s", 30.0)),
        tightRadius_m=float(rally_cfg_raw.get("tight_radius_m", 2.0)),
        expectedFollowerIds=expected_ids,
        staleTimeout_s=float(rally_cfg_raw.get("stale_timeout_s", 2.0)),
        targetPattern=pattern,
        dt_s=algorithm_period_s,
        loiter_radius_m=float(rally_cfg_raw.get("loiter_radius_m", 200.0)),
        arrival_radius_m=float(rally_cfg_raw.get("arrival_radius_m", 100.0)),
        last_arrival_threshold_s=float(rally_cfg_raw.get("last_arrival_threshold_s", 5.0)),
        mission_heading_deg=float(rally_cfg_raw.get("mission_heading_deg", 0.0)),
        catchup_radius_m=float(rally_cfg_raw.get("catchup_radius_m", 200.0)),
        catchup_kp_speed=float(rally_cfg_raw.get("catchup_kp_speed", 0.05)),
    )


def _build_rally_approach_speed(config: dict[str, object] | None) -> float:
    """从 rally_cfg.approach_speed_mps 读取僚机集结接近速度。注意：未配置时保持 EntityInitS 默认 20 m/s。"""
    # approach_speed_mps 是僚机本地 APPROACH 速度，不属于长机 Rally 状态机门控。
    # 因此它不放进 RallyTaskInitS，避免长机任务单元携带僚机控制细节。
    # 控制器在装配 RallyFollowerEntity 时单独把该值写入 EntityInitS。
    # 旧配置没有该字段时沿用 20 m/s，保持既有 demo 行为。
    # 若配置了负值，加载阶段直接失败，避免到实体初始化时才报模块初始化错误。
    rally_cfg_raw = (config or {}).get("rally_cfg")
    if rally_cfg_raw is None:
        return 20.0
    if not isinstance(rally_cfg_raw, dict):
        raise ValueError("rally_cfg must be an object")
    speed = float(rally_cfg_raw.get("approach_speed_mps", 20.0))
    if speed < 0.0:
        raise ValueError("rally_cfg.approach_speed_mps must be >= 0")
    return speed


class _ConfigLoader:
    """控制器首版使用的轻量 JSON/YAML 加载器。注意：YAML 依赖缺失时只支持 JSON。"""

    def load(self, path: str) -> dict[str, object]:
        """加载控制器配置并构造运行所需对象。注意：重复加载会覆盖当前场景。"""
        config_path = Path(path)
        if not config_path.exists():
            raise FileNotFoundError(path)
        text = config_path.read_text(encoding="utf-8")
        # 按扩展名选择解析器：JSON 内建，YAML 需可选依赖 PyYAML。
        if config_path.suffix.lower() == ".json":
            data = json.loads(text)
        elif config_path.suffix.lower() in {".yaml", ".yml"}:
            try:
                import yaml
            except ImportError as exc:  # pragma: no cover - 依赖运行环境
                raise ValueError("YAML config requires PyYAML") from exc
            data = yaml.safe_load(text)
        else:
            raise ValueError("config must be .json, .yaml, or .yml")
        # 根必须是对象；解析后立即做结构校验再返回副本。
        if not isinstance(data, dict):
            raise ValueError("config root must be an object")
        self.validate(data)
        return dict(data)

    def validate(self, config: dict[str, object]) -> None:
        """校验配置结构和关键字段。注意：这里只做控制器需要的基础校验。"""
        # 核心时序参数取值范围校验：时长/步长为正，倍率落在允许范围内。
        duration_s = float(config.get("duration_s", 120.0))
        step_s = float(config.get("step_s", 0.005))
        playback_rate = float(config.get("playback_rate", 1.0))
        algorithm_decimation = config.get("algorithm_decimation", _DEFAULT_ALGORITHM_DECIMATION)
        if duration_s <= 0:
            raise ValueError("duration_s must be positive")
        if step_s <= 0:
            raise ValueError("step_s must be positive")
        if not _MIN_PLAYBACK_RATE <= playback_rate <= _MAX_PLAYBACK_RATE:
            raise ValueError(f"playback_rate must be in [{_MIN_PLAYBACK_RATE}, {_MAX_PLAYBACK_RATE}]")
        if (
            isinstance(algorithm_decimation, bool)
            or not isinstance(algorithm_decimation, int)
            or algorithm_decimation <= 0
        ):
            raise ValueError("algorithm_decimation must be a positive integer")
        nodes = config.get("nodes", [])
        links = config.get("links", [])
        model = config.get("model", {})
        if nodes is not None and not isinstance(nodes, list):
            raise ValueError("nodes must be a list")
        if links is not None and not isinstance(links, list):
            raise ValueError("links must be a list")
        # 复用构造函数做深层校验：航线、编队/通信、模型配置任一非法都会在此抛错。
        _build_leader_route(config)
        _build_rally_route(config)
        _build_formation_comm_init(list(nodes or []), list(links or []), config)
        _build_vel_cmd_limit(config)
        step_s_v = float(config.get("step_s", 0.005))
        decimation_v = int(config.get("algorithm_decimation", _DEFAULT_ALGORITHM_DECIMATION))
        _build_rally_task_init(config, step_s_v * decimation_v, list(nodes or []))
        _build_rally_approach_speed(config)
        ModelIterator._parse_model_config(model)



class _NodeAlgorithm:
    """把可移植编队实体 API 适配到 SimulationController。注意：负责端口数据转换。"""

    def __init__(
        self,
        node_id: str,
        role: str,
        comm_init: FormCommInitS,
        initial_leader_state: MotionProfS | None,
        leader_route: list[WayPointInputS] | None,
        control_period_s: float,
        vel_cmd_limit: VelCmdLimitS | None = None,
        rally_route: list[WayPointInputS] | None = None,
        rally_cfg: object | None = None,
        rally_target: PosInEarthS | None = None,
        rally_leader_id: str = "",
        rally_approach_speed_mps: float = 20.0,
    ) -> None:
        """初始化 _NodeAlgorithm 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        self._node_id = node_id
        self._role = role
        # 标记长机是否已执行过算法步：未跑前 current_route 回退到航线首段。
        self._has_route_step = False
        # 集结角色从 RALLY 开始；其余角色（leader/wingman）默认 HOLD。
        self._remote_stage = FormStageE.RALLY if role in {"rally_leader", "rally_follower"} else FormStageE.HOLD
        self._initial_remote_stage = self._remote_stage
        # 按角色选择编队实体。
        if role == "leader":
            self._entity: EntityBase = LeaderEntity()
        elif role == "rally_leader":
            self._entity = RallyLeaderEntity()
        elif role == "rally_follower":
            self._entity = RallyFollowerEntity()
        else:
            self._entity = FollowerEntity()
        self._entity.init(
            EntityInitS(
                selfInit=FormSelfInitS(node_id),
                commInit=comm_init,
                route=leader_route or [],
                control_period_s=control_period_s,
                velCmdLimit=vel_cmd_limit or VelCmdLimitS(),
                rally_route=rally_route,
                rally_cfg=rally_cfg,
                rally_target=rally_target,
                rally_leader_id=rally_leader_id,
                rally_approach_speed_mps=rally_approach_speed_mps,
            )
        )
        # 保存长机初始航线（内部 WayLineS），供首步前 current_route() 回退显示。
        self._initial_route_lines: list[WayLineS] = []
        if role in {"leader", "rally_leader"}:
            for _attr in ("_tra_plan", "_tra_plan_mission"):
                tra_plan = getattr(self._entity, _attr, None)
                if tra_plan is not None and hasattr(tra_plan, "get_route"):
                    self._initial_route_lines = tra_plan.get_route()
                    break
        # 僚机预置：直接进入 HOLD/三角队形并写入长机初态，避免冷启动时无参考。
        if role not in {"leader", "rally_leader", "rally_follower"} and initial_leader_state is not None and hasattr(self._entity, "cxt"):
            self._entity.cxt.cmd.stage = FormStageE.HOLD  # type: ignore[attr-defined]
            self._entity.cxt.cmd.pattern = FormPatE.TRIANGLE  # type: ignore[attr-defined]
            copy_motion(initial_leader_state, self._entity.cxt.leaderState)  # type: ignore[attr-defined]

    def step(
        self,
        state: AircraftState,
        inbox: list[MessageEnvelope],
        time_s: float,
        health: str = "normal",
    ) -> _NodeAlgorithmOutput:
        """推进 _NodeAlgorithm 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        entity_output = EntityOutputS()
        self._entity.step(
            EntityInputS(
                selfState=_motion_from_aircraft_state(state),
                inbox=inbox,
                remote=RemoteCmdS(self._remote_stage),
                now_s=time_s,
            ),
            entity_output,
        )
        # 长机（包含集结长机）一旦跑过即标记，使 current_route() 从上下文取实时值。
        if self._role in {"leader", "rally_leader"}:
            self._has_route_step = True
        # 集结完成时自动切换为 HOLD，防止重复触发完成流程。
        formation_analysis = entity_output.formationAnalysis
        if formation_analysis is not None:
            self._remote_stage = FormStageE.HOLD
        # 优先用输出加速度，缺省回退到实体上下文中的加速度。
        acc_cmd = entity_output.selfAccCmd or self._entity.cxt.selfAccCmd  # type: ignore[attr-defined]
        control = AccelerationCommand(
            acc_cmd.accEast,
            acc_cmd.accNorth,
            acc_cmd.accUp,
        )
        # 给待发消息打上当前仿真时间戳，供接收端做时延/时序判断。
        outbox = [
            replace(message, timestamp=time_s)
            for message in entity_output.outbox
        ]
        # 节点非健康时上报"重构"，否则"组队"，供控制回报聚合。
        status = "reconfiguring" if health != "normal" else "forming"
        control_diag = entity_output.controlDiag or PosTrackDiagS()
        return _NodeAlgorithmOutput(control, outbox, status, control_diag, formation_analysis)

    def reset(self) -> None:
        """复位 _NodeAlgorithm 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        self._has_route_step = False
        self._remote_stage = self._initial_remote_stage
        self._entity.reset()
        return None

    def close(self) -> None:
        """释放 _NodeAlgorithm 持有的资源。注意：关闭后不应继续调用运行接口。"""
        self._entity.close()

    def current_stage(self) -> FormStageE:
        """读取当前编队阶段。注意：返回值用于 GUI 回报显示。"""
        # 实体无上下文时视为无阶段（NONE）。
        cxt = getattr(self._entity, "cxt", None)
        if cxt is None:
            return FormStageE.NONE
        return FormStageE(cxt.cmd.stage)

    def current_rally_phase_str(self) -> str:
        """返回人类可读的集结阶段字符串，JOINING 阶段含本机汇合状态。"""
        cxt = getattr(self._entity, "cxt", None)
        if cxt is None:
            return ""
        stage = cxt.cmd.stage
        step = cxt.cmd.step
        if stage == FormStageE.RALLY:
            _step_names = {0: "JOINING", 1: "CATCHUP", 2: "LOOSE", 3: "COMPRESS"}
            phase = _step_names.get(step, f"STEP{step}")
            if step == 0:
                rally_join = getattr(self._entity, "_rally_join", None)
                join_state = getattr(rally_join, "state", "") if rally_join is not None else ""
                _join_abbr = {"FLYING": "FLY", "LOITERING": "LOIT", "EXITED": "EXIT"}
                if join_state:
                    phase = f"JN·{_join_abbr.get(join_state, join_state)}"
            return phase
        if stage == FormStageE.HOLD:
            return "HOLD"
        return ""

    def current_route(self) -> WayLineS | None:
        """读取当前航线状态。注意：返回副本避免外部改写内部状态。"""
        cxt = getattr(self._entity, "cxt", None)
        # 仅长机（含集结长机）持有航线；无上下文或非长机返回 None。
        if cxt is None or self._role not in {"leader", "rally_leader"}:
            return None
        # 算法尚未跑过时上下文 wayLine 未初始化，回退到航线首段用于初始显示。
        if not self._has_route_step and self._initial_route_lines:
            return self._initial_route_lines[0]
        return cxt.wayLine


class _DisturbanceEngine:
    """动态扰动执行器。注意：当前实现覆盖风场、节点故障和链路扰动。"""

    def __init__(self) -> None:
        """初始化 _DisturbanceEngine 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        # 活跃扰动列表，元素为 (命令, 到期时刻)。
        self._active: list[tuple[DisturbanceCommand, float]] = []
        self._model: ModelIterator | None = None
        self._comm: CommunicationChannel | None = None
        self._node_health: dict[str, str] = {}  # 运行期节点健康（被扰动修改）。
        self._baseline_health: dict[str, str] = {}  # 健康基线，清除扰动时恢复目标。
        self._faulted_links: set[str] = set()  # 被中断链路集合，便于恢复。
        self._degraded_links: dict[str, float] = {}  # 降级链路 -> 原始丢包率，便于回填。

    def init(
        self,
        config: dict[str, object],
        seed: int,
        model: ModelIterator,
        comm: CommunicationChannel,
    ) -> None:
        """按配置初始化 _DisturbanceEngine。注意：调用方需先准备好必要依赖和输入数据。"""
        del seed
        self._active = []
        self._faulted_links = set()
        self._degraded_links = {}
        self._model = model
        self._comm = comm
        nodes = config.get("nodes") or []
        # 从配置记录各节点基线健康，作为扰动清除后的恢复目标。
        self._baseline_health = {
            node_id_from_config(node, i): str(node.get("health", "normal"))
            for i, node in enumerate(nodes)
            if isinstance(node, dict)
        }
        # 当前健康初始等于基线（副本，运行期被扰动修改）。
        self._node_health = dict(self._baseline_health)

    def read_health(self) -> dict[str, str]:
        """读取扰动模块健康状态。注意：用于状态表和回报显示。"""
        return dict(self._node_health)

    def inject(self, command: DisturbanceCommand, current_time_s: float) -> SimulationEvent:
        """注入扰动命令。注意：扰动类型和目标由命令字段决定。"""
        # clear 命令撤销全部已注入扰动并复位受影响子系统。
        if command.type == "clear":
            self.clear()
            return SimulationEvent(current_time_s, "INFO", "Disturbance", "清除扰动")
        # 其余扰动登记到活跃表（带到期时刻）并立即生效。
        until_s = current_time_s + float(command.duration_s or 0.0)
        self._active.append((command, until_s))
        self._apply(command, until_s)
        return SimulationEvent(current_time_s, "INFO", "Disturbance", f"注入扰动: {command.type}")

    def tick(self, time_s: float, dt_s: float) -> list[SimulationEvent]:
        """推进模块内部时钟或动态状态一个周期。注意：调用频率应与仿真步长一致。"""
        del dt_s
        events: list[SimulationEvent] = []
        remaining: list[tuple[DisturbanceCommand, float]] = []
        had_expiry = False
        # 扫描活跃扰动，超过到期时刻的剔除并生成"扰动结束"事件。
        for command, until_s in self._active:
            if time_s > until_s:
                events.append(SimulationEvent(time_s, "INFO", "Disturbance", f"扰动结束: {command.type}"))
                had_expiry = True
                continue
            remaining.append((command, until_s))
        self._active = remaining
        # 有扰动到期时，先清空所有动态影响，再重放仍活跃的扰动——
        # 这样能正确撤销过期项，又不误伤共享同一资源的未过期项。
        if had_expiry:
            self._clear_dynamic_effects()
            for command, until_s in self._active:
                self._apply(command, until_s)
        return events

    def clear(self) -> None:
        """清除动态扰动。注意：只撤销扰动影响，不重置仿真时间。"""
        self._active = []
        self._clear_dynamic_effects()

    def _apply(self, command: DisturbanceCommand, until_s: float) -> None:
        """把扰动命令分发到对应模型或通信模块。注意：新增扰动类型需同步扩展。"""
        # wind：交给模型施加风场扰动。
        if command.type == "wind" and self._model is not None:
            self._model.inject_wind(command)
        # node_fault：把目标节点健康置为给定模式（默认 degraded），影响算法状态判定。
        elif command.type == "node_fault":
            target = str(ModelIterator._command_value(command, "target") or "")
            if target in self._node_health:
                params = ModelIterator._command_params(command)
                self._node_health[target] = str(params.get("mode", "degraded"))
        # link_fault：使目标链路中断；记录到 faulted 集合以便后续恢复。
        elif command.type == "link_fault" and self._comm is not None:
            link_id = str(command.target or "")
            if link_id:
                try:
                    self._comm.inject_link_fault(link_id, "lost")
                    self._faulted_links.add(link_id)
                except (KeyError, ValueError):
                    pass
        # link_loss：临时抬高目标链路丢包率；先保存原始丢包率以便清除时回填。
        elif command.type == "link_loss" and self._comm is not None:
            link_id = str(command.target or "")
            # 同一链路已降级则不重复处理，避免覆盖已保存的原始值。
            if link_id and link_id not in self._degraded_links:
                params = ModelIterator._command_params(command)
                rate_raw = params.get("loss_rate", 1.0)
                try:
                    # 非法/布尔丢包率退化为 1.0（完全丢包）。
                    rate = float(rate_raw) if isinstance(rate_raw, (int, float)) and not isinstance(rate_raw, bool) else 1.0
                    states = {s.link_id: s for s in self._comm.read_link_states()}
                    original = states[link_id].loss_rate if link_id in states else 0.0
                    self._comm.inject_link_qos(link_id, latency_ms=None, loss_rate=rate)
                    self._degraded_links[link_id] = original
                except (KeyError, ValueError):
                    pass

    def _clear_dynamic_effects(self) -> None:
        """清除已注入的动态影响。注意：需要同时处理模型和通信两类扰动。"""
        # 撤风、把节点健康恢复到基线。
        if self._model is not None:
            self._model.clear_wind()
        self._node_health = dict(self._baseline_health)
        if self._comm is not None:
            # 恢复曾被中断的链路。
            for link_id in self._faulted_links:
                try:
                    self._comm.inject_link_fault(link_id, "normal")
                except (KeyError, ValueError):
                    pass
            # 把降级链路的丢包率回填为注入前的原始值。
            for link_id, original_rate in self._degraded_links.items():
                try:
                    self._comm.inject_link_qos(link_id, latency_ms=None, loss_rate=original_rate)
                except (KeyError, ValueError):
                    pass
        # 清空跟踪集合，标记动态影响已全部撤销。
        self._faulted_links = set()
        self._degraded_links = {}

    def reset(self) -> None:
        """复位 _DisturbanceEngine 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        self.clear()

    def close(self) -> None:
        """释放 _DisturbanceEngine 持有的资源。注意：关闭后不应继续调用运行接口。"""
        self._active = []
        self._faulted_links = set()
        self._degraded_links = {}
        self._model = None
        self._comm = None
        self._node_health.clear()
        self._baseline_health.clear()


class _DataLogger:
    """关键数据日志记录器。注意：同时保留内存副本并写入 JSONL 文件。"""

    _TIME_KEYS = {"time_s", "duration_s", "step_s"}
    _SNAPSHOT_OMIT_KEYS = {"step_s", "route", "route_segments"}
    _LOAD_FACTOR_KEYS = {"nx", "nz"}
    _ANGLE_SUFFIXES = ("_deg", "_deg_s")
    _ACCELERATION_SUFFIXES = ("_mps2", "_mps3")
    _SPEED_SUFFIXES = ("_mps",)
    _POSITION_SUFFIXES = ("_m",)

    def __init__(self) -> None:
        """初始化 _DataLogger 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        self.snapshots: list[SimulationSnapshot] = []
        self.events: list[SimulationEvent] = []
        self.opened = False
        self.run_dir: Path | None = None
        self._snapshot_file = None
        self._event_file = None
        self._file_logging_disabled = False
        self.last_error_message = ""

    def reset(self) -> None:
        """重置日志记录器状态。注意：只清当前运行，不创建文件目录。"""
        self.close()
        self.snapshots.clear()
        self.events.clear()
        self.run_dir = None
        self._file_logging_disabled = False
        self.last_error_message = ""

    def open(self, run_id: str, config: dict[str, object]) -> bool:
        """打开数据记录器资源。注意：文件打开失败时返回 False 而不打断仿真。"""
        if self.opened:
            return True
        if self._file_logging_disabled:
            return False
        try:
            self.run_dir = self._make_run_dir(run_id)
            self.run_dir.mkdir(parents=True, exist_ok=False)
            (self.run_dir / "config.json").write_text(
                json.dumps(config, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            # 使用行缓冲，仿真中断时也尽量保留已记录数据。
            self._snapshot_file = (self.run_dir / "snapshots.jsonl").open("w", encoding="utf-8", buffering=1)
            self._event_file = (self.run_dir / "events.jsonl").open("w", encoding="utf-8", buffering=1)
            for event in self.events:
                self._event_file.write(json.dumps(self._serialize_record(asdict(event)), ensure_ascii=False) + "\n")
        except OSError as exc:
            self._disable_file_logging(exc)
            return False
        self.opened = True
        return True

    def write_snapshot(self, snapshot: SimulationSnapshot) -> bool:
        """写入一帧仿真快照。注意：文件失败返回 False，内存记录仍保留。"""
        self.snapshots.append(snapshot)
        if self._snapshot_file is not None:
            record = self._serialize_record(asdict(snapshot), omit_keys=self._SNAPSHOT_OMIT_KEYS)
            try:
                self._snapshot_file.write(json.dumps(record, ensure_ascii=False) + "\n")
            except OSError as exc:
                self._disable_file_logging(exc)
                return False
        return True

    def write_event(self, event: SimulationEvent) -> bool:
        """写入一条仿真事件。注意：文件失败返回 False，内存记录仍保留。"""
        self.events.append(event)
        if self._event_file is not None:
            try:
                self._event_file.write(json.dumps(self._serialize_record(asdict(event)), ensure_ascii=False) + "\n")
            except OSError as exc:
                self._disable_file_logging(exc)
                return False
        return True

    def flush(self) -> None:
        """刷新记录缓冲。注意：频繁调用会增加 IO 开销。"""
        for handle in (self._snapshot_file, self._event_file):
            if handle is not None:
                try:
                    handle.flush()
                except OSError as exc:
                    self._disable_file_logging(exc)
                    break

    def close(self) -> None:
        """释放 _DataLogger 持有的资源。注意：关闭后不应继续调用运行接口。"""
        for handle in (self._snapshot_file, self._event_file):
            if handle is not None:
                handle.close()
        self._snapshot_file = None
        self._event_file = None
        self.opened = False

    def _disable_file_logging(self, exc: OSError) -> None:
        """停用当前运行的文件落盘。注意：调用方负责把错误转为 WARN 事件。"""
        self.last_error_message = str(exc)
        for handle in (self._snapshot_file, self._event_file):
            if handle is not None:
                try:
                    handle.close()
                except OSError:
                    pass
        self._snapshot_file = None
        self._event_file = None
        self.opened = False
        self._file_logging_disabled = True

    @staticmethod
    def _make_run_dir(run_id: str) -> Path:
        """生成不冲突的运行日志目录。注意：同一秒多次启动会自动加序号。"""
        base = Path("logs") / run_id
        if not base.exists():
            return base
        index = 1
        while True:
            candidate = Path("logs") / f"{run_id}-{index}"
            if not candidate.exists():
                return candidate
            index += 1

    @classmethod
    def _serialize_record(cls, record: dict[str, Any], *, omit_keys: set[str] | None = None) -> dict[str, Any]:
        """按日志精度规则序列化记录。注意：只改变落盘值，不改内存快照。"""
        ignored = omit_keys or set()
        return {key: cls._round_log_value(key, value) for key, value in record.items() if key not in ignored}

    @classmethod
    def _round_log_value(cls, key: str, value: Any) -> Any:
        """按字段语义四舍五入日志值。注意：嵌套列表和字典递归处理。"""
        if isinstance(value, dict):
            return cls._serialize_record(value)
        if isinstance(value, list):
            return [cls._round_log_value(key, item) for item in value]
        if not isinstance(value, float) or not math.isfinite(value):
            return value
        decimals = cls._decimals_for_key(key)
        if decimals is None:
            return value
        quant = Decimal("1").scaleb(-decimals)
        return float(Decimal(str(value)).quantize(quant, rounding=ROUND_HALF_UP))

    @classmethod
    def _decimals_for_key(cls, key: str) -> int | None:
        """返回日志字段小数位规则。注意：未知物理量保持原始精度。"""
        if key in cls._TIME_KEYS:
            return 3
        if key in cls._LOAD_FACTOR_KEYS:
            return 4
        if key.endswith(cls._ACCELERATION_SUFFIXES):
            return 3
        if key.endswith(cls._ANGLE_SUFFIXES):
            return 2
        if key.endswith(cls._SPEED_SUFFIXES) or key.endswith(cls._POSITION_SUFFIXES):
            return 2
        return None


class SimulationController:
    """顶层仿真编排门面。注意：对 GUI/CLI 暴露统一控制接口。"""

    _EVENT_BUFFER_SIZE = 1000
    _DISPLAY_REFRESH_S = 0.1

    def __init__(self) -> None:
        """初始化 SimulationController 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        self._lock = threading.RLock()
        self._config_loader = _ConfigLoader()
        self._model = ModelIterator()
        self._comm = CommunicationChannel()
        self._disturbance = _DisturbanceEngine()
        self._logger = _DataLogger()
        self._node_algorithms: dict[str, _NodeAlgorithm] = {}
        self._node_roles: dict[str, str] = {}
        self._configured_links: list[_ConfiguredLink] = []
        self._leader_route: list[WayPointInputS] | None = None
        self._display_route: list[WayLineS] | None = None  # 显示用航线(WayLineS)，仅供 GUI 画航段
        # 避障”采用”的长机航线覆盖：非 None 时替换配置生成的长机航线（reset 保留，load_config 清除）。
        self._leader_route_override: list[WayPointInputS] | None = None
        self._formation_completed_analysis: object | None = None  # FormationAnalysisS；集结完成后锁存
        self._current_controls: dict[str, AccelerationCommand] = {}
        self._control_diagnostics: dict[str, PosTrackDiagS] = {}
        self._config: dict[str, object] | None = None
        self._seed = 0
        self._duration_s = 0.0
        self._step_s = 0.005
        self._time_s = 0.0
        self._tick_index = 0
        self._next_log_sample_time_s = _LOG_SAMPLE_PERIOD_S
        self._playback_rate = 1.0
        self._cpu_utilization = 0.0
        self._algorithm_decimation = _DEFAULT_ALGORITHM_DECIMATION
        self._algorithm_period_s = self._step_s * self._algorithm_decimation
        self._run_state: RunState = "UNLOADED"
        self._control_report: ControlReport = "待命"
        self._latest_snapshot = self._make_snapshot_for_empty_controller()
        self._events: deque[SimulationEvent] = deque(maxlen=self._EVENT_BUFFER_SIZE)
        self._subscribers: dict[int, Callable[[SimulationSnapshot], None]] = {}
        self._subscriber_ids_by_callback: dict[Callable[[SimulationSnapshot], None], int] = {}
        self._next_subscription_id = 1
        self._last_display_wall_s = 0.0
        self._worker: threading.Thread | None = None
        self._stop_requested = threading.Event()
        self._closed = False

    @property
    def playback_rate(self) -> float:
        """返回当前播放倍率。注意：只反映墙钟调度倍率，不改变仿真步长。"""

        with self._lock:
            return self._playback_rate

    def load_config(self, path: str) -> CommandResult:
        """读取并解析仿真配置文件。注意：文件路径由调用方保证存在且可读。"""

        # 先做轻量前置校验：已关闭或运行中不允许加载新配置。
        with self._lock:
            if self._closed:
                return CommandResult("ERR_INVALID_STATE", "controller is closed")
            if self._run_state == "RUNNING":
                return CommandResult("ERR_BUSY", "pause or reset before loading a new config")
        # 文件读取与解析放在锁外（可能耗时 IO），按异常类型映射结果码。
        try:
            config = self._config_loader.load(path)
        except FileNotFoundError:
            return CommandResult("ERR_CONFIG_NOT_FOUND", f"config not found: {path}")
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            return CommandResult("ERR_CONFIG_INVALID", str(exc))

        # 再次持锁并复检状态（IO 期间状态可能变化），随后初始化模块。
        with self._lock:
            if self._closed:
                return CommandResult("ERR_INVALID_STATE", "controller is closed")
            if self._run_state == "RUNNING":
                return CommandResult("ERR_BUSY", "pause or reset before loading a new config")
            # 新配置：清除上一个配置遗留的避障航线覆盖，回到该配置的原始长机航线。
            self._leader_route_override = None
            try:
                self._init_modules_unlocked(config)
            except Exception as exc:  # noqa: BLE001 - 首版统一映射模块初始化失败
                return CommandResult("ERR_MODULE_INIT_FAILED", str(exc))
            # 加载成功转入 READY/待命，准备 start。
            self._run_state = "READY"
            self._control_report = "待命"
            self._latest_snapshot = self._make_snapshot_unlocked()
            self._append_event_unlocked("INFO", "SimControl", f"配置已加载: {path}")
            snapshot = self._latest_snapshot
        self._notify_subscribers(snapshot)
        return CommandResult("OK", "config loaded")

    def get_snapshot(self) -> SimulationSnapshot:
        """获取当前仿真快照。注意：该操作不推进仿真时间。"""

        with self._lock:
            if self._config is not None and self._run_state == "RUNNING":
                # 显式查询应返回当前状态；调用频率由 UI 计时器或外部调用方控制。
                self._latest_snapshot = self._make_snapshot_unlocked()
            return self._latest_snapshot

    def start(self) -> CommandResult:
        """启动或继续 SimulationController 的运行流程。注意：重复调用应保持状态一致。"""

        should_stop_worker = False
        # 第一段持锁：做状态前置校验，并判断是否需要先回收残留旧线程。
        with self._lock:
            if self._closed:
                return CommandResult("ERR_INVALID_STATE", "controller is closed")
            if self._config is None:
                return CommandResult("ERR_NO_CONFIG", "load config before start")
            # 已结束必须先 reset 才能重跑；运行中重复 start 视为幂等成功。
            if self._run_state == "FINISHED":
                return CommandResult("ERR_INVALID_STATE", "reset before restarting")
            if self._run_state == "RUNNING":
                return CommandResult("OK", "already running")
            should_stop_worker = self._worker is not None and self._worker.is_alive()

        # 停线程需阻塞 join，必须在锁外做，避免与 _run_loop 持锁互锁。
        if should_stop_worker:
            self._stop_worker()

        # 第二段持锁：释锁期间状态可能变化，重新校验后再真正启动。
        with self._lock:
            if self._closed:
                return CommandResult("ERR_INVALID_STATE", "controller is closed")
            if self._config is None:
                return CommandResult("ERR_NO_CONFIG", "load config before start")
            if self._run_state == "FINISHED":
                return CommandResult("ERR_INVALID_STATE", "reset before restarting")
            if self._run_state == "RUNNING":
                return CommandResult("OK", "already running")
            # 切到运行态，清停止标志并拉起后台线程开始自动推进。
            self._run_state = "RUNNING"
            self._control_report = self._derive_control_report_unlocked()
            self._cpu_utilization = 0.0
            self._stop_requested.clear()
            self._start_worker_unlocked()
            self._latest_snapshot = self._make_snapshot_unlocked()
            snapshot = self._latest_snapshot
        self._notify_subscribers(snapshot)
        return CommandResult("OK", "started")

    def pause(self) -> CommandResult:
        """暂停 SimulationController 的运行流程。注意：只暂停调度，不清空当前状态。"""

        with self._lock:
            # 运行->暂停：仅改状态与回报，不动模型数据，便于随后 step 或继续。
            if self._run_state == "RUNNING":
                self._run_state = "PAUSED"
                self._control_report = "保持"
                self._cpu_utilization = 0.0
                self._latest_snapshot = self._make_snapshot_unlocked()
                snapshot = self._latest_snapshot
            elif self._run_state == "PAUSED":
                # 重复暂停幂等返回成功。
                return CommandResult("OK", "already paused")
            else:
                # READY/FINISHED/UNLOADED 下暂停无意义，报状态错误。
                return CommandResult("ERR_INVALID_STATE", "pause requires RUNNING or PAUSED")
        # 注意：后台线程在下一圈检测到非 RUNNING 会自行退出，这里不显式停线程。
        self._notify_subscribers(snapshot)
        return CommandResult("OK", "paused")

    def step(self, count: int = 1) -> CommandResult:
        """推进 SimulationController 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""

        if count < 1:
            return CommandResult("ERR_INVALID_ARGUMENT", "count must be >= 1")
        snapshots_to_notify: list[SimulationSnapshot] = []
        with self._lock:
            if self._config is None:
                return CommandResult("ERR_NO_CONFIG", "load config before step")
            # 单步只在非自动运行时允许：RUNNING 下须先 pause，FINISHED 下须先 reset。
            if self._run_state == "RUNNING":
                return CommandResult("ERR_INVALID_STATE", "pause before manual step")
            if self._run_state == "FINISHED":
                return CommandResult("ERR_INVALID_STATE", "reset before stepping")
            # 单步语义即"暂停态下手动推进 count 个 tick"。
            self._run_state = "PAUSED"
            self._control_report = "保持"
            for _ in range(count):
                try:
                    # force_snapshot 保证每个手动步都产出快照，便于逐帧观察。
                    snapshot = self._tick_unlocked(force_snapshot=True)
                except Exception as exc:  # noqa: BLE001
                    self._append_event_unlocked("ERROR", "SimControl", f"tick failed: {exc}")
                    return CommandResult("ERR_TICK_FAILED", str(exc))
                if snapshot is not None:
                    snapshots_to_notify.append(snapshot)
                # 中途到达总时长即停止剩余步进。
                if self._run_state == "FINISHED":
                    break
            # 若全程无新快照，至少回传一帧最近快照以刷新 UI。
            if not snapshots_to_notify:
                snapshots_to_notify.append(self._latest_snapshot)
        for snapshot in snapshots_to_notify:
            self._notify_subscribers(snapshot)
        return CommandResult("OK", "stepped")

    def reset(self) -> CommandResult:
        """复位 SimulationController 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""

        with self._lock:
            if self._config is None:
                return CommandResult("ERR_NO_CONFIG", "load config before reset")
            # 取出当前配置副本，重置=用同一配置重新初始化所有模块（时间归零）。
            config = dict(self._config)
        # 先停后台线程（锁外），再持锁重建模块，避免线程与重建竞争。
        self._stop_worker()
        with self._lock:
            try:
                self._init_modules_unlocked(config)
            except Exception as exc:  # noqa: BLE001
                return CommandResult("ERR_MODULE_INIT_FAILED", str(exc))
            # 重置后回到 READY/待命，等待再次 start。
            self._run_state = "READY"
            self._control_report = "待命"
            self._latest_snapshot = self._make_snapshot_unlocked()
            self._append_event_unlocked("INFO", "SimControl", "仿真已重置")
            snapshot = self._latest_snapshot
        self._notify_subscribers(snapshot)
        return CommandResult("OK", "reset")

    def apply_avoidance_route(self, route: list[WayPointInputS]) -> CommandResult:
        """采用一条避障规划航线，替换长机航线并重置到 READY。注意：运行中需先暂停。"""
        if not isinstance(route, list) or len(route) < 2:
            return CommandResult("ERR_CONFIG_INVALID", "avoidance route must be a list of at least 2 WayPointInputS")
        with self._lock:
            if self._closed:
                return CommandResult("ERR_INVALID_STATE", "controller is closed")
            if self._config is None:
                return CommandResult("ERR_NO_CONFIG", "load config before applying a route")
            if self._run_state == "RUNNING":
                return CommandResult("ERR_BUSY", "pause or reset before applying a route")
            config = dict(self._config)
        # 先停后台线程（锁外），再持锁带覆盖重建模块（时间归零，等价一次 reset）。
        self._stop_worker()
        with self._lock:
            self._leader_route_override = route
            try:
                self._init_modules_unlocked(config)
            except Exception as exc:  # noqa: BLE001
                return CommandResult("ERR_MODULE_INIT_FAILED", str(exc))
            self._run_state = "READY"
            self._control_report = "待命"
            self._latest_snapshot = self._make_snapshot_unlocked()
            self._append_event_unlocked("INFO", "SimControl", "已采用避障航线")
            snapshot = self._latest_snapshot
        self._notify_subscribers(snapshot)
        return CommandResult("OK", "avoidance route applied")

    def clear_avoidance_route(self) -> CommandResult:
        """清除避障航线覆盖，恢复配置原始长机航线并重置到 READY。"""
        with self._lock:
            if self._closed:
                return CommandResult("ERR_INVALID_STATE", "controller is closed")
            if self._config is None:
                return CommandResult("ERR_NO_CONFIG", "load config first")
            if self._run_state == "RUNNING":
                return CommandResult("ERR_BUSY", "pause or reset before clearing the route")
            if self._leader_route_override is None:
                return CommandResult("OK", "no avoidance route to clear")
            config = dict(self._config)
        self._stop_worker()
        with self._lock:
            self._leader_route_override = None
            try:
                self._init_modules_unlocked(config)
            except Exception as exc:  # noqa: BLE001
                return CommandResult("ERR_MODULE_INIT_FAILED", str(exc))
            self._run_state = "READY"
            self._control_report = "待命"
            self._latest_snapshot = self._make_snapshot_unlocked()
            self._append_event_unlocked("INFO", "SimControl", "已清除避障航线")
            snapshot = self._latest_snapshot
        self._notify_subscribers(snapshot)
        return CommandResult("OK", "avoidance route cleared")

    def close(self) -> None:
        """释放 SimulationController 持有的资源。注意：关闭后不应继续调用运行接口。"""

        # 先停后台线程，再持锁逐个关闭子系统并清空订阅，最后置已关闭标志。
        self._stop_worker()
        with self._lock:
            self._logger.flush()
            self._logger.close()
            self._model.close()
            self._comm.close()
            self._disturbance.close()
            for algorithm in self._node_algorithms.values():
                algorithm.close()
            self._node_algorithms.clear()
            self._subscribers.clear()
            self._subscriber_ids_by_callback.clear()
            # 置位后所有控制接口都将拒绝服务。
            self._closed = True

    def set_playback_rate(self, rate: float) -> CommandResult:
        """设置播放倍率。注意：只影响墙钟调度，不改变仿真步长。"""

        # 倍率限定在允许范围内，仅改墙钟节流，不改仿真步长（结果可复现）。
        if not _MIN_PLAYBACK_RATE <= rate <= _MAX_PLAYBACK_RATE:
            return CommandResult(
                "ERR_INVALID_ARGUMENT",
                f"rate must be in [{_MIN_PLAYBACK_RATE}, {_MAX_PLAYBACK_RATE}]",
            )
        with self._lock:
            self._playback_rate = float(rate)
            if self._config is not None:
                # reset 会用当前配置副本重建模块，需同步运行期倍率避免回退到文件默认值。
                self._config["playback_rate"] = self._playback_rate
        return CommandResult("OK", "playback rate updated")

    def set_duration(self, duration_s: float) -> CommandResult:
        """设置仿真总时长。注意：只允许在未自动运行时修改。"""

        # 总时长必须是正有限值，避免进度条和结束条件进入不可判定状态。
        if not math.isfinite(duration_s) or duration_s <= 0.0:
            return CommandResult("ERR_INVALID_ARGUMENT", "duration_s must be positive")
        with self._lock:
            if self._closed:
                return CommandResult("ERR_INVALID_STATE", "controller is closed")
            if self._config is None:
                return CommandResult("ERR_NO_CONFIG", "load config before setting duration")
            if self._run_state == "RUNNING":
                return CommandResult("ERR_INVALID_STATE", "pause before setting duration")
            if self._run_state == "FINISHED":
                return CommandResult("ERR_INVALID_STATE", "reset before setting duration")
            # 缩短到当前时间之前会制造“时间回退但模型未回滚”的不一致快照，必须拒绝。
            if duration_s + _TIME_EPSILON_S < self._time_s:
                return CommandResult("ERR_INVALID_ARGUMENT", "duration_s must not be before current time")
            self._duration_s = float(duration_s)
            self._config["duration_s"] = self._duration_s
            # 若总时长刚好等于当前时间，应立即按新的边界结束。
            if self._time_s >= self._duration_s:
                self._time_s = self._duration_s
                self._run_state = "FINISHED"
                self._control_report = "保持"
            self._latest_snapshot = self._make_snapshot_unlocked()
        return CommandResult("OK", "duration updated")

    def inject_disturbance(self, command: DisturbanceCommand | dict[str, object]) -> CommandResult:
        """向仿真注入扰动。注意：调用方需提供合法扰动类型和参数。"""

        # 先把 dict/对象统一规范为 DisturbanceCommand，非法参数提前返回。
        try:
            normalized = self._normalize_disturbance(command)
        except (TypeError, ValueError) as exc:
            return CommandResult("ERR_INVALID_ARGUMENT", str(exc))
        with self._lock:
            if self._config is None:
                return CommandResult("ERR_NO_CONFIG", "load config before disturbance")
            # 仿真结束后不再接受扰动（轨迹已定型）。
            if self._run_state == "FINISHED":
                return CommandResult("ERR_INVALID_STATE", "disturbance is not accepted after finish")
            # 以当前仿真时间为基准注入扰动并记录事件。
            event = self._disturbance.inject(normalized, self._time_s)
            self._append_event_object_unlocked(event)
            self._logger.write_event(event)
            self._latest_snapshot = self._make_snapshot_unlocked()
        return CommandResult("OK", "disturbance injected")

    def subscribe_snapshot(self, callback: Callable[[SimulationSnapshot], None]) -> Subscription:
        """订阅快照刷新回调。注意：回调应快速返回，避免阻塞仿真线程。"""

        with self._lock:
            # 同一回调去重：已订阅则复用其 ID，避免重复登记导致多次触发。
            subscription_id = self._subscriber_ids_by_callback.get(callback)
            if subscription_id is None:
                subscription_id = self._next_subscription_id
                self._next_subscription_id += 1
                self._subscribers[subscription_id] = callback
                self._subscriber_ids_by_callback[callback] = subscription_id
            snapshot = self._latest_snapshot

        def unsubscribe() -> None:
            """取消订阅回调。注意：回调不存在时应保持幂等。"""
            with self._lock:
                # 双向映射一并清理，pop 默认值保证重复取消不报错。
                removed = self._subscribers.pop(subscription_id, None)
                if removed is not None:
                    self._subscriber_ids_by_callback.pop(removed, None)

        # 订阅时立即推一帧当前快照，让新订阅者无需等待下一 tick 即可初始化显示。
        try:
            callback(snapshot)
        except Exception as exc:  # noqa: BLE001
            with self._lock:
                self._append_event_unlocked("WARN", "SimControl", f"snapshot callback failed: {exc}")
        return Subscription(unsubscribe)

    def get_recent_events(
        self,
        limit: int = 200,
        min_level: EventLevel | None = None,
    ) -> list[SimulationEvent]:
        """读取最近事件列表。注意：返回副本供 UI 展示。"""

        if limit < 1:
            return []
        # 数值化日志级别用于阈值过滤（仅返回不低于 min_level 的事件）。
        level_order = {"DEBUG": 10, "INFO": 20, "WARN": 30, "ERROR": 40}
        min_value = level_order.get(min_level or "DEBUG", 10)
        with self._lock:
            events = [event for event in self._events if level_order[event.level] >= min_value]
            # 取最近 limit 条（事件队列已按时间追加）。
            return events[-limit:]

    def run_until_complete(self, config: object | str, *, seed: int | None = None) -> CommandResult:
        """同步运行到仿真结束。注意：主要供 CLI 或批处理使用。"""

        # config 可为文件路径（走 load_config）或内联 dict（直接校验+初始化）。
        if isinstance(config, str):
            result = self.load_config(config)
            if result.code != "OK":
                return result
        elif isinstance(config, dict):
            with self._lock:
                config_copy = dict(config)
                # 允许参数 seed 覆盖配置内 seed，便于批量复现实验。
                if seed is not None:
                    config_copy["seed"] = seed
                try:
                    self._config_loader.validate(config_copy)
                    self._init_modules_unlocked(config_copy)
                except Exception as exc:  # noqa: BLE001
                    return CommandResult("ERR_CONFIG_INVALID", str(exc))
                self._run_state = "READY"
                self._latest_snapshot = self._make_snapshot_unlocked()
        else:
            return CommandResult("ERR_INVALID_ARGUMENT", "config must be path or dict")

        # 同步推进：在当前线程持锁连续 tick 直到状态机离开 RUNNING（到时长结束）。
        with self._lock:
            if self._config is None:
                return CommandResult("ERR_NO_CONFIG", "load config before run")
            self._run_state = "RUNNING"
            self._control_report = self._derive_control_report_unlocked()
            while self._run_state == "RUNNING":
                try:
                    # force_snapshot 确保每帧都落日志/快照（批处理需完整轨迹）。
                    self._tick_unlocked(force_snapshot=True)
                except Exception as exc:  # noqa: BLE001
                    self._append_event_unlocked("ERROR", "SimControl", f"tick failed: {exc}")
                    return CommandResult("ERR_TICK_FAILED", str(exc))
        return CommandResult("OK", "finished")

    def _run_loop(self) -> None:
        """后台线程主循环。注意：所有共享状态访问必须受锁保护。"""
        current = threading.current_thread()
        last_wall_s = time.perf_counter()
        stats_start_wall_s = last_wall_s
        stats_busy_s = 0.0
        sim_budget_s = 0.0
        force_first_tick = True
        try:
            # 直到收到停止请求；按累计墙钟时间批量补拍，避免高倍率下依赖亚毫秒 sleep。
            while not self._stop_requested.is_set():
                now_wall_s = time.perf_counter()
                wall_delta_s = max(0.0, now_wall_s - last_wall_s)
                last_wall_s = now_wall_s
                snapshots_to_notify: list[SimulationSnapshot] = []
                cpu_snapshot: SimulationSnapshot | None = None
                should_sleep = False
                with self._lock:
                    # 运行态被外部改为非 RUNNING（暂停/结束）时退出循环。
                    if self._run_state != "RUNNING":
                        break
                    step_s = self._step_s
                    playback_rate = self._playback_rate
                    sim_budget_s += wall_delta_s * playback_rate
                    if force_first_tick and sim_budget_s < step_s:
                        sim_budget_s = step_s
                    force_first_tick = False

                    ticks_due = min(int(sim_budget_s / step_s), _MAX_RUN_LOOP_BATCH_TICKS)
                    if ticks_due <= 0:
                        should_sleep = True
                    for _ in range(ticks_due):
                        try:
                            snapshot = self._tick_unlocked()
                        except Exception as exc:  # noqa: BLE001
                            # tick 出错不崩线程：记录错误、转入暂停并产出一帧快照便于排查。
                            self._append_event_unlocked("ERROR", "SimControl", f"tick failed: {exc}")
                            self._run_state = "PAUSED"
                            snapshot = self._make_snapshot_unlocked()
                        sim_budget_s = max(0.0, sim_budget_s - step_s)
                        if snapshot is not None:
                            snapshots_to_notify.append(snapshot)
                        if self._run_state != "RUNNING":
                            break
                    if self._run_state == "RUNNING" and sim_budget_s < step_s:
                        should_sleep = True
                # 在锁外通知订阅者，避免回调阻塞持锁路径。
                for snapshot in snapshots_to_notify:
                    self._notify_subscribers(snapshot)
                busy_end_s = time.perf_counter()
                stats_busy_s += max(0.0, busy_end_s - now_wall_s)
                stats_wall_s = max(0.0, busy_end_s - stats_start_wall_s)
                if stats_wall_s >= _CPU_UTILIZATION_SAMPLE_PERIOD_S:
                    with self._lock:
                        self._cpu_utilization = min(1.0, max(0.0, stats_busy_s / stats_wall_s))
                        self._latest_snapshot = self._make_snapshot_unlocked()
                        cpu_snapshot = self._latest_snapshot
                    stats_start_wall_s = busy_end_s
                    stats_busy_s = 0.0
                if cpu_snapshot is not None:
                    self._notify_subscribers(cpu_snapshot)
                if should_sleep:
                    time.sleep(_RUN_LOOP_SLEEP_SLICE_S)
        finally:
            with self._lock:
                # 仅当自己仍是登记的工作线程时才清空引用，避免误清新线程。
                if self._worker is current:
                    self._worker = None

    def _stop_worker(self) -> None:
        """停止后台工作线程。注意：调用后需要等待线程退出。"""
        # 置停止标志，循环下一圈即退出。
        self._stop_requested.set()
        worker = self._worker
        # 等待线程真正退出（最多 2s）；但不能 join 自身，否则死锁。
        if worker is not None and worker.is_alive() and worker is not threading.current_thread():
            worker.join(timeout=2.0)
        self._worker = None
        # 清标志，为下次启动复位。
        self._stop_requested.clear()

    def _start_worker_unlocked(self) -> None:
        """在已持锁状态下启动工作线程。注意：调用方必须先持有控制器锁。"""
        # 已有存活线程则不重复创建。
        if self._worker is not None and self._worker.is_alive():
            return
        # 守护线程：主程序退出时不被其阻塞。
        self._worker = threading.Thread(target=self._run_loop, name="SimulationController", daemon=True)
        self._worker.start()

    def _init_modules_unlocked(self, config: dict[str, object]) -> None:
        """在已持锁状态下初始化仿真模块。注意：不得在未加载配置时调用。"""
        # 缓存配置并读取核心运行参数（种子、总时长、步长、倍率）。
        self._config = dict(config)
        self._seed = int(config.get("seed", 0))
        self._duration_s = float(config.get("duration_s", 120.0))
        self._step_s = float(config.get("step_s", 0.005))
        self._playback_rate = float(config.get("playback_rate", 1.0))
        self._algorithm_decimation = int(config.get("algorithm_decimation", _DEFAULT_ALGORITHM_DECIMATION))
        self._algorithm_period_s = self._step_s * self._algorithm_decimation
        # 时间与计数归零，保证每次初始化都是干净起点。
        self._time_s = 0.0
        self._tick_index = 0
        self._next_log_sample_time_s = _LOG_SAMPLE_PERIOD_S
        self._last_display_wall_s = 0.0
        self._cpu_utilization = 0.0
        # 按依赖顺序初始化各子系统：先模型（提供初始状态），再通信，再扰动（依赖前两者）。
        self._model.init(config, self._seed)
        raw_links = list(config.get("links") or [])
        comm_config = {
            "nodes": list(config.get("nodes") or []),
            "links": raw_links,
        }
        self._comm.init(comm_config, self._seed)
        # 保存折叠前的链路声明，供快照阶段合并双向状态。
        self._configured_links = self._parse_configured_links(raw_links)
        # 扰动引擎需持有模型与通信句柄，故最后初始化并注入二者。
        self._disturbance.init(config, self._seed, self._model, self._comm)
        nodes = config.get("nodes") or []
        # 建立 node_id->角色映射；首节点缺省 leader，其余 wingman。
        self._node_roles = {
            node_id_from_config(node, i): str(
                node.get("role") or ("leader" if i == 0 else "wingman")
            )
            for i, node in enumerate(nodes)
            if isinstance(node, dict)
        }
        # 从模型读取各机初始状态，用于构造算法与初始长机运动基准。
        states = self._model.read_states()
        # 由拓扑与队形配置生成编队通信初始化信息（网络连接 + 槽位）。
        formation_comm_init = _build_formation_comm_init(list(nodes), raw_links, config)
        leader_id = _leader_id_from_nodes(list(nodes))
        initial_leader_state = states.get(leader_id)
        # 把长机初始状态转换为算法侧运动表示，供僚机持队参考；无长机则为 None。
        initial_leader_motion = (
            _motion_from_aircraft_state(initial_leader_state)
            if initial_leader_state is not None
            else None
        )
        # 避障”采用”后用覆盖航线替换配置航线；否则按配置生成。
        if self._leader_route_override is not None:
            leader_route = self._leader_route_override
        else:
            leader_route = _build_leader_route(config)
        self._leader_route = leader_route
        # 前向/垂向速度指令限幅(串级 P+PI 外环输出)，由配置注入各节点实体。
        vel_cmd_limit = _build_vel_cmd_limit(config)
        # 显示用航线(list[WayLineS])：只画航段几何，去掉交接半径 r(转弯信息)，与配置航线显示一致。
        if self._leader_route_override is not None:
            _display_wpi = to_display_inputs(self._leader_route_override)
        else:
            _display_wpi = _build_leader_route(config, insert_arcs=False)
        self._display_route = waypoint_inputs_to_waylines(_display_wpi) if len(_display_wpi) >= 2 else None
        # 集结场景额外参数：集结航线、任务配置、每机目标集结点。
        rally_route = _build_rally_route(config)
        rally_task_init = _build_rally_task_init(config, self._algorithm_period_s, list(nodes))
        rally_approach_speed = _build_rally_approach_speed(config)
        self._formation_completed_analysis = None
        rally_leader_id = _leader_id_from_nodes(list(nodes))
        _node_rally_targets: dict[str, PosInEarthS | None] = {
            node_id_from_config(node, i): (
                _route_point_from_config(node["rally_target"], f"nodes[{i}].rally_target")
                if isinstance(node.get("rally_target"), dict)
                else None
            )
            for i, node in enumerate(nodes)
            if isinstance(node, dict)
        }
        # 为每个节点创建算法适配器（角色决定实体类型）。
        self._node_algorithms = {
            node_id: _NodeAlgorithm(
                node_id,
                self._node_roles.get(node_id, "wingman"),
                formation_comm_init,
                initial_leader_motion,
                leader_route,
                self._algorithm_period_s,
                vel_cmd_limit,
                rally_route=rally_route,
                rally_cfg=rally_task_init,
                rally_target=_node_rally_targets.get(node_id),
                rally_leader_id=rally_leader_id,
                rally_approach_speed_mps=rally_approach_speed,
            )
            for node_id in states
        }
        # 控制指令缓存初始化为零加速度，首个算法 tick 前模型保持初值。
        self._current_controls = {
            node_id: AccelerationCommand()
            for node_id in states
        }
        self._control_diagnostics = {
            node_id: PosTrackDiagS()
            for node_id in states
        }
        # 仅重置内存日志；文件目录延迟到首次实际 tick 时创建，避免空 run 目录。
        self._logger.reset()

    def _tick_unlocked(self, *, force_snapshot: bool = False) -> SimulationSnapshot | None:
        """在已持锁状态下推进一个仿真 tick。注意：调用方负责锁和阶段检查。"""
        # 仅在运行/暂停态推进；其他状态直接回最近快照，不产生副作用。
        if self._run_state not in {"RUNNING", "PAUSED"}:
            return self._latest_snapshot
        self._ensure_logger_open_unlocked()
        step_s = self._step_s
        tick_index = self._tick_index
        algorithm_tick = tick_index % self._algorithm_decimation == 0

        # 分频调度：算法链路按配置分频运行，控制频率低于积分频率以降低算力开销。
        if algorithm_tick:
            self._run_formation_algorithms_unlocked()
        # 通信分频推进，传入累计步长以保持时延计时一致。
        if tick_index % _COMM_DECIMATION == 0:
            self._comm.tick(step_s * _COMM_DECIMATION)

        # 先把当前控制指令施加到模型（算法分频更新，未更新的 tick 沿用上次控制）。
        self._model.apply_controls(self._current_controls)
        # 推进扰动引擎并落地其产生的事件（如扰动到期）。
        for event in self._disturbance.tick(self._time_s, step_s):
            self._append_event_object_unlocked(event)
            self._logger.write_event(event)
        # 模型积分一步，随后推进仿真时间；用 min 夹住，确保不越过总时长。
        self._model.step(step_s)
        self._time_s = min(self._duration_s, self._time_s + step_s)
        self._tick_index += 1

        # 状态机收尾：到达总时长则置 FINISHED 并锁定回报；否则在运行态刷新回报文本。
        if self._time_s >= self._duration_s:
            self._run_state = "FINISHED"
            self._control_report = "保持"
        elif self._run_state == "RUNNING":
            self._control_report = self._derive_control_report_unlocked()

        should_refresh_display = self._should_refresh_display_unlocked() or self._run_state == "FINISHED"
        # 日志按仿真时间固定 10Hz 采样，保证不同播放倍率得到一致的离线数据点。
        should_log_snapshot = self._time_s + _TIME_EPSILON_S >= self._next_log_sample_time_s
        # 快照生成按墙钟显示频率限流；日志采样点额外生成，避免漏记关键状态。
        snapshot: SimulationSnapshot | None = None
        if force_snapshot or should_refresh_display or should_log_snapshot:
            self._latest_snapshot = self._make_snapshot_unlocked()
            snapshot = self._latest_snapshot
        # 关键数据定时记录固定 10Hz；若单个 tick 跨过多个采样点，只记录当前最新状态一次。
        if should_log_snapshot and snapshot is not None:
            if not self._logger.write_snapshot(snapshot):
                self._append_event_unlocked("WARN", "DataLogger", f"snapshot log failed: {self._logger.last_error_message}")
            while self._time_s + _TIME_EPSILON_S >= self._next_log_sample_time_s:
                self._next_log_sample_time_s += _LOG_SAMPLE_PERIOD_S
        # 仅当强制产帧、达到显示刷新间隔或仿真结束时才回传快照，否则返回 None 抑制 UI 刷新。
        if force_snapshot or should_refresh_display:
            return self._latest_snapshot
        return None

    def _ensure_logger_open_unlocked(self) -> None:
        """确保当前运行已创建日志目录。注意：打开失败只记录 WARN，不阻断 tick。"""
        if self._config is None or self._logger.opened or self._logger._file_logging_disabled:
            return
        if not self._logger.open(f"run-{int(time.time())}", self._config):
            self._append_event_unlocked("WARN", "DataLogger", f"open log failed: {self._logger.last_error_message}")

    def _run_formation_algorithms_unlocked(self) -> None:
        """运行编队算法链路。注意：算法输入应使用当前模型状态快照。"""
        # 取一致的输入快照：所有节点基于同一时刻的模型状态与健康表计算，避免步内串扰。
        states = self._model.read_states()
        health_map = self._disturbance.read_health()
        controls: dict[str, AccelerationCommand] = {}
        diagnostics: dict[str, PosTrackDiagS] = {}
        outbox: list[MessageEnvelope] = []
        status_values: list[str] = []
        for node_id, state in states.items():
            # 每个节点先取走自己的收件箱（读取即清空），再驱动其算法一步。
            inbox = self._comm.read_inbox(node_id)
            output = self._node_algorithms[node_id].step(
                state, inbox, self._time_s, health_map.get(node_id, "normal")
            )
            controls[node_id] = output.control
            diagnostics[node_id] = replace(output.control_diag)
            # 汇总各节点待发消息，统一在本轮末尾交给通信模块。
            outbox.extend(output.outbox)
            status_values.append(output.status)
            # 集结完成首帧：锁存分析结果，供快照透传给 UI。
            if output.formation_analysis is not None:
                self._formation_completed_analysis = output.formation_analysis
        # 缓存本轮控制，供后续未跑算法的 tick 继续施加（保持-上次值语义）。
        self._current_controls = controls
        self._control_diagnostics = diagnostics
        self._model.apply_controls(controls)
        # 集中发送：消息在通信模块内按时延/丢包规则投递。
        self._comm.send(outbox)
        # 任一节点非正常组队（如重构）即把全局控制回报置为"重构"。
        if any(status != "forming" for status in status_values):
            self._control_report = "重构"

    def _make_snapshot_unlocked(self) -> SimulationSnapshot:
        """在已持锁状态下生成完整快照。注意：不得把内部可变对象直接暴露出去。"""
        # 汇总各子系统当前态：健康表、当前航段、全部航段，再逐节点组装显示状态。
        health_map = self._disturbance.read_health()
        route = self._make_route_snapshot()
        route_segments = self._make_route_segment_snapshots()
        nodes: list[NodeState] = []
        rally_phases = {nid: alg.current_rally_phase_str() for nid, alg in self._node_algorithms.items()}
        for state in self._model.read_states().values():
            diag = self._control_diagnostics.get(state.node_id, PosTrackDiagS())
            cmd_pos_e = diag.cmd_pos_east_m
            cmd_pos_n = diag.cmd_pos_north_m
            nodes.append(
                NodeState(
                    node_id=state.node_id,
                    role=self._node_roles.get(state.node_id, "unknown"),
                    health=health_map.get(state.node_id, "normal"),
                    x_m=state.x_m,
                    y_m=state.y_m,
                    altitude_m=state.altitude_m,
                    psi_v_deg=state.psi_v_deg,
                    theta_deg=state.theta_deg,
                    speed_mps=state.speed_mps,
                    vx_mps=state.vx_mps,
                    vy_mps=state.vy_mps,
                    vz_mps=state.vz_mps,
                    nx=state.nx,
                    nz=state.nz,
                    phi_deg=state.phi_deg,
                    psi_dot_deg_s=state.psi_dot_deg_s,
                    cmd_pos_east_m=cmd_pos_e,
                    cmd_pos_north_m=cmd_pos_n,
                    cmd_pos_h_m=diag.cmd_pos_h_m,
                    cmd_vel_east_mps=diag.cmd_vel_east_mps,
                    cmd_vel_north_mps=diag.cmd_vel_north_mps,
                    cmd_vel_up_mps=diag.cmd_vel_up_mps,
                    pos_err_east_m=diag.pos_err_east_m,
                    pos_err_north_m=diag.pos_err_north_m,
                    pos_err_h_m=diag.pos_err_h_m,
                    vel_err_east_mps=diag.vel_err_east_mps,
                    vel_err_north_mps=diag.vel_err_north_mps,
                    vel_err_up_mps=diag.vel_err_up_mps,
                    track_pos_err_x_m=diag.track_pos_err_x_m,
                    track_pos_err_y_m=diag.track_pos_err_y_m,
                    track_pos_err_z_m=diag.track_pos_err_z_m,
                    track_vel_err_x_mps=diag.track_vel_err_x_mps,
                    track_vel_err_y_mps=diag.track_vel_err_y_mps,
                    track_vel_err_z_mps=diag.track_vel_err_z_mps,
                    # 侧偏与待飞距相对"当前航段"计算，供 UI 显示跟踪误差。
                    cross_track_error_m=self._cross_track_error(state, route),
                    distance_to_go_m=self._distance_to_go(state, route),
                    rally_phase=rally_phases.get(state.node_id, ""),
                )
            )
        # 链路快照已折叠双向状态。
        links = self._make_configured_link_snapshots()
        return SimulationSnapshot(
            time_s=self._time_s,
            duration_s=self._duration_s,
            step_s=self._step_s,
            run_state=self._run_state,
            control_report=self._control_report,
            nodes=nodes,
            links=links,
            route=route,
            route_segments=route_segments,
            cpu_utilization=self._cpu_utilization,
            rally_analysis=self._formation_completed_analysis,
        )

    def _parse_configured_links(self, raw_links: list[object]) -> list[_ConfiguredLink]:
        """解析配置中的通信链路。注意：链路 ID 需能反向映射双向状态。"""
        configured: list[_ConfiguredLink] = []
        for link in raw_links:
            # 跳过缺 link_id 的非法条目。
            if not isinstance(link, dict) or not link.get("link_id"):
                continue
            configured.append(
                _ConfiguredLink(
                    link_id=str(link["link_id"]),
                    direction=str(link.get("direction") or "duplex"),
                )
            )
        return configured

    def _make_configured_link_snapshots(self) -> list[LinkState]:
        """生成配置链路快照。注意：需要合并正反向通信状态。"""
        # 把通信模块返回的有向状态建索引，便于按 link_id 查找。
        states = {state.link_id: state for state in self._comm.read_link_states()}
        links: list[LinkState] = []
        for configured in self._configured_links:
            # 双工链路需同时取正反两个方向，合并为面向 UI 的一条。
            ids = [configured.link_id]
            if configured.direction == "duplex":
                ids.append(self._reverse_link_id(configured.link_id))
            directional_states = [states[link_id] for link_id in ids if link_id in states]
            if not directional_states:
                continue
            # 折叠取最坏值：任一方向中断即显示 lost，时延/丢包取两向最大（保守显示）。
            status = "lost" if any(state.status == "lost" for state in directional_states) else directional_states[0].status
            links.append(
                LinkState(
                    link_id=configured.link_id,
                    direction=configured.direction,
                    latency_ms=max(state.latency_ms for state in directional_states),
                    loss_rate=max(state.loss_rate for state in directional_states),
                    status=status,
                )
            )
        return links

    def _make_route_snapshot(self) -> RouteState | None:
        """生成当前航线快照。注意：无航线时返回空状态。"""
        # 取第一个能给出当前航段的算法（通常是长机）作为显示航线。
        for algorithm in self._node_algorithms.values():
            route = algorithm.current_route()
            if route is None:
                continue
            return _route_state_from_wayline(route)
        return None

    def _make_route_segment_snapshots(self) -> list[RouteState]:
        """生成全部航段快照。注意：用于 GUI 绘制多航段轨迹。"""
        if not self._node_algorithms or self._leader_route is None:
            return []
        display_route = self._display_route
        if not display_route:
            return []
        return [_route_state_from_wayline(line) for line in display_route]

    @staticmethod
    def _cross_track_error(state: AircraftState, route: RouteState | None) -> float | None:
        """计算节点相对当前航段的侧偏。注意：退化航段返回零偏差。"""
        if route is None:
            return None
        if route.radius_m > 0.0:
            # 圆弧段侧偏应取径向误差；按转向符号定号，保持左右偏差语义稳定。
            radial_distance = math.hypot(state.x_m - route.center_x_m, state.y_m - route.center_y_m)
            turn_sign = 1.0 if route.turn_sign >= 0.0 else -1.0
            return (radial_distance - route.radius_m) * turn_sign
        # 航段方向向量（ENU 平面，x 东 y 北）。
        dx = route.end_x_m - route.start_x_m
        dy = route.end_y_m - route.start_y_m
        length = math.hypot(dx, dy)
        # 退化航段（首尾重合）无法定义法向，返回 None。
        if length <= 1e-9:
            return None
        # 单位右法向量（航段方向顺时针旋转 90°），与航迹系 z 右侧向为正保持一致。
        normal_x = dy / length
        normal_y = -dx / length
        # 侧偏 = 起点->节点向量在右法向上的投影；正值表示位于航迹右侧。
        return (state.x_m - route.start_x_m) * normal_x + (state.y_m - route.start_y_m) * normal_y

    @staticmethod
    def _distance_to_go(state: AircraftState, route: RouteState | None) -> float | None:
        """计算节点到当前航段终点的待飞距。注意：结果不包含后续航段距离。"""
        if route is None:
            return None
        dx = route.end_x_m - route.start_x_m
        dy = route.end_y_m - route.start_y_m
        length = math.hypot(dx, dy)
        if length <= 1e-9:
            return None
        # 沿航段方向的单位向量。
        track_x = dx / length
        track_y = dy / length
        # 待飞距 = 节点->终点向量在航段方向上的投影；越过终点时夹到 0。
        return max(0.0, (route.end_x_m - state.x_m) * track_x + (route.end_y_m - state.y_m) * track_y)

    @staticmethod
    def _reverse_link_id(link_id: str) -> str:
        """生成通信链路反向 ID。注意：仅处理约定格式的双机链路。"""
        # 交换 "src-dst" 两端得到反向 ID；无分隔符则原样返回。
        src, sep, dst = link_id.partition("-")
        if not sep:
            return link_id
        return f"{dst}-{src}"

    def _make_snapshot_for_empty_controller(self) -> SimulationSnapshot:
        """生成空控制器快照。注意：用于未加载配置时的 GUI 初始显示。"""
        return SimulationSnapshot(
            time_s=0.0,
            duration_s=0.0,
            step_s=self._step_s,
            run_state=self._run_state,
            control_report=self._control_report,
            nodes=[],
            links=[],
            cpu_utilization=0.0,
        )

    def _derive_control_report_unlocked(self) -> ControlReport:
        """根据当前状态推导控制回报文本。注意：调用方需持锁。"""
        # 任一节点非健康即优先判为"重构"——故障会触发队形重构。
        if any(h != "normal" for h in self._disturbance.read_health().values()):
            return "重构"
        stages = [
            algorithm.current_stage()
            for algorithm in self._node_algorithms.values()
        ]
        # 按优先级聚合各节点编队阶段：重构 > 集结 > 保持。
        if any(stage == FormStageE.RECONFIG for stage in stages):
            return "重构"
        if any(stage == FormStageE.RALLY for stage in stages):
            return "集结"
        if any(stage == FormStageE.HOLD for stage in stages):
            return "保持"
        # 有节点但无明确阶段则"保持"，完全无算法时为"待命"。
        return "保持" if self._node_algorithms else "待命"

    def _should_refresh_display_unlocked(self) -> bool:
        """判断本 tick 是否需要刷新显示。注意：用于降低 GUI 刷新频率。"""
        # 按墙钟节流：距上次刷新满 _DISPLAY_REFRESH_S 才允许，避免高频 tick 压垮 UI。
        now_s = time.monotonic()
        if self._last_display_wall_s == 0.0 or now_s - self._last_display_wall_s >= self._DISPLAY_REFRESH_S:
            self._last_display_wall_s = now_s
            return True
        return False

    def _notify_subscribers(self, snapshot: SimulationSnapshot) -> None:
        """通知所有快照订阅者。注意：回调异常不应破坏控制器状态。"""
        # 先在锁内拷贝订阅者列表，再在锁外回调，避免回调期间长时间持锁。
        with self._lock:
            subscribers = list(self._subscribers.values())
        for callback in subscribers:
            try:
                callback(snapshot)
            except Exception as exc:  # noqa: BLE001
                # 单个回调异常被隔离记录，不影响其他订阅者与仿真推进。
                with self._lock:
                    self._append_event_unlocked("WARN", "SimControl", f"snapshot callback failed: {exc}")

    def _append_event_unlocked(self, level: EventLevel, source: str, message: str) -> None:
        """在已持锁状态下追加事件文本。注意：事件列表会按容量裁剪。"""
        # 用当前仿真时间打戳，事件同时入内存队列并落日志。
        event = SimulationEvent(self._time_s, level, source, message)
        self._append_event_object_unlocked(event)
        self._logger.write_event(event)

    def _append_event_object_unlocked(self, event: SimulationEvent) -> None:
        """在已持锁状态下追加事件对象。注意：时间戳使用当前仿真时间。"""
        self._events.append(event)

    def _normalize_disturbance(self, command: DisturbanceCommand | dict[str, object]) -> DisturbanceCommand:
        """规范化扰动命令。注意：兼容 GUI 和脚本的不同字段写法。"""
        # 已是结构化命令则直接透传。
        if isinstance(command, DisturbanceCommand):
            return command
        if not isinstance(command, dict):
            raise TypeError("command must be DisturbanceCommand or dict")
        # 校验扰动类型在允许集合内。
        command_type = command.get("type")
        if command_type not in {"wind", "node_fault", "link_loss", "link_fault", "clear"}:
            raise ValueError("invalid disturbance type")
        params = command.get("params", {})
        if not isinstance(params, dict):
            raise ValueError("params must be a dict")
        # duration_s 可缺省（None 表示持续到显式 clear）。
        duration = command.get("duration_s")
        return DisturbanceCommand(
            type=command_type,  # type: ignore[arg-type]
            target=str(command["target"]) if command.get("target") is not None else None,
            duration_s=float(duration) if duration is not None else None,
            params=dict(params),
        )
