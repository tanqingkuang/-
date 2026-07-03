"""仿真控制器的航线、队形与算法初始化构造函数。注意：不持有运行状态。"""

from __future__ import annotations

import math

from src.algorithm.context.leaf_types import (
    CommDirE,
    FormCommInitS,
    FormPosS,
    FormSelfInitS,
    MotionProfS,
    NetWorkS,
    PosInEarthS,
    RemoteCmdS,
    VdInEarthS,
    WayLineS,
    WayPointInputS,
    WayPointS,
)
from src.algorithm.entity.leader_follower_rally import (
    rally_loose_target,
    rally_route_heading_rad,
    resolve_formation_slot,
)
from src.algorithm.entity.types import VelCmdLimitS
from src.algorithm.units.algo.pos_calc.rally_join_pos import _ccw_entry_tangent
from src.algorithm.units.process.formation_task.rally import RallyTaskInitS
from src.environment.model import AircraftState, node_id_from_config
from src.runner.sim_control_constants import (
    _DEFAULT_TRIANGLE_WING_SLOTS,
    _FORMATION_COORDINATE_SYSTEM,
)
from src.runner.sim_control_types import RallyJoinGeometryState, RouteState

def _motion_from_aircraft_state(state: AircraftState) -> MotionProfS:
    """把环境模型状态转换为算法运动状态。注意：单位和坐标系必须保持一致。"""
    # 本函数是环境模型和编队算法之间的单位边界，新增字段时优先在这里转换。
    # 环境状态内部角度为弧度，日志/快照显示角度另在快照模块转换。
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
    # 通信拓扑和队形槽位同时进入 FormCommInitS，算法初始化只接受这个聚合结构。
    # links 中的非法条目被跳过，formation 配置错误则抛异常，二者语义不同。
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

    # 多队形（名字 + 各队形槽位）与通信网络一并打包为编队初始化结构。
    names, rows, initial_index = _build_formation_slots(nodes, config)
    return FormCommInitS(
        netWork=network,
        formPat=names,
        formPos=rows,
        initialPattern=initial_index,
    )


def _build_formation_slots(
    nodes: list[object],
    config: dict[str, object] | None,
) -> tuple[list[str], list[list[FormPosS]], int]:
    """根据配置生成多队形槽位定义。注意：槽位是队形定义，不应依赖飞机初始位置。

    返回 (队形名列表, 各队形槽位表, 初始队形索引)；三者以队形索引对齐。
    支持两种写法：
    - 多队形：formation.formations = [{name, slots}, ...]，初始索引取 formation.initial_index；
    - 单队形（历史）：formation.pattern + formation.slots，等价于只有一个队形、初始索引 0。
    """
    # 未配置 formation 时回退到默认三角单队形。
    formation_config = (config or {}).get("formation")
    if formation_config is None:
        return ["default"], [_default_formation_slots(nodes)], 0
    if not isinstance(formation_config, dict):
        raise ValueError("formation must be an object")

    formations = formation_config.get("formations")
    if formations is not None:
        return _build_multi_formations(nodes, formation_config, formations)

    # —— 历史单队形写法 ——
    name = str(formation_config.get("pattern", "default"))
    slot_config = formation_config.get("slots")
    if slot_config is None:
        # 给了 pattern 但未给 slots 时同样用默认槽位。
        return [name], [_default_formation_slots(nodes)], 0
    _validate_formation_coordinate_system(formation_config)
    slots = _parse_slot_list(nodes, slot_config, "formation.slots")
    return [name], [slots], 0


def _build_multi_formations(
    nodes: list[object],
    formation_config: dict[str, object],
    formations: object,
) -> tuple[list[str], list[list[FormPosS]], int]:
    """解析 formation.formations 多队形列表。注意：每个队形都必须给全部节点槽位。"""
    if not isinstance(formations, list) or not formations:
        raise ValueError("formation.formations must be a non-empty list")
    # 坐标轴声明在 formation 顶层统一校验一次，各队形共用同一轴序。
    _validate_formation_coordinate_system(formation_config)
    names: list[str] = []
    rows: list[list[FormPosS]] = []
    for index, item in enumerate(formations):
        if not isinstance(item, dict):
            raise ValueError(f"formation.formations[{index}] must be an object")
        names.append(str(item.get("name", f"formation_{index}")))
        slot_config = item.get("slots")
        if not isinstance(slot_config, list) or not slot_config:
            raise ValueError(f"formation.formations[{index}].slots must be a non-empty list")
        rows.append(_parse_slot_list(nodes, slot_config, f"formation.formations[{index}].slots"))
    # 初始队形索引默认 0，必须落在队形数量范围内。
    initial_index = int(formation_config.get("initial_index", 0))
    if initial_index < 0 or initial_index >= len(rows):
        raise ValueError(f"formation.initial_index out of range: {initial_index}")
    return names, rows, initial_index


def _parse_slot_list(
    nodes: list[object],
    slot_config: object,
    prefix: str,
) -> list[FormPosS]:
    """解析单个队形的槽位列表并按节点顺序输出。注意：每个已知节点都必须有槽位，缺一即报错。"""
    if not isinstance(slot_config, list) or not slot_config:
        raise ValueError(f"{prefix} must be a non-empty list")

    # 收集已知节点 ID，用于校验槽位引用的节点是否存在。
    known_node_ids = {
        node_id_from_config(node, index)
        for index, node in enumerate(nodes)
        if isinstance(node, dict)
    }
    slots_by_id: dict[str, FormPosS] = {}
    for index, slot in enumerate(slot_config):
        if not isinstance(slot, dict):
            raise ValueError(f"{prefix}[{index}] must be an object")
        # 兼容 node_id / id 两种键名。
        node_id = str(slot.get("node_id", slot.get("id", "")))
        if not node_id:
            raise ValueError(f"{prefix}[{index}].node_id is required")
        # 槽位不得重复或引用未知节点。
        if node_id in slots_by_id:
            raise ValueError(f"{prefix} contains duplicate node_id {node_id!r}")
        if known_node_ids and node_id not in known_node_ids:
            raise ValueError(f"{prefix} contains unknown node_id {node_id!r}")
        slots_by_id[node_id] = FormPosS(
            node_id,
            _float_from_keys(slot, prefix, index, ("x_m", "x")),
            _float_from_keys(slot, prefix, index, ("y_m", "y")),
            _float_from_keys(slot, prefix, index, ("z_m", "z")),
        )

    # 每个已知节点都必须有对应槽位，缺一即报错。
    missing = [node_id for node_id in known_node_ids if node_id not in slots_by_id]
    if missing:
        raise ValueError(f"{prefix} missing node_id {missing[0]!r}")

    # 按节点声明顺序输出槽位，保证与模型节点顺序对齐。
    ordered_slots: list[FormPosS] = []
    for index, node in enumerate(nodes):
        if not isinstance(node, dict):
            continue
        ordered_slots.append(slots_by_id[node_id_from_config(node, index)])
    return ordered_slots


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
    # 默认槽位只服务最小三机示例；更多僚机必须显式给出 formation.slots。
    # 输出顺序仍跟随 nodes，保证算法实体和模型节点顺序一致。
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
    # route.waypoints 优先级最高，segments/lines 次之，最后才是历史单段写法。
    # insert_arcs 只影响算法输入的圆弧过渡；GUI 显示原始航段时会关闭它。
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


def _waypoint_turn_sign(raw_point: object, field_name: str) -> float:
    """读取已烘焙圆弧的转向符号。注意：普通折线航点缺省为 0。"""
    if not isinstance(raw_point, dict):
        return 0.0
    # 导出航线使用 snake_case；历史/内部对象字段使用 camelCase，两者都兼容。
    turn_sign = float(raw_point.get("turn_sign", raw_point.get("turnSign", 0.0)))
    # 当前路径几何只定义左转、右转和直线三种状态，不接受任意连续值。
    if turn_sign not in {-1.0, 0.0, 1.0}:
        raise ValueError(f"{field_name}.turn_sign must be -1, 0, or 1")
    return turn_sign


def _waypoint_center(raw_point: object, altitude_m: float, field_name: str) -> PosInEarthS:
    """读取已烘焙圆弧圆心。注意：只在 turn_sign 非 0 时有意义。"""
    if not isinstance(raw_point, dict):
        raise ValueError(f"{field_name} must be an object")
    center = raw_point.get("center")
    if isinstance(center, dict):
        # 嵌套 center 复用普通航点坐标解析，保持 x_m/east 等别名一致。
        return _route_point_from_config(center, f"{field_name}.center")
    if "center_x_m" in raw_point or "center_y_m" in raw_point:
        # 扁平 center_x_m/center_y_m 便于外部工具导出，缺省高度跟随航点。
        return PosInEarthS(
            float(raw_point.get("center_x_m", 0.0)),
            float(raw_point.get("center_y_m", 0.0)),
            altitude_m,
        )
    raise ValueError(f"{field_name}.center is required when turn_sign is non-zero")


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
    turn_signs = [
        _waypoint_turn_sign(raw_point, f"route.waypoints[{index}]")
        for index, raw_point in enumerate(raw_waypoints)
    ]
    # 先完成退化校验，再决定 R 或已知圆弧，错误信息仍指向原始 waypoint 下标。
    for index in range(len(points) - 1):
        if _same_xy(points[index], points[index + 1]) and points[index].h == points[index + 1].h:
            raise ValueError(f"route.waypoints[{index}] and route.waypoints[{index + 1}] must be different")
    n = len(points)
    return [
        WayPointInputS(
            idx=index,
            pos=points[index],
            vdCmd=speed,
            # 已烘焙圆弧已经给出真实曲率，不能再同时按 R 插入交接圆弧。
            r=radii[index] if (insert_arcs and turn_signs[index] == 0.0 and 0 < index < n - 1) else 0.0,
            turnSign=turn_signs[index],
            # center 只在 turnSign 非 0 时解析，普通折线避免强制填写无意义字段。
            center=_waypoint_center(raw_waypoints[index], points[index].h, f"route.waypoints[{index}]")
            if turn_signs[index] != 0.0
            else PosInEarthS(),
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
    # 集结只用单队形（formPos 只有第 0 行），目标队形索引恒为 0。
    return RallyTaskInitS(
        looseScale=float(rally_cfg_raw.get("loose_scale", 3.0)),
        convergenceRadius_m=float(rally_cfg_raw.get("convergence_radius_m", 5.0)),
        stableHold_s=float(rally_cfg_raw.get("stable_hold_s", 5.0)),
        compressTime_s=float(rally_cfg_raw.get("compress_time_s", 30.0)),
        tightRadius_m=float(rally_cfg_raw.get("tight_radius_m", 2.0)),
        expectedFollowerIds=expected_ids,
        staleTimeout_s=float(rally_cfg_raw.get("stale_timeout_s", 2.0)),
        targetPattern=0,
        dt_s=algorithm_period_s,
        loiter_radius_m=float(rally_cfg_raw.get("loiter_radius_m", 200.0)),
        arrival_radius_m=float(rally_cfg_raw.get("arrival_radius_m", 100.0)),
        catchup_radius_m=float(rally_cfg_raw.get("catchup_radius_m", 200.0)),
        catchup_kp_speed=float(rally_cfg_raw.get("catchup_kp_speed", 0.05)),
    )


def _build_rally_join_geometry(
    nodes: list[object],
    rally_route: list[WayPointInputS] | None,
    formation_comm_init: FormCommInitS,
    rally_task_init: RallyTaskInitS | None,
    initial_states: dict[str, AircraftState],
) -> dict[str, RallyJoinGeometryState]:
    """按静态配置预计算各集结节点的盘旋圆、切入点 T、切出点 M_i，仅供 GUI 辅助展示。

    注意：只依赖配置和模型已解析的初始状态（不依赖仿真推进后的运行时状态），几何公式复用
    RallyJoinPos 的实际实现（`rally_route_heading_rad`/`rally_loose_target`/`_ccw_entry_tangent`），
    避免和算法行为分叉。切入点 T 的起点取自 initial_states（模型对 x_m/y_m 缺省时按节点序号错位
    摆放的解析结果），不能直接读原始配置的 x_m/y_m——配置省略该字段时二者默认值不同，直接读配置
    会用错误的起点算出错误的 T。集结长机不依赖队形/槽位，即使 targetPattern 在 formPat 中找不到
    （只有 rally_follower 才需要它），长机自己的圆和切出点仍应正常给出。
    """
    if rally_route is None or rally_task_init is None or len(rally_route) < 2:
        return {}
    heading = rally_route_heading_rad(rally_route)
    a = rally_route[0].pos
    radius = rally_task_init.loiter_radius_m

    geometry: dict[str, RallyJoinGeometryState] = {}
    for index, node in enumerate(nodes):
        if not isinstance(node, dict):
            continue
        role = str(node.get("role") or "")
        if role not in ("rally_leader", "rally_follower"):
            continue
        node_id = node_id_from_config(node, index)
        if role == "rally_leader":
            slot_pos = a
        else:
            slot = resolve_formation_slot(formation_comm_init, rally_task_init.targetPattern, node_id)
            if slot is None:
                continue
            slot_pos = rally_loose_target(a, heading, rally_task_init.looseScale, slot)
        center_e = slot_pos.east - radius * math.sin(heading)
        center_n = slot_pos.north + radius * math.cos(heading)
        initial_state = initial_states.get(node_id)
        start_e = initial_state.x_m if initial_state is not None else slot_pos.east
        start_n = initial_state.y_m if initial_state is not None else slot_pos.north
        tangent = _ccw_entry_tangent(start_e, start_n, center_e, center_n, radius)
        entry_e, entry_n = (tangent[0], tangent[1]) if tangent is not None else (slot_pos.east, slot_pos.north)
        geometry[node_id] = RallyJoinGeometryState(
            slot_east_m=slot_pos.east,
            slot_north_m=slot_pos.north,
            loiter_center_east_m=center_e,
            loiter_center_north_m=center_n,
            loiter_radius_m=radius,
            entry_east_m=entry_e,
            entry_north_m=entry_n,
        )
    return geometry


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
