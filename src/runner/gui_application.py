"""GUI 专用应用层服务。注意：统一承接界面需要的规划、配置与坐标转换。"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from enum import StrEnum
from pathlib import Path
from typing import TYPE_CHECKING

from src.algorithm.context.leaf_types import WayLineS, WayPointInputS, to_display_inputs
from src.algorithm.entity.leader_follower_hold.leader import waypoint_inputs_to_waylines
from src.algorithm.units.algo.arc_path import arc_radius as _arc_radius_fn, arc_swept_rad
from src.algorithm.units.process.tra_plan.avoidance.obstacle import ObstacleS, make_circle, make_polygon, make_rect
from src.algorithm.units.process.tra_plan.avoidance.planner import plan_avoidance_route
from src.data.config_loader import _LINE_FILE_MANAGER, resolve_config_references
from src.data.geo import GeoOrigin, enu_to_geodetic
from src.data.geo_config import geo_origin_from_dict, route_to_external, route_to_internal

if TYPE_CHECKING:
    from src.runner.sim_controller import SimulationController


@dataclass(frozen=True)
class GeoReference:
    """应用层暴露给 GUI 的地理原点。注意：不泄漏 data 层具体类型。"""

    latitude_deg: float
    longitude_deg: float


class ObstacleKind(StrEnum):
    """应用层与 GUI 共用的障碍类型。"""

    # 枚举值属于 JSON 配置兼容契约，不能随 Python 成员名重命名。
    CIRCLE = "circle"
    RECT = "rect"
    POLYGON = "polygon"


@dataclass(frozen=True)
class ObstacleSpec:
    """应用层障碍描述。注意：只承载规划和显示共同需要的稳定字段。"""

    # 标识与类型供列表、规划和导出三条链路共同使用。
    obstacle_id: str
    kind: ObstacleKind
    enabled: bool = True
    # 圆形障碍只使用中心与半径字段。
    center_x: float = 0.0
    center_y: float = 0.0
    radius: float = 0.0
    # 轴对齐矩形只使用四条边界字段。
    min_x: float = 0.0
    min_y: float = 0.0
    max_x: float = 0.0
    max_y: float = 0.0
    # 任意多边形使用不可变顶点快照，避免 GUI 勾选之外的意外回写。
    vertices: tuple[tuple[float, float], ...] = ()

    def __post_init__(self) -> None:
        """把配置边界传入的字符串归一化为共享枚举。"""

        object.__setattr__(self, "kind", ObstacleKind(self.kind))


@dataclass
class AvoidanceParams:
    """避障规划参数与基础航线。注意：GUI 可以修改交互参数，但不接触算法类型。"""

    # 飞行约束与安全距离来自当前配置，GUI 控件可在生成前覆盖。
    turn_radius_m: float = 0.0
    leg_margin_m: float = 0.0
    clearance_m: float = 0.0
    simplify_clearance_m: float = 0.0
    simplify_clearance_explicit: bool = False
    # 搜索代价参数保持独立，避免和真实几何距离混淆。
    turn_switch_penalty_m: float = 0.0
    turn_angle_weight_m: float = 0.0
    resolution_m: float = 10.0
    margin_m: float = 0.0
    speed_mps: float = 0.0
    allow_arc: bool = True
    # 基础航点只暴露 ENU 三元组，不把 WayPointInputS 传进 GUI。
    waypoints: list[tuple[float, float, float]] = field(default_factory=list)
    geo_reference: GeoReference | None = None

    @property
    def geo_origin(self) -> GeoReference | None:
        """兼容旧字段名读取地理原点。注意：返回的仍是应用层类型。"""

        return self.geo_reference

    @geo_origin.setter
    def geo_origin(self, value: object | None) -> None:
        """兼容旧测试写入原点对象。注意：只复制经纬度，不保留 data 层实例。"""

        if value is None:
            # 清空兼容字段时同时清空正式应用层字段。
            self.geo_reference = None
            return
        # 使用属性协议复制值，兼容旧测试对象但不保存其类型。
        self.geo_reference = GeoReference(
            latitude_deg=float(getattr(value, "latitude_deg")),
            longitude_deg=float(getattr(value, "longitude_deg")),
        )


@dataclass(frozen=True)
class PlannedRoute:
    """应用层规划航线。注意：算法航点只保存在私有载荷中，GUI 只读显示摘要。"""

    # 私有载荷只在 runner 内用于采用和导出，不进入界面绘制逻辑。
    _waypoints: tuple[WayPointInputS, ...] = field(repr=False)
    # 下列字段都是不可变显示投影，界面可安全缓存。
    polyline: tuple[tuple[float, float], ...]
    markers: tuple[tuple[float, float], ...]
    segment_count: int
    arc_count: int
    transition_radius_count: int


@dataclass(frozen=True)
class AvoidancePlanOutcome:
    """避障规划对 GUI 的稳定返回对象。注意：失败时 route 为 None。"""

    ok: bool
    route: PlannedRoute | None = None
    code: str = "OK"
    detail: str = ""


@dataclass(frozen=True)
class GuiConfigData:
    """控制器成功加载配置后供 GUI 消费的辅助配置快照。"""

    obstacles: tuple[ObstacleSpec, ...] = ()
    obstacle_clearance_m: float = 0.0
    avoidance_params: AvoidanceParams | None = None
    geo_reference: GeoReference | None = None
    terrain_display_file: str | None = None


def load_gui_config(path: str) -> GuiConfigData:
    """安全读取 GUI 辅助配置。注意：失败时返回空快照，不影响控制器正式加载结论。"""

    # 障碍和航线分别容错解析，坏掉一条引用不会抹掉另一条显示信息。
    obstacles, clearance = _parse_obstacles(path)
    params = _parse_avoidance_params(path)
    if params is not None:
        # 障碍解析口径是安全间距的权威来源，两份 DTO 必须保持一致。
        params.clearance_m = clearance
    return GuiConfigData(
        obstacles=tuple(obstacles),
        obstacle_clearance_m=clearance,
        avoidance_params=params,
        geo_reference=_geo_reference_from_config(path),
        terrain_display_file=terrain_display_file_from_config(path),
    )


def plan_route_for_gui(
    waypoints: list[tuple[float, float, float]],
    obstacles: list[ObstacleSpec],
    **kwargs: object,
) -> AvoidancePlanOutcome:
    """调用避障算法并封装 GUI 只读航线。注意：参数错误仍以 ValueError 反馈调用方。"""

    # 仅在应用层边界把稳定 DTO 转换成算法障碍对象。
    result = plan_avoidance_route(
        waypoints,
        [_obstacle_to_backend(obstacle) for obstacle in obstacles],
        **kwargs,
    )
    # 失败结果不构造空航线，避免 GUI 把失败误当作可采用预览。
    planned = planned_route_from_waypoints(result.route) if result.ok and result.route is not None else None
    return AvoidancePlanOutcome(ok=result.ok, route=planned, code=result.code, detail=result.detail)


def planned_route_from_waypoints(route: list[WayPointInputS] | tuple[WayPointInputS, ...]) -> PlannedRoute:
    """把算法航点封装成应用层航线。注意：主要供规划服务和兼容测试构造使用。"""

    # 元组封装阻止调用方在规划完成后改写算法返回列表。
    waypoints = tuple(route)
    # 显示语义先去掉交接半径，再统一转换成航段。
    display_lines = waypoint_inputs_to_waylines(to_display_inputs(list(waypoints))) if len(waypoints) >= 2 else []
    return PlannedRoute(
        _waypoints=waypoints,
        polyline=tuple(_route_to_polyline_from_lines(display_lines)),
        markers=tuple(_route_marker_points_from_lines(display_lines)),
        segment_count=len(display_lines),
        arc_count=sum(1 for line in display_lines if line.start.turnSign != 0.0),
        transition_radius_count=sum(1 for point in waypoints if point.r > 0.0),
    )


def apply_planned_route(controller: SimulationController, route: PlannedRoute):  # noqa: ANN201
    """把应用层航线下发控制器。注意：算法航点不会暴露给 GUI 调用方。"""

    # 控制器仍接收自身算法输入类型，转换细节留在 runner 内部。
    return controller.apply_avoidance_route(list(route._waypoints))


def export_planned_route(
    config_path: Path,
    route_path: Path,
    route: PlannedRoute,
    speed_mps: float,
    geo_reference: GeoReference | None,
) -> Path:
    """按航线文件策略输出规划结果。注意：相对路径仍以主配置目录为基准。"""

    # 先生成格式无关对象，再交给 LineFileManager 选择具体策略。
    route_config = route_inputs_to_config(list(route._waypoints), speed_mps, geo_reference)
    return _LINE_FILE_MANAGER.save_route(config_path, str(route_path), route_config)


def route_export_defaults(config_path: Path) -> tuple[Path, str]:
    """返回导出默认路径和过滤器。注意：读取失败时回退 JSON。"""

    json_filter = "JSON 文件 (*.json)"
    xml_filter = "钻石 XML (*.XML *.xml)"
    # 已有 route_file 决定建议文件格式；缺失时固定回退 JSON。
    route_file = _route_file_from_config(config_path)
    if route_file is None:
        return config_path.parent / "avoidance_route.json", json_filter
    # 格式策略只查看解析后的路径后缀，不读取原航线内容。
    route_path = _LINE_FILE_MANAGER.resolve_path(config_path, route_file)
    try:
        filename = _LINE_FILE_MANAGER.default_output_filename(route_path)
    except ValueError:
        return config_path.parent / "avoidance_route.json", json_filter
    selected_filter = xml_filter if Path(filename).suffix.lower() == ".xml" else json_filter
    return config_path.parent / filename, selected_filter


def persist_config_duration(path: Path, duration_s: float) -> None:
    """只更新配置文件的 duration_s。注意：格式错误与写盘错误由调用方记录。"""

    # 保留原文件格式，避免修改一个字段却把 YAML 整体改写成 JSON。
    suffix = path.suffix.lower()
    text = path.read_text(encoding="utf-8")
    if suffix == ".json":
        # JSON 根对象之外的输入不具备 duration_s 字段语义。
        config = json.loads(text)
        if not isinstance(config, dict):
            raise ValueError("config root must be an object")
        config["duration_s"] = duration_s
        path.write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        return
    if suffix in {".yaml", ".yml"}:
        # PyYAML 为可选依赖，只有用户实际编辑 YAML 时才要求安装。
        try:
            import yaml
        except ImportError as exc:  # pragma: no cover - 依赖运行环境
            raise ValueError("YAML config requires PyYAML") from exc
        config = yaml.safe_load(text)
        if not isinstance(config, dict):
            raise ValueError("config root must be an object")
        config["duration_s"] = duration_s
        path.write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False), encoding="utf-8")
        return
    raise ValueError("config must be .json, .yaml, or .yml")


def geodetic_from_enu(east_m: float, north_m: float, reference: GeoReference | None) -> tuple[float, float] | None:
    """把 ENU 点转换为经纬度。注意：无地理原点时返回 None。"""

    if reference is None:
        # 缺少 origin 时禁止用零点伪造看似有效的经纬度。
        return None
    return enu_to_geodetic(east_m, north_m, _geo_origin(reference))


def terrain_display_file_from_config(path: str) -> str | None:
    """从主配置读取 3D 地形文件路径。注意：该字段只影响显示层。"""

    # 地形布局只影响显示，任何解析失败都按“未配置”降级。
    config_path = Path(path)
    try:
        text = config_path.read_text(encoding="utf-8")
        if config_path.suffix.lower() == ".json":
            # 与主配置支持范围一致，仅接受 JSON 和 YAML。
            data = json.loads(text)
        elif config_path.suffix.lower() in {".yaml", ".yml"}:
            try:
                import yaml
            except ImportError:
                return None
            data = yaml.safe_load(text)
        else:
            return None
    except (OSError, json.JSONDecodeError, ValueError):
        return None
    if not isinstance(data, dict):
        return None
    # 空白字符串和非字符串都视为未配置。
    raw_file = data.get("terrain_display_file")
    if not isinstance(raw_file, str) or not raw_file.strip():
        return None
    display_path = Path(raw_file)
    if not display_path.is_absolute():
        # 相对路径始终相对主配置目录，避免受启动 cwd 影响。
        display_path = config_path.parent / display_path
    return str(display_path.resolve())


def route_inputs_to_config(
    route: list[WayPointInputS],
    speed_mps: float,
    geo_reference: GeoReference | GeoOrigin | None = None,
) -> dict[str, object]:
    """把避障航点转换为航线文件对象。注意：兼容内部测试传入 GeoOrigin。"""

    # 先保持内部 ENU 表示，最后按需整体转换为经纬高。
    waypoints: list[dict[str, object]] = []
    for point in route:
        waypoint: dict[str, object] = {
            "x_m": point.pos.east,
            "y_m": point.pos.north,
            "altitude_m": point.pos.h,
            "R": point.r,
        }
        if point.turnSign != 0.0:
            # 已烘焙圆弧必须携带转向和圆心，才能无损读回。
            waypoint["turn_sign"] = point.turnSign
            waypoint["center"] = {
                "x_m": point.center.east,
                "y_m": point.center.north,
                "altitude_m": point.center.h,
            }
        waypoints.append(waypoint)
    # 速度是航线级字段，不能从末段航点猜测。
    route_config = {"speed_mps": speed_mps, "waypoints": waypoints}
    if geo_reference is None:
        return route_config
    # 旧内部测试可传 GeoOrigin，正式 GUI 只会传 GeoReference。
    origin = geo_reference if isinstance(geo_reference, GeoOrigin) else _geo_origin(geo_reference)
    return route_to_external(route_config, origin)


def route_to_polyline(route: list[WayPointInputS]) -> list[tuple[float, float]]:
    """兼容入口：把算法航点展开为显示折线。注意：正式 GUI 使用 PlannedRoute.polyline。"""

    if len(route) < 2:
        # 单点无法形成可绘制航段。
        return []
    return _route_to_polyline_from_lines(waypoint_inputs_to_waylines(to_display_inputs(route)))


def preview_route_marker_points(route: list[WayPointInputS]) -> list[tuple[float, float]]:
    """兼容入口：返回航点标记。注意：正式 GUI 使用 PlannedRoute.markers。"""

    if len(route) < 2:
        return []
    return _route_marker_points_from_lines(waypoint_inputs_to_waylines(to_display_inputs(route)))


def _safe_float(value: object, default: float = 0.0) -> float:
    """把配置值安全转换为 float。注意：非法值回退默认值。"""

    try:
        # bool 也能转 float，沿用既有宽松配置兼容语义。
        return float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return default


def _load_json_config(path: str) -> dict[str, object] | None:
    """读取 JSON 主配置。注意：GUI 辅助解析失败不改变控制器结论。"""

    # 辅助配置不承担正式错误上报，控制器加载链路会给出权威诊断。
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    return data if isinstance(data, dict) else None


def _resolve_obstacles(data: dict[str, object], path: str) -> dict[str, object] | None:
    """展开障碍引用。注意：坏航线文件不应清空旧 ENU 障碍。"""

    # 首选完整展开，以便经纬度障碍能复用航线 origin。
    try:
        return resolve_config_references(data, Path(path))
    except (OSError, ValueError):
        try:
            # 回退路径移除坏 route_file，只保住旧 ENU 障碍的显示能力。
            resolved = dict(data)
            avoidance = resolved.get("avoidance")
            if isinstance(avoidance, dict) and "obstacles_file" in avoidance:
                resolved.pop("route_file", None)
                resolved = resolve_config_references(resolved, Path(path))
            return resolved
        except (OSError, ValueError):
            return None


def _resolve_route(data: dict[str, object], path: str) -> dict[str, object] | None:
    """展开航线引用。注意：坏障碍文件不应污染规划参数。"""

    # 只展开 route_file，不读取 obstacles_file，隔离两类外部资源错误。
    try:
        resolved = dict(data)
        route_file = resolved.get("route_file")
        if route_file is not None:
            resolved["route"] = _LINE_FILE_MANAGER.load_route(Path(path), route_file)
        route = resolved.get("route")
        if isinstance(route, dict):
            resolved["route"], _origin_value = route_to_internal(route)
        return resolved
    except (OSError, ValueError):
        return None


def _parse_obstacles(path: str) -> tuple[list[ObstacleSpec], float]:
    """解析避障障碍与安全间距。注意：任何非法辅助字段都安全降级。"""

    # 非 JSON 配置暂不提供避障编辑面板，但不影响控制器运行。
    raw_data = _load_json_config(path)
    data = _resolve_obstacles(raw_data, path) if raw_data is not None else None
    avoidance = data.get("avoidance") if isinstance(data, dict) else None
    if not isinstance(avoidance, dict) or not avoidance.get("enabled", True):
        # 总开关关闭时障碍不参与显示和规划。
        return [], 0.0
    clearance = _safe_float(avoidance.get("clearance_m", 0.0))
    raw_obstacles = avoidance.get("obstacles", [])
    if not isinstance(raw_obstacles, list):
        return [], clearance
    obstacles: list[ObstacleSpec] = []
    for index, raw in enumerate(raw_obstacles):
        # 非对象条目跳过，保持辅助面板的安全解析契约。
        if not isinstance(raw, dict):
            continue
        obstacle_id = str(raw.get("id", f"OB{index + 1}"))
        enabled = bool(raw.get("enabled", True))
        raw_type = str(raw.get("type", ObstacleKind.CIRCLE))
        vertices = raw.get("vertices")
        if raw_type == ObstacleKind.POLYGON and isinstance(vertices, list):
            # 多边形至少三个有效顶点才有面积语义。
            points = tuple(
                (_safe_float(point.get("east_m", 0.0)), _safe_float(point.get("north_m", 0.0)))
                for point in vertices
                if isinstance(point, dict)
            )
            if len(points) >= 3:
                obstacles.append(ObstacleSpec(obstacle_id, ObstacleKind.POLYGON, enabled, vertices=points))
        elif raw_type == ObstacleKind.RECT:
            # 矩形上下界允许缺字段，缺失值按零保持旧兼容行为。
            lo = raw.get("min", {})
            hi = raw.get("max", {})
            lo = lo if isinstance(lo, dict) else {}
            hi = hi if isinstance(hi, dict) else {}
            obstacles.append(
                ObstacleSpec(
                    obstacle_id,
                    ObstacleKind.RECT,
                    enabled,
                    min_x=_safe_float(lo.get("east_m", 0.0)),
                    min_y=_safe_float(lo.get("north_m", 0.0)),
                    max_x=_safe_float(hi.get("east_m", 0.0)),
                    max_y=_safe_float(hi.get("north_m", 0.0)),
                )
            )
        else:
            # 未知类型沿用旧规则退化为圆形，而不是阻断整个配置。
            # 该兼容降级只存在于配置入口，内部 ObstacleSpec 不接受未知类型。
            center = raw.get("center", {})
            center = center if isinstance(center, dict) else {}
            obstacles.append(
                ObstacleSpec(
                    obstacle_id,
                    ObstacleKind.CIRCLE,
                    enabled,
                    center_x=_safe_float(center.get("east_m", 0.0)),
                    center_y=_safe_float(center.get("north_m", 0.0)),
                    radius=_safe_float(raw.get("radius_m", 0.0)),
                )
            )
    return obstacles, clearance


def _parse_avoidance_params(path: str) -> AvoidanceParams | None:
    """解析规划参数和基础航线。注意：航点不足时返回 None。"""

    # 航线参数与障碍列表分开解析，避免外部障碍文件错误污染航点。
    raw_data = _load_json_config(path)
    data = _resolve_route(raw_data, path) if raw_data is not None else None
    if data is None:
        return None
    avoidance = data.get("avoidance")
    if not isinstance(avoidance, dict) or not avoidance.get("enabled", True):
        return None
    grid = avoidance.get("grid") if isinstance(avoidance.get("grid"), dict) else {}
    route = data.get("route") if isinstance(data.get("route"), dict) else {}
    origin = geo_origin_from_dict(route.get("_geo_origin")) if isinstance(route, dict) else None
    raw_waypoints = route.get("waypoints", []) if isinstance(route, dict) else []
    waypoints = [
        (
            _safe_float(raw.get("x_m", raw.get("east", 0.0))),
            _safe_float(raw.get("y_m", raw.get("north", 0.0))),
            _safe_float(raw.get("altitude_m", raw.get("h", 0.0))),
        )
        for raw in raw_waypoints
        if isinstance(raw, dict)
    ] if isinstance(raw_waypoints, list) else []
    if len(waypoints) < 2:
        # 不足两点无法规划任何航段，因此不开放生成按钮。
        return None
    clearance_m = _safe_float(avoidance.get("clearance_m", 0.0))
    return AvoidanceParams(
        turn_radius_m=_safe_float(avoidance.get("turn_radius_m", 0.0)),
        leg_margin_m=_safe_float(avoidance.get("leg_length_margin_m", 0.0)),
        clearance_m=clearance_m,
        simplify_clearance_m=_safe_float(avoidance.get("simplify_clearance_m", clearance_m)),
        simplify_clearance_explicit="simplify_clearance_m" in avoidance,
        turn_switch_penalty_m=_safe_float(avoidance.get("turn_switch_penalty_m", 0.0)),
        turn_angle_weight_m=_safe_float(avoidance.get("turn_angle_weight_m", 0.0)),
        resolution_m=_safe_float(grid.get("resolution_m", 10.0)) if isinstance(grid, dict) else 10.0,
        margin_m=_safe_float(grid.get("margin_m", 0.0)) if isinstance(grid, dict) else 0.0,
        speed_mps=_safe_float(route.get("speed_mps", 0.0)) if isinstance(route, dict) else 0.0,
        allow_arc=bool(avoidance.get("allow_arc", True)),
        waypoints=waypoints,
        geo_reference=_geo_reference(origin),
    )


def _geo_reference_from_config(path: str) -> GeoReference | None:
    """读取基础航线的地理原点。注意：旧 ENU 配置返回 None。"""

    # 经纬 origin 和规划参数使用同一条 route_file 展开规则。
    raw_data = _load_json_config(path)
    data = _resolve_route(raw_data, path) if raw_data is not None else None
    route = data.get("route") if isinstance(data, dict) else None
    origin = geo_origin_from_dict(route.get("_geo_origin")) if isinstance(route, dict) else None
    return _geo_reference(origin)


def _route_file_from_config(path: Path) -> str | None:
    """读取 route_file。注意：只用于导出默认文件名。"""

    data = _load_json_config(str(path))
    route_file = data.get("route_file") if isinstance(data, dict) else None
    return route_file if isinstance(route_file, str) and route_file.strip() else None


def _obstacle_to_backend(obstacle: ObstacleSpec) -> ObstacleS:
    """把应用层障碍转换为算法对象。注意：转换仅发生在 runner 边界内。"""

    if obstacle.kind is ObstacleKind.POLYGON:
        # 顶点 tuple 在边界处复制为算法期望的可变列表。
        return make_polygon(obstacle.obstacle_id, list(obstacle.vertices))
    if obstacle.kind is ObstacleKind.RECT:
        return make_rect(obstacle.obstacle_id, obstacle.min_x, obstacle.min_y, obstacle.max_x, obstacle.max_y)
    return make_circle(obstacle.obstacle_id, obstacle.center_x, obstacle.center_y, obstacle.radius)


def _sample_wayline_arc(line: WayLineS, step_deg: float = 6.0) -> list[tuple[float, float]]:
    """把圆弧航段采样为显示折线。注意：复用算法层统一圆弧语义。"""

    # 圆弧起点、圆心和转向共同决定唯一扫掠方向。
    center = line.start.center
    radius = _arc_radius_fn(line)
    start_angle = math.atan2(line.start.pos.north - center.north, line.start.pos.east - center.east)
    swept = arc_swept_rad(line)
    segments = max(1, int(abs(math.degrees(swept)) / step_deg))
    return [
        (
            center.east + radius * math.cos(start_angle + swept * (index / segments)),
            center.north + radius * math.sin(start_angle + swept * (index / segments)),
        )
        for index in range(segments + 1)
    ]


def _route_to_polyline_from_lines(lines: list[WayLineS]) -> list[tuple[float, float]]:
    """把显示航段展开为去重折线。注意：圆弧保留采样形状。"""

    # 先按航段展开，再统一删除相邻共点。
    raw: list[tuple[float, float]] = []
    for line in lines:
        if line.start.turnSign != 0.0:
            raw.extend(_sample_wayline_arc(line))
        else:
            raw.extend(((line.start.pos.east, line.start.pos.north), (line.end.pos.east, line.end.pos.north)))
    return _deduplicate_points(raw)


def _route_marker_points_from_lines(lines: list[WayLineS]) -> list[tuple[float, float]]:
    """从显示航段生成航点标记。注意：相邻共用端点只保留一次。"""

    if not lines:
        return []
    raw = [(lines[0].start.pos.east, lines[0].start.pos.north)]
    raw.extend((line.end.pos.east, line.end.pos.north) for line in lines)
    return _deduplicate_points(raw)


def _deduplicate_points(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """删除相邻重复显示点。注意：保持原始顺序。"""

    result: list[tuple[float, float]] = []
    for point in points:
        if not result or math.hypot(point[0] - result[-1][0], point[1] - result[-1][1]) > 1e-6:
            result.append(point)
    return result


def _geo_reference(origin: GeoOrigin | None) -> GeoReference | None:
    """把 data 层原点转换成应用层对象。"""

    return None if origin is None else GeoReference(origin.latitude_deg, origin.longitude_deg)


def _geo_origin(reference: GeoReference) -> GeoOrigin:
    """把应用层原点转换成 data 层对象。"""

    return GeoOrigin(reference.latitude_deg, reference.longitude_deg)
