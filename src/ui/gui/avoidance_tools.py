"""避障配置解析、预览航线转换与规划窗口辅助。注意：只提供 UI 辅助逻辑。"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

from PySide6.QtWidgets import QDialog, QWidget

from src.algorithm.context.leaf_types import WayLineS, WayPointInputS, to_display_inputs
from src.algorithm.entity.leader_follower_hold.leader import waypoint_inputs_to_waylines
from src.algorithm.units.algo.arc_path import arc_radius as _arc_radius_fn, arc_swept_rad
from src.algorithm.units.process.tra_plan.avoidance.obstacle import ObstacleS, make_circle, make_polygon, make_rect
from src.data.config_loader import _LINE_FILE_MANAGER, resolve_config_references
from src.data.geo import GeoOrigin
from src.data.geo_config import geo_origin_from_dict, route_to_external, route_to_internal
from src.ui.gui.view_models import ObstacleView

def _safe_float(value: object, default: float = 0.0) -> float:
    """把任意配置值安全转成 float。注意：非法值（如字符串）返回默认值，避免 UI-only 字段拖垮加载流程。"""
    try:
        return float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return default


def _load_json_config_for_ui(path: str) -> dict[str, object] | None:
    """读取 GUI 需要的 JSON 配置。注意：失败返回 None，避免影响主加载流程。"""
    try:
        with open(path, "r", encoding="utf-8") as handle:
            data = json.load(handle)
        if not isinstance(data, dict):
            return None
    except (OSError, ValueError):
        # GUI 侧解析是辅助显示；配置错误由控制器主加载路径给出正式错误。
        return None
    return data


def _resolve_obstacles_reference_for_ui(data: dict[str, object], path: str) -> dict[str, object] | None:
    """只展开 avoidance.obstacles_file，避免航线文件错误污染障碍 UI 显示。"""
    try:
        # 优先走完整配置展开：经纬度障碍需要基础航线 origin 才能转 ENU。
        return resolve_config_references(data, Path(path))
    except (OSError, ValueError):
        try:
            resolved = dict(data)
            avoidance = resolved.get("avoidance")
            if isinstance(avoidance, dict) and "obstacles_file" in avoidance:
                # 兼容旧 ENU 障碍：坏 route_file 不应把障碍列表清空。
                resolved.pop("route_file", None)
                resolved = resolve_config_references(resolved, Path(path))
            return resolved
        except (OSError, ValueError):
            return None


def _resolve_route_reference_for_ui(data: dict[str, object], path: str) -> dict[str, object] | None:
    """只展开 route_file，供避障规划参数读取航线，避免障碍文件错误污染航线参数。"""
    try:
        resolved = dict(data)
        route_file = resolved.get("route_file")
        if route_file is not None:
            # parse_avoidance_params 只需要 route 与 avoidance 参数，不需要读取障碍库。
            resolved["route"] = _LINE_FILE_MANAGER.load_route(Path(path), route_file)
        route = resolved.get("route")
        if isinstance(route, dict):
            resolved["route"], _origin = route_to_internal(route)
        return resolved
    except (OSError, ValueError):
        return None


def _geo_origin_from_config_for_ui(path: str) -> GeoOrigin | None:
    """从配置 route_file 读取经纬 origin。注意：仅供 GUI 点击坐标显示使用。"""
    raw_data = _load_json_config_for_ui(path)
    data = _resolve_route_reference_for_ui(raw_data, path) if raw_data is not None else None
    route = data.get("route") if isinstance(data, dict) else None
    return geo_origin_from_dict(route.get("_geo_origin")) if isinstance(route, dict) else None


def route_inputs_to_config(
    route: list[WayPointInputS],
    speed_mps: float,
    geo_origin: GeoOrigin | None = None,
) -> dict[str, object]:
    """把避障航点转换为航线文件对象。注意：有 origin 时输出经纬高。"""
    waypoints: list[dict[str, object]] = []
    for point in route:
        # 普通字段保持与 configs/element/line.json 一致，方便人工编辑和复用。
        waypoint: dict[str, object] = {
            "x_m": point.pos.east,
            "y_m": point.pos.north,
            "altitude_m": point.pos.h,
            "R": point.r,
        }
        if point.turnSign != 0.0:
            # 已烘焙圆弧必须带圆心，否则再次加载时无法恢复同一几何。
            waypoint["turn_sign"] = point.turnSign
            waypoint["center"] = {
                "x_m": point.center.east,
                "y_m": point.center.north,
                "altitude_m": point.center.h,
            }
        waypoints.append(waypoint)
    route_config = {"speed_mps": speed_mps, "waypoints": waypoints}
    return route_to_external(route_config, geo_origin) if geo_origin is not None else route_config


def parse_avoidance_config(path: str) -> tuple[list[ObstacleView], float]:
    """从配置 JSON 解析 avoidance 障碍与膨胀间距，供 UI 显示。

    注意：仅读取、不校验飞行约束。本函数为“安全解析”——任何缺失/非法字段都退化为默认值或被跳过，
    绝不抛异常拖垮 _apply_config_path（该流程在控制器加载成功后调用）。
    约定与文档 §4.2 一致：顶层 enabled=false 或 obstacles 为空 → 完全跳过避障，返回空。
    """
    raw_data = _load_json_config_for_ui(path)
    data = _resolve_obstacles_reference_for_ui(raw_data, path) if raw_data is not None else None
    if data is None:
        # 非 JSON（如 YAML）或读取失败时不影响主流程，返回空障碍集。
        return [], 0.0
    avoidance = data.get("avoidance") if isinstance(data, dict) else None
    if not isinstance(avoidance, dict):
        return [], 0.0
    # 顶层总开关：显式关闭即完全跳过避障，等价于现状（不显示、不绘制、不可生成）。
    if not avoidance.get("enabled", True):
        return [], 0.0
    clearance = _safe_float(avoidance.get("clearance_m", 0.0))
    obstacles: list[ObstacleView] = []
    raw_obstacles = avoidance.get("obstacles", [])
    if not isinstance(raw_obstacles, list):
        return [], clearance
    for index, raw in enumerate(raw_obstacles):
        if not isinstance(raw, dict):
            continue
        obstacle_id = str(raw.get("id", f"OB{index + 1}"))
        enabled = bool(raw.get("enabled", True))
        raw_type = str(raw.get("type", "circle"))
        vertices = raw.get("vertices")
        if raw_type == "polygon" and isinstance(vertices, list):
            points: list[tuple[float, float]] = []
            for point in vertices:
                if isinstance(point, dict):
                    points.append((_safe_float(point.get("east_m", 0.0)), _safe_float(point.get("north_m", 0.0))))
            if len(points) >= 3:
                obstacles.append(ObstacleView(obstacle_id=obstacle_id, kind="polygon", enabled=enabled, vertices=points))
        elif raw_type == "rect":
            lo = raw.get("min", {})
            hi = raw.get("max", {})
            lo = lo if isinstance(lo, dict) else {}
            hi = hi if isinstance(hi, dict) else {}
            obstacles.append(
                ObstacleView(
                    obstacle_id=obstacle_id,
                    kind="rect",
                    enabled=enabled,
                    min_x=_safe_float(lo.get("east_m", 0.0)),
                    min_y=_safe_float(lo.get("north_m", 0.0)),
                    max_x=_safe_float(hi.get("east_m", 0.0)),
                    max_y=_safe_float(hi.get("north_m", 0.0)),
                )
            )
        else:
            center = raw.get("center", {})
            center = center if isinstance(center, dict) else {}
            obstacles.append(
                ObstacleView(
                    obstacle_id=obstacle_id,
                    kind="circle",
                    enabled=enabled,
                    center_x=_safe_float(center.get("east_m", 0.0)),
                    center_y=_safe_float(center.get("north_m", 0.0)),
                    radius=_safe_float(raw.get("radius_m", 0.0)),
                )
            )
    return obstacles, clearance


@dataclass
class AvoidanceParams:
    """避障规划参数与长机原航线，从配置 JSON 解析，供 plan_avoidance_route 调用。"""

    turn_radius_m: float = 0.0
    leg_margin_m: float = 0.0
    clearance_m: float = 0.0
    simplify_clearance_m: float = 0.0
    simplify_clearance_explicit: bool = False
    turn_switch_penalty_m: float = 0.0
    turn_angle_weight_m: float = 0.0
    resolution_m: float = 10.0
    margin_m: float = 0.0
    speed_mps: float = 0.0
    allow_arc: bool = True  # 航段自身是否可为曲线：开启则折叠贴障大弧并把拐点烘焙成圆弧段；关闭只留直线骨架+交接圆弧
    waypoints: list[tuple[float, float, float]] = field(default_factory=list)  # (east, north, altitude)
    geo_origin: GeoOrigin | None = None  # 外部经纬高航线 origin；旧 ENU 配置为 None


class AvoidanceWindow(QDialog):
    """避障规划子窗口。注意：控件由 MainWindow 填充，本类只固化窗口元数据。"""

    param_order = [
        # 1. 先锁定飞机物理转弯能力。
        "turn_radius_m",
        # 2. 再确认圆弧之间保留的直线余度。
        "leg_length_margin_m",
        # 3. 安全边界优先于搜索质量参数。
        "clearance_m",
        # 4. 栅格精度决定 A* 离散程度。
        "grid.resolution_m",
        # 5. 搜索包围盒大小影响能否绕开障碍。
        "grid.margin_m",
        # 6. 去冗余参数只在安全约束稳定后调整。
        "simplify_clearance_m",
        # 7. 方向切换惩罚用于减少碎段。
        "turn_switch_penalty_m",
        # 8. 航迹角惩罚最后再微调硬拐。
        "turn_angle_weight_m",
    ]

    def __init__(self, parent: QWidget | None = None) -> None:
        """初始化避障规划窗口。注意：非模态窗口，便于对照主画布预览。"""
        super().__init__(parent)
        self.setWindowTitle("避障规划")
        self.setModal(False)
        self.setMinimumSize(820, 560)


def parse_avoidance_params(path: str) -> AvoidanceParams | None:
    """从配置 JSON 解析避障规划参数与长机航点。注意：安全解析；缺 avoidance/航点不足时返回 None。"""
    raw_data = _load_json_config_for_ui(path)
    data = _resolve_route_reference_for_ui(raw_data, path) if raw_data is not None else None
    if data is None:
        return None
    avoidance = data.get("avoidance")
    if not isinstance(avoidance, dict) or not avoidance.get("enabled", True):
        return None
    grid = avoidance.get("grid") if isinstance(avoidance.get("grid"), dict) else {}
    route = data.get("route") if isinstance(data.get("route"), dict) else {}
    geo_origin = geo_origin_from_dict(route.get("_geo_origin")) if isinstance(route, dict) else None
    raw_waypoints = route.get("waypoints", []) if isinstance(route, dict) else []
    waypoints: list[tuple[float, float, float]] = []
    if isinstance(raw_waypoints, list):
        for raw in raw_waypoints:
            if isinstance(raw, dict):
                # 与控制器 _route_point_from_config 一致：兼容 x_m/east、y_m/north、altitude_m/h 两套字段名。
                waypoints.append(
                    (
                        _safe_float(raw.get("x_m", raw.get("east", 0.0))),
                        _safe_float(raw.get("y_m", raw.get("north", 0.0))),
                        _safe_float(raw.get("altitude_m", raw.get("h", 0.0))),
                    )
                )
    if len(waypoints) < 2:
        return None
    clearance_m = _safe_float(avoidance.get("clearance_m", 0.0))
    simplify_clearance_explicit = "simplify_clearance_m" in avoidance
    simplify_clearance_m = _safe_float(
        avoidance.get("simplify_clearance_m", clearance_m)
    )
    return AvoidanceParams(
        turn_radius_m=_safe_float(avoidance.get("turn_radius_m", 0.0)),
        leg_margin_m=_safe_float(avoidance.get("leg_length_margin_m", 0.0)),
        clearance_m=clearance_m,
        simplify_clearance_m=simplify_clearance_m,
        simplify_clearance_explicit=simplify_clearance_explicit,
        turn_switch_penalty_m=_safe_float(avoidance.get("turn_switch_penalty_m", 0.0)),
        turn_angle_weight_m=_safe_float(avoidance.get("turn_angle_weight_m", 0.0)),
        resolution_m=_safe_float(grid.get("resolution_m", 10.0)) if isinstance(grid, dict) else 10.0,
        margin_m=_safe_float(grid.get("margin_m", 0.0)) if isinstance(grid, dict) else 0.0,
        speed_mps=_safe_float(route.get("speed_mps", 0.0)) if isinstance(route, dict) else 0.0,
        allow_arc=bool(avoidance.get("allow_arc", True)),
        waypoints=waypoints,
        geo_origin=geo_origin,
    )


def _obstacle_view_to_backend(view: "ObstacleView") -> ObstacleS:
    """把 UI 障碍转成后端 ObstacleS（供规划调用）。"""
    if view.kind == "polygon":
        return make_polygon(view.obstacle_id, view.vertices)
    if view.kind == "rect":
        return make_rect(view.obstacle_id, view.min_x, view.min_y, view.max_x, view.max_y)
    return make_circle(view.obstacle_id, view.center_x, view.center_y, view.radius)


def _inflated_polygon_vertices(vertices: list[tuple[float, float]], inflate: float) -> list[tuple[float, float]]:
    """返回用于 GUI 显示的多边形外扩顶点。注意：面向旋转矩形/凸多边形显示近似。"""
    if inflate <= 0.0 or len(vertices) < 3:
        return list(vertices)
    signed_area = 0.0
    # 用有向面积判断顶点绕序，从而确定每条边的外法线方向。
    for (x0, y0), (x1, y1) in zip(vertices, vertices[1:] + vertices[:1]):
        signed_area += x0 * y1 - y0 * x1
    if abs(signed_area) <= 1e-9:
        return _inflate_polygon_from_centroid(vertices, inflate)

    edge_lines: list[tuple[tuple[float, float], tuple[float, float]]] = []
    for start, end in zip(vertices, vertices[1:] + vertices[:1]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.hypot(dx, dy)
        if length <= 1e-9:
            return _inflate_polygon_from_centroid(vertices, inflate)
        # Qt 画布使用 east/north 世界坐标，外扩只影响显示，不改变后端 inside(clearance) 语义。
        if signed_area > 0.0:
            normal = (dy / length, -dx / length)
        else:
            normal = (-dy / length, dx / length)
        # 每条边沿外法线平移 inflate，再取相邻平移边交点作为新的角点。
        point = (start[0] + normal[0] * inflate, start[1] + normal[1] * inflate)
        edge_lines.append((point, (dx, dy)))

    inflated: list[tuple[float, float]] = []
    for index, _ in enumerate(vertices):
        previous_point, previous_dir = edge_lines[index - 1]
        current_point, current_dir = edge_lines[index]
        # 相邻外移边的交点就是凸多边形的外扩角点；平行退化时改用径向兜底。
        intersection = _line_intersection(previous_point, previous_dir, current_point, current_dir)
        if intersection is None:
            return _inflate_polygon_from_centroid(vertices, inflate)
        inflated.append(intersection)
    return inflated


def _line_intersection(
    point_a: tuple[float, float],
    dir_a: tuple[float, float],
    point_b: tuple[float, float],
    dir_b: tuple[float, float],
) -> tuple[float, float] | None:
    """求两条参数直线交点。注意：平行或近似平行时返回 None。"""
    # 二维叉积接近 0 表示两条外移边平行，无法稳定求 miter 角点。
    cross = dir_a[0] * dir_b[1] - dir_a[1] * dir_b[0]
    if abs(cross) <= 1e-9:
        return None
    # 解 point_a + dir_a * t = point_b + dir_b * u，只需要 t 即可还原交点。
    delta = (point_b[0] - point_a[0], point_b[1] - point_a[1])
    t = (delta[0] * dir_b[1] - delta[1] * dir_b[0]) / cross
    return point_a[0] + dir_a[0] * t, point_a[1] + dir_a[1] * t


def _inflate_polygon_from_centroid(vertices: list[tuple[float, float]], inflate: float) -> list[tuple[float, float]]:
    """退化多边形的显示兜底：各顶点沿几何中心径向外推。"""
    center_x = sum(point[0] for point in vertices) / len(vertices)
    center_y = sum(point[1] for point in vertices) / len(vertices)
    inflated: list[tuple[float, float]] = []
    for east, north in vertices:
        dx = east - center_x
        dy = north - center_y
        length = math.hypot(dx, dy)
        if length <= 1e-9:
            inflated.append((east, north))
        else:
            # 兜底路径只保证外扩圈不和本体重合，不承诺精确等距 offset。
            scale = (length + inflate) / length
            inflated.append((center_x + dx * scale, center_y + dy * scale))
    return inflated


def _rounded_inflated_polygon_points(
    vertices: list[tuple[float, float]], inflate: float, corner_segments: int = 6
) -> list[tuple[float, float]]:
    """返回用于 GUI 显示的多边形圆角外扩折线点，与后端 inside() 的圆角膨胀语义一致。

    每条边沿外法线平移 inflate，相邻边在凸顶点用半径 = inflate 的圆弧衔接，圆弧离散成折
    线点。这样角部是圆角（等价 Minkowski 和），而不像 _inflated_polygon_vertices 的 miter
    尖角向外凸出。inflate<=0 或退化时回退到原始顶点/径向兜底。
    """
    if inflate <= 0.0 or len(vertices) < 3:
        return list(vertices)
    signed_area = 0.0
    # 有向面积定绕序，进而确定每条边的外法线方向（与 _inflated_polygon_vertices 保持一致）。
    for (x0, y0), (x1, y1) in zip(vertices, vertices[1:] + vertices[:1]):
        signed_area += x0 * y1 - y0 * x1
    if abs(signed_area) <= 1e-9:
        return _inflate_polygon_from_centroid(vertices, inflate)

    normals: list[tuple[float, float]] = []
    for start, end in zip(vertices, vertices[1:] + vertices[:1]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.hypot(dx, dy)
        if length <= 1e-9:
            return _inflate_polygon_from_centroid(vertices, inflate)
        # 正绕序取右手法线，负绕序取反向，二者都指向多边形外侧。
        if signed_area > 0.0:
            normals.append((dy / length, -dx / length))
        else:
            normals.append((-dy / length, dx / length))

    count = len(vertices)
    points: list[tuple[float, float]] = []
    for index in range(count):
        start = vertices[index]
        end = vertices[(index + 1) % count]
        nx, ny = normals[index]
        # 当前边沿外法线整体平移 inflate，两端点构成一段偏移直段。
        points.append((start[0] + nx * inflate, start[1] + ny * inflate))
        points.append((end[0] + nx * inflate, end[1] + ny * inflate))
        # 在顶点 end 处，用半径 inflate 的圆弧把本边外法线转到下一条边外法线。
        next_nx, next_ny = normals[(index + 1) % count]
        angle_from = math.atan2(ny, nx)
        sweep = math.atan2(next_ny, next_nx) - angle_from
        # 归一到与绕序一致的方向，保证圆弧只走多边形外侧那段。
        if signed_area > 0.0:
            while sweep < 0.0:
                sweep += 2.0 * math.pi
        else:
            while sweep > 0.0:
                sweep -= 2.0 * math.pi
        steps = max(1, int(corner_segments * abs(sweep) / (math.pi / 2.0)))
        for step in range(1, steps):
            angle = angle_from + sweep * (step / steps)
            points.append((end[0] + math.cos(angle) * inflate, end[1] + math.sin(angle) * inflate))
    return points


def _sample_wayline_arc(line: WayLineS, step_deg: float = 6.0) -> list[tuple[float, float]]:
    """采样圆弧航段为折线点（含两端）。注意：复用 arc_swept_rad 求扫掠角。"""
    center = line.start.center
    radius = _arc_radius_fn(line)
    a_start = math.atan2(line.start.pos.north - center.north, line.start.pos.east - center.east)
    swept = arc_swept_rad(line)
    segments = max(1, int(abs(math.degrees(swept)) / step_deg))
    return [
        (
            center.east + radius * math.cos(a_start + swept * (k / segments)),
            center.north + radius * math.sin(a_start + swept * (k / segments)),
        )
        for k in range(segments + 1)
    ]


def route_to_polyline(route: list[WayPointInputS]) -> list[tuple[float, float]]:
    """把 WayPointInputS 列表展开为折线点，供俯视图绘制预览航线。

    显示只画航段几何：去掉转弯信息（交接半径 r），直线航段画直线、曲率航段（turnSign）画弧。
    与配置航线/避障采用后的显示规则一致（见 leaf_types.to_display_inputs）。
    """
    if len(route) < 2:
        return []
    lines = waypoint_inputs_to_waylines(to_display_inputs(route))
    raw: list[tuple[float, float]] = []
    for line in lines:
        if line.start.turnSign != 0.0:
            raw.extend(_sample_wayline_arc(line))
        else:
            raw.append((line.start.pos.east, line.start.pos.north))
            raw.append((line.end.pos.east, line.end.pos.north))
    polyline: list[tuple[float, float]] = []
    for point in raw:
        if not polyline or math.hypot(point[0] - polyline[-1][0], point[1] - polyline[-1][1]) > 1e-6:
            polyline.append(point)
    return polyline


def preview_route_marker_points(route: list[WayPointInputS]) -> list[tuple[float, float]]:
    """把预览航线转换为航点标记坐标。注意：圆弧只标记端点，不标记中间采样点。"""
    if len(route) < 2:
        return []
    # 与预览折线一致先转 display inputs，避免把过渡半径 r 当成真实航点。
    lines = waypoint_inputs_to_waylines(to_display_inputs(route))
    if not lines:
        return []
    raw = [(lines[0].start.pos.east, lines[0].start.pos.north)]
    raw.extend((line.end.pos.east, line.end.pos.north) for line in lines)
    markers: list[tuple[float, float]] = []
    for point in raw:
        # 相邻航段共用端点只画一个黑点，避免连接处显得更粗。
        if not markers or math.hypot(point[0] - markers[-1][0], point[1] - markers[-1][1]) > 1e-6:
            markers.append(point)
    return markers
