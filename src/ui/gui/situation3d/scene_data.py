"""3D 态势显示数据适配。注意：这里只做 GUI 坐标映射，不改变仿真坐标语义。"""

from __future__ import annotations

import json
import math
from functools import lru_cache
from pathlib import Path
from typing import Iterable

import numpy as np

from src.ui.gui.situation3d.aircraft_model_style import (
    DEFAULT_AIRCRAFT_MODEL_TYPE,
    AircraftModelType,
    available_model_options,
    create_aircraft_model_style,
)
from src.ui.gui.situation3d.terrain_field import (
    DEFAULT_TERRAIN_RESOLUTION,
    TerrainField,
    generate_terrain_field,
    risk_zones_from_layout,
    terrain_extent_from_layout,
    load_terrain_layout,
)
from src.ui.gui.view_models import (
    ObstacleView,
    ReferenceRoute,
    Snapshot,
    reference_route_points,
)

MAX_TRAIL_POINTS_PER_NODE = 28
ENABLE_TRAIL_SMOOTHING = True
TRAIL_SMOOTHING_PASSES = 2
TRAIL_SMOOTHING_MAX_POINTS = 96
MAX_ROUTE_POINTS_PER_SEGMENT = 32
ROUTE_DASH_LENGTH_M = 140.0
ROUTE_DASH_GAP_M = 90.0
MAX_ROUTE_DASHES_PER_SEGMENT = 96
ROUTE_DASH_WIDTH_M = 16.0
ROUTE_DASH_COLOR = "#22d3ee"
DEFAULT_GROUND_MARGIN_M = 420.0
DEFAULT_TERRAIN_SPAN_M = 20000.0
DEFAULT_TERRAIN_RELIEF_M = 2200.0

# 3D payload 设计说明：
# 1. QML 侧只接收 JSON 字符串，因此 payload 不能携带完整高度场大数组。
# 2. 布局地形只把 layoutFile、resolution、中心和范围传给 TerrainGeometry。
# 3. TerrainGeometry 在 Python/QML 类型内部生成网格，避免 JSON 序列化数十 MB 数据。
# 4. riskZones 是语义数据，riskZoneLines/riskZoneBuffers 是为 QML 简化准备的渲染线段。
# 5. 风险区线段保持米制 Quick3D 坐标，复用 TrailRibbonGeometry 的三角带实现细线。
# 6. 旧配置没有 terrain_display_file 时，payload 仍然走 procedural 地形路径。
# 7. 旧路径的 surface.width/depth/height 字段保持不变，避免历史 QML 绑定失效。
# 8. 新布局模式只影响显示层，不改变 Snapshot 的 ENU 坐标、航线和障碍语义。
# 9. 障碍物 obstacles 仍表示避障模块二维障碍；riskZones 表示手工标记山峰风险罩面。
# 10. 航线虚线、尾迹和风险细线都用同一几何契约：JSON 编码的 Quick3D 三元组数组。
# 11. 红色风险线宽固定为 7m，明显细于历史粗网纹和默认尾迹。
# 12. 淡青缓冲圈拆成 24 个短弧，视觉上是虚线，QML 不需要计算三角函数。
# 13. 风险罩面高度略高于峰顶，避免和地形 z-fighting。
# 14. 缓冲虚线放在山脚高度附近，表达安全缓冲而不是禁飞实体墙。
# 15. 布局文件读取使用 lru_cache，实时刷新快照时不会重复解析 JSON。
# 16. 文件不可读只回退旧地形，不阻断 3D 窗口打开。
# 17. 但 terrain_field 的单元测试会直接读取正式布局，确保验收文件本身有效。
# 18. 相机 bounds 仍由飞机、航线和障碍推导；布局地形范围由 terrain payload 独立声明。
# 19. 这样无地形配置时相机行为完全沿用旧场景。
# 20. 有地形配置时 QML 的 terrainSpan 会辅助俯视/侧视按钮拉开距离。
# 21. riskZones 计数进入 sceneSummary，便于截图时确认布局风险数据已到达 QML。
# 22. 路径字段使用绝对路径传给 QML，避免工作目录变化导致 TerrainGeometry 找不到文件。
# 23. `terrain_display_file` 可以由 Snapshot 或 build_scene_payload 参数传入，测试可直接覆盖。
# 24. payload 字段名使用 QML 现有 camelCase 风格，减少绑定样板。
# 25. 这里不做 P2 的可通行性判断；S 绕航线仅作为冻结布局参考线。
# 26. 原始航线仍来自仿真 snapshot.route_segments，保证飞机飞行坐标语义不变。
# 27. planned_route_uv 暂不进入正式 3D 航线，避免 P1 误导为已接避障。
# 28. 后续 P2 接入时可把规划结果写入 route_segments，同一渲染管线即可显示。
# 29. 颜色和材质在 QML 层控制，本文件只提供几何和数据。
# 30. 单个风险区生成 10 条红色网格线，覆盖但不遮蔽山体细节。
# 31. 线段 y 坐标取风险峰高度，缓冲圈 y 坐标取低值，形成上下层次。
# 32. 通过 json.dumps separators 压缩 pathValue，降低实时 payload 字符串体积。
# 33. 失败回退只吞 IO/JSON/值错误，普通编程错误仍由测试暴露。
# 34. terrain payload 的 ground 字段保留给后续需要地平面或雾墙时复用。
# 35. 这里的 Quick3D z 是 -north，所有风险线同样遵守 enu_to_quick3d。
# 36. route dash 切分仍按空间距离，巡航 900m 地形不会影响虚线节奏。
# 37. 旧 obstacles 与新 riskZones 可同时存在，二者分别显示为柱体和贴地风险罩面。
# 38. scene_data 不缓存生成后的 mesh，避免和 TerrainGeometry 的重建生命周期竞争。
# 39. 如果布局文件变更，改变路径或清理 _cached_layout 即可刷新元数据。
# 40. P1 默认风险区由正式 JSON 手工标记，符合任务书“先由布局 JSON 驱动”的边界。
# 41. 注释中的“显示层”均指 GUI/QML，不包含 runner、algorithm 或 environment 模块。
# 42. 所有数值都用 float 输出，QML ListModel 不需要再做类型归一化。
# 43. 风险线材质发光很弱，最终视觉厚度主要由几何 width 控制。
# 44. 风险缓冲虚线 alpha 更低，避免抢过航线蓝色主视觉。
# 45. payload 中保留 label，后续需要 3D 标注时不用再改 terrain_field。


def build_scene_payload(
    snapshot: Snapshot,
    obstacles: Iterable[ObstacleView] = (),
    *,
    clearance_m: float = 0.0,
    model_type: AircraftModelType = DEFAULT_AIRCRAFT_MODEL_TYPE,
    terrain_display_file: str | None = None,
) -> dict[str, object]:
    """把 UI 快照转换为 QML 场景数据。注意：输入仍采用 x/y/z=东/北/天。"""

    aircraft = [_aircraft_payload(node) for node in snapshot.nodes]
    aircraft_style = create_aircraft_model_style(model_type).style_payload()
    trail_ribbons = [
        ribbon
        for node in snapshot.nodes
        for ribbon in _trail_ribbon_payload(node.node_id, node.trail, _node_color(node.role, node.health))
    ]
    route_segments = _route_segments(snapshot)
    route_polylines = [_route_polyline(segment) for segment in route_segments]
    # 航点球和虚线共用同一组采样折线，避免同一航段被重复展开。
    route_points = [
        point
        for polyline in route_polylines
        for point in _route_payload(polyline)
    ]
    route_dashes = [
        dash
        for polyline in route_polylines
        for dash in _route_dash_payload(polyline)
    ]
    obstacle_items = [_obstacle_payload(obstacle, clearance_m) for obstacle in obstacles if obstacle.enabled]
    bounds = _scene_bounds(aircraft, route_points, obstacle_items)
    display_file = terrain_display_file or snapshot.terrain_display_file
    terrain = _terrain_payload(bounds, display_file)
    risk_zones = list(terrain.get("riskZones", []))
    risk_zone_lines = list(terrain.get("riskZoneLines", []))
    if not risk_zone_lines:
        risk_zone_lines = [
            line
            for zone in risk_zones
            for line in _risk_zone_line_payload(zone)
        ]
    risk_zone_buffers = list(terrain.get("riskZoneBuffers", []))
    if not risk_zone_buffers:
        risk_zone_buffers = [
            dash
            for zone in risk_zones
            for dash in _risk_zone_buffer_payload(zone)
        ]
    camera_payload = _camera_payload(bounds, aircraft)
    if terrain.get("surface", {}).get("mode") == "layout":
        camera_payload = _layout_camera_payload(terrain["surface"])
    return {
        "time": snapshot.time,
        "runState": snapshot.run_state,
        "controlReport": snapshot.control_report,
        "aircraft": aircraft,
        "trailRibbons": trail_ribbons,
        "aircraftStyle": aircraft_style,
        "modelOptions": available_model_options(),
        "routePoints": route_points,
        "routeDashes": route_dashes,
        "obstacles": obstacle_items,
        "terrain": terrain,
        "riskZones": risk_zones,
        "riskZoneLines": risk_zone_lines,
        "riskZoneBuffers": risk_zone_buffers,
        "camera": camera_payload,
        "counts": {
            "aircraft": len(aircraft),
            "trailRibbons": len(trail_ribbons),
            "routePoints": len(route_points),
            "routeDashes": len(route_dashes),
            "obstacles": len(obstacle_items),
            "riskZones": len(risk_zones),
        },
    }


def enu_to_quick3d(east_m: float, north_m: float, up_m: float) -> dict[str, float]:
    """把 ENU 坐标映射到 Qt Quick 3D 坐标。注意：项目内部坐标不随此函数改变。"""

    # Qt Quick 3D 的屏幕前后轴与项目 north 方向相反，只在显示层翻转 z。
    return {
        "x": float(east_m),
        "y": float(up_m),
        "z": float(-north_m),
    }


def _aircraft_payload(node) -> dict[str, object]:  # noqa: ANN001
    """生成单机显示数据。注意：飞机朝向只用于显示，不回写控制器。"""

    coord = enu_to_quick3d(node.x, node.y, node.altitude)
    return {
        "nodeId": node.node_id,
        "role": node.role,
        "health": node.health,
        "color": _node_color(node.role, node.health),
        "yawDeg": _heading_yaw_deg(node.vx, node.vy),
        "speed": math.hypot(node.vx, node.vy),
        **coord,
    }


def _trail_ribbon_payload(node_id: str, trail: list, color: str) -> list[dict[str, object]]:
    """生成连续尾迹拖尾带数据。注意：每架飞机一条 ribbon，避免段间接缝。"""

    if len(trail) < 2:
        return []
    sampled = _evenly_sample(trail, MAX_TRAIL_POINTS_PER_NODE)
    path: list[list[float]] = []
    for point in sampled:
        coord = enu_to_quick3d(point.x, point.y, point.altitude)
        # pathValue 契约是 Quick3D 坐标 [x, y, z] 三元组数组，供 TrailRibbonGeometry 直接解析。
        path.append([coord["x"], coord["y"], coord["z"]])
    path = _smooth_trail_path(path)
    return [
        {
            "nodeId": node_id,
            "color": color,
            "width": 44.0,
            "pathValue": json.dumps(path, ensure_ascii=False, separators=(",", ":")),
        }
    ]


def _smooth_trail_path(path: list[list[float]]) -> list[list[float]]:
    """返回平滑后的尾迹路径。注意：只供尾迹使用，不处理航线或其他 ribbon。"""

    # 手动调试需要原始折线时，把 ENABLE_TRAIL_SMOOTHING 改为 False 即可关闭。
    if not ENABLE_TRAIL_SMOOTHING or len(path) < 3:
        return path
    smoothed = [list(point) for point in path]
    for _ in range(TRAIL_SMOOTHING_PASSES):
        if len(smoothed) >= TRAIL_SMOOTHING_MAX_POINTS:
            break
        smoothed = _chaikin_smooth_once(smoothed)
    return _evenly_sample(smoothed, TRAIL_SMOOTHING_MAX_POINTS)


def _chaikin_smooth_once(points: list[list[float]]) -> list[list[float]]:
    """对折线路径执行一次 Chaikin 平滑。注意：保留首尾点，避免尾迹端点漂移。"""

    smoothed = [points[0]]
    for previous, current in zip(points, points[1:]):
        # Q/R 点分别靠近当前线段的两端，连续迭代后尖角会被圆滑过渡替代。
        q_point = [previous[index] * 0.75 + current[index] * 0.25 for index in range(3)]
        r_point = [previous[index] * 0.25 + current[index] * 0.75 for index in range(3)]
        smoothed.extend([q_point, r_point])
    smoothed.append(points[-1])
    return smoothed


def _route_payload(polyline: list[tuple[float, float, float]]) -> list[dict[str, object]]:
    """生成航线采样点显示数据。注意：圆弧按现有二维视图采样口径展开。"""

    return [{"color": ROUTE_DASH_COLOR, "size": 9.0, "x": x, "y": y, "z": z} for x, y, z in polyline]


def _route_dash_payload(polyline: list[tuple[float, float, float]]) -> list[dict[str, object]]:
    """生成 3D 航线虚线段。注意：每段 dash 复用 ribbon 几何，宽度比尾迹更细。"""

    if len(polyline) < 2:
        return []
    # 先把折线转成累计里程，再按 dash/gap 周期切片；圆弧采样点会自然落入切片中。
    cumulative = _polyline_distances(polyline)
    total_length = cumulative[-1]
    if total_length <= 1e-6:
        return []

    dashes: list[dict[str, object]] = []
    dash_length, period = _route_dash_layout(total_length)
    dash_start = 0.0
    while dash_start < total_length and len(dashes) < MAX_ROUTE_DASHES_PER_SEGMENT:
        dash_end = min(dash_start + dash_length, total_length)
        dash_path = _polyline_slice(polyline, cumulative, dash_start, dash_end)
        if len(dash_path) >= 2:
            # 每段 dash 独立成一个 ribbon，材质层保持连续线宽，段间空隙由模型缺失形成。
            dashes.append(
                {
                    "color": ROUTE_DASH_COLOR,
                    "width": ROUTE_DASH_WIDTH_M,
                    "pathValue": json.dumps(dash_path, ensure_ascii=False, separators=(",", ":")),
                }
            )
        # 无论当前 dash 是否因退化被跳过，都按完整周期推进，保持虚线节奏稳定。
        dash_start += period
    return dashes


def _route_dash_layout(total_length: float) -> tuple[float, float]:
    """返回 dash 长度和周期。注意：超长航段会放大周期，限制 QML delegate 数量。"""

    base_period = ROUTE_DASH_LENGTH_M + ROUTE_DASH_GAP_M
    estimated_count = math.ceil(total_length / base_period)
    if estimated_count <= MAX_ROUTE_DASHES_PER_SEGMENT:
        return ROUTE_DASH_LENGTH_M, base_period
    # 保留原 dash/gap 比例，把同样的虚线节奏均匀铺满整条长航段。
    period = total_length / MAX_ROUTE_DASHES_PER_SEGMENT
    dash_ratio = ROUTE_DASH_LENGTH_M / base_period
    return period * dash_ratio, period


def _route_segments(snapshot: Snapshot) -> list[ReferenceRoute]:
    """返回 3D 需要绘制的参考航段。注意：对齐俯视图的多航段优先、当前航段兜底规则。"""

    if snapshot.route_segments:
        return snapshot.route_segments
    # 单航段或控制器只暴露当前航段时，仍然要显示目标航段虚线。
    if snapshot.route is not None:
        return [snapshot.route]
    return []


def _route_polyline(route: ReferenceRoute) -> list[tuple[float, float, float]]:
    """把单条参考航段采样为 Quick3D 折线点。注意：高度随采样序号线性插值。"""

    xy_points = reference_route_points(route)
    sampled_xy = _evenly_sample(xy_points, MAX_ROUTE_POINTS_PER_SEGMENT)
    count = max(1, len(sampled_xy) - 1)
    points: list[tuple[float, float, float]] = []
    for index, (east_m, north_m) in enumerate(sampled_xy):
        # reference_route_points 只给水平位置，高度按采样序号沿航段端点线性插值。
        mix = index / count
        altitude_m = route.start_altitude + (route.end_altitude - route.start_altitude) * mix
        coord = enu_to_quick3d(east_m, north_m, altitude_m)
        points.append((coord["x"], coord["y"], coord["z"]))
    return points


def _polyline_distances(points: list[tuple[float, float, float]]) -> list[float]:
    """返回折线各点累计距离。注意：3D 虚线按空间距离分段，爬升航段间距也稳定。"""

    distances = [0.0]
    for previous, current in zip(points, points[1:]):
        # 用三维距离切 dash，爬升/下降段不会因为水平距离短而虚线过密。
        distances.append(distances[-1] + math.dist(previous, current))
    return distances


def _polyline_slice(
    points: list[tuple[float, float, float]],
    distances: list[float],
    start_m: float,
    end_m: float,
) -> list[list[float]]:
    """截取折线上的一段路径。注意：保留中间采样点，使圆弧 dash 仍贴合曲线。"""

    sliced: list[list[float]] = [_interpolate_polyline(points, distances, start_m)]
    for point, distance in zip(points[1:-1], distances[1:-1]):
        # 中间点只有落在当前 dash 里才保留，保证每个 dash 自身仍是一条小折线。
        if start_m < distance < end_m:
            sliced.append([point[0], point[1], point[2]])
    sliced.append(_interpolate_polyline(points, distances, end_m))
    return sliced


def _interpolate_polyline(
    points: list[tuple[float, float, float]],
    distances: list[float],
    target_m: float,
) -> list[float]:
    """按累计距离在线性折线上插值。注意：返回 QML 可直接 JSON 化的三元组。"""

    clamped = max(0.0, min(target_m, distances[-1]))
    for index in range(1, len(distances)):
        if clamped <= distances[index]:
            previous_distance = distances[index - 1]
            span = distances[index] - previous_distance
            # 退化子段直接取前点，避免零长航段造成除零。
            mix = 0.0 if span <= 1e-9 else (clamped - previous_distance) / span
            previous = points[index - 1]
            current = points[index]
            return [
                previous[0] + (current[0] - previous[0]) * mix,
                previous[1] + (current[1] - previous[1]) * mix,
                previous[2] + (current[2] - previous[2]) * mix,
            ]
    last = points[-1]
    return [last[0], last[1], last[2]]


def _obstacle_payload(obstacle: ObstacleView, clearance_m: float) -> dict[str, object]:
    """生成障碍/风险区显示数据。注意：二维障碍在 3D 中按无限高柱体近似显示。"""

    # 安全间距在显示层膨胀半径/包围盒，便于和避障预览里的风险边界对应。
    safe_clearance = max(0.0, float(clearance_m))
    column_height_m = 720.0
    if obstacle.kind == "circle":
        radius = max(1.0, obstacle.radius + safe_clearance)
        coord = enu_to_quick3d(obstacle.center_x, obstacle.center_y, column_height_m / 2.0)
        return {
            "kind": "circle",
            "id": obstacle.obstacle_id,
            "radius": radius,
            "width": radius * 2.0,
            "depth": radius * 2.0,
            "height": column_height_m,
            **coord,
        }
    min_x, max_x, min_y, max_y = _obstacle_bounds(obstacle)
    width = max(1.0, max_x - min_x + safe_clearance * 2.0)
    depth = max(1.0, max_y - min_y + safe_clearance * 2.0)
    coord = enu_to_quick3d((min_x + max_x) / 2.0, (min_y + max_y) / 2.0, column_height_m / 2.0)
    return {
        "kind": "box",
        "id": obstacle.obstacle_id,
        "radius": max(width, depth) / 2.0,
        "width": width,
        "depth": depth,
        "height": column_height_m,
        **coord,
    }


def _obstacle_bounds(obstacle: ObstacleView) -> tuple[float, float, float, float]:
    """计算障碍的 ENU 包围盒。注意：polygon/rect 共用此结果做 3D 近似。"""

    if obstacle.vertices:
        xs = [point[0] for point in obstacle.vertices]
        ys = [point[1] for point in obstacle.vertices]
        return min(xs), max(xs), min(ys), max(ys)
    return obstacle.min_x, obstacle.max_x, obstacle.min_y, obstacle.max_y


def _scene_bounds(
    aircraft: list[dict[str, object]],
    route_points: list[dict[str, object]],
    obstacles: list[dict[str, object]],
) -> dict[str, float]:
    """计算场景包围盒。注意：用于地面、山体和初始相机范围。"""

    xs = [float(item["x"]) for item in aircraft + route_points]
    zs = [float(item["z"]) for item in aircraft + route_points]
    ys = [float(item["y"]) for item in aircraft + route_points]
    for obstacle in obstacles:
        half_width = float(obstacle["width"]) / 2.0
        half_depth = float(obstacle["depth"]) / 2.0
        xs.extend([float(obstacle["x"]) - half_width, float(obstacle["x"]) + half_width])
        zs.extend([float(obstacle["z"]) - half_depth, float(obstacle["z"]) + half_depth])
        ys.extend([0.0, float(obstacle["height"])])
    if not xs:
        xs = [-800.0, 800.0]
        zs = [-500.0, 500.0]
        ys = [0.0, 1200.0]
    min_x = min(xs) - DEFAULT_GROUND_MARGIN_M
    max_x = max(xs) + DEFAULT_GROUND_MARGIN_M
    min_z = min(zs) - DEFAULT_GROUND_MARGIN_M
    max_z = max(zs) + DEFAULT_GROUND_MARGIN_M
    return {
        "minX": min_x,
        "maxX": max_x,
        "minZ": min_z,
        "maxZ": max_z,
        "minY": min(ys),
        "maxY": max(ys),
        "centerX": (min_x + max_x) / 2.0,
        "centerZ": (min_z + max_z) / 2.0,
        "spanX": max_x - min_x,
        "spanZ": max_z - min_z,
    }


def _terrain_payload(bounds: dict[str, float], terrain_display_file: str | None = None) -> dict[str, object]:
    """生成连续高度场地形参数。注意：只影响 3D 显示背景，不改变仿真状态。"""

    if terrain_display_file:
        layout_payload = _layout_terrain_payload(terrain_display_file)
        if layout_payload is not None:
            return layout_payload

    # 地图默认保持 20km x 20km，较大的仿真范围再按实际包围盒外扩。
    span_x = max(bounds["spanX"], DEFAULT_TERRAIN_SPAN_M)
    span_z = max(bounds["spanZ"], DEFAULT_TERRAIN_SPAN_M)
    center_x = bounds["centerX"]
    center_z = bounds["centerZ"]
    return {
        "ground": {
            "mode": "procedural",
            "x": center_x,
            "y": -8.0,
            "z": center_z,
            "width": span_x,
            "depth": span_z,
            "height": 16.0,
        },
        "surface": {
            "mode": "procedural",
            "x": center_x,
            "y": 0.0,
            "z": center_z,
            "width": span_x,
            "depth": span_z,
            "height": DEFAULT_TERRAIN_RELIEF_M,
            "layoutFile": "",
            "resolution": 0,
        },
        "riskZones": [],
    }


def _layout_terrain_payload(terrain_display_file: str) -> dict[str, object] | None:
    """生成布局地形 payload。注意：文件不可读时返回 None 触发旧地形回退。"""

    layout_path = str(Path(terrain_display_file).resolve())
    try:
        layout = _cached_layout(layout_path)
    except (OSError, ValueError, json.JSONDecodeError):
        return None
    extent = terrain_extent_from_layout(layout)
    center_east = (extent["min_east_m"] + extent["max_east_m"]) / 2.0
    center_north = (extent["min_north_m"] + extent["max_north_m"]) / 2.0
    width = extent["max_east_m"] - extent["min_east_m"]
    depth = extent["max_north_m"] - extent["min_north_m"]
    detail = layout.get("detail") if isinstance(layout.get("detail"), dict) else {}
    resolution = int(detail.get("grid_resolution", DEFAULT_TERRAIN_RESOLUTION)) if isinstance(detail, dict) else DEFAULT_TERRAIN_RESOLUTION
    effective_span = float(layout.get("map", {}).get("effective_extent_km", 32.0)) * 1000.0 if isinstance(layout.get("map"), dict) else 32000.0
    field = _cached_terrain_field(layout_path, resolution)
    # 风险区线网和地形 mesh 使用同一高度场，避免 QML 里出现悬浮平面。
    risk_zones = [_risk_zone_payload(zone, field) for zone in risk_zones_from_layout(layout)]
    # 线网在 Python 侧提前采样成多点折线，QML 只负责按正式 TrailRibbonGeometry 渲染。
    risk_zone_lines = [
        line
        for zone in risk_zones
        for line in _risk_zone_line_payload(zone, field)
    ]
    # 缓冲圈同样贴地，但 offset 更低，用来表达山脚安全边界。
    risk_zone_buffers = [
        dash
        for zone in risk_zones
        for dash in _risk_zone_buffer_payload(zone, field)
    ]
    return {
        "ground": {
            "mode": "layout",
            "x": center_east,
            "y": -10.0,
            "z": -center_north,
            "width": width,
            "depth": depth,
            "height": 20.0,
        },
        "surface": {
            "mode": "layout",
            "x": center_east,
            "y": 0.0,
            "z": -center_north,
            "width": width,
            "depth": depth,
            "height": max([zone["height"] for zone in risk_zones] + [2600.0]),
            "layoutFile": layout_path,
            "resolution": resolution,
            "effectiveSpan": effective_span,
        },
        "riskZones": risk_zones,
        "riskZoneLines": risk_zone_lines,
        "riskZoneBuffers": risk_zone_buffers,
    }


@lru_cache(maxsize=4)
def _cached_layout(path: str) -> dict[str, object]:
    """缓存布局文件内容。注意：避免每帧刷新重复读取 JSON。"""

    return load_terrain_layout(path)


@lru_cache(maxsize=4)
def _cached_terrain_field(path: str, resolution: int) -> TerrainField:
    """缓存布局高度场。注意：风险区贴地线和 TerrainGeometry 使用同一生成逻辑。"""

    return generate_terrain_field(_cached_layout(path), resolution=resolution)


def _risk_zone_payload(zone, field: TerrainField | None = None) -> dict[str, object]:  # noqa: ANN001
    """生成风险区显示数据。注意：中心坐标转换到 Quick3D，半径仍为米。"""

    center_height = _sample_field_height(field, zone.east_m, zone.north_m) if field is not None else zone.height_m
    # 中心高度只作为语义参考，实际罩面形状由每条风险线逐点采样决定。
    coord = enu_to_quick3d(zone.east_m, zone.north_m, center_height + 30.0)
    foot = enu_to_quick3d(zone.east_m, zone.north_m, center_height + 18.0)
    return {
        "id": zone.zone_id,
        "label": zone.label,
        "radius": zone.radius_m,
        "bufferRadius": zone.buffer_radius_m,
        "height": zone.height_m,
        "x": coord["x"],
        "y": coord["y"],
        "z": coord["z"],
        "footY": foot["y"],
    }


def _risk_zone_line_payload(zone: dict[str, object], field: TerrainField | None = None) -> list[dict[str, object]]:
    """生成风险区红色细线罩面。注意：线宽显著小于航线和尾迹。"""

    center_x = float(zone["x"])
    center_z = float(zone["z"])
    y = float(zone["y"])
    radius = float(zone["radius"])
    lines: list[dict[str, object]] = []
    for ratio in (-0.66, -0.33, 0.0, 0.33, 0.66):
        half = radius * math.sqrt(max(0.0, 1.0 - ratio * ratio))
        offset = radius * ratio
        for angle in (0.0, math.pi / 2.0):
            cos_a = math.cos(angle)
            sin_a = math.sin(angle)
            p1 = [center_x + offset * -sin_a - half * cos_a, y, center_z + offset * cos_a - half * sin_a]
            p2 = [center_x + offset * -sin_a + half * cos_a, y, center_z + offset * cos_a + half * sin_a]
            path = _surface_line_path(field, p1, p2, 34.0) if field is not None else [p1, p2]
            lines.append(
                {
                    "color": "#ff5a45",
                    "width": 18.0,
                    "pathValue": json.dumps(path, ensure_ascii=False, separators=(",", ":")),
                }
            )
    return lines


def _risk_zone_buffer_payload(zone: dict[str, object], field: TerrainField | None = None) -> list[dict[str, object]]:
    """生成风险区山脚淡青虚线缓冲轮廓。注意：用短弧段组成虚线，减少 QML 逻辑。"""

    center_x = float(zone["x"])
    center_z = float(zone["z"])
    y = float(zone["footY"])
    radius = float(zone["bufferRadius"])
    dashes: list[dict[str, object]] = []
    dash_count = 24
    dash_angle = math.tau / dash_count * 0.55
    for index in range(dash_count):
        start = index * math.tau / dash_count
        points = []
        for sample in range(4):
            angle = start + dash_angle * sample / 3.0
            x = center_x + math.cos(angle) * radius
            z = center_z - math.sin(angle) * radius
            points.append([x, _sample_quick3d_height(field, x, z, 18.0) if field is not None else y, z])
        dashes.append(
            {
                "color": "#62d9e7",
                "width": 12.0,
                "pathValue": json.dumps(points, ensure_ascii=False, separators=(",", ":")),
            }
        )
    return dashes


def _surface_line_path(field: TerrainField, start: list[float], end: list[float], offset_m: float) -> list[list[float]]:
    """把水平线段披挂到地形表面。注意：输入输出均为 Quick3D 坐标。"""

    samples = 18
    points: list[list[float]] = []
    for index in range(samples):
        ratio = index / max(1, samples - 1)
        # 在水平线段上均匀取样，再把每个点投到高度场上。
        x = float(start[0]) + (float(end[0]) - float(start[0])) * ratio
        z = float(start[2]) + (float(end[2]) - float(start[2])) * ratio
        points.append([x, _sample_quick3d_height(field, x, z, offset_m), z])
    return points


def _sample_quick3d_height(field: TerrainField | None, x_m: float, z_m: float, offset_m: float) -> float:
    """采样 Quick3D 坐标下的地形高度。注意：Quick3D z 轴为 north 取反。"""

    if field is None:
        return float(offset_m)
    return _sample_field_height(field, float(x_m), -float(z_m)) + offset_m


def _sample_field_height(field: TerrainField, east_m: float, north_m: float) -> float:
    """双线性采样高度场。注意：坐标越界时夹到最近网格边界。"""

    min_e = field.center_east_m - field.width_m / 2.0
    min_n = field.center_north_m - field.depth_m / 2.0
    u = np.clip((float(east_m) - min_e) / max(field.width_m, 1.0), 0.0, 1.0) * (field.resolution - 1)
    v = np.clip((float(north_m) - min_n) / max(field.depth_m, 1.0), 0.0, 1.0) * (field.resolution - 1)
    # 高度场行列方向仍是 ENU north/east，Quick3D 的 z 翻转已在调用侧处理。
    col = int(math.floor(u))
    row = int(math.floor(v))
    col1 = min(col + 1, field.resolution - 1)
    row1 = min(row + 1, field.resolution - 1)
    tx = float(u - col)
    ty = float(v - row)
    h00 = float(field.heights_m[row, col])
    h10 = float(field.heights_m[row, col1])
    h01 = float(field.heights_m[row1, col])
    h11 = float(field.heights_m[row1, col1])
    return (h00 * (1.0 - tx) + h10 * tx) * (1.0 - ty) + (h01 * (1.0 - tx) + h11 * tx) * ty


def _camera_payload(bounds: dict[str, float], aircraft: list[dict[str, object]]) -> dict[str, float]:
    """生成初始相机参数。注意：QML 仍允许用户拖拽、缩放和重置。"""

    focus_x = _average([float(item["x"]) for item in aircraft], bounds["centerX"])
    focus_y = _average([float(item["y"]) for item in aircraft], max(320.0, bounds["maxY"] * 0.55))
    focus_z = _average([float(item["z"]) for item in aircraft], bounds["centerZ"])
    # 相机距离跟随 20km 基准跨度，避免初始视角只看到局部山包。
    span = max(bounds["spanX"], bounds["spanZ"], bounds["maxY"] - bounds["minY"], DEFAULT_TERRAIN_SPAN_M)
    return {
        "focusX": focus_x,
        "focusY": focus_y,
        "focusZ": focus_z,
        "distance": max(950.0, span * 1.15),
        "yaw": -38.0,
        "pitch": -34.0,
    }


def _layout_camera_payload(surface: dict[str, object]) -> dict[str, float]:
    """生成布局地形相机。注意：取景按内容跨度，不按 90km 渲染裙边。"""

    effective_span = float(surface.get("effectiveSpan", 32000.0))
    # 定稿布局是"平原+走廊"的稀疏构图，大俯角远机位会让暗平原占满画面；
    # 默认机位改为顺峡谷走廊的低角度近景，让两侧受光山坡填充画面下部（style_a 同族构图）。
    distance = max(8200.0, effective_span * 0.30)
    return {
        # 焦点略偏走廊东段，主光来自西南，北山链朝南受光大坡正对相机成为画面主体。
        "focusX": float(surface.get("x", 0.0)) + effective_span * 0.05,
        "focusY": 900.0,
        "focusZ": float(surface.get("z", 0.0)),
        "distance": distance,
        "yaw": -62.0,
        "pitch": -20.0,
    }


def _node_color(role: str, health: str) -> str:
    """返回节点显示颜色。注意：异常健康状态优先于角色颜色。"""

    if health != "normal":
        return "#f59e0b"
    if role.strip().lower() in {"leader", "rally_leader"}:
        return "#60a5fa"
    return "#c084fc"


def _heading_yaw_deg(vx_mps: float, vy_mps: float) -> float:
    """把 ENU 水平速度转换为 Quick3D 绕 Y 轴偏航角。注意：显示模型机头默认朝 +X。

    Quick3D 的 z 轴在坐标映射时已相对 ENU north 取反(见 enu_to_quick3d)，
    Y 轴偏航角的正方向与该取反抵消，此处不能再对 vy 取负，否则斜向航向会被镜像。
    """

    if math.hypot(vx_mps, vy_mps) < 1e-6:
        return 0.0
    return math.degrees(math.atan2(vy_mps, vx_mps))


def _evenly_sample(items: list, limit: int) -> list:
    """按上限均匀抽样。注意：保留首尾点，避免轨迹端点丢失。"""

    if len(items) <= limit:
        return list(items)
    if limit <= 1:
        return [items[-1]]
    last = len(items) - 1
    return [items[round(last * index / (limit - 1))] for index in range(limit)]


def _average(values: list[float], fallback: float) -> float:
    """求平均值。注意：空列表使用 fallback。"""

    return sum(values) / len(values) if values else fallback
