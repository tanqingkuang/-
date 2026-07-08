"""3D 态势显示数据适配。注意：这里只做 GUI 坐标映射，不改变仿真坐标语义。"""

from __future__ import annotations

import json
import math
from typing import Iterable

from src.ui.gui.view_models import (
    ObstacleView,
    ReferenceRoute,
    Snapshot,
    reference_route_points,
)

MAX_TRAIL_POINTS_PER_NODE = 28
MAX_ROUTE_POINTS_PER_SEGMENT = 32
DEFAULT_GROUND_MARGIN_M = 420.0
DEFAULT_TERRAIN_SPAN_M = 20000.0
DEFAULT_TERRAIN_RELIEF_M = 2200.0


def build_scene_payload(
    snapshot: Snapshot,
    obstacles: Iterable[ObstacleView] = (),
    *,
    clearance_m: float = 0.0,
) -> dict[str, object]:
    """把 UI 快照转换为 QML 场景数据。注意：输入仍采用 x/y/z=东/北/天。"""

    aircraft = [_aircraft_payload(node) for node in snapshot.nodes]
    trail_ribbons = [
        ribbon
        for node in snapshot.nodes
        for ribbon in _trail_ribbon_payload(node.node_id, node.trail, _node_color(node.role, node.health))
    ]
    route_points = [
        point
        for segment in snapshot.route_segments
        for point in _route_payload(segment)
    ]
    obstacle_items = [_obstacle_payload(obstacle, clearance_m) for obstacle in obstacles if obstacle.enabled]
    bounds = _scene_bounds(aircraft, route_points, obstacle_items)
    terrain = _terrain_payload(bounds)
    return {
        "time": snapshot.time,
        "runState": snapshot.run_state,
        "controlReport": snapshot.control_report,
        "aircraft": aircraft,
        "trailRibbons": trail_ribbons,
        "routePoints": route_points,
        "obstacles": obstacle_items,
        "terrain": terrain,
        "camera": _camera_payload(bounds, aircraft),
        "counts": {
            "aircraft": len(aircraft),
            "trailRibbons": len(trail_ribbons),
            "routePoints": len(route_points),
            "obstacles": len(obstacle_items),
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
    return [
        {
            "nodeId": node_id,
            "color": color,
            "width": 44.0,
            "pathValue": json.dumps(path, ensure_ascii=False, separators=(",", ":")),
        }
    ]


def _route_payload(route: ReferenceRoute) -> list[dict[str, object]]:
    """生成航线采样点显示数据。注意：圆弧按现有二维视图采样口径展开。"""

    xy_points = reference_route_points(route)
    sampled_xy = _evenly_sample(xy_points, MAX_ROUTE_POINTS_PER_SEGMENT)
    count = max(1, len(sampled_xy) - 1)
    points: list[dict[str, object]] = []
    for index, (east_m, north_m) in enumerate(sampled_xy):
        mix = index / count
        altitude_m = route.start_altitude + (route.end_altitude - route.start_altitude) * mix
        coord = enu_to_quick3d(east_m, north_m, altitude_m)
        points.append({"color": "#22d3ee", "size": 9.0, **coord})
    return points


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


def _terrain_payload(bounds: dict[str, float]) -> dict[str, object]:
    """生成连续高度场地形参数。注意：只影响 3D 显示背景，不改变仿真状态。"""

    # 地图默认保持 20km x 20km，较大的仿真范围再按实际包围盒外扩。
    span_x = max(bounds["spanX"], DEFAULT_TERRAIN_SPAN_M)
    span_z = max(bounds["spanZ"], DEFAULT_TERRAIN_SPAN_M)
    center_x = bounds["centerX"]
    center_z = bounds["centerZ"]
    return {
        "ground": {
            "x": center_x,
            "y": -8.0,
            "z": center_z,
            "width": span_x,
            "depth": span_z,
            "height": 16.0,
        },
        "surface": {
            "x": center_x,
            "y": 0.0,
            "z": center_z,
            "width": span_x,
            "depth": span_z,
            "height": DEFAULT_TERRAIN_RELIEF_M,
        },
    }


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
