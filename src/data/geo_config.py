"""经纬高外部配置与内部 ENU 配置互转。注意：上层负责传入 origin，不在障碍文件内查航线。"""

from __future__ import annotations

import copy
from typing import Iterable

from src.data.geo import GeoOrigin, enu_to_geodetic, geodetic_to_enu


_LAT_KEYS = ("latitude_deg", "lat_deg", "lat")
_LON_KEYS = ("longitude_deg", "lon_deg", "lon")
_GEO_DECIMALS = 7


def format_geodetic_degree(value: float) -> float:
    """格式化对外经纬度。注意：统一保留 7 位小数，内部计算不使用该截断值。"""
    return round(value, _GEO_DECIMALS)


def geo_origin_to_dict(origin: GeoOrigin) -> dict[str, float]:
    """把 GeoOrigin 转成可嵌入配置副本的字典。注意：不写回用户文件。"""
    return {"latitude_deg": format_geodetic_degree(origin.latitude_deg), "longitude_deg": format_geodetic_degree(origin.longitude_deg)}


def geo_origin_from_dict(raw: object) -> GeoOrigin | None:
    """从配置副本读取 origin。注意：非法结构返回 None。"""
    if not isinstance(raw, dict):
        return None
    if not _has_geodetic_point(raw):
        return None
    return GeoOrigin(_read_float(raw, _LAT_KEYS), _read_float(raw, _LON_KEYS))


def _read_float(data: dict[str, object], keys: Iterable[str], default: float = 0.0) -> float:
    """按候选键读取浮点数。注意：用于兼容简写和正式字段名。"""
    # 正式字段优先，兼容字段只作为输入便利。
    for key in keys:
        if key in data:
            return float(data[key])
    return default


def _has_geodetic_point(data: object) -> bool:
    """判断对象是否包含经纬度字段。注意：不校验数值范围，交给 float 转换报错。"""
    if not isinstance(data, dict):
        return False
    # 纬度和经度必须同时存在，单边字段按普通 ENU/非法结构处理。
    return any(key in data for key in _LAT_KEYS) and any(key in data for key in _LON_KEYS)


def _point_to_enu(raw: dict[str, object], origin: GeoOrigin) -> dict[str, object]:
    """把单个经纬高点转换为内部 ENU 点。注意：高度字段原样作为 altitude_m。"""
    # 只把经纬度变成 east/north；高度不参与 ECEF/ENU 投影。
    east, north = geodetic_to_enu(_read_float(raw, _LAT_KEYS), _read_float(raw, _LON_KEYS), origin)
    converted = dict(raw)
    # 保留原经纬字段，便于调试；控制器会优先读取 x_m/y_m。
    converted["x_m"] = east
    converted["y_m"] = north
    converted["altitude_m"] = float(raw.get("altitude_m", raw.get("h", 0.0)))
    return converted


def _point_to_geodetic(raw: dict[str, object], origin: GeoOrigin) -> dict[str, object]:
    """把内部 ENU 点转换为经纬高点。注意：只转换水平坐标，高度原样输出。"""
    # 支持 x_m/y_m 和 east/north 两套内部字段，统一转成经纬度。
    lat, lon = enu_to_geodetic(float(raw.get("x_m", raw.get("east", 0.0))), float(raw.get("y_m", raw.get("north", 0.0))), origin)
    converted = dict(raw)
    # 输出文件面向客户，不再暴露内部 ENU 字段。
    converted.pop("x_m", None)
    converted.pop("y_m", None)
    converted.pop("east", None)
    converted.pop("north", None)
    converted["latitude_deg"] = format_geodetic_degree(lat)
    converted["longitude_deg"] = format_geodetic_degree(lon)
    converted["altitude_m"] = float(raw.get("altitude_m", raw.get("h", 0.0)))
    return converted


def route_to_internal(route: dict[str, object], origin: GeoOrigin | None = None) -> tuple[dict[str, object], GeoOrigin | None]:
    """把外部航线对象转换为内部 ENU route。注意：origin 缺省时取第一个经纬航点。"""
    resolved = copy.deepcopy(route)
    waypoints = resolved.get("waypoints")
    if not isinstance(waypoints, list) or not waypoints:
        # 无 waypoints（空列表 / segments 等其它写法）：无经纬信息可转，原样透传。
        return resolved, origin
    if not _has_geodetic_point(waypoints[0]):
        # 原则：航线必须是经纬度。非经纬(如 ENU x_m/y_m)航线在加载期直接拒绝，
        # 由控制器映射为配置错误、界面提示加载失败。
        raise ValueError(
            "route waypoints must be geodetic (latitude_deg + longitude_deg); "
            "ENU x_m/y_m routes are no longer supported"
        )
    # 按约定使用基础航线第一个航点作为 origin；上层可为 rally_route 传入同一 origin。
    route_origin = origin or GeoOrigin(_read_float(waypoints[0], _LAT_KEYS), _read_float(waypoints[0], _LON_KEYS))
    converted_waypoints: list[object] = []
    for index, raw in enumerate(waypoints):
        if not isinstance(raw, dict):
            converted_waypoints.append(raw)
            continue
        if not _has_geodetic_point(raw):
            raise ValueError(
                f"route.waypoints[{index}] must be geodetic (latitude_deg + longitude_deg); "
                "ENU x_m/y_m routes are no longer supported"
            )
        point = _point_to_enu(raw, route_origin)
        center = point.get("center")
        if isinstance(center, dict):
            if not _has_geodetic_point(center):
                raise ValueError(
                    f"route.waypoints[{index}].center must be geodetic (latitude_deg + longitude_deg); "
                    "ENU x_m/y_m route centers are no longer supported"
                )
            # 已烘焙圆弧的圆心也属于水平几何，必须使用同一个 origin 转 ENU。
            point["center"] = _point_to_enu(center, route_origin)
        converted_waypoints.append(point)
    resolved["waypoints"] = converted_waypoints
    # origin 只放在内存配置副本，供障碍转换和航线输出复用；不写回用户文件。
    resolved["_geo_origin"] = geo_origin_to_dict(route_origin)
    return resolved, route_origin


def route_to_external(route: dict[str, object], origin: GeoOrigin) -> dict[str, object]:
    """把内部 ENU route 转换为外部经纬高 route。注意：用于避障航线输出。"""
    resolved = copy.deepcopy(route)
    waypoints = resolved.get("waypoints")
    if not isinstance(waypoints, list):
        return resolved
    converted_waypoints: list[object] = []
    for raw in waypoints:
        if not isinstance(raw, dict):
            converted_waypoints.append(raw)
            continue
        point = _point_to_geodetic(raw, origin)
        center = raw.get("center")
        if isinstance(center, dict):
            # 圆弧航段的 center 和端点一样输出经纬度，保证文件再次读回几何不丢。
            point["center"] = _point_to_geodetic(center, origin)
        converted_waypoints.append(point)
    resolved["waypoints"] = converted_waypoints
    # 明确移除内存元数据，满足“origin 不写入配置文件”的约定。
    resolved.pop("_geo_origin", None)
    return resolved


def _obstacle_has_geodetic(obstacle: object) -> bool:
    """判断障碍是否含经纬度坐标。注意：圆心或四点任一满足即可。"""
    if not isinstance(obstacle, dict):
        return False
    center = obstacle.get("center")
    if _has_geodetic_point(center):
        return True
    points = obstacle.get("points")
    # 旋转矩形使用 points[4]，只要任一点为经纬度就要求 origin。
    return isinstance(points, list) and any(_has_geodetic_point(point) for point in points)


def obstacles_to_internal(obstacles: list[object], origin: GeoOrigin | None) -> list[object]:
    """把外部障碍数组转换为内部 ENU 障碍。注意：经纬障碍必须由上层注入 origin。"""
    if origin is None and any(_obstacle_has_geodetic(obstacle) for obstacle in obstacles):
        raise ValueError("geodetic obstacles require route origin")
    converted: list[object] = []
    for obstacle in obstacles:
        if not isinstance(obstacle, dict) or origin is None:
            converted.append(copy.deepcopy(obstacle))
            continue
        item = copy.deepcopy(obstacle)
        center = item.get("center")
        if isinstance(center, dict) and _has_geodetic_point(center):
            # 圆障碍只转换中心，半径 radius_m 仍然是米，不做任何缩放。
            east, north = geodetic_to_enu(_read_float(center, _LAT_KEYS), _read_float(center, _LON_KEYS), origin)
            item["center"] = {"east_m": east, "north_m": north}
        points = item.get("points")
        if isinstance(points, list) and any(_has_geodetic_point(point) for point in points):
            vertices: list[dict[str, float]] = []
            for point in points:
                if not isinstance(point, dict):
                    raise ValueError("rect.points must contain point objects")
                # 四点矩形可能旋转，不能折算成 min/max；转成 polygon 顶点交给后端判定。
                east, north = geodetic_to_enu(_read_float(point, _LAT_KEYS), _read_float(point, _LON_KEYS), origin)
                vertices.append({"east_m": east, "north_m": north})
            item["type"] = "polygon"
            item["vertices"] = vertices
            item.pop("points", None)
        converted.append(item)
    return converted
