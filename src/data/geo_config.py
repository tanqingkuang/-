"""经纬高外部配置与内部 ENU 配置互转。注意：上层负责传入 origin，不在障碍文件内查航线。"""

from __future__ import annotations

import copy
import math
from typing import Iterable

from src.data.geo import GeoOrigin, enu_to_geodetic, geodetic_to_enu


_LAT_KEYS = ("latitude_deg", "lat_deg", "lat")
_LON_KEYS = ("longitude_deg", "lon_deg", "lon")
_ENU_KEY_PAIRS = (
    ("x_m", "y_m"),
    ("east_m", "north_m"),
    ("east", "north"),
)
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
    try:
        latitude, longitude = _read_geodetic_coordinates(raw, "geo origin")
    except ValueError:
        return None
    return GeoOrigin(latitude, longitude)


def _has_geodetic_point(data: object) -> bool:
    """判断对象是否包含经纬度字段。注意：不校验数值范围，交给 float 转换报错。"""
    if not isinstance(data, dict):
        return False
    # 纬度和经度必须同时存在，单边字段按普通 ENU/非法结构处理。
    return any(key in data for key in _LAT_KEYS) and any(key in data for key in _LON_KEYS)


def _has_any_key(data: dict[str, object], keys: Iterable[str]) -> bool:
    """判断对象是否包含任一候选键。注意：只判断字段存在，不读取字段值。"""
    return any(key in data for key in keys)


def _finite_float(value: object, field_name: str) -> float:
    """读取有限浮点数。注意：坐标转换禁止 NaN/Inf 静默传播。"""
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field_name} must be numeric") from exc
    if not math.isfinite(number):
        raise ValueError(f"{field_name} must be finite")
    return number


def _read_required_float(data: dict[str, object], keys: Iterable[str], field_name: str) -> float:
    """按候选键读取必填有限浮点数。注意：缺失时不得回退到坐标原点。"""
    for key in keys:
        if key in data:
            return _finite_float(data[key], field_name)
    raise ValueError(f"{field_name} is required")


def _read_geodetic_coordinates(data: dict[str, object], where: str) -> tuple[float, float]:
    """读取并校验经纬度。注意：纬度和经度必须成对出现且落在合法范围。"""
    has_latitude = _has_any_key(data, _LAT_KEYS)
    has_longitude = _has_any_key(data, _LON_KEYS)
    if has_latitude != has_longitude:
        raise ValueError(f"{where} must provide both latitude and longitude")
    if not has_latitude:
        raise ValueError(f"{where} must be geodetic (latitude + longitude)")

    latitude = _read_required_float(data, _LAT_KEYS, f"{where}.latitude")
    longitude = _read_required_float(data, _LON_KEYS, f"{where}.longitude")
    if not -90.0 <= latitude <= 90.0:
        raise ValueError(f"{where}.latitude must be in [-90, 90]")
    if not -180.0 <= longitude <= 180.0:
        raise ValueError(f"{where}.longitude must be in [-180, 180]")
    return latitude, longitude


def _read_enu_coordinates(data: dict[str, object], where: str) -> tuple[float, float]:
    """读取一对内部 ENU 水平坐标。注意：同一点只能使用一套完整字段。"""
    complete_pairs: list[tuple[str, str]] = []
    for east_key, north_key in _ENU_KEY_PAIRS:
        has_east = east_key in data
        has_north = north_key in data
        if has_east != has_north:
            raise ValueError(f"{where} must provide both {east_key} and {north_key}")
        if has_east:
            complete_pairs.append((east_key, north_key))
    if not complete_pairs:
        raise ValueError(f"{where} must provide ENU coordinates")
    if len(complete_pairs) > 1:
        raise ValueError(f"{where} must use exactly one ENU coordinate representation")

    east_key, north_key = complete_pairs[0]
    return (
        _finite_float(data[east_key], f"{where}.{east_key}"),
        _finite_float(data[north_key], f"{where}.{north_key}"),
    )


def _coordinate_kind(data: object, where: str) -> str:
    """识别单点的 geodetic/enu 表示。注意：禁止半边字段和两套表示混用。"""
    # 坐标种类先按字段集合判定，再读取数值；这样缺半轴不会被默认零掩盖。
    if not isinstance(data, dict):
        raise ValueError(f"{where} must be a point object")
    has_geodetic_key = _has_any_key(data, (*_LAT_KEYS, *_LON_KEYS))
    has_enu_key = any(east_key in data or north_key in data for east_key, north_key in _ENU_KEY_PAIRS)
    if has_geodetic_key and has_enu_key:
        # 经纬度与 ENU 都是完整位置表示，不能靠字段优先级猜调用方意图。
        raise ValueError(f"{where} mixes geodetic and ENU coordinates")
    if has_geodetic_key:
        _read_geodetic_coordinates(data, where)
        return "geodetic"
    if has_enu_key:
        _read_enu_coordinates(data, where)
        return "enu"
    raise ValueError(f"{where} has no supported coordinates")


def _point_to_enu(raw: dict[str, object], origin: GeoOrigin, where: str = "point") -> dict[str, object]:
    """把单个经纬高点转换为内部 ENU 点。注意：高度字段原样作为 altitude_m。"""
    # 转换前先裁决坐标种类，禁止经纬度点夹带任一套 ENU 别名形成双重权威。
    _coordinate_kind(raw, where)
    # 只把经纬度变成 east/north；高度不参与 ECEF/ENU 投影。
    # 增大经度对应东向正值、增大纬度对应北向正值，基准方向由独立 UT 锁定。
    latitude, longitude = _read_geodetic_coordinates(raw, where)
    east, north = geodetic_to_enu(latitude, longitude, origin)
    converted = dict(raw)
    # 保留原经纬字段，便于调试；控制器会优先读取 x_m/y_m。
    converted["x_m"] = east
    converted["y_m"] = north
    converted["altitude_m"] = float(raw.get("altitude_m", raw.get("h", 0.0)))
    return converted


def _point_to_geodetic(raw: dict[str, object], origin: GeoOrigin, where: str = "point") -> dict[str, object]:
    """把内部 ENU 点转换为经纬高点。注意：只转换水平坐标，高度原样输出。"""
    # 支持航线 x_m/y_m、障碍 east_m/north_m 和旧 east/north，缺字段直接报错。
    east, north = _read_enu_coordinates(raw, where)
    lat, lon = enu_to_geodetic(east, north, origin)
    converted = dict(raw)
    # 输出文件面向客户，不再暴露内部 ENU 字段。
    converted.pop("x_m", None)
    converted.pop("y_m", None)
    converted.pop("east_m", None)
    converted.pop("north_m", None)
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
    if not isinstance(waypoints[0], dict) or not _has_geodetic_point(waypoints[0]):
        # 原则：航线必须是经纬度。非经纬(如 ENU x_m/y_m)航线在加载期直接拒绝，
        # 由控制器映射为配置错误、界面提示加载失败。
        raise ValueError(
            "route waypoints must be geodetic (latitude_deg + longitude_deg); "
            "ENU x_m/y_m routes are no longer supported"
        )
    # 按约定使用统一航线第一个航点作为 origin。
    first_latitude, first_longitude = _read_geodetic_coordinates(waypoints[0], "route.waypoints[0]")
    route_origin = origin or GeoOrigin(first_latitude, first_longitude)
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
        point = _point_to_enu(raw, route_origin, f"route.waypoints[{index}]")
        center = point.get("center")
        if isinstance(center, dict):
            if not _has_geodetic_point(center):
                raise ValueError(
                    f"route.waypoints[{index}].center must be geodetic (latitude_deg + longitude_deg); "
                    "ENU x_m/y_m route centers are no longer supported"
                )
            # 已烘焙圆弧的圆心也属于水平几何，必须使用同一个 origin 转 ENU。
            point["center"] = _point_to_enu(center, route_origin, f"route.waypoints[{index}].center")
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
    for index, raw in enumerate(waypoints):
        if not isinstance(raw, dict):
            converted_waypoints.append(raw)
            continue
        point = _point_to_geodetic(raw, origin, f"route.waypoints[{index}]")
        center = raw.get("center")
        if isinstance(center, dict):
            # 圆弧航段的 center 和端点一样输出经纬度，保证文件再次读回几何不丢。
            point["center"] = _point_to_geodetic(center, origin, f"route.waypoints[{index}].center")
        converted_waypoints.append(point)
    resolved["waypoints"] = converted_waypoints
    # 明确移除内存元数据，满足“origin 不写入配置文件”的约定。
    resolved.pop("_geo_origin", None)
    return resolved


def _obstacle_coordinate_kind(obstacle: dict[str, object], index: int) -> str | None:
    """校验单个障碍几何的坐标表示。注意：圆心和全部顶点必须使用同一坐标系。"""
    # 多边形逐点收集坐标种类，避免一个 ENU 顶点被误当成缺省经纬度投到远处。
    kinds: set[str] = set()
    center = obstacle.get("center")
    if center is not None:
        kinds.add(_coordinate_kind(center, f"obstacles[{index}].center"))

    points = obstacle.get("points")
    if points is not None:
        if not isinstance(points, list):
            raise ValueError(f"obstacles[{index}].points must be a list")
        for point_index, point in enumerate(points):
            kinds.add(_coordinate_kind(point, f"obstacles[{index}].points[{point_index}]"))

    if len(kinds) > 1:
        # 同一障碍必须在一次整体投影中保持几何关系，混合坐标无法安全解释。
        raise ValueError(f"obstacles[{index}] mixes geodetic and ENU coordinates")
    return next(iter(kinds), None)


def obstacles_to_internal(obstacles: list[object], origin: GeoOrigin | None) -> list[object]:
    """把外部障碍数组转换为内部 ENU 障碍。注意：经纬障碍必须由上层注入 origin。"""
    converted: list[object] = []
    for obstacle_index, obstacle in enumerate(obstacles):
        if not isinstance(obstacle, dict):
            converted.append(copy.deepcopy(obstacle))
            continue
        coordinate_kind = _obstacle_coordinate_kind(obstacle, obstacle_index)
        if coordinate_kind == "geodetic" and origin is None:
            raise ValueError("geodetic obstacles require route origin")
        if coordinate_kind != "geodetic":
            converted.append(copy.deepcopy(obstacle))
            continue

        assert origin is not None
        item = copy.deepcopy(obstacle)
        center = item.get("center")
        if isinstance(center, dict):
            # 圆障碍只转换中心，半径 radius_m 仍然是米，不做任何缩放。
            latitude, longitude = _read_geodetic_coordinates(center, f"obstacles[{obstacle_index}].center")
            east, north = geodetic_to_enu(latitude, longitude, origin)
            item["center"] = {"east_m": east, "north_m": north}
        points = item.get("points")
        if isinstance(points, list):
            vertices: list[dict[str, float]] = []
            for point_index, point in enumerate(points):
                assert isinstance(point, dict)
                # 四点矩形可能旋转，不能折算成 min/max；转成 polygon 顶点交给后端判定。
                latitude, longitude = _read_geodetic_coordinates(
                    point,
                    f"obstacles[{obstacle_index}].points[{point_index}]",
                )
                east, north = geodetic_to_enu(latitude, longitude, origin)
                vertices.append({"east_m": east, "north_m": north})
            item["type"] = "polygon"
            item["vertices"] = vertices
            item.pop("points", None)
        converted.append(item)
    return converted
