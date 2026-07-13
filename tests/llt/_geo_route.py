"""测试辅助：把 ENU 航线转成等价经纬度航线。

产品约定：JSON 航线只支持经纬度（latitude_deg/longitude_deg），加载期由
route_to_internal 转回内部 ENU。历史测试用 ENU(x_m/y_m) 写航线，这里提供转换器，
使这些测试改用经纬度航线而语义不变。

注意：route_to_internal 会把首航点重定为 ENU 原点(0,0)。故只有首航点本就是 (0,0)
的航线，转换后各航点 ENU 值不变；首航点非 (0,0) 的航线会整体平移到以首点为原点。
"""

from __future__ import annotations

from src.data.geo import GeoOrigin, enu_to_geodetic

# 固定锚点（北京附近），与 configs/element/line.json 一致，仅用于把测试 ENU 值映射到经纬。
GEO_ANCHOR = GeoOrigin(latitude_deg=39.0, longitude_deg=116.0)


def enu_waypoint_to_geodetic(waypoint: dict) -> dict:
    """把单个 ENU 航点(x_m/y_m)转成经纬航点(latitude_deg/longitude_deg)。注意：其余字段(altitude_m/R 等)原样保留。"""
    east = float(waypoint.get("x_m", waypoint.get("east", 0.0)))
    north = float(waypoint.get("y_m", waypoint.get("north", 0.0)))
    latitude_deg, longitude_deg = enu_to_geodetic(east, north, GEO_ANCHOR)
    converted = {key: value for key, value in waypoint.items() if key not in ("x_m", "y_m", "east", "north")}
    converted["latitude_deg"] = latitude_deg
    converted["longitude_deg"] = longitude_deg
    return converted


def geodetic_route(route: dict) -> dict:
    """把一个 route 对象里的 ENU waypoints 转成经纬 waypoints。注意：非 dict 航点或已是经纬的原样保留。"""
    converted = dict(route)
    waypoints = route.get("waypoints")
    if isinstance(waypoints, list):
        converted["waypoints"] = [
            enu_waypoint_to_geodetic(wp) if isinstance(wp, dict) and ("x_m" in wp or "east" in wp) else wp
            for wp in waypoints
        ]
    return converted


def geodetic_config(config: dict) -> dict:
    """把配置里的 route ENU 航线转成经纬航线。注意：不改动 nodes/formation/obstacles。"""
    converted = dict(config)
    route = converted.get("route")
    if isinstance(route, dict):
        converted["route"] = geodetic_route(route)
    return converted
