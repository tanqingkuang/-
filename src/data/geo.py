"""WGS84 经纬度与本地 ENU 平面坐标转换。注意：高度按业务字段透传，不参与水平坐标转换。"""

from __future__ import annotations

from dataclasses import dataclass
import math


_WGS84_A_M = 6378137.0
_WGS84_F = 1.0 / 298.257223563
# 第一偏心率平方用于卯酉圈半径计算，保持和 WGS84 标准椭球一致。
_WGS84_E2 = _WGS84_F * (2.0 - _WGS84_F)


@dataclass(frozen=True)
class GeoOrigin:
    """本地 ENU 原点。注意：只使用经纬度定义水平切平面。"""

    latitude_deg: float
    longitude_deg: float


def _origin_basis(origin: GeoOrigin) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """生成 origin 对应的东向、北向单位向量。注意：返回 ECEF 坐标系下的基向量。"""
    lat = math.radians(origin.latitude_deg)
    lon = math.radians(origin.longitude_deg)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)
    # 东向只和经度相关，是 ECEF 水平切向量。
    east = (-sin_lon, cos_lon, 0.0)
    # 北向是沿子午线增加纬度方向的切向量。
    north = (-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat)
    return east, north


def geodetic_to_ecef(latitude_deg: float, longitude_deg: float, altitude_m: float = 0.0) -> tuple[float, float, float]:
    """把 WGS84 大地坐标转换为 ECEF。注意：altitude_m 对本项目水平转换通常传 0。"""
    lat = math.radians(latitude_deg)
    lon = math.radians(longitude_deg)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)
    # radius 是卯酉圈曲率半径，随纬度变化。
    radius = _WGS84_A_M / math.sqrt(1.0 - _WGS84_E2 * sin_lat * sin_lat)
    # 标准大地坐标转 ECEF，不做球面近似。
    x = (radius + altitude_m) * cos_lat * cos_lon
    y = (radius + altitude_m) * cos_lat * sin_lon
    z = (radius * (1.0 - _WGS84_E2) + altitude_m) * sin_lat
    return x, y, z


def ecef_to_geodetic(x_m: float, y_m: float, z_m: float) -> tuple[float, float, float]:
    """把 ECEF 转回 WGS84 大地坐标。注意：用于 ENU 反解的初值和收敛校验。"""
    # 经度可直接由 ECEF x/y 平面反解。
    lon = math.atan2(y_m, x_m)
    horizontal = math.hypot(x_m, y_m)
    # 初值采用常见地表附近近似，随后迭代修正。
    lat = math.atan2(z_m, horizontal * (1.0 - _WGS84_E2))
    altitude = 0.0
    for _ in range(8):
        sin_lat = math.sin(lat)
        radius = _WGS84_A_M / math.sqrt(1.0 - _WGS84_E2 * sin_lat * sin_lat)
        altitude = horizontal / max(math.cos(lat), 1e-15) - radius
        # 该迭代式对地表附近点收敛很快，100 km 切平面反解也稳定。
        lat = math.atan2(z_m, horizontal * (1.0 - _WGS84_E2 * radius / (radius + altitude)))
    return math.degrees(lat), math.degrees(lon), altitude


def geodetic_to_enu(latitude_deg: float, longitude_deg: float, origin: GeoOrigin) -> tuple[float, float]:
    """把经纬度投影到 origin 切平面的 ENU 坐标。注意：只返回 east/north。"""
    # 水平转换不使用业务高度，避免高度变化污染 east/north。
    x, y, z = geodetic_to_ecef(latitude_deg, longitude_deg, 0.0)
    ox, oy, oz = geodetic_to_ecef(origin.latitude_deg, origin.longitude_deg, 0.0)
    dx, dy, dz = x - ox, y - oy, z - oz
    east_axis, north_axis = _origin_basis(origin)
    # ECEF 差向量投影到局部东/北基向量，得到内部 ENU 平面坐标。
    east = dx * east_axis[0] + dy * east_axis[1] + dz * east_axis[2]
    north = dx * north_axis[0] + dy * north_axis[1] + dz * north_axis[2]
    return east, north


def enu_to_geodetic(east_m: float, north_m: float, origin: GeoOrigin) -> tuple[float, float]:
    """把 ENU 坐标反解为经纬度。注意：用牛顿迭代反解投影，避免大范围线性近似误差。"""
    # 先把 ENU 平面点放回 ECEF 切平面，作为经纬度反解初值。
    ox, oy, oz = geodetic_to_ecef(origin.latitude_deg, origin.longitude_deg, 0.0)
    east_axis, north_axis = _origin_basis(origin)
    x = ox + east_m * east_axis[0] + north_m * north_axis[0]
    y = oy + east_m * east_axis[1] + north_m * north_axis[1]
    z = oz + east_m * east_axis[2] + north_m * north_axis[2]
    lat, lon, _ = ecef_to_geodetic(x, y, z)
    for _ in range(10):
        # 用正向投影残差做收敛条件，保证输出经纬度再读回时仍落在目标 ENU。
        cur_e, cur_n = geodetic_to_enu(lat, lon, origin)
        err_e, err_n = cur_e - east_m, cur_n - north_m
        if math.hypot(err_e, err_n) <= 1e-7:
            break
        step_deg = 1e-6
        # 经纬两个方向分别扰动，构造 2x2 数值雅可比。
        lat_e, lat_n = geodetic_to_enu(lat + step_deg, lon, origin)
        lon_e, lon_n = geodetic_to_enu(lat, lon + step_deg, origin)
        # 有限差分雅可比单位是 m/deg，解出来的修正量就是 degree。
        j11 = (lat_e - cur_e) / step_deg
        j21 = (lat_n - cur_n) / step_deg
        j12 = (lon_e - cur_e) / step_deg
        j22 = (lon_n - cur_n) / step_deg
        det = j11 * j22 - j12 * j21
        if abs(det) <= 1e-12:
            break
        # 直接解线性方程 J * delta = -error，通常 2~3 次即可到亚毫米级。
        lat += (-err_e * j22 + j12 * err_n) / det
        lon += (-j11 * err_n + err_e * j21) / det
    return lat, lon
