"""生成演示地形布局候选方案图。

输出 3 个方案的俯视布局图和 style A 斜俯视效果图。
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib
import numpy as np
from PySide6.QtCore import QByteArray, Property, QEventLoop, QTimer, QUrl, Signal
from PySide6.QtGui import QGuiApplication, QVector3D
from PySide6.QtQml import qmlRegisterType
from PySide6.QtQuick import QQuickView
from PySide6.QtQuick3D import QQuick3DGeometry

from generate_previews import (
    box_blur,
    lerp_color,
    smoothstep,
    srgb_to_linear,
    warped_ridged_fbm,
)

matplotlib.use("Agg")
from matplotlib import pyplot as plt  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402
from matplotlib.patches import Ellipse  # noqa: E402

WIDTH_PX = 1600
HEIGHT_PX = 900
TOP_WIDTH_PX = 1600
TOP_HEIGHT_PX = 1200
ROUTE_ALTITUDE_M = 900.0
FLOAT_SIZE = 4
VERTEX_COMPONENTS = 12
VERTEX_STRIDE = VERTEX_COMPONENTS * FLOAT_SIZE
SCENE: "LayoutScene | None" = None


@dataclass(frozen=True)
class Peak:
    """布局峰体定义。u/v 为航线坐标系 km，半径单位 km，高度单位 m。"""

    u: float
    v: float
    height: float
    radius_u: float
    radius_v: float
    angle_deg: float
    role: str
    label: str = ""


@dataclass(frozen=True)
class RidgeSegment:
    """山脉链相邻峰之间的脊线定义。"""

    start_u: float
    start_v: float
    end_u: float
    end_v: float
    height: float
    width_km: float
    role: str = "ridge"
    label: str = ""


@dataclass(frozen=True)
class Saddle:
    """需要在布局图上标注的鞍部。"""

    u: float
    v: float
    height: float
    label: str


@dataclass(frozen=True)
class LayoutSpec:
    """单个候选方案定义。"""

    key: str
    title: str
    description: str
    peaks: tuple[Peak, ...]
    route_offsets: tuple[tuple[float, float, float], ...]
    ridges: tuple[RidgeSegment, ...] = ()
    saddles: tuple[Saddle, ...] = ()
    original_route_uv: tuple[tuple[float, float], ...] = ()
    planned_route_uv: tuple[tuple[float, float], ...] = ()
    chain_polylines: tuple[tuple[tuple[float, float], ...], ...] = ()


def route_frame() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """返回航线坐标系起点、前向和左法向。"""

    start = np.array([-13.0, -7.2], dtype=np.float32)
    angle = math.radians(35.0)
    forward = np.array([math.cos(angle), math.sin(angle)], dtype=np.float32)
    left = np.array([-forward[1], forward[0]], dtype=np.float32)
    return start, forward, left


def uv_to_xy(u_coord: np.ndarray | float, v_coord: np.ndarray | float) -> tuple[np.ndarray, np.ndarray] | tuple[float, float]:
    """把航线坐标 u/v 转为 ENU x/y km。"""

    start, forward, left = route_frame()
    u_arr = np.asarray(u_coord, dtype=np.float32)
    v_arr = np.asarray(v_coord, dtype=np.float32)
    xy = start[:, None] + forward[:, None] * u_arr.reshape(1, -1) + left[:, None] * v_arr.reshape(1, -1)
    x = xy[0].reshape(u_arr.shape)
    y = xy[1].reshape(u_arr.shape)
    if np.isscalar(u_coord) and np.isscalar(v_coord):
        return float(x), float(y)
    return x, y


def xy_to_uv(x_coord: np.ndarray, y_coord: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """把 ENU x/y km 转为航线坐标 u/v。"""

    start, forward, left = route_frame()
    dx = x_coord - start[0]
    dy = y_coord - start[1]
    u_coord = dx * forward[0] + dy * forward[1]
    v_coord = dx * left[0] + dy * left[1]
    return u_coord, v_coord


def route_lateral_offset(spec: LayoutSpec, u_coord: np.ndarray) -> np.ndarray:
    """按方案返回重规划航线横向偏移。"""

    offset = np.zeros_like(u_coord, dtype=np.float32)
    for center_u, amplitude, sigma in spec.route_offsets:
        offset += amplitude * np.exp(-0.5 * ((u_coord - center_u) / sigma) ** 2)
    return offset.astype(np.float32)


def interpolate_uv_route(waypoints_uv: tuple[tuple[float, float], ...], count: int) -> np.ndarray:
    """按 u/v 航点折线插值并转成 ENU 点列。"""

    uv_points = np.array(waypoints_uv, dtype=np.float32)
    if uv_points.shape[0] < 2:
        raise ValueError("航线至少需要两个航点")
    distance = [0.0]
    for first, second in zip(uv_points[:-1], uv_points[1:]):
        distance.append(distance[-1] + float(np.linalg.norm(second - first)))
    distance_arr = np.array(distance, dtype=np.float32)
    targets = np.linspace(0.0, float(distance_arr[-1]), count, dtype=np.float32)
    u_values = np.interp(targets, distance_arr, uv_points[:, 0]).astype(np.float32)
    v_values = np.interp(targets, distance_arr, uv_points[:, 1]).astype(np.float32)
    x, y = uv_to_xy(u_values, v_values)
    return np.column_stack([x, y]).astype(np.float32)


def original_route(spec: LayoutSpec | None = None, count: int = 180) -> np.ndarray:
    """生成红色原始航线点列，单位 km。"""

    if spec is not None and spec.original_route_uv:
        return interpolate_uv_route(spec.original_route_uv, count)
    u = np.linspace(0.0, 26.0, count, dtype=np.float32)
    x, y = uv_to_xy(u, np.zeros_like(u))
    return np.column_stack([x, y]).astype(np.float32)


def planned_route(spec: LayoutSpec, count: int = 220) -> np.ndarray:
    """生成青色重规划航线点列，单位 km。"""

    if spec.planned_route_uv:
        return interpolate_uv_route(spec.planned_route_uv, count)
    u = np.linspace(0.0, 26.0, count, dtype=np.float32)
    v = route_lateral_offset(spec, u)
    x, y = uv_to_xy(u, v)
    return np.column_stack([x, y]).astype(np.float32)


def make_layout_specs() -> dict[str, LayoutSpec]:
    """定义三个候选方案。"""

    common_background = (
        Peak(4.0, 5.8, 980, 2.5, 1.6, 12, "background", "远山 980m"),
        Peak(9.0, -5.5, 1280, 2.3, 1.4, -18, "background", "远山 1280m"),
        Peak(22.5, 4.8, 1250, 2.4, 1.5, 24, "background", "远山 1250m"),
        Peak(24.0, -4.0, 980, 2.1, 1.3, -28, "background", "远山 980m"),
    )
    valley_standard = (
        Peak(7.6, 2.35, 1780, 2.0, 1.15, -12, "valley", "北侧 1780m"),
        Peak(8.5, -2.40, 1680, 1.9, 1.10, 18, "valley", "南侧 1680m"),
        Peak(10.8, 2.55, 1960, 2.1, 1.20, 24, "valley", "北侧 1960m"),
        Peak(11.1, -2.55, 1820, 2.0, 1.10, -22, "valley", "南侧 1820m"),
    )
    valley_narrow = (
        Peak(7.3, 1.30, 2200, 2.0, 0.95, -14, "valley", "北侧 2200m"),
        Peak(8.4, -1.35, 2080, 1.8, 0.90, 22, "valley", "南侧 2080m"),
        Peak(10.7, 1.40, 2380, 2.0, 1.00, 20, "valley", "北侧 2380m"),
        Peak(11.3, -1.45, 2240, 1.9, 0.95, -24, "valley", "南侧 2240m"),
    )
    low_hills = (
        Peak(6.6, 0.20, 420, 0.55, 0.45, 0, "low", "低丘 420m"),
        Peak(8.8, -0.45, 540, 0.60, 0.45, 0, "low", "低丘 540m"),
        Peak(10.4, 0.35, 470, 0.55, 0.42, 0, "low", "低丘 470m"),
    )
    return {
        "a": LayoutSpec(
            "a",
            "方案 A：单峰绕飞",
            "段3 一座大主峰挡路，重规划航线形成单次平滑大弧。",
            common_background
            + valley_standard
            + low_hills
            + (
                Peak(16.1, 0.05, 2480, 1.65, 1.35, 12, "obstacle", "障碍峰 2480m"),
                Peak(18.8, 2.8, 1120, 1.8, 1.1, -18, "background", "伴峰 1120m"),
            ),
            ((16.1, -2.75, 2.2),),
        ),
        "b": LayoutSpec(
            "b",
            "方案 B：双峰 S 绕（推荐）",
            "段3 两座主峰错位挡路，重规划航线先左绕再右绕，演示效果最强。",
            common_background
            + valley_standard
            + low_hills
            + (
                Peak(14.4, 0.65, 2280, 1.55, 1.25, 18, "obstacle", "障碍峰 2280m"),
                Peak(18.3, -0.70, 2420, 1.60, 1.28, -20, "obstacle", "障碍峰 2420m"),
                Peak(20.5, 3.2, 1200, 1.8, 1.1, 15, "background", "伴峰 1200m"),
            ),
            ((14.4, -2.35, 1.65), (18.3, 2.35, 1.75)),
        ),
        "c": LayoutSpec(
            "c",
            "方案 C：峡谷 + 单峰",
            "段2 山谷收窄、两侧高山更陡，段3 单峰绕飞。",
            common_background
            + valley_narrow
            + low_hills
            + (
                Peak(17.0, 0.05, 2380, 1.55, 1.25, -8, "obstacle", "障碍峰 2380m"),
                Peak(14.5, -3.4, 1350, 1.8, 1.1, 16, "background", "伴峰 1350m"),
            ),
            ((17.0, 2.70, 2.0),),
        ),
    }


def point_on_polyline(polyline: list[list[float]], station: float) -> tuple[float, float, float]:
    """按 0~1 里程比例返回折线上的 u/v 坐标和局部走向角。"""

    points = np.array(polyline, dtype=np.float32)
    if points.shape[0] < 2:
        raise ValueError("山脉链走向折线至少需要两个控制点")
    lengths = np.linalg.norm(points[1:] - points[:-1], axis=1)
    total = max(float(np.sum(lengths)), 1e-6)
    target = float(np.clip(station, 0.0, 1.0)) * total
    accumulated = 0.0
    for index, segment_length in enumerate(lengths):
        next_accumulated = accumulated + float(segment_length)
        if target <= next_accumulated or index == len(lengths) - 1:
            mix = (target - accumulated) / max(float(segment_length), 1e-6)
            point = points[index] * (1.0 - mix) + points[index + 1] * mix
            direction = points[index + 1] - points[index]
            angle = math.degrees(math.atan2(float(direction[1]), float(direction[0]))) - 35.0
            return float(point[0]), float(point[1]), angle
        accumulated = next_accumulated
    direction = points[-1] - points[-2]
    angle = math.degrees(math.atan2(float(direction[1]), float(direction[0]))) - 35.0
    return float(points[-1, 0]), float(points[-1, 1]), angle


def load_layout_spec_from_json(path: Path) -> LayoutSpec:
    """从布局层 JSON 读取定稿方案。"""

    data = json.loads(path.read_text(encoding="utf-8"))
    peaks: list[Peak] = []
    ridges: list[RidgeSegment] = []
    saddles: list[Saddle] = []
    chain_polylines: list[tuple[tuple[float, float], ...]] = []
    for chain in data["mountain_chains"]:
        polyline = chain["polyline_uv"]
        chain_polylines.append(tuple((float(point[0]), float(point[1])) for point in polyline))
        resolved: list[Peak] = []
        for item in chain["peaks"]:
            u_coord, v_coord, angle = point_on_polyline(polyline, float(item["station"]))
            v_coord += float(item.get("lateral_offset_km", 0.0))
            peak = Peak(
                u=u_coord,
                v=v_coord,
                height=float(item["height_m"]),
                radius_u=float(item["base_radius_km"]),
                radius_v=float(item["base_radius_km"]) / max(float(item.get("aspect_ratio", 1.0)), 0.35),
                angle_deg=angle + float(item.get("angle_offset_deg", 0.0)),
                role=str(item.get("role", chain.get("role", "background"))),
                label=str(item.get("label", "")),
            )
            resolved.append(peak)
            peaks.append(peak)
        saddle_factor = float(chain.get("saddle_height_factor", 0.30))
        ridge_width = float(chain.get("ridge_width_km", 0.78))
        for first, second in zip(resolved[:-1], resolved[1:]):
            height = min(first.height, second.height) * saddle_factor
            role = "saddle" if first.role == "obstacle" and second.role == "obstacle" else "ridge"
            label = "低鞍部脊线 ≤650m" if role == "saddle" else ""
            ridges.append(RidgeSegment(first.u, first.v, second.u, second.v, height, ridge_width, role, label))
            if role == "saddle":
                saddles.append(Saddle((first.u + second.u) * 0.5, (first.v + second.v) * 0.5, height, label))
    flight = data["flight"]
    return LayoutSpec(
        key=str(data["key"]),
        title=str(data["title"]),
        description=str(data["description"]),
        peaks=tuple(peaks),
        route_offsets=(),
        ridges=tuple(ridges),
        saddles=tuple(saddles),
        original_route_uv=tuple((float(point[0]), float(point[1])) for point in flight["original_route_uv"]),
        planned_route_uv=tuple((float(point[0]), float(point[1])) for point in flight["planned_route_uv"]),
        chain_polylines=tuple(chain_polylines),
    )


def layout_height(spec: LayoutSpec, x_grid: np.ndarray, y_grid: np.ndarray, seed: int) -> np.ndarray:
    """按方案生成高度场，单位 m。"""

    u_grid, v_grid = xy_to_uv(x_grid, y_grid)
    extent = 60.0
    uu = (x_grid + extent) / (extent * 2.0)
    vv = (y_grid + extent) / (extent * 2.0)
    height = np.full(x_grid.shape, 45.0, dtype=np.float32)
    for index, ridge in enumerate(spec.ridges):
        start_x, start_y = uv_to_xy(ridge.start_u, ridge.start_v)
        end_x, end_y = uv_to_xy(ridge.end_u, ridge.end_v)
        sx = end_x - start_x
        sy = end_y - start_y
        length_sq = max(sx * sx + sy * sy, 1e-6)
        t = np.clip(((x_grid - start_x) * sx + (y_grid - start_y) * sy) / length_sq, 0.0, 1.0)
        closest_x = start_x + sx * t
        closest_y = start_y + sy * t
        cross_distance = np.sqrt((x_grid - closest_x) ** 2 + (y_grid - closest_y) ** 2)
        along_sine = np.clip(np.sin(np.pi * t), 0.0, 1.0)
        along_profile = 0.58 + 0.42 * along_sine**0.72
        cross_profile = np.exp(-1.55 * (cross_distance / max(ridge.width_km, 0.18)) ** 1.42)
        ridge_noise = warped_ridged_fbm(uu + index * 0.029, vv - index * 0.017, 11 + index % 4, 3, seed + 211 + index, 0.020)
        height += ridge.height * along_profile * cross_profile * (0.86 + 0.20 * ridge_noise)
    for index, peak in enumerate(spec.peaks):
        cx, cy = uv_to_xy(peak.u, peak.v)
        angle = math.radians(peak.angle_deg + 35.0)
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        dx = x_grid - cx
        dy = y_grid - cy
        local_x = (dx * cos_a + dy * sin_a) / peak.radius_u
        local_y = (-dx * sin_a + dy * cos_a) / peak.radius_v
        body = np.exp(-1.20 * (np.abs(local_x) ** 1.65 + np.abs(local_y) ** 1.80))
        ridge = warped_ridged_fbm(uu + index * 0.037, vv - index * 0.023, 16 + index % 5, 4, seed + 41 + index, 0.030)
        drainage = warped_ridged_fbm(uu - index * 0.011, vv + index * 0.019, 22 + index % 7, 3, seed + 91 + index, 0.025)
        rugged = np.clip(0.72 + 0.42 * ridge - 0.24 * (1.0 - drainage) ** 2.0, 0.42, 1.26)
        height += peak.height * body * rugged
    # 航线走廊在段1和段4更平缓，段2保留低丘层次。
    corridor = np.exp(-0.5 * (v_grid / 0.75) ** 2)
    gather = smoothstep((6.0 - u_grid) / 2.2)
    finish = smoothstep((u_grid - 20.0) / 2.4)
    flatten = np.clip((gather + finish) * corridor, 0.0, 1.0)
    height = height * (1.0 - flatten * 0.50)
    rolling = 120.0 * warped_ridged_fbm(uu, vv, 9, 3, seed + 7, 0.025)
    height += rolling
    height -= float(np.min(height))
    height *= 2650.0 / max(float(np.max(height)), 1.0)
    return height.astype(np.float32)


def style_a_colors(height: np.ndarray, x_grid: np.ndarray, y_grid: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """生成 style A 顶点色和法线。"""

    grad_y, grad_x = np.gradient(height, y_grid[:, 0] * 1000.0, x_grid[0, :] * 1000.0)
    nx = -grad_x
    ny = np.ones_like(nx)
    nz = -grad_y
    length = np.sqrt(nx * nx + ny * ny + nz * nz)
    normals = np.stack([nx / length, ny / length, nz / length], axis=-1).astype(np.float32)
    slope = np.sqrt(grad_x * grad_x + grad_y * grad_y)
    h = np.clip(height / 2650.0, 0.0, 1.0)
    height_low = box_blur(h, 7, passes=3)
    slope_low = box_blur(slope, 8, passes=3)
    rock_weight = smoothstep((height_low - 0.62) / 0.24) + 0.12 * smoothstep((slope_low - 0.50) / 0.52)
    rock_weight = np.clip(box_blur(rock_weight, 12, passes=2), 0.0, 1.0)
    vegetation = lerp_color((0.045, 0.075, 0.060), (0.235, 0.265, 0.155), smoothstep(height_low / 0.42))
    alpine = lerp_color((0.235, 0.265, 0.155), (0.345, 0.335, 0.245), smoothstep((height_low - 0.30) / 0.36))
    veg_mix = smoothstep((height_low - 0.28) / 0.36)[..., None]
    vegetation = vegetation * (1.0 - veg_mix) + alpine * veg_mix
    rock = lerp_color((0.30, 0.30, 0.29), (0.74, 0.745, 0.715), smoothstep((height_low - 0.62) / 0.30))
    mixed = vegetation * (1.0 - rock_weight[..., None]) + rock * rock_weight[..., None]
    far_distance = np.sqrt(x_grid * x_grid + y_grid * y_grid)
    far_mix = smoothstep((far_distance - 12.0) / 18.0)[..., None]
    mixed = mixed * (1.0 - far_mix * 0.65) + np.array([0.08, 0.12, 0.15], dtype=np.float32) * (far_mix * 0.65)
    light_dir = np.array([-0.58, 0.66, 0.48], dtype=np.float32)
    light_dir /= np.linalg.norm(light_dir)
    lambert = np.clip(np.sum(normals * light_dir, axis=-1), 0.0, 1.0)
    veg_relief = 0.86 + 0.24 * (lambert[..., None] ** 0.82)
    rock_relief = 0.46 + 0.96 * (lambert[..., None] ** 0.78)
    relief = veg_relief * (1.0 - rock_weight[..., None]) + rock_relief * rock_weight[..., None]
    color = np.maximum(mixed * relief, np.array([0.036, 0.052, 0.070], dtype=np.float32))
    linear = srgb_to_linear(np.clip(color, 0.0, 0.85))
    alpha = np.ones((*height.shape, 1), dtype=np.float32)
    return np.concatenate([linear.astype(np.float32), alpha], axis=-1), normals


class LayoutScene:
    """布局候选 3D 场景数据。"""

    def __init__(self, spec: LayoutSpec, grid_size: int, seed: int) -> None:
        """生成地形、颜色和线层。"""

        self.spec = spec
        self.grid_size = grid_size
        self.extent_km = 58.0
        axis = np.linspace(-self.extent_km, self.extent_km, grid_size, dtype=np.float32)
        self.x_grid, self.y_grid = np.meshgrid(axis, axis)
        self.height = layout_height(spec, self.x_grid, self.y_grid, seed)
        self.colors, self.normals = style_a_colors(self.height, self.x_grid, self.y_grid)
        self.original_route = original_route(spec)
        self.planned_route = planned_route(spec)
        self.hazard_peaks = tuple(peak for peak in spec.peaks if peak.role == "obstacle")
        self.waypoints = self._build_waypoints()
        self.hazard_grid = self._build_hazard_grid()
        self.hazard_contours = self._build_hazard_contours()
        self.buffers = self._build_buffers()
        self.original_dashes = self._build_original_dashes()

    def height_at(self, x_km: float, y_km: float) -> float:
        """双线性采样高度。"""

        axis = self.x_grid[0, :]
        ix = int(np.clip(np.searchsorted(axis, x_km) - 1, 0, self.grid_size - 2))
        iy = int(np.clip(np.searchsorted(axis, y_km) - 1, 0, self.grid_size - 2))
        tx = (x_km - float(axis[ix])) / max(float(axis[ix + 1] - axis[ix]), 1e-6)
        ty = (y_km - float(axis[iy])) / max(float(axis[iy + 1] - axis[iy]), 1e-6)
        h00 = float(self.height[iy, ix])
        h10 = float(self.height[iy, ix + 1])
        h01 = float(self.height[iy + 1, ix])
        h11 = float(self.height[iy + 1, ix + 1])
        return (h00 * (1 - tx) + h10 * tx) * (1 - ty) + (h01 * (1 - tx) + h11 * tx) * ty

    def _build_waypoints(self) -> list[np.ndarray]:
        """生成约 2km 间隔的航点圆环。"""

        route = self.planned_route
        distance = [0.0]
        for first, second in zip(route[:-1], route[1:]):
            distance.append(distance[-1] + float(np.linalg.norm(second - first)))
        distance_arr = np.array(distance, dtype=np.float32)
        rings: list[np.ndarray] = []
        for target in np.arange(1.2, distance_arr[-1] - 0.6, 2.0, dtype=np.float32):
            index = int(np.searchsorted(distance_arr, target))
            index = max(1, min(index, len(route) - 1))
            mix = float((target - distance_arr[index - 1]) / max(distance_arr[index] - distance_arr[index - 1], 1e-6))
            center = route[index - 1] * (1.0 - mix) + route[index] * mix
            points = []
            for theta in np.linspace(0.0, math.tau, 64, endpoint=True, dtype=np.float32):
                points.append((float(center[0] * 1000.0 + math.cos(float(theta)) * 230.0), ROUTE_ALTITUDE_M + 22.0, float(center[1] * 1000.0 + math.sin(float(theta)) * 230.0)))
            rings.append(np.array(points, dtype=np.float32))
        return rings

    def _build_hazard_grid(self) -> list[np.ndarray]:
        """生成细风险区线框。"""

        lines: list[np.ndarray] = []
        for peak in self.hazard_peaks:
            cx, cy = uv_to_xy(peak.u, peak.v)
            angle = math.radians(peak.angle_deg + 35.0)
            cos_a = math.cos(angle)
            sin_a = math.sin(angle)
            rx = peak.radius_u * 1.0
            ry = peak.radius_v * 1.0
            for local_x in np.linspace(-rx, rx, 8, dtype=np.float32):
                points: list[tuple[float, float, float]] = []
                for local_y in np.linspace(-ry, ry, 80, dtype=np.float32):
                    if (local_x / rx) ** 2 + (local_y / ry) ** 2 > 1.22:
                        if len(points) >= 2:
                            lines.append(np.array(points, dtype=np.float32))
                        points = []
                        continue
                    x = cx + local_x * cos_a - local_y * sin_a
                    y = cy + local_x * sin_a + local_y * cos_a
                    points.append((x * 1000.0, self.height_at(float(x), float(y)) + 42.0, y * 1000.0))
                if len(points) >= 2:
                    lines.append(np.array(points, dtype=np.float32))
            for local_y in np.linspace(-ry, ry, 8, dtype=np.float32):
                points = []
                for local_x in np.linspace(-rx, rx, 80, dtype=np.float32):
                    if (local_x / rx) ** 2 + (local_y / ry) ** 2 > 1.22:
                        if len(points) >= 2:
                            lines.append(np.array(points, dtype=np.float32))
                        points = []
                        continue
                    x = cx + local_x * cos_a - local_y * sin_a
                    y = cy + local_x * sin_a + local_y * cos_a
                    points.append((x * 1000.0, self.height_at(float(x), float(y)) + 44.0, y * 1000.0))
                if len(points) >= 2:
                    lines.append(np.array(points, dtype=np.float32))
        return lines

    def _build_hazard_contours(self) -> list[np.ndarray]:
        """生成风险峰贴地红色轮廓。"""

        lines: list[np.ndarray] = []
        for peak in self.hazard_peaks:
            cx, cy = uv_to_xy(peak.u, peak.v)
            for scale in (0.55, 0.78, 1.0):
                points = []
                for theta in np.linspace(0.0, math.tau, 120, endpoint=True, dtype=np.float32):
                    x = cx + math.cos(float(theta)) * peak.radius_u * scale
                    y = cy + math.sin(float(theta)) * peak.radius_v * scale
                    points.append((x * 1000.0, self.height_at(float(x), float(y)) + 38.0, y * 1000.0))
                lines.append(np.array(points, dtype=np.float32))
        return lines

    def _build_buffers(self) -> list[np.ndarray]:
        """生成淡青色安全缓冲虚线圈。"""

        lines: list[np.ndarray] = []
        for peak in self.hazard_peaks:
            cx, cy = uv_to_xy(peak.u, peak.v)
            samples = []
            for theta in np.linspace(0.0, math.tau, 144, endpoint=False, dtype=np.float32):
                x = cx + math.cos(float(theta)) * peak.radius_u * 1.25
                y = cy + math.sin(float(theta)) * peak.radius_v * 1.25
                samples.append((x * 1000.0, self.height_at(float(x), float(y)) + 54.0, y * 1000.0))
            for start in range(0, len(samples), 12):
                dash = samples[start : start + 7]
                if len(dash) >= 2:
                    lines.append(np.array(dash, dtype=np.float32))
        return lines

    def _build_original_dashes(self) -> list[np.ndarray]:
        """生成红色虚线原始航线。"""

        points = [(float(x * 1000.0), ROUTE_ALTITUDE_M + 180.0, float(y * 1000.0)) for x, y in self.original_route]
        dashes = []
        for start in range(0, len(points) - 1, 12):
            dash = points[start : start + 7]
            if len(dash) >= 2:
                dashes.append(np.array(dash, dtype=np.float32))
        return dashes


class LayoutTerrainGeometry(QQuick3DGeometry):
    """布局候选地形几何。"""

    def __init__(self, parent: object | None = None) -> None:
        """初始化几何。"""

        super().__init__(parent)
        self._rebuild()

    def _rebuild(self) -> None:
        """提交地形网格。"""

        scene = require_scene()
        vertices = bytearray()
        for row in range(scene.grid_size):
            for col in range(scene.grid_size):
                vertices.extend(
                    struct.pack(
                        "<ffffffffffff",
                        float(scene.x_grid[row, col] * 1000.0),
                        float(scene.height[row, col]),
                        float(scene.y_grid[row, col] * 1000.0),
                        float(scene.normals[row, col, 0]),
                        float(scene.normals[row, col, 1]),
                        float(scene.normals[row, col, 2]),
                        col / (scene.grid_size - 1),
                        row / (scene.grid_size - 1),
                        float(scene.colors[row, col, 0]),
                        float(scene.colors[row, col, 1]),
                        float(scene.colors[row, col, 2]),
                        1.0,
                    )
                )
        indices = bytearray()
        for row in range(scene.grid_size - 1):
            for col in range(scene.grid_size - 1):
                top_left = row * scene.grid_size + col
                top_right = top_left + 1
                bottom_left = top_left + scene.grid_size
                bottom_right = bottom_left + 1
                indices.extend(struct.pack("<IIIIII", top_left, bottom_left, top_right, top_right, bottom_left, bottom_right))
        self.clear()
        self.setPrimitiveType(QQuick3DGeometry.PrimitiveType.Triangles)
        self.setStride(VERTEX_STRIDE)
        self.addAttribute(QQuick3DGeometry.Attribute.Semantic.PositionSemantic, 0, QQuick3DGeometry.Attribute.ComponentType.F32Type)
        self.addAttribute(QQuick3DGeometry.Attribute.Semantic.NormalSemantic, 3 * FLOAT_SIZE, QQuick3DGeometry.Attribute.ComponentType.F32Type)
        self.addAttribute(QQuick3DGeometry.Attribute.Semantic.TexCoordSemantic, 6 * FLOAT_SIZE, QQuick3DGeometry.Attribute.ComponentType.F32Type)
        self.addAttribute(QQuick3DGeometry.Attribute.Semantic.ColorSemantic, 8 * FLOAT_SIZE, QQuick3DGeometry.Attribute.ComponentType.F32Type)
        self.addAttribute(QQuick3DGeometry.Attribute.Semantic.IndexSemantic, 0, QQuick3DGeometry.Attribute.ComponentType.U32Type)
        extent = scene.extent_km * 1000.0
        self.setBounds(QVector3D(-extent, 0.0, -extent), QVector3D(extent, float(scene.height.max()) + 100.0, extent))
        self.setVertexData(QByteArray(bytes(vertices)))
        self.setIndexData(QByteArray(bytes(indices)))
        self.update()


class LayoutLineGeometry(QQuick3DGeometry):
    """布局候选线层几何。"""

    kindChanged = Signal()
    widthValueChanged = Signal()

    def __init__(self, parent: object | None = None) -> None:
        """初始化线层。"""

        super().__init__(parent)
        self._kind = "routeCore"
        self._width_value = 80.0
        self._rebuild()

    @Property(str, notify=kindChanged)
    def kind(self) -> str:
        """返回线层类型。"""

        return self._kind

    @kind.setter
    def kind(self, value: str) -> None:
        """设置线层类型。"""

        allowed = {"routeCore", "routeGlow", "original", "waypoint", "hazardGrid", "hazardContour", "buffer"}
        normalized = value if value in allowed else "routeCore"
        if normalized == self._kind:
            return
        self._kind = normalized
        self._rebuild()
        self.kindChanged.emit()

    @Property(float, notify=widthValueChanged)
    def widthValue(self) -> float:
        """返回线宽。"""

        return self._width_value

    @widthValue.setter
    def widthValue(self, value: float) -> None:
        """设置线宽。"""

        try:
            normalized = float(value)
        except (TypeError, ValueError):
            normalized = self._width_value
        normalized = max(2.0, min(400.0, normalized))
        if math.isclose(normalized, self._width_value, rel_tol=1e-6):
            return
        self._width_value = normalized
        self._rebuild()
        self.widthValueChanged.emit()

    def _rebuild(self) -> None:
        """按类型生成三角带。"""

        scene = require_scene()
        if self._kind in {"routeCore", "routeGlow"}:
            line = [(float(x * 1000.0), ROUTE_ALTITUDE_M, float(y * 1000.0)) for x, y in scene.planned_route]
            polylines = [np.array(line, dtype=np.float32)]
        elif self._kind == "original":
            polylines = scene.original_dashes
        elif self._kind == "waypoint":
            polylines = scene.waypoints
        elif self._kind == "hazardGrid":
            polylines = scene.hazard_grid
        elif self._kind == "hazardContour":
            polylines = scene.hazard_contours
        else:
            polylines = scene.buffers
        vertices, indices, bounds = build_ribbons(polylines, self._width_value)
        self.clear()
        self.setPrimitiveType(QQuick3DGeometry.PrimitiveType.Triangles)
        self.setStride(VERTEX_STRIDE)
        self.addAttribute(QQuick3DGeometry.Attribute.Semantic.PositionSemantic, 0, QQuick3DGeometry.Attribute.ComponentType.F32Type)
        self.addAttribute(QQuick3DGeometry.Attribute.Semantic.NormalSemantic, 3 * FLOAT_SIZE, QQuick3DGeometry.Attribute.ComponentType.F32Type)
        self.addAttribute(QQuick3DGeometry.Attribute.Semantic.TexCoordSemantic, 6 * FLOAT_SIZE, QQuick3DGeometry.Attribute.ComponentType.F32Type)
        self.addAttribute(QQuick3DGeometry.Attribute.Semantic.ColorSemantic, 8 * FLOAT_SIZE, QQuick3DGeometry.Attribute.ComponentType.F32Type)
        self.addAttribute(QQuick3DGeometry.Attribute.Semantic.IndexSemantic, 0, QQuick3DGeometry.Attribute.ComponentType.U32Type)
        if bounds is None:
            self.setBounds(QVector3D(), QVector3D())
        else:
            self.setBounds(QVector3D(bounds[0], bounds[1], bounds[2]), QVector3D(bounds[3], bounds[4], bounds[5]))
        self.setVertexData(QByteArray(bytes(vertices)))
        self.setIndexData(QByteArray(bytes(indices)))
        self.update()


def build_ribbons(polylines: list[np.ndarray], width: float) -> tuple[bytearray, bytearray, tuple[float, float, float, float, float, float] | None]:
    """把折线转成三角带。"""

    vertices = bytearray()
    indices = bytearray()
    all_points: list[np.ndarray] = []
    vertex_base = 0
    half_width = width / 2.0
    for line in polylines:
        if len(line) < 2:
            continue
        all_points.append(line)
        for index, point in enumerate(line):
            if index == 0:
                previous = line[index]
                current = line[index + 1]
            elif index == len(line) - 1:
                previous = line[index - 1]
                current = line[index]
            else:
                previous = line[index - 1]
                current = line[index + 1]
            dx = float(current[0] - previous[0])
            dz = float(current[2] - previous[2])
            length = math.hypot(dx, dz)
            side_x, side_z = (1.0, 0.0) if length <= 1e-6 else (-dz / length, dx / length)
            left = (float(point[0] - side_x * half_width), float(point[1]), float(point[2] - side_z * half_width))
            right = (float(point[0] + side_x * half_width), float(point[1]), float(point[2] + side_z * half_width))
            append_vertex(vertices, left, index / max(1, len(line) - 1), 0.0)
            append_vertex(vertices, right, index / max(1, len(line) - 1), 1.0)
        for index in range(len(line) - 1):
            left_a = vertex_base + index * 2
            right_a = left_a + 1
            left_b = left_a + 2
            right_b = left_a + 3
            indices.extend(struct.pack("<IIIIII", left_a, left_b, right_a, right_a, left_b, right_b))
        vertex_base += len(line) * 2
    if not all_points:
        return vertices, indices, None
    merged = np.concatenate(all_points, axis=0)
    margin = width + 80.0
    return vertices, indices, (
        float(np.min(merged[:, 0]) - margin),
        float(np.min(merged[:, 1]) - margin),
        float(np.min(merged[:, 2]) - margin),
        float(np.max(merged[:, 0]) + margin),
        float(np.max(merged[:, 1]) + margin),
        float(np.max(merged[:, 2]) + margin),
    )


def append_vertex(vertices: bytearray, position: tuple[float, float, float], u_coord: float, v_coord: float) -> None:
    """追加线顶点。"""

    vertices.extend(
        struct.pack(
            "<ffffffffffff",
            position[0],
            position[1],
            position[2],
            0.0,
            1.0,
            0.0,
            u_coord,
            v_coord,
            1.0,
            1.0,
            1.0,
            1.0,
        )
    )


def require_scene() -> LayoutScene:
    """返回当前布局场景。"""

    if SCENE is None:
        raise RuntimeError("布局场景尚未初始化")
    return SCENE


def render_top_view(spec: LayoutSpec, output_path: Path, seed: int) -> None:
    """生成俯视布局图。"""

    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "Arial Unicode MS"]
    plt.rcParams["axes.unicode_minus"] = False
    axis = np.linspace(-16.0, 16.0, 420, dtype=np.float32)
    x_grid, y_grid = np.meshgrid(axis, axis)
    height = layout_height(spec, x_grid, y_grid, seed)
    cmap = LinearSegmentedColormap.from_list(
        "style_a_terrain",
        ["#0d1420", "#183322", "#526327", "#8a8357", "#c7c4ad"],
    )
    fig, ax = plt.subplots(figsize=(16, 12), dpi=100)
    levels = np.linspace(0, 2650, 26)
    filled = ax.contourf(x_grid, y_grid, height, levels=levels, cmap=cmap, alpha=0.96)
    ax.contour(x_grid, y_grid, height, levels=np.arange(300, 2700, 200), colors="#d5d7c080", linewidths=0.45)
    original = original_route(spec)
    planned = planned_route(spec)
    for chain in spec.chain_polylines:
        if len(chain) < 2:
            continue
        chain_uv = np.array(chain, dtype=np.float32)
        chain_x, chain_y = uv_to_xy(chain_uv[:, 0], chain_uv[:, 1])
        ax.plot(chain_x, chain_y, color="#d5d7c055", linewidth=1.1, linestyle="-", alpha=0.55)
    ax.plot(original[:, 0], original[:, 1], "--", color="#ff3b30", linewidth=3.0, label="原始航线")
    ax.plot(planned[:, 0], planned[:, 1], "-", color="#22d3ee", linewidth=3.2, label="重规划航线")
    for peak in spec.peaks:
        cx, cy = uv_to_xy(peak.u, peak.v)
        if peak.role == "obstacle":
            ax.add_patch(Ellipse((cx, cy), peak.radius_u * 2.35, peak.radius_v * 2.35, angle=peak.angle_deg + 35, facecolor="#ff1f1f55", edgecolor="#ff3b30", linewidth=2.2))
            ax.text(cx, cy + peak.radius_v * 1.45, peak.label, color="#ffdad6", fontsize=13, ha="center", weight="bold")
        elif peak.role == "valley":
            ax.text(cx, cy, peak.label, color="#f3ead2", fontsize=11, ha="center", va="center")
    for saddle in spec.saddles:
        sx, sy = uv_to_xy(saddle.u, saddle.v)
        ax.scatter([sx], [sy], s=86, color="#22d3ee", edgecolors="#ffffff", linewidths=1.0, zorder=5)
        ax.text(sx, sy - 1.0, f"鞍部约 {saddle.height:.0f}m", color="#dffbff", fontsize=12, ha="center", va="top", bbox={"facecolor": "#101923cc", "edgecolor": "#22d3ee88", "boxstyle": "round,pad=0.20"})
    segment_marks = ((3.0, "集结段"), (9.0, "穿越段"), (16.0, "避障段"), (23.0, "收尾段"))
    for u_coord, label in segment_marks:
        x, y = uv_to_xy(u_coord, 0.0)
        ax.text(x, y + 1.1, label, color="#ffffff", fontsize=15, ha="center", va="bottom", bbox={"facecolor": "#101923cc", "edgecolor": "#22d3ee88", "boxstyle": "round,pad=0.25"})
    ax.set_title(spec.title, fontsize=22, color="#f4f7fb", pad=14)
    ax.set_xlabel("E / km")
    ax.set_ylabel("N / km")
    ax.set_xlim(-16, 16)
    ax.set_ylim(-16, 16)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, color="#ffffff33", linewidth=0.6)
    ax.legend(loc="upper left", framealpha=0.82)
    cbar = fig.colorbar(filled, ax=ax, fraction=0.030, pad=0.02)
    cbar.set_label("相对高度 / m")
    ax.text(-15.5, -15.0, "网格间距 2 km", color="#e5edf4", fontsize=12, bbox={"facecolor": "#0d1420bb", "edgecolor": "#ffffff55"})
    fig.patch.set_facecolor("#0b1118")
    ax.set_facecolor("#0b1118")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, facecolor=fig.get_facecolor())
    plt.close(fig)


def render_3d(spec: LayoutSpec, output_path: Path, grid_size: int, seed: int, wait_ms: int) -> None:
    """生成 3D 斜俯视效果图。"""

    global SCENE
    SCENE = LayoutScene(spec, grid_size, seed)
    app = QGuiApplication.instance() or QGuiApplication(sys.argv)
    view = QQuickView()
    view.setResizeMode(QQuickView.ResizeMode.SizeRootObjectToView)
    view.setWidth(WIDTH_PX)
    view.setHeight(HEIGHT_PX)
    view.setSource(QUrl.fromLocalFile(str(Path(__file__).resolve().parent / "layout_scene.qml")))
    if view.status() == QQuickView.Status.Error:
        for error in view.errors():
            print(error.toString(), file=sys.stderr)
        raise RuntimeError("布局 QML 加载失败")
    view.show()
    view.requestActivate()
    app.processEvents()
    loop = QEventLoop()
    QTimer.singleShot(wait_ms, loop.quit)
    loop.exec()
    image = view.grabWindow()
    if image.isNull():
        raise RuntimeError(f"抓取窗口失败：{spec.key}")
    if image.width() != WIDTH_PX or image.height() != HEIGHT_PX:
        image = image.scaled(WIDTH_PX, HEIGHT_PX)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not image.save(str(output_path), "PNG"):
        raise RuntimeError(f"保存 3D PNG 失败：{output_path}")
    view.close()
    view.deleteLater()
    app.processEvents()


def parse_args() -> argparse.Namespace:
    """解析命令行。"""

    parser = argparse.ArgumentParser(description="生成三个演示地形布局候选方案。")
    parser.add_argument("--grid-size", type=int, default=385, help="3D 高度场采样边长，默认 385。")
    parser.add_argument("--seed", type=int, default=20260712, help="随机种子。")
    parser.add_argument("--wait-ms", type=int, default=1500, help="3D 渲染等待毫秒数。")
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parent / "output" / "layouts", help="输出目录。")
    parser.add_argument("--layout-json", type=Path, default=None, help="从布局层 JSON 读取并渲染单个定稿方案。")
    return parser.parse_args()


def main() -> int:
    """生成候选方案图或 JSON 驱动的定稿方案图。"""

    args = parse_args()
    qmlRegisterType(LayoutTerrainGeometry, "TerrainLayout", 1, 0, "LayoutTerrainGeometry")
    qmlRegisterType(LayoutLineGeometry, "TerrainLayout", 1, 0, "LayoutLineGeometry")
    if args.layout_json is not None:
        spec = load_layout_spec_from_json(args.layout_json)
        top_path = args.output_dir / f"layout_{spec.key}_top.png"
        view_path = args.output_dir / f"layout_{spec.key}_3d.png"
        render_top_view(spec, top_path, args.seed + ord(spec.key[0]))
        render_3d(spec, view_path, args.grid_size, args.seed + ord(spec.key[0]), args.wait_ms)
        print(f"{spec.title}: {top_path.resolve()}  {view_path.resolve()}")
        return 0
    specs = make_layout_specs()
    for key in ("a", "b", "c"):
        spec = specs[key]
        top_path = args.output_dir / f"layout_{key}_top.png"
        view_path = args.output_dir / f"layout_{key}_3d.png"
        render_top_view(spec, top_path, args.seed + ord(key))
        render_3d(spec, view_path, args.grid_size, args.seed + ord(key), args.wait_ms)
        print(f"{spec.title}: {top_path.resolve()}  {view_path.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
