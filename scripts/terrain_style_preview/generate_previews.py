"""生成 3D 山地地形风格样张。

脚本只写入 scripts/terrain_style_preview/output，不修改正式 src/QML。
"""

from __future__ import annotations

import argparse
import math
import os
import struct
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PySide6.QtCore import QByteArray, Property, QEventLoop, QTimer, QUrl, Signal
from PySide6.QtGui import QGuiApplication, QVector3D
from PySide6.QtQml import qmlRegisterType
from PySide6.QtQuick import QQuickView
from PySide6.QtQuick3D import QQuick3DGeometry

WIDTH_PX = 1600
HEIGHT_PX = 900
MAP_SIZE_M = 50000.0
ROUTE_ALTITUDE_M = 900.0
FLOAT_SIZE = 4
VERTEX_COMPONENTS = 12
VERTEX_STRIDE = VERTEX_COMPONENTS * FLOAT_SIZE
SCENE: "TerrainPreviewScene | None" = None


@dataclass(frozen=True)
class PeakSpec:
    """主峰参数。中心坐标单位为米，高度为相对高度。"""

    x: float
    z: float
    radius_x: float
    radius_z: float
    angle_deg: float
    height: float


def srgb_to_linear(value: np.ndarray | float) -> np.ndarray | float:
    """把 sRGB 转为线性空间，匹配 Qt Quick 3D 光照计算。"""

    arr = np.asarray(value)
    converted = np.where(arr <= 0.04045, arr / 12.92, ((arr + 0.055) / 1.055) ** 2.4)
    if np.isscalar(value):
        return float(converted)
    return converted


def smoothstep(value: np.ndarray | float) -> np.ndarray | float:
    """返回 0 到 1 的三次平滑阶跃。"""

    clipped = np.clip(value, 0.0, 1.0)
    return clipped * clipped * (3.0 - 2.0 * clipped)


def lerp_color(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    mix: np.ndarray,
) -> np.ndarray:
    """按 mix 插值两个 sRGB 颜色。"""

    a = np.array(start, dtype=np.float32)
    b = np.array(end, dtype=np.float32)
    return a + (b - a) * mix[..., None]


def value_noise(u_coord: np.ndarray, v_coord: np.ndarray, frequency: int, rng: np.random.Generator) -> np.ndarray:
    """二维 value noise，输入坐标范围约为 0 到 1。"""

    grid = rng.uniform(-1.0, 1.0, size=(frequency + 2, frequency + 2)).astype(np.float32)
    x = np.clip(u_coord * frequency, 0.0, frequency - 1e-4)
    y = np.clip(v_coord * frequency, 0.0, frequency - 1e-4)
    ix = np.floor(x).astype(np.int32)
    iy = np.floor(y).astype(np.int32)
    fx = smoothstep(x - ix)
    fy = smoothstep(y - iy)
    a = grid[iy, ix]
    b = grid[iy, ix + 1]
    c = grid[iy + 1, ix]
    d = grid[iy + 1, ix + 1]
    return ((a * (1.0 - fx) + b * fx) * (1.0 - fy) + (c * (1.0 - fx) + d * fx) * fy).astype(np.float32)


def fbm(
    u_coord: np.ndarray,
    v_coord: np.ndarray,
    base_frequency: int,
    octaves: int,
    seed: int,
) -> np.ndarray:
    """分形布朗噪声，输出大致在 -1 到 1。"""

    rng = np.random.default_rng(seed)
    total = np.zeros_like(u_coord, dtype=np.float32)
    amplitude = 0.55
    amplitude_sum = 0.0
    frequency = base_frequency
    for _ in range(octaves):
        total += amplitude * value_noise(u_coord, v_coord, frequency, rng)
        amplitude_sum += amplitude
        amplitude *= 0.52
        frequency *= 2
    return total / max(amplitude_sum, 1e-6)


def ridged_fbm(
    u_coord: np.ndarray,
    v_coord: np.ndarray,
    base_frequency: int,
    octaves: int,
    seed: int,
) -> np.ndarray:
    """脊状分形噪声，输出 0 到 1，越亮表示越接近山脊。"""

    rng = np.random.default_rng(seed)
    total = np.zeros_like(u_coord, dtype=np.float32)
    amplitude = 0.62
    amplitude_sum = 0.0
    frequency = base_frequency
    for _ in range(octaves):
        noise = value_noise(u_coord, v_coord, frequency, rng)
        ridge = (1.0 - np.abs(noise)) ** 2.0
        total += amplitude * ridge
        amplitude_sum += amplitude
        amplitude *= 0.50
        frequency *= 2
    return np.clip(total / max(amplitude_sum, 1e-6), 0.0, 1.0)


def route_z_from_x(x_coord: np.ndarray | float) -> np.ndarray | float:
    """蜿蜒航线走廊中心线的 z 坐标。"""

    t = (np.asarray(x_coord) + 8800.0) / 17600.0
    z = 1420.0 * np.sin(math.tau * (1.10 * t + 0.06)) + 820.0 * np.sin(math.tau * (2.55 * t - 0.20))
    if np.isscalar(x_coord):
        return float(z)
    return z


def route_points(count: int = 150) -> np.ndarray:
    """生成固定高度航线点列。"""

    xs = np.linspace(-8800.0, 8800.0, count, dtype=np.float32)
    zs = route_z_from_x(xs).astype(np.float32)
    ys = np.full_like(xs, ROUTE_ALTITUDE_M)
    return np.column_stack([xs, ys, zs]).astype(np.float32)


def route_normal_at(t: float) -> tuple[float, float]:
    """返回航线在参数 t 处的水平法向。"""

    x0 = -8800.0 + 17600.0 * t
    x1 = x0 + 20.0
    tangent_x = 20.0
    tangent_z = route_z_from_x(x1) - route_z_from_x(x0)
    length = math.hypot(tangent_x, tangent_z)
    return -tangent_z / length, tangent_x / length


def build_peak_specs() -> list[PeakSpec]:
    """按航线走廊两侧布置 8 座主峰。"""

    layout = [
        (0.04, -1.0, 2380.0, 2100.0, 1300.0, -26.0),
        (0.15, 1.0, 2720.0, 2400.0, 1450.0, 34.0),
        (0.28, -1.0, 3000.0, 2550.0, 1500.0, 10.0),
        (0.40, 1.0, 2440.0, 2200.0, 1320.0, -30.0),
        (0.52, -1.0, 2580.0, 2300.0, 1380.0, 42.0),
        (0.64, 1.0, 2860.0, 2550.0, 1450.0, -18.0),
        (0.77, -1.0, 2480.0, 2250.0, 1360.0, 30.0),
        (0.90, 1.0, 2320.0, 2100.0, 1260.0, -38.0),
    ]
    peaks: list[PeakSpec] = []
    for t, side, height, radius_x, radius_z, angle in layout:
        x = -8800.0 + 17600.0 * t
        z = route_z_from_x(x)
        normal_x, normal_z = route_normal_at(t)
        offset = side * (2600.0 + 620.0 * math.sin(math.tau * t * 1.7))
        peaks.append(PeakSpec(x + normal_x * offset, z + normal_z * offset, radius_x, radius_z, angle, height))
    return peaks


def build_preview_axis(grid_size: int, extent: float) -> np.ndarray:
    """生成中心高分辨率、外围低分辨率的地形采样轴。"""

    core_half = 12000.0
    core_count = 513
    if grid_size < core_count + 2 or extent <= core_half:
        return np.linspace(-extent, extent, grid_size, dtype=np.float32)
    side_count = (grid_size - core_count) // 2
    extra = grid_size - core_count - side_count * 2
    if extra:
        core_count += extra
    left = np.linspace(-extent, -core_half, side_count, endpoint=False, dtype=np.float32)
    core = np.linspace(-core_half, core_half, core_count, dtype=np.float32)
    right = np.linspace(core_half, extent, side_count + 1, dtype=np.float32)[1:]
    return np.concatenate([left, core, right]).astype(np.float32)


def terrain_distance_to_route(x_grid: np.ndarray, z_grid: np.ndarray, samples: np.ndarray) -> np.ndarray:
    """计算每个格点到航线采样点的近似水平距离。"""

    distance_sq = np.full(x_grid.shape, np.inf, dtype=np.float32)
    for point in samples[::2]:
        dx = x_grid - point[0]
        dz = z_grid - point[2]
        distance_sq = np.minimum(distance_sq, dx * dx + dz * dz)
    return np.sqrt(distance_sq)


class TerrainPreviewScene:
    """承载同一份地形几何和派生线层。"""

    def __init__(self, grid_size: int, seed: int, contour_step: float) -> None:
        """生成高度场、颜色、风险区和线层缓存。"""

        self.grid_size = grid_size
        self.seed = seed
        self.contour_step = contour_step
        self.extent = MAP_SIZE_M / 2.0
        self.axis = build_preview_axis(grid_size, self.extent)
        self.x_grid, self.z_grid = np.meshgrid(self.axis, self.axis)
        self.route = route_points()
        self.peaks = build_peak_specs()
        self.height = self._build_height_field()
        self.grad_z, self.grad_x = np.gradient(self.height, self.axis, self.axis)
        self.slope = np.sqrt(self.grad_x * self.grad_x + self.grad_z * self.grad_z)
        self.normals = self._build_normals()
        self.risk_peaks = self._select_risk_peaks()
        self.risk_mask = self._build_risk_mask()
        self.colors_a = self._build_style_a_colors()
        self.colors_b = self._build_style_b_colors()
        self.contour_segments = self._build_contour_segments(style="b")
        self.hazard_segments = self._build_contour_segments(style="hazard")
        self.hazard_grid_lines = self._build_hazard_grid_lines()
        self.buffer_loops = self._build_buffer_loops()
        self.grid_lines = self._build_grid_lines()
        self.waypoint_rings = self._build_waypoint_rings()
        self.blocked_route = self._build_blocked_route()
        self.blocked_cross = self._build_blocked_cross()
        self.drone_icons = self._build_drone_icons()

    def _build_height_field(self) -> np.ndarray:
        """生成险峻山地高度场，主峰外叠加脊状噪声和域扭曲。"""

        u = (self.x_grid + self.extent) / MAP_SIZE_M
        v = (self.z_grid + self.extent) / MAP_SIZE_M
        warp_x = fbm(u, v, 3, 4, self.seed + 11) * 760.0 + fbm(u + 0.31, v - 0.17, 7, 3, self.seed + 12) * 230.0
        warp_z = fbm(u - 0.23, v + 0.29, 3, 4, self.seed + 21) * 760.0 + fbm(u + 0.14, v + 0.36, 7, 3, self.seed + 22) * 230.0
        xw = self.x_grid + warp_x
        zw = self.z_grid + warp_z
        uw = (xw + self.extent) / MAP_SIZE_M
        vw = (zw + self.extent) / MAP_SIZE_M
        global_ridge = ridged_fbm(uw, vw, 10, 5, self.seed + 31)
        sharp_ridge = ridged_fbm(uw + 0.13, vw - 0.07, 28, 4, self.seed + 32)
        micro_ridge = ridged_fbm(uw - 0.19, vw + 0.11, 58, 3, self.seed + 33)
        distance_to_route = terrain_distance_to_route(self.x_grid, self.z_grid, self.route)
        corridor_gate = 0.30 + 0.70 * smoothstep((distance_to_route - 780.0) / 3000.0)
        main = np.zeros_like(self.x_grid, dtype=np.float32)
        for index, peak in enumerate(self.peaks):
            angle = math.radians(peak.angle_deg)
            cos_a = math.cos(angle)
            sin_a = math.sin(angle)
            dx = xw - peak.x
            dz = zw - peak.z
            local_x = (dx * cos_a + dz * sin_a) / peak.radius_x
            local_z = (-dx * sin_a + dz * cos_a) / peak.radius_z
            body = np.exp(-1.12 * (np.abs(local_x) ** 1.55 + np.abs(local_z) ** 1.82))
            striation = 0.5 + 0.5 * np.sin(local_x * 18.0 + local_z * 7.2 + global_ridge * 7.0 + index)
            knife_ridge = np.maximum(0.0, np.sin(local_x * 28.0 - local_z * 5.5 + sharp_ridge * 8.0 + index)) ** 3.2
            gullies = (1.0 - sharp_ridge) ** 2.4 * smoothstep(body * 1.85)
            rugged = 0.58 + 0.64 * global_ridge + 0.28 * sharp_ridge + 0.24 * striation + 0.22 * knife_ridge - 0.34 * gullies
            main += peak.height * body * np.clip(rugged, 0.42, 1.42)
            local_detail = (micro_ridge - 0.42) * (0.72 + knife_ridge) - gullies * 0.30
            main += peak.height * 0.115 * (body ** 0.82) * np.clip(local_detail, -0.55, 0.85)
        small = self._build_corridor_hills()
        connectors = self._build_connector_hills()
        rolling = (
            115.0 * fbm(u, v, 5, 4, self.seed + 41)
            + 90.0 * ridged_fbm(uw, vw, 16, 3, self.seed + 42)
            + 42.0 * (ridged_fbm(uw + 0.07, vw - 0.21, 42, 3, self.seed + 43) - 0.42)
        )
        terrain = main * corridor_gate + connectors + small + rolling + 65.0
        edge_x = smoothstep((self.extent - np.abs(self.x_grid)) / 2100.0)
        edge_z = smoothstep((self.extent - np.abs(self.z_grid)) / 2100.0)
        edge = np.minimum(edge_x, edge_z)
        terrain = 28.0 + terrain * edge
        terrain -= np.min(terrain)
        terrain += 35.0
        terrain *= 3040.0 / np.max(terrain)
        return terrain.astype(np.float32)

    def _build_corridor_hills(self) -> np.ndarray:
        """在航线走廊内布置低矮山头，增加穿越层次。"""

        rng = np.random.default_rng(self.seed + 101)
        hills = np.zeros_like(self.x_grid, dtype=np.float32)
        for t in np.linspace(0.07, 0.93, 18):
            x = -8800.0 + 17600.0 * t
            z = route_z_from_x(x)
            normal_x, normal_z = route_normal_at(float(t))
            offset = rng.uniform(-720.0, 720.0)
            center_x = x + normal_x * offset + rng.uniform(-160.0, 160.0)
            center_z = z + normal_z * offset + rng.uniform(-160.0, 160.0)
            radius = rng.uniform(260.0, 540.0)
            height = rng.uniform(300.0, 600.0)
            d2 = ((self.x_grid - center_x) ** 2 + (self.z_grid - center_z) ** 2) / (radius * radius)
            hills += height * np.exp(-1.85 * d2)
        return hills

    def _build_connector_hills(self) -> np.ndarray:
        """生成连接主峰之间的中等丘陵，避免画面出现平坦空地。"""

        rng = np.random.default_rng(self.seed + 151)
        hills = np.zeros_like(self.x_grid, dtype=np.float32)
        for first, second in zip(self.peaks[:-1], self.peaks[1:]):
            for mix in (0.28, 0.58):
                center_x = first.x * (1.0 - mix) + second.x * mix + rng.uniform(-520.0, 520.0)
                center_z = first.z * (1.0 - mix) + second.z * mix + rng.uniform(-520.0, 520.0)
                radius_x = rng.uniform(760.0, 1350.0)
                radius_z = rng.uniform(620.0, 1160.0)
                angle = rng.uniform(-55.0, 55.0)
                height = rng.uniform(360.0, 920.0)
                hills += height * self._elliptic_bump(center_x, center_z, radius_x, radius_z, angle)
        for _ in range(64):
            x = rng.uniform(-self.extent * 0.90, self.extent * 0.90)
            z = rng.uniform(-self.extent * 0.86, self.extent * 0.86)
            radius_x = rng.uniform(1000.0, 2600.0)
            radius_z = rng.uniform(760.0, 1900.0)
            angle = rng.uniform(-75.0, 75.0)
            height = rng.uniform(240.0, 860.0)
            hills += height * self._elliptic_bump(x, z, radius_x, radius_z, angle)
        return hills

    def _elliptic_bump(self, center_x: float, center_z: float, radius_x: float, radius_z: float, angle_deg: float) -> np.ndarray:
        """返回旋转椭圆丘陵权重。"""

        angle = math.radians(angle_deg)
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        dx = self.x_grid - center_x
        dz = self.z_grid - center_z
        local_x = (dx * cos_a + dz * sin_a) / radius_x
        local_z = (-dx * sin_a + dz * cos_a) / radius_z
        return np.exp(-1.45 * (local_x * local_x + local_z * local_z))

    def _build_normals(self) -> np.ndarray:
        """按高度梯度生成平滑法线。"""

        nx = -self.grad_x
        ny = np.ones_like(nx)
        nz = -self.grad_z
        length = np.sqrt(nx * nx + ny * ny + nz * nz)
        return np.stack([nx / length, ny / length, nz / length], axis=-1).astype(np.float32)

    def _select_risk_peaks(self) -> list[PeakSpec]:
        """选择航线两侧各一座近中景主峰作为风险区。"""

        # 固定选中段两侧山体，避免自动评分把两座风险区都落在画面同侧。
        return [self.peaks[2], self.peaks[5]]

    def _build_risk_mask(self) -> np.ndarray:
        """生成两座风险山体的范围遮罩。"""

        mask = np.zeros(self.height.shape, dtype=bool)
        for peak in self.risk_peaks:
            angle = math.radians(peak.angle_deg)
            cos_a = math.cos(angle)
            sin_a = math.sin(angle)
            dx = self.x_grid - peak.x
            dz = self.z_grid - peak.z
            local_x = (dx * cos_a + dz * sin_a) / (peak.radius_x * 0.88)
            local_z = (-dx * sin_a + dz * cos_a) / (peak.radius_z * 0.88)
            mask |= (local_x * local_x + local_z * local_z) <= 1.45
        mask &= self.height > ROUTE_ALTITUDE_M * 0.72
        return mask

    def _build_style_a_colors(self) -> np.ndarray:
        """写实风格颜色：海拔和坡度共同驱动植被、岩石和雪线。"""

        h = np.clip(self.height / 3040.0, 0.0, 1.0)
        slope_mix = smoothstep((self.slope - 0.30) / 0.80)
        low = lerp_color((0.10, 0.17, 0.12), (0.29, 0.34, 0.18), smoothstep(h / 0.30))
        mid = lerp_color((0.29, 0.34, 0.18), (0.46, 0.40, 0.31), smoothstep((h - 0.22) / 0.38))
        rock = lerp_color((0.34, 0.32, 0.30), (0.70, 0.68, 0.63), smoothstep((h - 0.50) / 0.36))
        snow = lerp_color((0.70, 0.68, 0.63), (0.84, 0.84, 0.79), smoothstep((h - 0.80) / 0.16))
        base = np.where((h < 0.30)[..., None], low, mid)
        base = np.where((h > 0.58)[..., None], rock, base)
        base = np.where((h > 0.82)[..., None], snow, base)
        mixed = base * (1.0 - slope_mix[..., None] * 0.68) + rock * (slope_mix[..., None] * 0.68)
        edge_distance = self.extent - np.maximum(np.abs(self.x_grid), np.abs(self.z_grid))
        edge_visibility = smoothstep(edge_distance / 7600.0)[..., None]
        mixed = mixed * edge_visibility + np.array([0.03, 0.055, 0.080], dtype=np.float32) * (1.0 - edge_visibility)
        light_dir = np.array([-0.56, 0.68, 0.47], dtype=np.float32)
        light_dir /= np.linalg.norm(light_dir)
        lambert = np.clip(np.sum(self.normals * light_dir, axis=-1), 0.0, 1.0)
        relief = 0.62 + 0.54 * (lambert[..., None] ** 0.85)
        ridge_shadow = np.clip(1.03 - self.slope[..., None] * 0.11, 0.72, 1.06)
        linear = srgb_to_linear(np.clip(mixed * relief * ridge_shadow, 0.0, 1.0))
        alpha = np.ones((*self.height.shape, 1), dtype=np.float32)
        return np.concatenate([linear.astype(np.float32), alpha], axis=-1)

    def _build_style_b_colors(self) -> np.ndarray:
        """指挥风颜色：低亮度墨绿到深灰海拔渐变。"""

        h = np.clip(self.height / 3040.0, 0.0, 1.0)
        valley = lerp_color((0.03, 0.055, 0.085), (0.05, 0.12, 0.13), smoothstep(h / 0.35))
        high = lerp_color((0.05, 0.12, 0.13), (0.20, 0.22, 0.24), smoothstep((h - 0.25) / 0.70))
        base = np.where((h < 0.35)[..., None], valley, high)
        edge_distance = self.extent - np.maximum(np.abs(self.x_grid), np.abs(self.z_grid))
        edge_visibility = smoothstep(edge_distance / 7000.0)[..., None]
        base = base * edge_visibility + np.array([0.01, 0.025, 0.035], dtype=np.float32) * (1.0 - edge_visibility)
        shade = np.clip(0.82 + self.slope[..., None] * 0.12, 0.78, 1.04)
        linear = srgb_to_linear(np.clip(base * shade, 0.0, 1.0))
        alpha = np.ones((*self.height.shape, 1), dtype=np.float32)
        return np.concatenate([linear.astype(np.float32), alpha], axis=-1)

    def height_at(self, x_coord: float, z_coord: float) -> float:
        """双线性采样高度。"""

        ix = int(np.clip(np.searchsorted(self.axis, x_coord) - 1, 0, self.grid_size - 2))
        iz = int(np.clip(np.searchsorted(self.axis, z_coord) - 1, 0, self.grid_size - 2))
        tx = (x_coord - float(self.axis[ix])) / max(float(self.axis[ix + 1] - self.axis[ix]), 1e-6)
        tz = (z_coord - float(self.axis[iz])) / max(float(self.axis[iz + 1] - self.axis[iz]), 1e-6)
        h00 = float(self.height[iz, ix])
        h10 = float(self.height[iz, ix + 1])
        h01 = float(self.height[iz + 1, ix])
        h11 = float(self.height[iz + 1, ix + 1])
        return (h00 * (1 - tx) + h10 * tx) * (1 - tz) + (h01 * (1 - tx) + h11 * tx) * tz

    def _build_contour_segments(self, style: str) -> list[np.ndarray]:
        """用 marching squares 生成等高线线段。"""

        sample_step = 2
        heights = self.height[::sample_step, ::sample_step]
        risk = self.risk_mask[::sample_step, ::sample_step]
        xs = self.x_grid[0, ::sample_step]
        zs = self.z_grid[::sample_step, 0]
        if style == "hazard":
            levels = np.arange(940.0, 3060.0, 145.0, dtype=np.float32)
        else:
            levels = np.arange(240.0, 3040.0, self.contour_step, dtype=np.float32)
        segments: list[np.ndarray] = []
        rows, cols = heights.shape
        for level in levels:
            for row in range(rows - 1):
                z0 = float(zs[row])
                z1 = float(zs[row + 1])
                for col in range(cols - 1):
                    if style == "hazard" and not bool(risk[row : row + 2, col : col + 2].any()):
                        continue
                    corners = [
                        (float(xs[col]), z0, float(heights[row, col])),
                        (float(xs[col + 1]), z0, float(heights[row, col + 1])),
                        (float(xs[col + 1]), z1, float(heights[row + 1, col + 1])),
                        (float(xs[col]), z1, float(heights[row + 1, col])),
                    ]
                    points: list[tuple[float, float, float]] = []
                    for a, b in ((0, 1), (1, 2), (2, 3), (3, 0)):
                        p0 = corners[a]
                        p1 = corners[b]
                        d0 = p0[2] - level
                        d1 = p1[2] - level
                        if d0 == 0.0 and d1 == 0.0:
                            continue
                        if d0 * d1 <= 0.0:
                            denom = p1[2] - p0[2]
                            mix = 0.0 if abs(denom) < 1e-6 else (float(level) - p0[2]) / denom
                            x = p0[0] + (p1[0] - p0[0]) * mix
                            z = p0[1] + (p1[1] - p0[1]) * mix
                            points.append((x, float(level) + (32.0 if style == "hazard" else 24.0), z))
                    if len(points) == 2:
                        segments.append(np.array(points, dtype=np.float32))
                    elif len(points) == 4:
                        segments.append(np.array([points[0], points[1]], dtype=np.float32))
                        segments.append(np.array([points[2], points[3]], dtype=np.float32))
        return segments

    def _build_grid_lines(self) -> list[np.ndarray]:
        """生成贴地经纬网格线。"""

        lines: list[np.ndarray] = []
        coords = np.arange(-23000.0, 23001.0, 1000.0, dtype=np.float32)
        samples = np.linspace(-23500.0, 23500.0, 210, dtype=np.float32)
        for x_coord in coords:
            pts = [(float(x_coord), self.height_at(float(x_coord), float(z)) + 18.0, float(z)) for z in samples]
            lines.append(np.array(pts, dtype=np.float32))
        for z_coord in coords:
            pts = [(float(x), self.height_at(float(x), float(z_coord)) + 18.0, float(z_coord)) for x in samples]
            lines.append(np.array(pts, dtype=np.float32))
        return lines

    def _build_hazard_grid_lines(self) -> list[np.ndarray]:
        """生成风险山体红色线框网格。"""

        lines: list[np.ndarray] = []
        for peak in self.risk_peaks:
            angle = math.radians(peak.angle_deg)
            cos_a = math.cos(angle)
            sin_a = math.sin(angle)
            rx = peak.radius_x * 0.92
            rz = peak.radius_z * 0.92
            for local_x in np.linspace(-rx, rx, 9, dtype=np.float32):
                points: list[tuple[float, float, float]] = []
                for local_z in np.linspace(-rz, rz, 96, dtype=np.float32):
                    if (local_x / rx) ** 2 + (local_z / rz) ** 2 > 1.28:
                        if len(points) >= 2:
                            lines.append(np.array(points, dtype=np.float32))
                        points = []
                        continue
                    x = peak.x + local_x * cos_a - local_z * sin_a
                    z = peak.z + local_x * sin_a + local_z * cos_a
                    points.append((float(x), self.height_at(float(x), float(z)) + 44.0, float(z)))
                if len(points) >= 2:
                    lines.append(np.array(points, dtype=np.float32))
            for local_z in np.linspace(-rz, rz, 9, dtype=np.float32):
                points = []
                for local_x in np.linspace(-rx, rx, 96, dtype=np.float32):
                    if (local_x / rx) ** 2 + (local_z / rz) ** 2 > 1.28:
                        if len(points) >= 2:
                            lines.append(np.array(points, dtype=np.float32))
                        points = []
                        continue
                    x = peak.x + local_x * cos_a - local_z * sin_a
                    z = peak.z + local_x * sin_a + local_z * cos_a
                    points.append((float(x), self.height_at(float(x), float(z)) + 46.0, float(z)))
                if len(points) >= 2:
                    lines.append(np.array(points, dtype=np.float32))
        return lines

    def _build_buffer_loops(self) -> list[np.ndarray]:
        """生成风险山脚淡青色虚线安全缓冲轮廓。"""

        loops: list[np.ndarray] = []
        for peak in self.risk_peaks:
            angle = math.radians(peak.angle_deg)
            cos_a = math.cos(angle)
            sin_a = math.sin(angle)
            rx = peak.radius_x * 1.12
            rz = peak.radius_z * 1.12
            samples = []
            for theta in np.linspace(0.0, math.tau, 144, endpoint=False, dtype=np.float32):
                local_x = math.cos(float(theta)) * rx
                local_z = math.sin(float(theta)) * rz
                x = peak.x + local_x * cos_a - local_z * sin_a
                z = peak.z + local_x * sin_a + local_z * cos_a
                samples.append((float(x), self.height_at(float(x), float(z)) + 54.0, float(z)))
            for start in range(0, len(samples), 12):
                dash = samples[start : start + 7]
                if len(dash) >= 2:
                    loops.append(np.array(dash, dtype=np.float32))
        return loops

    def _build_waypoint_rings(self) -> list[np.ndarray]:
        """沿航线每约 2km 生成一个发光圆环航点。"""

        rings: list[np.ndarray] = []
        cumulative = [0.0]
        for first, second in zip(self.route[:-1], self.route[1:]):
            cumulative.append(cumulative[-1] + float(np.linalg.norm(second[[0, 2]] - first[[0, 2]])))
        cumulative_arr = np.array(cumulative, dtype=np.float32)
        targets = np.arange(1200.0, cumulative_arr[-1] - 600.0, 2000.0, dtype=np.float32)
        for target in targets:
            index = int(np.searchsorted(cumulative_arr, target))
            index = max(1, min(index, len(self.route) - 1))
            prev_len = cumulative_arr[index - 1]
            next_len = cumulative_arr[index]
            mix = 0.0 if next_len <= prev_len else float((target - prev_len) / (next_len - prev_len))
            center = self.route[index - 1] * (1.0 - mix) + self.route[index] * mix
            radius = 230.0
            points = []
            for theta in np.linspace(0.0, math.tau, 64, endpoint=True, dtype=np.float32):
                points.append((float(center[0] + math.cos(float(theta)) * radius), ROUTE_ALTITUDE_M + 22.0, float(center[2] + math.sin(float(theta)) * radius)))
            rings.append(np.array(points, dtype=np.float32))
        return rings

    def _build_blocked_route(self) -> list[np.ndarray]:
        """生成被风险区截断的红色虚线原始航线。"""

        start = self.route[0]
        end = self.route[-1]
        mid_x = sum(peak.x for peak in self.risk_peaks) / len(self.risk_peaks)
        mid_z = sum(peak.z for peak in self.risk_peaks) / len(self.risk_peaks)
        samples = []
        for t in np.linspace(0.0, 1.0, 220, dtype=np.float32):
            one_minus = 1.0 - float(t)
            x = one_minus * one_minus * float(start[0]) + 2.0 * one_minus * float(t) * mid_x + float(t) * float(t) * float(end[0])
            z = one_minus * one_minus * float(start[2]) + 2.0 * one_minus * float(t) * mid_z + float(t) * float(t) * float(end[2])
            samples.append((x, ROUTE_ALTITUDE_M + 220.0, z))
        dashes: list[np.ndarray] = []
        for start_index in range(0, len(samples) - 1, 12):
            t_mid = (start_index + 4) / max(1, len(samples) - 1)
            if 0.42 <= t_mid <= 0.62:
                continue
            dash = samples[start_index : start_index + 7]
            if len(dash) >= 2:
                dashes.append(np.array(dash, dtype=np.float32))
        return dashes

    def _build_blocked_cross(self) -> list[np.ndarray]:
        """生成原始航线进入风险区处的红圈叉号。"""

        point = self._blocked_route_point(0.42)
        radius = 320.0
        circle = []
        for theta in np.linspace(0.0, math.tau, 72, endpoint=True, dtype=np.float32):
            circle.append((point[0] + math.cos(float(theta)) * radius, point[1], point[2] + math.sin(float(theta)) * radius))
        lines = [np.array(circle, dtype=np.float32)]
        lines.append(np.array([(point[0] - radius * 0.58, point[1], point[2] - radius * 0.58), (point[0] + radius * 0.58, point[1], point[2] + radius * 0.58)], dtype=np.float32))
        lines.append(np.array([(point[0] - radius * 0.58, point[1], point[2] + radius * 0.58), (point[0] + radius * 0.58, point[1], point[2] - radius * 0.58)], dtype=np.float32))
        return lines

    def _blocked_route_point(self, t: float) -> tuple[float, float, float]:
        """按二次贝塞尔返回原始航线点。"""

        start = self.route[0]
        end = self.route[-1]
        mid_x = sum(peak.x for peak in self.risk_peaks) / len(self.risk_peaks)
        mid_z = sum(peak.z for peak in self.risk_peaks) / len(self.risk_peaks)
        one_minus = 1.0 - t
        x = one_minus * one_minus * float(start[0]) + 2.0 * one_minus * t * mid_x + t * t * float(end[0])
        z = one_minus * one_minus * float(start[2]) + 2.0 * one_minus * t * mid_z + t * t * float(end[2])
        return x, ROUTE_ALTITUDE_M + 240.0, z

    def _build_drone_icons(self) -> list[np.ndarray]:
        """生成白色无人机俯视剪影的简化线框。"""

        icons: list[np.ndarray] = []
        for route_index in (34, 76, 118):
            center = self.route[min(route_index, len(self.route) - 1)]
            previous = self.route[max(0, route_index - 2)]
            next_point = self.route[min(len(self.route) - 1, route_index + 2)]
            dx = float(next_point[0] - previous[0])
            dz = float(next_point[2] - previous[2])
            length = math.hypot(dx, dz)
            forward = (1.0, 0.0) if length <= 1e-6 else (dx / length, dz / length)
            side = (-forward[1], forward[0])
            scale = 270.0
            cx = float(center[0])
            cz = float(center[2])
            cy = ROUTE_ALTITUDE_M + 120.0
            nose = (cx + forward[0] * scale, cy, cz + forward[1] * scale)
            tail = (cx - forward[0] * scale * 0.55, cy, cz - forward[1] * scale * 0.55)
            left = (cx + side[0] * scale * 0.72, cy, cz + side[1] * scale * 0.72)
            right = (cx - side[0] * scale * 0.72, cy, cz - side[1] * scale * 0.72)
            icons.append(np.array([nose, left, tail, right, nose], dtype=np.float32))
            icons.append(np.array([left, right], dtype=np.float32))
        return icons


class TerrainPreviewGeometry(QQuick3DGeometry):
    """地形网格几何，顶点布局与正式 terrain_geometry.py 保持一致。"""

    styleNameChanged = Signal()

    def __init__(self, parent: object | None = None) -> None:
        """初始化地形几何。"""

        super().__init__(parent)
        self._style_name = "a"
        self._rebuild()

    @Property(str, notify=styleNameChanged)
    def styleName(self) -> str:
        """返回样张风格名。"""

        return self._style_name

    @styleName.setter
    def styleName(self, value: str) -> None:
        """切换样张风格并重建顶点色。"""

        normalized = value if value in {"a", "b"} else "a"
        if normalized == self._style_name:
            return
        self._style_name = normalized
        self._rebuild()
        self.styleNameChanged.emit()

    def _rebuild(self) -> None:
        """重建地形顶点和索引。"""

        scene = require_scene()
        colors = scene.colors_a if self._style_name == "a" else scene.colors_b
        vertices = bytearray()
        for row in range(scene.grid_size):
            for col in range(scene.grid_size):
                vertices.extend(
                    struct.pack(
                        "<ffffffffffff",
                        float(scene.x_grid[row, col]),
                        float(scene.height[row, col]),
                        float(scene.z_grid[row, col]),
                        float(scene.normals[row, col, 0]),
                        float(scene.normals[row, col, 1]),
                        float(scene.normals[row, col, 2]),
                        col / (scene.grid_size - 1),
                        row / (scene.grid_size - 1),
                        float(colors[row, col, 0]),
                        float(colors[row, col, 1]),
                        float(colors[row, col, 2]),
                        float(colors[row, col, 3]),
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
        self._apply_common_layout()
        self.setBounds(QVector3D(-scene.extent, 0.0, -scene.extent), QVector3D(scene.extent, float(scene.height.max()) + 80.0, scene.extent))
        self.setVertexData(QByteArray(bytes(vertices)))
        self.setIndexData(QByteArray(bytes(indices)))
        self.update()

    def _apply_common_layout(self) -> None:
        """提交 Quick3D 顶点布局描述。"""

        self.clear()
        self.setPrimitiveType(QQuick3DGeometry.PrimitiveType.Triangles)
        self.setStride(VERTEX_STRIDE)
        self.addAttribute(QQuick3DGeometry.Attribute.Semantic.PositionSemantic, 0, QQuick3DGeometry.Attribute.ComponentType.F32Type)
        self.addAttribute(QQuick3DGeometry.Attribute.Semantic.NormalSemantic, 3 * FLOAT_SIZE, QQuick3DGeometry.Attribute.ComponentType.F32Type)
        self.addAttribute(QQuick3DGeometry.Attribute.Semantic.TexCoordSemantic, 6 * FLOAT_SIZE, QQuick3DGeometry.Attribute.ComponentType.F32Type)
        self.addAttribute(QQuick3DGeometry.Attribute.Semantic.ColorSemantic, 8 * FLOAT_SIZE, QQuick3DGeometry.Attribute.ComponentType.F32Type)
        self.addAttribute(QQuick3DGeometry.Attribute.Semantic.IndexSemantic, 0, QQuick3DGeometry.Attribute.ComponentType.U32Type)


class HazardPatchGeometry(TerrainPreviewGeometry):
    """风险区贴地半透明面片。"""

    def _rebuild(self) -> None:
        """只为风险区网格单元生成索引，颜色统一为暗红。"""

        scene = require_scene()
        red = srgb_to_linear(np.array([1.0, 0.02, 0.01], dtype=np.float32))
        vertices = bytearray()
        for row in range(scene.grid_size):
            for col in range(scene.grid_size):
                alpha = 0.72 if scene.risk_mask[row, col] else 0.0
                vertices.extend(
                    struct.pack(
                        "<ffffffffffff",
                        float(scene.x_grid[row, col]),
                        float(scene.height[row, col] + 16.0),
                        float(scene.z_grid[row, col]),
                        float(scene.normals[row, col, 0]),
                        float(scene.normals[row, col, 1]),
                        float(scene.normals[row, col, 2]),
                        col / (scene.grid_size - 1),
                        row / (scene.grid_size - 1),
                        float(red[0]),
                        float(red[1]),
                        float(red[2]),
                        alpha,
                    )
                )
        indices = bytearray()
        for row in range(scene.grid_size - 1):
            for col in range(scene.grid_size - 1):
                if not bool(scene.risk_mask[row : row + 2, col : col + 2].any()):
                    continue
                top_left = row * scene.grid_size + col
                top_right = top_left + 1
                bottom_left = top_left + scene.grid_size
                bottom_right = bottom_left + 1
                indices.extend(struct.pack("<IIIIII", top_left, bottom_left, top_right, top_right, bottom_left, bottom_right))
        self._apply_common_layout()
        self.setBounds(QVector3D(-scene.extent, 0.0, -scene.extent), QVector3D(scene.extent, float(scene.height.max()) + 100.0, scene.extent))
        self.setVertexData(QByteArray(bytes(vertices)))
        self.setIndexData(QByteArray(bytes(indices)))
        self.update()


class LinePreviewGeometry(QQuick3DGeometry):
    """航线、等高线和网格线三角带几何。"""

    kindChanged = Signal()
    styleNameChanged = Signal()
    widthValueChanged = Signal()

    def __init__(self, parent: object | None = None) -> None:
        """初始化线层几何。"""

        super().__init__(parent)
        self._kind = "routeCore"
        self._style_name = "a"
        self._width_value = 80.0
        self._rebuild()

    @Property(str, notify=kindChanged)
    def kind(self) -> str:
        """返回线层类型。"""

        return self._kind

    @kind.setter
    def kind(self, value: str) -> None:
        """切换线层类型。"""

        allowed = {
            "routeCore",
            "routeGlow",
            "contour",
            "grid",
            "hazard",
            "hazardGrid",
            "buffer",
            "waypoint",
            "blockedRoute",
            "blockedCross",
            "drone",
        }
        normalized = value if value in allowed else "routeCore"
        if normalized == self._kind:
            return
        self._kind = normalized
        self._rebuild()
        self.kindChanged.emit()

    @Property(str, notify=styleNameChanged)
    def styleName(self) -> str:
        """返回样张风格名。"""

        return self._style_name

    @styleName.setter
    def styleName(self, value: str) -> None:
        """设置样张风格名。"""

        normalized = value if value in {"a", "b"} else "a"
        if normalized == self._style_name:
            return
        self._style_name = normalized
        self._rebuild()
        self.styleNameChanged.emit()

    @Property(float, notify=widthValueChanged)
    def widthValue(self) -> float:
        """返回线宽，单位为米。"""

        return self._width_value

    @widthValue.setter
    def widthValue(self, value: float) -> None:
        """设置线宽并重建几何。"""

        try:
            normalized = float(value)
        except (TypeError, ValueError):
            normalized = self._width_value
        normalized = max(2.0, min(420.0, normalized))
        if math.isclose(normalized, self._width_value, rel_tol=1e-6):
            return
        self._width_value = normalized
        self._rebuild()
        self.widthValueChanged.emit()

    def _rebuild(self) -> None:
        """按线层类型重建三角带。"""

        scene = require_scene()
        if self._kind in {"routeCore", "routeGlow"}:
            polylines = [scene.route.copy()]
        elif self._kind == "contour":
            polylines = scene.contour_segments
        elif self._kind == "grid":
            polylines = scene.grid_lines
        elif self._kind == "hazardGrid":
            polylines = scene.hazard_grid_lines
        elif self._kind == "buffer":
            polylines = scene.buffer_loops
        elif self._kind == "waypoint":
            polylines = scene.waypoint_rings
        elif self._kind == "blockedRoute":
            polylines = scene.blocked_route
        elif self._kind == "blockedCross":
            polylines = scene.blocked_cross
        elif self._kind == "drone":
            polylines = scene.drone_icons
        else:
            polylines = scene.hazard_segments
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
    """把多条折线转成水平展开的三角带。"""

    half_width = width / 2.0
    vertices = bytearray()
    indices = bytearray()
    vertex_base = 0
    all_points: list[np.ndarray] = []
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
            u_coord = index / max(1, len(line) - 1)
            append_line_vertex(vertices, left, u_coord, 0.0)
            append_line_vertex(vertices, right, u_coord, 1.0)
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
    margin = width + 60.0
    bounds = (
        float(np.min(merged[:, 0]) - margin),
        float(np.min(merged[:, 1]) - margin),
        float(np.min(merged[:, 2]) - margin),
        float(np.max(merged[:, 0]) + margin),
        float(np.max(merged[:, 1]) + margin),
        float(np.max(merged[:, 2]) + margin),
    )
    return vertices, indices, bounds


def append_line_vertex(vertices: bytearray, position: tuple[float, float, float], u_coord: float, v_coord: float) -> None:
    """追加线层顶点。"""

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


def require_scene() -> TerrainPreviewScene:
    """返回已初始化的场景缓存。"""

    if SCENE is None:
        raise RuntimeError("场景尚未初始化")
    return SCENE


def render_style(app: QGuiApplication, qml_path: Path, style: str, output_path: Path, wait_ms: int) -> None:
    """显示 Quick3D 窗口并抓取 1600x900 PNG。"""

    view = QQuickView()
    view.setResizeMode(QQuickView.ResizeMode.SizeRootObjectToView)
    view.rootContext().setContextProperty("initialPreviewStyle", style)
    view.setWidth(WIDTH_PX)
    view.setHeight(HEIGHT_PX)
    view.setTitle(f"terrain style {style}")
    view.setSource(QUrl.fromLocalFile(str(qml_path)))
    if view.status() == QQuickView.Status.Error:
        for error in view.errors():
            print(error.toString(), file=sys.stderr)
        raise RuntimeError(f"QML 加载失败：{qml_path}")
    view.show()
    view.requestActivate()
    app.processEvents()
    loop = QEventLoop()
    QTimer.singleShot(wait_ms, loop.quit)
    loop.exec()
    image = view.grabWindow()
    if image.isNull():
        raise RuntimeError(f"抓取窗口失败：{style}")
    if image.width() != WIDTH_PX or image.height() != HEIGHT_PX:
        image = image.scaled(WIDTH_PX, HEIGHT_PX)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not image.save(str(output_path), "PNG"):
        raise RuntimeError(f"保存 PNG 失败：{output_path}")
    view.close()
    view.deleteLater()
    app.processEvents()


def parse_args() -> argparse.Namespace:
    """解析命令行参数。"""

    parser = argparse.ArgumentParser(description="生成 3D 山地地形两种视觉风格样张。")
    parser.add_argument("--grid-size", type=int, default=641, help="高度场采样边长，默认 641。")
    parser.add_argument("--seed", type=int, default=20260711, help="随机种子，默认 20260711。")
    parser.add_argument("--contour-step", type=float, default=185.0, help="风格 B 普通等高线间隔，单位米。")
    parser.add_argument("--wait-ms", type=int, default=1500, help="每张图显示后等待渲染稳定的毫秒数。")
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parent / "output", help="PNG 输出目录。")
    return parser.parse_args()


def main() -> int:
    """生成 style_a.png 和 style_b.png。"""

    args = parse_args()
    if args.grid_size < 128:
        raise ValueError("--grid-size 不能小于 128")
    os.environ.setdefault("QSG_RHI_BACKEND", "d3d11")
    os.environ.setdefault("QT_ENABLE_HIGHDPI_SCALING", "0")
    global SCENE
    SCENE = TerrainPreviewScene(args.grid_size, args.seed, args.contour_step)
    qmlRegisterType(TerrainPreviewGeometry, "TerrainPreview", 1, 0, "TerrainPreviewGeometry")
    qmlRegisterType(HazardPatchGeometry, "TerrainPreview", 1, 0, "HazardPatchGeometry")
    qmlRegisterType(LinePreviewGeometry, "TerrainPreview", 1, 0, "LinePreviewGeometry")
    app = QGuiApplication.instance() or QGuiApplication(sys.argv)
    qml_path = Path(__file__).resolve().parent / "preview_scene.qml"
    render_style(app, qml_path, "a", args.output_dir / "style_a.png", args.wait_ms)
    render_style(app, qml_path, "b", args.output_dir / "style_b.png", args.wait_ms)
    for filename in ("style_a.png", "style_b.png"):
        path = args.output_dir / filename
        size_kb = path.stat().st_size / 1024.0
        print(f"{path.resolve()}  {size_kb:.1f} KB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
