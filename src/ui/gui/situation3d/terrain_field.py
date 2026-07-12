"""布局驱动的 3D 山地高度场生成器。注意：只服务 GUI 显示层，不参与仿真计算。"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import time
from typing import Any

import numpy as np


DEFAULT_TERRAIN_RESOLUTION = 641
LOW_SPEC_TERRAIN_RESOLUTION = 384
_MIN_RESOLUTION = 96
_MAX_RESOLUTION = 1024
_SRGB_LOW = np.array([0.045, 0.082, 0.105], dtype=np.float32)
_SRGB_MID = np.array([0.205, 0.260, 0.225], dtype=np.float32)
_SRGB_HIGH = np.array([0.650, 0.655, 0.610], dtype=np.float32)
_SRGB_SNOW = np.array([0.845, 0.855, 0.825], dtype=np.float32)

# 细节层生成策略说明：
# 1. 布局 JSON 只描述山脉链、峰、鞍部和风险区，不直接携带网格点。
# 2. 高度场先按山脉链生成主体，再叠加多角度 ridged fBm。
# 3. 噪声只作用在视觉细节上，不回写布局 JSON，也不影响仿真航线。
# 4. 风险区只来自显式 risk_zone 标记，避免 P1 阶段误把所有高山当障碍。
# 5. 坐标在本模块仍使用 ENU；Quick3D 的 z 轴翻转留给几何打包阶段。
# 6. 颜色预计算为线性 RGB，完全对齐原型 style A 顶点色管线。
# 7. 边缘高度淡出服务远景雾化，避免相机低角度看到地形硬截面。
# 8. 默认 641 分辨率对应约 41 万顶点，生成后由 QQuick3DGeometry 上传 GPU。
# 9. 低配 384 分辨率保留同一布局和噪声，只降低采样密度。
# 10. 所有随机数都由布局 detail.seed 固定，保证验收截图可复现。
# 11. 山峰高度使用指数压缩，防止多个山脉链叠加处形成针状尖峰。
# 12. 山脊沿 polyline 分段取最大值，保留连绵走向而不是孤立圆包。
# 13. 鞍部用低宽高斯脊连接，不在航线语义上产生“可通行”判断。
# 14. 颜色只表达海拔、坡度和冷暖光感，不加入碎斑贴图。
# 15. 该模块不导入 PySide6，方便 LLT 直接在无 GUI 环境验证高度场。
# 16. terrain_geometry 是唯一把输出转成 Qt 顶点缓冲的使用方。
# 17. scene_data 只读取轻量元数据和风险区，不把完整高度数组塞进 JSON payload。
# 18. 大数组保持 numpy.float32，减少上传前内存和 QByteArray 拷贝压力。
# 19. 风险区半径和缓冲半径均保留米制单位，便于 QML 直接缩放线宽和圆盘。
# 20. 峰体椭圆长轴跟随山脉链方向，避免“馒头山”或规则网格感。
# 21. 低丘、远山和风险峰共用同一数学核，只由布局高度和半径区分。
# 22. 视觉验收重点是自然锋利山脊、冷峻色彩和无平行波纹。
# 23. 若后续 P2 接避障，本模块仍应只输出显示数据，不参与碰撞检测。
# 24. 如需热更新布局，调用方应清理缓存或改变 layoutFile 触发几何重建。
# 25. Matplotlib 验收图同样从这里取数据，避免样张与正式模块分叉。
# 26. 所有高度单位为米；截图脚本显示为 km 只是展示坐标轴换算。
# 27. 局部 u/v 原点由布局 geo_reference 说明，生成高度场时不需要经纬度。
# 28. 经纬度转换留在配置航线文件中完成，保持地形层和 data.geo 低耦合。
# 29. 域扭曲幅度刻意较小，只破坏规则波纹，不改变山脉大轮廓。
# 30. 法线用 numpy.gradient 计算，和最终高度数组完全一致。
# 31. 若布局文件损坏，terrain_geometry 会回退旧地形，避免 3D 视图空白。
# 32. 生成耗时写入 TerrainField，供最终验收报告和性能回归使用。
# 33. 海拔颜色的绿色分量被压低，避免田园风暖绿。
# 34. 山顶颜色偏冷灰白，和 style_a 的受光面基准一致。
# 35. 暗部底色偏深蓝黑，靠顶点色和 QML 冷补光共同形成氛围。
# 36. 安全缓冲轮廓由 scene_data 生成线段，本模块只提供中心和半径。
# 37. 输出 risk_zones 使用 dataclass，减少下游对原始 JSON 结构的依赖。
# 38. 布局字段缺失时采用保守默认值，便于局部测试最小布局。
# 39. 但正式配置仍应保留字段说明和显式 risk_zone，便于总设计师审阅。
# 40. 高度场生成无外部 IO，除读取布局文件外没有运行期副作用。


@dataclass(frozen=True)
class TerrainRiskZone:
    """风险山峰显示数据。注意：坐标仍为布局 ENU 米制坐标。"""

    zone_id: str
    label: str
    east_m: float
    north_m: float
    radius_m: float
    height_m: float
    buffer_radius_m: float


@dataclass(frozen=True)
class TerrainField:
    """高度场网格数据。注意：positions 的 x/z 已转换到 Quick3D 局部坐标。"""

    resolution: int
    center_east_m: float
    center_north_m: float
    width_m: float
    depth_m: float
    heights_m: np.ndarray
    normals: np.ndarray
    colors: np.ndarray
    risk_zones: tuple[TerrainRiskZone, ...]
    generation_time_ms: float


def load_terrain_layout(path: str | Path) -> dict[str, Any]:
    """读取地形布局 JSON。注意：调用方负责决定缺失文件是否回退旧地形。"""

    layout_path = Path(path)
    data = json.loads(layout_path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("terrain layout root must be an object")
    return data


def generate_terrain_field_from_file(path: str | Path, *, resolution: int = DEFAULT_TERRAIN_RESOLUTION) -> TerrainField:
    """从布局文件生成高度场。注意：耗时操作应只在布局或分辨率变化时触发。"""

    return generate_terrain_field(load_terrain_layout(path), resolution=resolution)


def generate_terrain_field(layout: dict[str, Any], *, resolution: int = DEFAULT_TERRAIN_RESOLUTION) -> TerrainField:
    """生成山脉链高度场。注意：三层架构中这里属于细节层，不读取 QML 样式。"""

    started = time.perf_counter()
    safe_resolution = _normalized_resolution(resolution)
    extent = terrain_extent_from_layout(layout)
    east = np.linspace(extent["min_east_m"], extent["max_east_m"], safe_resolution, dtype=np.float32)
    north = np.linspace(extent["min_north_m"], extent["max_north_m"], safe_resolution, dtype=np.float32)
    east_grid, north_grid = np.meshgrid(east, north)

    base = _base_relief(east_grid, north_grid, layout)
    detail = _detail_noise(east_grid, north_grid, layout)
    heights = np.maximum(0.0, base + detail).astype(np.float32)
    # 多条山脉交汇处会自然叠加，视觉层用指数压缩保留山势但避免针状过高峰。
    heights = (3300.0 * (1.0 - np.exp(-heights / 3300.0))).astype(np.float32)
    heights *= _edge_fade(east_grid, north_grid, extent)
    max_height = float(np.max(heights))
    if max_height > 1.0:
        # 原型高度场最终归一到 3040m；正式显示层照抄该高度域以匹配配色雪线。
        heights *= 3040.0 / max_height
    # 法线必须在最终高度归一化后计算，否则烘焙光照会偏软。
    normals = _normal_grid(heights, float(east[1] - east[0]), float(north[1] - north[0]))
    colors = _color_grid(heights, normals, east_grid, north_grid, extent)
    generation_time_ms = (time.perf_counter() - started) * 1000.0
    return TerrainField(
        resolution=safe_resolution,
        center_east_m=(extent["min_east_m"] + extent["max_east_m"]) / 2.0,
        center_north_m=(extent["min_north_m"] + extent["max_north_m"]) / 2.0,
        width_m=extent["max_east_m"] - extent["min_east_m"],
        depth_m=extent["max_north_m"] - extent["min_north_m"],
        heights_m=heights,
        normals=normals,
        colors=colors,
        risk_zones=tuple(risk_zones_from_layout(layout)),
        generation_time_ms=generation_time_ms,
    )


def terrain_extent_from_layout(layout: dict[str, Any]) -> dict[str, float]:
    """计算布局渲染范围。注意：输入 u/v 单位为 km，输出为 ENU 米。"""

    render_extent_km = float(_read_path(layout, ("map", "render_extent_km"), 58.0))
    chains = layout.get("mountain_chains")
    u_values: list[float] = []
    v_values: list[float] = []
    if isinstance(chains, list):
        for chain in chains:
            if not isinstance(chain, dict):
                continue
            for point in chain.get("polyline_uv", []):
                if _is_pair(point):
                    u_values.append(float(point[0]))
                    v_values.append(float(point[1]))
            for peak in chain.get("peaks", []):
                if not isinstance(peak, dict):
                    continue
                center = _peak_center_uv(chain, peak)
                radius = float(peak.get("base_radius_km", 1.0)) * 2.2
                u_values.extend([center[0] - radius, center[0] + radius])
                v_values.extend([center[1] - radius, center[1] + radius])
    flight = layout.get("flight")
    if isinstance(flight, dict):
        for key in ("original_route_uv", "planned_route_uv"):
            for point in flight.get(key, []):
                if _is_pair(point):
                    u_values.append(float(point[0]))
                    v_values.append(float(point[1]))
    if not u_values or not v_values:
        half = render_extent_km / 2.0
        return {"min_east_m": -half * 1000.0, "max_east_m": half * 1000.0, "min_north_m": -half * 1000.0, "max_north_m": half * 1000.0}
    center_u = (min(u_values) + max(u_values)) / 2.0
    center_v = (min(v_values) + max(v_values)) / 2.0
    half = render_extent_km / 2.0
    return {
        "min_east_m": (center_u - half) * 1000.0,
        "max_east_m": (center_u + half) * 1000.0,
        "min_north_m": (center_v - half) * 1000.0,
        "max_north_m": (center_v + half) * 1000.0,
    }


def risk_zones_from_layout(layout: dict[str, Any]) -> list[TerrainRiskZone]:
    """读取布局中显式标记的风险区。注意：只认 risk_zone 标记，不按高度自动推断。"""

    zones: list[TerrainRiskZone] = []
    chains = layout.get("mountain_chains")
    if not isinstance(chains, list):
        return zones
    for chain in chains:
        if not isinstance(chain, dict):
            continue
        for index, peak in enumerate(chain.get("peaks", [])):
            if not isinstance(peak, dict) or not bool(peak.get("risk_zone", False)):
                continue
            center_u, center_v = _peak_center_uv(chain, peak)
            radius_m = float(peak.get("risk_radius_km", peak.get("base_radius_km", 1.0))) * 1000.0
            zones.append(
                TerrainRiskZone(
                    zone_id=str(peak.get("id") or f"{chain.get('id', 'risk')}_{index}"),
                    label=str(peak.get("label") or chain.get("name") or "风险峰"),
                    east_m=center_u * 1000.0,
                    north_m=center_v * 1000.0,
                    radius_m=radius_m,
                    height_m=float(peak.get("height_m", 0.0)),
                    buffer_radius_m=float(peak.get("buffer_radius_km", peak.get("base_radius_km", 1.0) * 1.55)) * 1000.0,
                )
            )
    return zones


def _base_relief(east_grid: np.ndarray, north_grid: np.ndarray, layout: dict[str, Any]) -> np.ndarray:
    """按山脉链骨架生成主体高度。注意：连续山脊优先于单个孤立峰。"""

    height = np.zeros_like(east_grid, dtype=np.float32)
    chains = layout.get("mountain_chains")
    if not isinstance(chains, list):
        return height
    for chain in chains:
        if not isinstance(chain, dict):
            continue
        height += _chain_ridge_height(east_grid, north_grid, chain)
        for peak in chain.get("peaks", []):
            if isinstance(peak, dict):
                height += _peak_height(east_grid, north_grid, chain, peak)
    links = layout.get("saddle_links")
    if isinstance(links, list):
        for link in links:
            if isinstance(link, dict):
                height += _link_height(east_grid, north_grid, link)
    return height


def _chain_ridge_height(east_grid: np.ndarray, north_grid: np.ndarray, chain: dict[str, Any]) -> np.ndarray:
    """生成山脉链连续脊线。注意：沿 polyline 分段取最大值以保留走向。"""

    polyline = [_uv_to_m(point) for point in chain.get("polyline_uv", []) if _is_pair(point)]
    if len(polyline) < 2:
        return np.zeros_like(east_grid, dtype=np.float32)
    ridge_width = float(chain.get("ridge_width_km", 0.65)) * 860.0
    ridge_height = _chain_average_height(chain) * float(chain.get("saddle_height_factor", 0.35)) * 1.55
    ridge = np.zeros_like(east_grid, dtype=np.float32)
    for start, end in zip(polyline, polyline[1:]):
        distance = _segment_distance(east_grid, north_grid, start, end)
        ridge = np.maximum(ridge, np.exp(-0.5 * (distance / max(1.0, ridge_width)) ** 2) * ridge_height)
    return ridge.astype(np.float32)


def _peak_height(east_grid: np.ndarray, north_grid: np.ndarray, chain: dict[str, Any], peak: dict[str, Any]) -> np.ndarray:
    """生成单峰高度核。注意：椭圆方向跟随山脉走向，避免圆锥馒头山。"""

    center_u, center_v = _peak_center_uv(chain, peak)
    center_e = center_u * 1000.0
    center_n = center_v * 1000.0
    radius = float(peak.get("base_radius_km", 1.0)) * 1000.0
    aspect = max(0.35, float(peak.get("aspect_ratio", 1.3)))
    angle = _chain_angle_rad(chain)
    local_e, local_n = _rotated_offsets(east_grid - center_e, north_grid - center_n, angle)
    long_radius = radius * aspect * 0.84
    short_radius = radius * 0.74
    shoulder_long = np.abs(local_e / max(1.0, long_radius * 1.38))
    shoulder_short = np.abs(local_n / max(1.0, short_radius * 1.32))
    local_long = np.abs(local_e / max(1.0, long_radius))
    local_short = np.abs(local_n / max(1.0, short_radius))
    shoulder = np.exp(-0.82 * (shoulder_long ** 1.42 + shoulder_short ** 1.62))
    body = np.exp(-1.42 * (local_long ** 1.58 + local_short ** 1.86))
    summit = np.exp(-3.65 * (local_long ** 2.1 + local_short ** 2.35))
    return float(peak.get("height_m", 0.0)) * (0.36 * shoulder + 0.63 * body + 0.34 * summit)


def _link_height(east_grid: np.ndarray, north_grid: np.ndarray, link: dict[str, Any]) -> np.ndarray:
    """生成山脉链之间的低鞍部连接。注意：高度低于障碍峰，供航线穿越可读。"""

    if not _is_pair(link.get("from_uv")) or not _is_pair(link.get("to_uv")):
        return np.zeros_like(east_grid, dtype=np.float32)
    start = _uv_to_m(link["from_uv"])
    end = _uv_to_m(link["to_uv"])
    width = float(link.get("ridge_width_km", 0.42)) * 1000.0
    distance = _segment_distance(east_grid, north_grid, start, end)
    return np.exp(-0.5 * (distance / max(1.0, width)) ** 2) * float(link.get("height_m", 0.0))


def _detail_noise(east_grid: np.ndarray, north_grid: np.ndarray, layout: dict[str, Any]) -> np.ndarray:
    """生成原型同款细节噪声。注意：只叠加在布局山脉链骨架上。"""

    detail_cfg = layout.get("detail") if isinstance(layout.get("detail"), dict) else {}
    seed = int(detail_cfg.get("seed", 1949)) if isinstance(detail_cfg, dict) else 1949
    min_e = float(np.min(east_grid))
    max_e = float(np.max(east_grid))
    min_n = float(np.min(north_grid))
    max_n = float(np.max(north_grid))
    span_e = max(max_e - min_e, 1.0)
    span_n = max(max_n - min_n, 1.0)
    # 原型在归一化 u/v 上做两层域扭曲，再取多组 ridged fBm。
    u = (east_grid - min_e) / span_e
    v = (north_grid - min_n) / span_n
    warp_x = _fbm(u, v, 3, 4, seed + 11) * 760.0 + _fbm(u + 0.31, v - 0.17, 7, 3, seed + 12) * 230.0
    warp_n = _fbm(u - 0.23, v + 0.29, 3, 4, seed + 21) * 760.0 + _fbm(u + 0.14, v + 0.36, 7, 3, seed + 22) * 230.0
    # 域扭曲后的 uv 只供噪声采样，不改变布局层的峰心和航线坐标。
    uw = (east_grid + warp_x - min_e) / span_e
    vw = (north_grid + warp_n - min_n) / span_n
    rolling = (
        115.0 * _fbm(u, v, 5, 4, seed + 41)
        + 90.0 * _warped_ridged_fbm(uw, vw, 16, 3, seed + 42, 0.030)
        + 42.0 * (_warped_ridged_fbm(uw + 0.07, vw - 0.21, 38, 3, seed + 43, 0.020) - 0.42)
        + 330.0 * (_warped_ridged_fbm(uw * 1.017 + 0.041, vw * 0.983 - 0.027, 58, 5, seed + 503, 0.034) - 0.45)
        + 195.0 * (_warped_ridged_fbm(uw * 1.039 + 0.012, vw * 1.011 + 0.071, 96, 3, seed + 719, 0.020) - 0.42)
    )
    # rolling 只提供样张同款低丘和沟壑细节，主体山势仍来自布局 JSON。
    return rolling.astype(np.float32)


def _normal_grid(heights: np.ndarray, step_east_m: float, step_north_m: float) -> np.ndarray:
    """计算高度场法线。注意：法线数组顺序为 x/y/z，对应 Quick3D 坐标。"""

    grad_north, grad_east = np.gradient(heights, step_north_m, step_east_m)
    normal_x = -grad_east
    normal_y = np.ones_like(heights, dtype=np.float32)
    normal_z = grad_north
    length = np.sqrt(normal_x * normal_x + normal_y * normal_y + normal_z * normal_z)
    return np.dstack((normal_x / length, normal_y / length, normal_z / length)).astype(np.float32)


def _color_grid(
    heights: np.ndarray,
    normals: np.ndarray,
    east_grid: np.ndarray,
    north_grid: np.ndarray,
    extent: dict[str, float],
) -> np.ndarray:
    """预计算原型 style A 顶点色。注意：逐项照抄原型并输出线性 RGB。"""

    # 原型以 3040m 归一化高度；正式布局保留该常量，确保色阶和样张一致。
    h = np.clip(heights / 3040.0, 0.0, 1.0)
    # 颜色权重必须低通，不能直接追随细碎高度噪声。
    height_low = _box_blur(h, 7, passes=3)
    slope = _slope_grid(heights, east_grid, north_grid)
    # 坡度低通后只决定岩石区域，实际几何法线仍保留高频细节。
    slope_low = _box_blur(slope, 8, passes=3)
    curvature_ao = _curvature_ao_grid(heights, east_grid, north_grid, slope)
    # ridge_source 来自 AO 的反相，用来把山脊区域推向岩石色。
    ridge_source = _box_blur(np.clip(1.0 - curvature_ao, 0.0, 1.0), 10, passes=3)
    elevation_rock = _smoothstep((height_low - 0.60) / 0.22)
    ridge_rock = _smoothstep((ridge_source - 0.12) / 0.24) * _smoothstep((height_low - 0.50) / 0.24)
    wall_rock = _smoothstep((slope_low - 0.50) / 0.58) * _smoothstep((height_low - 0.56) / 0.26)
    rock_weight = np.clip(0.78 * elevation_rock + 0.18 * ridge_rock + 0.08 * wall_rock, 0.0, 1.0)
    # 原型先模糊再 smoothstep，岩石边界才不会出现噪点碎斑。
    rock_weight = _box_blur(rock_weight, 14, passes=2)
    rock_weight = _smoothstep((rock_weight - 0.10) / 0.82)
    snow_weight = _smoothstep((height_low - 0.86) / 0.12)
    snow_weight = _box_blur(snow_weight, 8, passes=2)
    vegetation = _lerp_color((0.045, 0.075, 0.060), (0.235, 0.265, 0.155), _smoothstep(height_low / 0.42))
    alpine = _lerp_color((0.235, 0.265, 0.155), (0.345, 0.335, 0.245), _smoothstep((height_low - 0.30) / 0.36))
    # 山腰植被只保留为暗橄榄底色，高处继续过渡到岩灰。
    vegetation = vegetation * (1.0 - _smoothstep((height_low - 0.28) / 0.36)[..., None]) + alpine * _smoothstep((height_low - 0.28) / 0.36)[..., None]
    rock_dark = _lerp_color((0.280, 0.280, 0.270), (0.500, 0.500, 0.480), _smoothstep((height_low - 0.52) / 0.30))
    rock_light = _lerp_color((0.500, 0.500, 0.480), (0.720, 0.725, 0.700), _smoothstep((height_low - 0.76) / 0.18))
    rock = rock_dark * (1.0 - snow_weight[..., None]) + rock_light * snow_weight[..., None]
    mixed = vegetation * (1.0 - rock_weight[..., None]) + rock * rock_weight[..., None]
    center_e = (float(extent["min_east_m"]) + float(extent["max_east_m"])) / 2.0
    center_n = (float(extent["min_north_m"]) + float(extent["max_north_m"])) / 2.0
    x_grid = east_grid - center_e
    z_grid = north_grid - center_n
    half_extent = min(float(extent["max_east_m"] - extent["min_east_m"]), float(extent["max_north_m"] - extent["min_north_m"])) / 2.0
    distance_scale = max(half_extent / 25000.0, 1e-6)
    far_distance = np.sqrt(x_grid * x_grid + z_grid * z_grid)
    # 远景蓝化在顶点色里完成，QML 不再叠额外雾光层。
    far_mix = _smoothstep((far_distance - 8800.0 * distance_scale) / (12800.0 * distance_scale))[..., None]
    far_blue = np.array([0.082, 0.118, 0.148], dtype=np.float32)
    mixed = mixed * (1.0 - far_mix * 0.74) + far_blue * (far_mix * 0.74)
    edge_distance = half_extent - np.maximum(np.abs(x_grid), np.abs(z_grid))
    # 边缘可见性压暗地形边界，配合深度雾避免露出裁切硬边。
    edge_visibility = _smoothstep(edge_distance / (18000.0 * distance_scale))[..., None]
    mixed = mixed * edge_visibility + np.array([0.036, 0.058, 0.088], dtype=np.float32) * (1.0 - edge_visibility)
    light_dir = np.array([-0.58, 0.66, 0.48], dtype=np.float32)
    light_dir /= np.linalg.norm(light_dir)
    # 光照方向同原型，暖光和冷影都烘焙进顶点色。
    lambert = np.clip(np.sum(normals * light_dir, axis=-1), 0.0, 1.0)
    rock_channel = rock_weight[..., None]
    warm_light = np.array([1.06, 1.00, 0.88], dtype=np.float32)
    neutral_light = np.array([0.82, 0.88, 0.74], dtype=np.float32)
    cool_shadow = np.array([0.42, 0.56, 0.74], dtype=np.float32)
    veg_temperature = cool_shadow * 0.20 + neutral_light * 0.80
    rock_temperature = cool_shadow * (1.0 - lambert[..., None]) + warm_light * lambert[..., None]
    temperature = veg_temperature * (1.0 - rock_channel) + rock_temperature * rock_channel
    veg_relief = 0.86 + 0.24 * (lambert[..., None] ** 0.82)
    rock_relief = 0.42 + 1.02 * (lambert[..., None] ** 0.78)
    # 植被 relief 小、岩石 relief 大，峰体才有冷硬层次。
    relief = veg_relief * (1.0 - rock_channel) + rock_relief * rock_channel
    ridge_shadow = np.clip(1.05 - slope[..., None] * 0.090, 0.68, 1.12)
    ao = curvature_ao[..., None]
    color = mixed * temperature * relief * ridge_shadow * ao
    # AO 二次混合暗蓝底色，复刻沟壑被压暗的样张观感。
    color = color * ao + np.array([0.020, 0.034, 0.052], dtype=np.float32) * (1.0 - ao)
    highlight = _smoothstep((lambert - 0.62) / 0.34)[..., None] * _smoothstep((height_low - 0.68) / 0.24)[..., None]
    color += np.array([0.035, 0.038, 0.035], dtype=np.float32) * highlight
    color *= 1.10 - 0.22 * far_mix
    # 最后三段对比拉伸和上下限保持原型原数值。
    color = np.maximum(color, np.array([0.040, 0.057, 0.074], dtype=np.float32))
    color = np.clip((color - 0.055) * 1.16 + 0.050, 0.0, 1.0)
    color = np.maximum(color, np.array([0.036, 0.052, 0.070], dtype=np.float32))
    color = np.minimum(color, np.array([0.85, 0.85, 0.82], dtype=np.float32))
    # 原型最后上传线性 RGB，正式 QML 地形材质也按素色接收器处理。
    return _srgb_to_linear(np.clip(color, 0.0, 1.0)).astype(np.float32)


def _edge_fade(east_grid: np.ndarray, north_grid: np.ndarray, extent: dict[str, float]) -> np.ndarray:
    """生成边缘淡出高度系数。注意：远景配合 QML 雾避免露出硬边。"""

    min_e, max_e = extent["min_east_m"], extent["max_east_m"]
    min_n, max_n = extent["min_north_m"], extent["max_north_m"]
    margin_e = (max_e - min_e) * 0.13
    margin_n = (max_n - min_n) * 0.13
    edge_e = np.minimum((east_grid - min_e) / margin_e, (max_e - east_grid) / margin_e)
    edge_n = np.minimum((north_grid - min_n) / margin_n, (max_n - north_grid) / margin_n)
    edge = np.clip(np.minimum(edge_e, edge_n), 0.0, 1.0)
    return edge * edge * (3.0 - 2.0 * edge)


def _segment_distance(
    east_grid: np.ndarray,
    north_grid: np.ndarray,
    start: tuple[float, float],
    end: tuple[float, float],
) -> np.ndarray:
    """计算网格点到线段距离。注意：完全向量化，避免逐点 Python 循环。"""

    sx, sy = start
    ex, ey = end
    vx = ex - sx
    vy = ey - sy
    denom = max(vx * vx + vy * vy, 1e-6)
    t = np.clip(((east_grid - sx) * vx + (north_grid - sy) * vy) / denom, 0.0, 1.0)
    px = sx + t * vx
    py = sy + t * vy
    return np.hypot(east_grid - px, north_grid - py)


def _peak_center_uv(chain: dict[str, Any], peak: dict[str, Any]) -> tuple[float, float]:
    """按 station 和 lateral_offset 计算峰心。注意：station 沿山脉折线弧长取点。"""

    polyline = [point for point in chain.get("polyline_uv", []) if _is_pair(point)]
    if not polyline:
        return 0.0, 0.0
    if len(polyline) == 1:
        base = (float(polyline[0][0]), float(polyline[0][1]))
        angle = 0.0
    else:
        station = min(1.0, max(0.0, float(peak.get("station", 0.5))))
        base, angle = _point_on_polyline(polyline, station)
    lateral = float(peak.get("lateral_offset_km", 0.0))
    return base[0] - math.sin(angle) * lateral, base[1] + math.cos(angle) * lateral


def _point_on_polyline(polyline: list[Any], station: float) -> tuple[tuple[float, float], float]:
    """返回折线指定比例处坐标和切向角。注意：用于峰体沿山脉链排布。"""

    lengths: list[float] = []
    total = 0.0
    for left, right in zip(polyline, polyline[1:]):
        segment = math.hypot(float(right[0]) - float(left[0]), float(right[1]) - float(left[1]))
        lengths.append(segment)
        total += segment
    target = total * station
    walked = 0.0
    for index, length in enumerate(lengths):
        left = polyline[index]
        right = polyline[index + 1]
        if walked + length >= target or index == len(lengths) - 1:
            ratio = 0.0 if length <= 1e-9 else (target - walked) / length
            x = float(left[0]) + (float(right[0]) - float(left[0])) * ratio
            y = float(left[1]) + (float(right[1]) - float(left[1])) * ratio
            angle = math.atan2(float(right[1]) - float(left[1]), float(right[0]) - float(left[0]))
            return (x, y), angle
        walked += length
    return (float(polyline[-1][0]), float(polyline[-1][1])), 0.0


def _chain_angle_rad(chain: dict[str, Any]) -> float:
    """估计山脉链整体方向。注意：峰体椭圆长轴使用该方向。"""

    polyline = [point for point in chain.get("polyline_uv", []) if _is_pair(point)]
    if len(polyline) < 2:
        return 0.0
    first = polyline[0]
    last = polyline[-1]
    return math.atan2(float(last[1]) - float(first[1]), float(last[0]) - float(first[0]))


def _chain_average_height(chain: dict[str, Any]) -> float:
    """读取山脉链平均峰高。注意：无峰时给低鞍部一个保守高度。"""

    heights = [float(peak.get("height_m", 0.0)) for peak in chain.get("peaks", []) if isinstance(peak, dict)]
    return sum(heights) / len(heights) if heights else 420.0


def _rotated_offsets(dx: np.ndarray, dy: np.ndarray, angle: float) -> tuple[np.ndarray, np.ndarray]:
    """把平面偏移旋入山体局部坐标。注意：返回长轴和短轴方向分量。"""

    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    return dx * cos_a + dy * sin_a, -dx * sin_a + dy * cos_a


def _value_noise(u_coord: np.ndarray, v_coord: np.ndarray, frequency: int, rng: np.random.Generator) -> np.ndarray:
    """二维 value noise。注意：逐项移植原型，用于 fBm 和域扭曲。"""

    grid = rng.uniform(-1.0, 1.0, size=(frequency + 2, frequency + 2)).astype(np.float32)
    # 原型使用 smoothstep 插值格点噪声，避免线性插值产生方格边。
    x = np.clip(u_coord * frequency, 0.0, frequency - 1e-4)
    y = np.clip(v_coord * frequency, 0.0, frequency - 1e-4)
    ix = np.floor(x).astype(np.int32)
    iy = np.floor(y).astype(np.int32)
    fx = _smoothstep(x - ix)
    fy = _smoothstep(y - iy)
    v00 = grid[iy, ix]
    v10 = grid[iy, ix + 1]
    v01 = grid[iy + 1, ix]
    v11 = grid[iy + 1, ix + 1]
    return ((v00 * (1.0 - fx) + v10 * fx) * (1.0 - fy) + (v01 * (1.0 - fx) + v11 * fx) * fy).astype(np.float32)


def _value_noise_wrapped(u_coord: np.ndarray, v_coord: np.ndarray, frequency: int, rng: np.random.Generator) -> np.ndarray:
    """二维周期 value noise。注意：旋转后的坐标允许越界。"""

    grid = rng.uniform(-1.0, 1.0, size=(frequency, frequency)).astype(np.float32)
    # wrapped 版本用于随机旋转后的 uv，越界后仍能连续采样。
    x = np.mod(u_coord * frequency, frequency)
    y = np.mod(v_coord * frequency, frequency)
    ix0 = np.floor(x).astype(np.int32)
    iy0 = np.floor(y).astype(np.int32)
    ix1 = (ix0 + 1) % frequency
    iy1 = (iy0 + 1) % frequency
    fx = _smoothstep(x - ix0)
    fy = _smoothstep(y - iy0)
    v00 = grid[iy0, ix0]
    v10 = grid[iy0, ix1]
    v01 = grid[iy1, ix0]
    v11 = grid[iy1, ix1]
    return ((v00 * (1.0 - fx) + v10 * fx) * (1.0 - fy) + (v01 * (1.0 - fx) + v11 * fx) * fy).astype(np.float32)


def _fbm(u_coord: np.ndarray, v_coord: np.ndarray, base_frequency: int, octaves: int, seed: int) -> np.ndarray:
    """原型 fBm。注意：振幅衰减、频率翻倍参数保持不变。"""

    rng = np.random.default_rng(seed)
    total = np.zeros_like(u_coord, dtype=np.float32)
    amplitude = 0.55
    amplitude_sum = 0.0
    frequency = base_frequency
    for _ in range(octaves):
        # fBm 的随机数生成器不重置，严格复刻原型每个倍频程的格点序列。
        total += amplitude * _value_noise(u_coord, v_coord, frequency, rng)
        amplitude_sum += amplitude
        amplitude *= 0.52
        frequency *= 2
    return total / max(amplitude_sum, 1e-6)


def _ridged_fbm_rotated(u_coord: np.ndarray, v_coord: np.ndarray, base_frequency: int, octaves: int, seed: int) -> np.ndarray:
    """原型随机旋转 ridged fBm。注意：每倍频程旋转角来自固定 seed。"""

    rng = np.random.default_rng(seed)
    total = np.zeros_like(u_coord, dtype=np.float32)
    amplitude = 0.64
    amplitude_sum = 0.0
    frequency = base_frequency
    center_u = u_coord - 0.5
    center_v = v_coord - 0.5
    for _ in range(octaves):
        # 每个倍频程独立旋转坐标，破坏固定方向波纹。
        angle = float(rng.uniform(0.0, math.tau))
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        rotated_u = center_u * cos_a - center_v * sin_a + 0.5 + rng.uniform(-0.25, 0.25)
        rotated_v = center_u * sin_a + center_v * cos_a + 0.5 + rng.uniform(-0.25, 0.25)
        noise = _value_noise_wrapped(rotated_u, rotated_v, frequency, rng)
        ridge = (1.0 - np.abs(noise)) ** 2.15
        total += amplitude * ridge
        amplitude_sum += amplitude
        amplitude *= 0.49
        frequency *= 2
    return np.clip(total / max(amplitude_sum, 1e-6), 0.0, 1.0)


def _warped_ridged_fbm(
    u_coord: np.ndarray,
    v_coord: np.ndarray,
    base_frequency: int,
    octaves: int,
    seed: int,
    warp_strength: float,
) -> np.ndarray:
    """原型域扭曲 ridged fBm。注意：warp_strength 数值由调用点照抄。"""

    warp_u = _fbm(u_coord + 0.37, v_coord - 0.19, 4, 3, seed + 101) * warp_strength
    warp_v = _fbm(u_coord - 0.23, v_coord + 0.41, 4, 3, seed + 102) * warp_strength
    # 先扭曲再 ridged，和原型一样把沟壑从规则网格里打散。
    return _ridged_fbm_rotated(u_coord + warp_u, v_coord + warp_v, base_frequency, octaves, seed)


def _slope_grid(heights: np.ndarray, east_grid: np.ndarray, north_grid: np.ndarray) -> np.ndarray:
    """计算原型配色所需坡度。注意：使用高度梯度长度，不从法线反推。"""

    step_e = float(east_grid[0, 1] - east_grid[0, 0])
    step_n = float(north_grid[1, 0] - north_grid[0, 0])
    grad_n, grad_e = np.gradient(heights, step_n, step_e)
    # 原型 slope 是水平梯度长度，不是法线 y 的反推值。
    return np.sqrt(grad_e * grad_e + grad_n * grad_n).astype(np.float32)


def _curvature_ao_grid(
    heights: np.ndarray,
    east_grid: np.ndarray,
    north_grid: np.ndarray,
    slope: np.ndarray,
) -> np.ndarray:
    """用原型曲率近似 AO。注意：压暗沟壑和凹陷区域。"""

    step_e = float(east_grid[0, 1] - east_grid[0, 0])
    step_n = float(north_grid[1, 0] - north_grid[0, 0])
    grad_n, grad_e = np.gradient(heights, step_n, step_e)
    grad_ee = np.gradient(grad_e, step_e, axis=1)
    grad_nn = np.gradient(grad_n, step_n, axis=0)
    laplacian = grad_ee + grad_nn
    scale = float(np.percentile(np.abs(laplacian), 94))
    scale = max(scale, 1e-5)
    # 用曲率的高分位归一化，避免少数尖峰让 AO 全局失效。
    concavity = _smoothstep(np.clip(laplacian / (scale * 0.95), 0.0, 1.0))
    steep_valleys = _smoothstep((slope - 0.20) / 0.80) * concavity
    return np.clip(1.0 - 0.30 * steep_valleys, 0.68, 1.0).astype(np.float32)


def _smoothstep(value: np.ndarray) -> np.ndarray:
    """返回 0 到 1 的平滑阶跃。注意：用于复刻 style A 颜色权重。"""

    clipped = np.clip(value, 0.0, 1.0)
    return clipped * clipped * (3.0 - 2.0 * clipped)


def _box_blur(field: np.ndarray, radius: int, *, passes: int = 2) -> np.ndarray:
    """用方盒低通滤波颜色权重。注意：只处理二维 float32 数组。"""

    result = field.astype(np.float32, copy=True)
    if radius <= 0:
        return result
    kernel_size = radius * 2 + 1
    for _ in range(passes):
        padded_x = np.pad(result, ((0, 0), (radius, radius)), mode="edge")
        cumsum_x = np.pad(np.cumsum(padded_x, axis=1, dtype=np.float64), ((0, 0), (1, 0)), mode="constant")
        result = ((cumsum_x[:, kernel_size:] - cumsum_x[:, :-kernel_size]) / kernel_size).astype(np.float32)
        padded_y = np.pad(result, ((radius, radius), (0, 0)), mode="edge")
        cumsum_y = np.pad(np.cumsum(padded_y, axis=0, dtype=np.float64), ((1, 0), (0, 0)), mode="constant")
        result = ((cumsum_y[kernel_size:, :] - cumsum_y[:-kernel_size, :]) / kernel_size).astype(np.float32)
    return result


def _mix(start: np.ndarray, end: np.ndarray, ratio: np.ndarray) -> np.ndarray:
    """按 ratio 对 RGB 颜色插值。注意：ratio 可为二维数组。"""

    return start + (end - start) * ratio[:, :, None]


def _lerp_color(start: tuple[float, float, float], end: tuple[float, float, float], ratio: np.ndarray) -> np.ndarray:
    """按原型函数签名插值 sRGB 颜色。注意：用于逐项移植 style A 配色链。"""

    a = np.array(start, dtype=np.float32)
    b = np.array(end, dtype=np.float32)
    return a + (b - a) * ratio[..., None]


def _srgb_to_linear(color: np.ndarray) -> np.ndarray:
    """把 sRGB 颜色转换到线性空间。注意：Quick3D 顶点色按线性值上传。"""

    return np.where(color <= 0.04045, color / 12.92, ((color + 0.055) / 1.055) ** 2.4)


def _uv_to_m(point: Any) -> tuple[float, float]:
    """把布局 u/v km 坐标转换成 ENU 米。注意：u 对应 east，v 对应 north。"""

    return float(point[0]) * 1000.0, float(point[1]) * 1000.0


def _is_pair(value: Any) -> bool:
    """判断对象是否为二维坐标。注意：列表和元组均接受。"""

    return isinstance(value, (list, tuple)) and len(value) >= 2


def _read_path(data: dict[str, Any], keys: tuple[str, ...], default: float) -> Any:
    """按嵌套键读取值。注意：缺失时返回默认值。"""

    current: Any = data
    for key in keys:
        if not isinstance(current, dict) or key not in current:
            return default
        current = current[key]
    return current


def _normalized_resolution(value: int) -> int:
    """规范化地形网格分辨率。注意：低配可用 384，默认 641。"""

    try:
        resolution = int(value)
    except (TypeError, ValueError):
        resolution = DEFAULT_TERRAIN_RESOLUTION
    return max(_MIN_RESOLUTION, min(_MAX_RESOLUTION, resolution))
