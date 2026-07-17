"""布局驱动的 3D 山地高度场生成器。注意：只服务 GUI 显示层，不参与仿真计算。"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
import json
import logging
import math
from pathlib import Path
import threading
import time
from typing import Any

import numpy as np

from src.ui.gui.situation3d.color_space import srgb_to_linear


DEFAULT_TERRAIN_RESOLUTION = 641
LOW_SPEC_TERRAIN_RESOLUTION = 384
# 共享缓存锁:预热线程与 GUI 线程可能并发请求同一高度场。
_FIELD_CACHE_LOCK = threading.Lock()
# 非阻塞入口的就绪表与在途集合:GUI 线程 peek 时绝不等待生成。
_PENDING_LOCK = threading.Lock()
_PENDING_KEYS: set[tuple[str, int, int]] = set()
_READY_FIELDS: dict[tuple[str, int, int], "TerrainField"] = {}
# 失败表按文件版本与分辨率记忆诊断，避免窗口轮询时无限重启同一失败任务。
_FAILED_FIELDS: dict[tuple[str, int, int], str] = {}
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
# 11. 高度是米制语义：峰中心=height_m、脊线=均高×saddle_height_factor，
#     脊/峰/鞍取最大值合成，仅超过 2800m 膝点后向下软压，禁止向上归一。
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
# 41. heights_m 始终保存布局米制语义，禁止被材质或修形流程覆盖。
# 42. display_heights_m 是纯显示副本，航线净空与障碍提取不得读取它。
# 43. 低于 120m 的低地显示位移严格为零，保护航迹走廊的平面关系。
# 44. 山区显示位移硬限制为正负 220m，避免视觉细节重画整体轮廓。
# 45. 中尺度滤波半径按米换算，不允许分辨率变化改变岩脊物理尺度。
# 46. 峰肩修形只作用于当前峰占主导的网格，避免误切相邻山脉链。
# 47. 径向岩脊采用两组互质近似频率，减少规则星芒和重复沟槽。
# 48. 峰体相位只从稳定 id 派生，禁止使用进程随机 hash 破坏截图复现。
# 49. 相邻峰重叠区域按影响权重平均，不能直接累加成异常高墙。
# 50. 显示法线和顶点色都从显示高度重算，保证几何与明暗一致。
# 51. 米制法线继续保留在 normals，便于既有数据回归不受显示层影响。
# 52. 岩石反照率贴图只乘细裂隙，不承担海拔颜色与风险语义。
# 53. 切线空间法线贴图由 terrain_geometry 的完整 TBN 顶点基驱动。
# 54. 自阴影与屏幕空间 AO 在 QML 层启用，不烘焙进任何语义数组。
# 55. 低配网格复用同一物理尺度算法，只减少采样点，不切换视觉规则。
# 56. 所有显示增强保持向量化，正式 768 网格仍由后台线程一次生成。


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
    """高度场网格数据。注意：米制语义高度与显示细节高度彼此隔离。"""

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
    display_heights_m: np.ndarray | None = None
    display_normals: np.ndarray | None = None


def load_terrain_layout(path: str | Path) -> dict[str, Any]:
    """读取地形布局 JSON。注意：调用方负责决定缺失文件是否回退旧地形。"""

    layout_path = Path(path)
    data = json.loads(layout_path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("terrain layout root must be an object")
    _validate_layout_numbers(data)
    return data


# 布局数值字段清单:前置校验只做类型与有限性检查,正负/范围约束仍由各消费点按原语义处理。
_DETAIL_INTEGER_FIELDS = ("grid_resolution", "low_spec_grid_resolution", "seed")
_MAP_NUMERIC_FIELDS = ("effective_extent_km", "render_extent_km")
_CHAIN_NUMERIC_FIELDS = ("ridge_width_km", "saddle_height_factor")
_PEAK_NUMERIC_FIELDS = (
    "height_m",
    "base_radius_km",
    "aspect_ratio",
    "risk_radius_km",
    "buffer_radius_km",
    "station",
    "lateral_offset_km",
)
_LINK_NUMERIC_FIELDS = ("ridge_width_km", "height_m")


def _validate_layout_numbers(layout: dict[str, Any]) -> None:
    """前置校验布局数值字段。注意：768² 场生成约 4s,坏数据(如字符串 \"NaN\")必须在
    加载阶段毫秒级拒绝;否则后台线程会把整张场算完才被末端 isfinite 检查拦下,
    且主线程 peek 已经把这条注定失败的生成线程踢了出去。"""

    def _ensure_value_finite(value: Any, label: str) -> None:
        """校验单个值可转 float 且有限。注意：错误标签必须指向布局中的完整路径。"""

        try:
            numeric = float(value)
        except (TypeError, ValueError) as error:
            raise ValueError(f"{label} 不是数值: {value!r}") from error
        if not math.isfinite(numeric):
            raise ValueError(f"{label} 非有限值: {numeric}")

    def _ensure_finite(container: dict[str, Any], field: str, label: str) -> None:
        """字段存在时校验可转 float 且有限。注意：缺省字段由消费点默认值兜底,不在此报错。"""

        if field in container:
            _ensure_value_finite(container[field], f"{label}.{field}")

    def _ensure_integer(container: dict[str, Any], field: str, label: str) -> None:
        """字段存在时校验可转整数。注意：保持消费点 int(...) 的既有兼容语义。"""

        if field not in container:
            return
        try:
            int(container[field])
        except (TypeError, ValueError, OverflowError) as error:
            raise ValueError(f"{label}.{field} 不是整数: {container[field]!r}") from error

    def _ensure_pair_finite(value: Any, label: str) -> None:
        """二维坐标被消费点识别时校验前两轴。注意：非坐标条目仍按既有语义跳过。"""

        if not _is_pair(value):
            return
        _ensure_value_finite(value[0], f"{label}[0]")
        _ensure_value_finite(value[1], f"{label}[1]")

    def _ensure_coordinate_list(container: dict[str, Any], field: str, label: str) -> None:
        """校验坐标序列内会被消费的二维点。注意：字段类型异常仍由消费点按原语义处理。"""

        points = container.get(field)
        for point_index, point in enumerate(points if isinstance(points, (list, tuple)) else []):
            _ensure_pair_finite(point, f"{label}.{field}[{point_index}]")

    detail = layout.get("detail")
    if isinstance(detail, dict):
        for field in _DETAIL_INTEGER_FIELDS:
            _ensure_integer(detail, field, "detail")
    map_config = layout.get("map")
    if isinstance(map_config, dict):
        for field in _MAP_NUMERIC_FIELDS:
            _ensure_finite(map_config, field, "map")
    flight = layout.get("flight")
    if isinstance(flight, dict):
        for field in ("original_route_uv", "planned_route_uv"):
            _ensure_coordinate_list(flight, field, "flight")

    chains = layout.get("mountain_chains")
    for chain_index, chain in enumerate(chains if isinstance(chains, list) else []):
        # 非字典条目由各消费点按既有语义跳过,这里保持同样的容忍度。
        if not isinstance(chain, dict):
            continue
        chain_label = f"mountain_chains[{chain_index}]"
        for field in _CHAIN_NUMERIC_FIELDS:
            _ensure_finite(chain, field, chain_label)
        _ensure_coordinate_list(chain, "polyline_uv", chain_label)
        peaks = chain.get("peaks")
        for peak_index, peak in enumerate(peaks if isinstance(peaks, list) else []):
            if not isinstance(peak, dict):
                continue
            for field in _PEAK_NUMERIC_FIELDS:
                _ensure_finite(peak, field, f"{chain_label}.peaks[{peak_index}]")
    links = layout.get("saddle_links")
    for link_index, link in enumerate(links if isinstance(links, list) else []):
        if not isinstance(link, dict):
            continue
        link_label = f"saddle_links[{link_index}]"
        for field in _LINK_NUMERIC_FIELDS:
            _ensure_finite(link, field, link_label)
        for field in ("from_uv", "to_uv"):
            if field in link:
                _ensure_pair_finite(link[field], f"{link_label}.{field}")


def generate_terrain_field_from_file(path: str | Path, *, resolution: int = DEFAULT_TERRAIN_RESOLUTION) -> TerrainField:
    """从布局文件生成高度场。注意：耗时操作应只在布局或分辨率变化时触发。"""

    return generate_terrain_field(load_terrain_layout(path), resolution=resolution)


def get_terrain_field(path: str | Path, *, resolution: int = DEFAULT_TERRAIN_RESOLUTION) -> TerrainField:
    """获取共享缓存的高度场(阻塞)。注意：scene_data 与 TerrainGeometry 必须走同一入口，
    避免 768² 场在 GUI 主线程重复生成(实测单次约 4s,重复即翻倍卡顿)。"""

    resolved = Path(path).resolve()
    mtime_ns = resolved.stat().st_mtime_ns
    key = (str(resolved), mtime_ns, _normalized_resolution(resolution))
    # 锁保证并发预热线程与 GUI 线程不会同时算两份同参数场。
    with _FIELD_CACHE_LOCK:
        field = _cached_field(*key)
    # 阻塞入口同样登记就绪表,预热完成后 peek 立即命中。
    with _PENDING_LOCK:
        _READY_FIELDS[key] = field
        _FAILED_FIELDS.pop(key, None)
    return field


def peek_terrain_field(path: str | Path, *, resolution: int = DEFAULT_TERRAIN_RESOLUTION) -> TerrainField | None:
    """非阻塞获取高度场:已就绪立即返回,否则触发后台生成并返回 None。
    注意：GUI 主线程一律走本入口,未就绪时先用占位地形,就绪后由 payload 驱动替换。"""

    resolved = Path(path).resolve()
    mtime_ns = resolved.stat().st_mtime_ns
    key = (str(resolved), mtime_ns, _normalized_resolution(resolution))
    with _PENDING_LOCK:
        if key in _READY_FIELDS:
            return _READY_FIELDS[key]
        if key in _FAILED_FIELDS:
            return None
        if key not in _PENDING_KEYS:
            _PENDING_KEYS.add(key)
            threading.Thread(target=_background_generate, args=(key,), name="terrain-field-async", daemon=True).start()
    return None


def _background_generate(key: tuple[str, int, int]) -> None:
    """后台线程体:生成高度场并登记就绪表。注意：失败仅记诊断,不重试不抛出。"""

    try:
        field = get_terrain_field(key[0], resolution=key[2])
        with _PENDING_LOCK:
            _READY_FIELDS[key] = field
    except Exception as error:  # noqa: BLE001
        with _PENDING_LOCK:
            _FAILED_FIELDS[key] = f"{type(error).__name__}: {error}"
        logging.getLogger(__name__).warning("后台生成高度场失败 %s: %s", key[0], error)
    finally:
        with _PENDING_LOCK:
            _PENDING_KEYS.discard(key)


def terrain_field_error(path: str | Path, *, resolution: int = DEFAULT_TERRAIN_RESOLUTION) -> str | None:
    """返回当前文件版本的后台生成错误。注意：文件或分辨率变化后会形成新键并允许重试。"""

    resolved = Path(path).resolve()
    key = (str(resolved), resolved.stat().st_mtime_ns, _normalized_resolution(resolution))
    with _PENDING_LOCK:
        return _FAILED_FIELDS.get(key)


@lru_cache(maxsize=4)
def _cached_field(resolved_path: str, mtime_ns: int, resolution: int) -> TerrainField:
    """按(路径,修改时间,分辨率)缓存高度场。注意：文件被改动后 mtime 变化自动失效。"""

    return generate_terrain_field_from_file(resolved_path, resolution=resolution)


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
    # 细节噪声按主体山势 mask 衰减:高度是米制语义(峰高=height_m,供避障与净空判断),
    # 空布局/平原必须接近平坦,不允许全域噪声凭空造山。
    detail_mask = _smoothstep((base - 150.0) / 900.0)
    # 峰顶核心区二次衰减:细节长在山坡上,峰顶保持声明高度(偏差控制在数十米内)。
    summit_damp = 1.0 - 0.85 * _smoothstep((base - 1800.0) / 600.0)
    heights = np.maximum(0.0, base + detail * (0.05 + 0.95 * detail_mask) * summit_damp).astype(np.float32)
    # 只做超上限的向下软压(膝点以下保持精确米制),禁止任何向上归一。
    knee = 2800.0
    cap = 3040.0
    over = heights > knee
    heights[over] = knee + (cap - knee) * (1.0 - np.exp(-(heights[over] - knee) / (cap - knee)))
    heights *= _edge_fade(east_grid, north_grid, extent)
    if not np.isfinite(heights).all():
        # 坏配置(如零范围 extent)可能算出 NaN 高度,必须在出口拦截由上层回退。
        raise ValueError("高度场包含非有限值,布局配置非法")
    # 米制法线跟随语义高度，供数据回归和潜在净空诊断；显示层另建受控岩脊高度，
    # 两张数组互不共享内存，避免为了视觉效果改写航线/避障采样依据。
    normals = _normal_grid(heights, float(east[1] - east[0]), float(north[1] - north[0]))
    display_heights = _display_relief(heights, detail, east_grid, north_grid, layout)
    display_normals = _normal_grid(display_heights, float(east[1] - east[0]), float(north[1] - north[0]))
    colors = _color_grid(display_heights, display_normals, east_grid, north_grid, extent)
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
        display_heights_m=display_heights,
        display_normals=display_normals,
    )


def terrain_extent_from_layout(layout: dict[str, Any]) -> dict[str, float]:
    """计算布局渲染范围。注意：输入 u/v 单位为 km，输出为 ENU 米。"""

    render_extent_km = _finite_positive(float(_read_path(layout, ("map", "render_extent_km"), 58.0)), "render_extent_km")
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
            radius_m = _finite_positive(float(peak.get("risk_radius_km", peak.get("base_radius_km", 1.0))), "risk_radius_km") * 1000.0
            zones.append(
                TerrainRiskZone(
                    zone_id=str(peak.get("id") or f"{chain.get('id', 'risk')}_{index}"),
                    label=str(peak.get("label") or chain.get("name") or "风险峰"),
                    east_m=_finite(center_u * 1000.0, "risk center east"),
                    north_m=_finite(center_v * 1000.0, "risk center north"),
                    radius_m=radius_m,
                    height_m=_finite_positive(float(peak.get("height_m", 0.0)), "height_m", allow_zero=True),
                    buffer_radius_m=_finite_positive(float(peak.get("buffer_radius_km", peak.get("base_radius_km", 1.0) * 1.55)), "buffer_radius_km") * 1000.0,
                )
            )
    return zones


def _base_relief(east_grid: np.ndarray, north_grid: np.ndarray, layout: dict[str, Any]) -> np.ndarray:
    """按山脉链骨架生成主体高度。注意：连续山脊优先于单个孤立峰。"""

    height = np.zeros_like(east_grid, dtype=np.float32)
    chains = layout.get("mountain_chains")
    if not isinstance(chains, list):
        return height
    # 脊线/峰体/鞍部一律取最大值合成:峰中心严格等于声明 height_m(米制语义,
    # 供避障与净空判断);相加合成会把相邻体量叠高,历史上靠削顶归一掩盖,已废弃。
    for chain in chains:
        if not isinstance(chain, dict):
            continue
        height = np.maximum(height, _chain_ridge_height(east_grid, north_grid, chain))
        for peak in chain.get("peaks", []):
            if isinstance(peak, dict):
                height = np.maximum(height, _peak_height(east_grid, north_grid, chain, peak))
    links = layout.get("saddle_links")
    if isinstance(links, list):
        for link in links:
            if isinstance(link, dict):
                height = np.maximum(height, _link_height(east_grid, north_grid, link))
    return height


def _chain_ridge_height(east_grid: np.ndarray, north_grid: np.ndarray, chain: dict[str, Any]) -> np.ndarray:
    """生成山脉链连续脊线。注意：沿 polyline 分段取最大值以保留走向。"""

    polyline = [_uv_to_m(point) for point in chain.get("polyline_uv", []) if _is_pair(point)]
    if len(polyline) < 2:
        return np.zeros_like(east_grid, dtype=np.float32)
    ridge_width = float(chain.get("ridge_width_km", 0.65)) * 860.0
    # 脊线高度严格等于 峰均高×saddle_height_factor(布局语义,如 hazard 链 2410×0.2≈472m 鞍部),
    # 不允许再乘视觉系数抬高——山势的陡峭感由峰体剖面和细节层负责。
    ridge_height = _chain_average_height(chain) * float(chain.get("saddle_height_factor", 0.35))
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
    short_radius = radius * 0.68
    shoulder_long = np.abs(local_e / max(1.0, long_radius * 1.38))
    shoulder_short = np.abs(local_n / max(1.0, short_radius * 1.32))
    local_long = np.abs(local_e / max(1.0, long_radius))
    local_short = np.abs(local_n / max(1.0, short_radius))
    shoulder = np.exp(-0.82 * (shoulder_long ** 1.42 + shoulder_short ** 1.62))
    body = np.exp(-1.42 * (local_long ** 1.58 + local_short ** 1.86))
    summit = np.exp(-3.65 * (local_long ** 2.1 + local_short ** 2.35))
    profile = 0.36 * shoulder + 0.63 * body + 0.34 * summit
    # 峰核按中心值归一:峰中心严格输出声明 height_m,不允许权重和(1.33)抬高峰顶。
    return float(peak.get("height_m", 0.0)) * profile / 1.33


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


def _display_relief(
    heights: np.ndarray,
    detail: np.ndarray,
    east_grid: np.ndarray,
    north_grid: np.ndarray,
    layout: dict[str, Any],
) -> np.ndarray:
    """生成受控的显示层岩脊沟壑。注意：不得回写米制语义高度。"""

    step_e = abs(float(east_grid[0, 1] - east_grid[0, 0]))
    step_n = abs(float(north_grid[1, 0] - north_grid[0, 0]))
    sample_step = max(1.0, (step_e + step_n) * 0.5)
    # 半径按物理米制换算，保证 384/641/768 网格看到同尺度的坡面结构。
    mid_radius = max(1, int(round(360.0 / sample_step)))
    broad_radius = max(mid_radius + 1, int(round(840.0 / sample_step)))
    mid_reference = _box_blur(heights, mid_radius, passes=2)
    broad_reference = _box_blur(heights, broad_radius, passes=1)
    detail_reference = _box_blur(detail, mid_radius, passes=1)
    mid_landform = heights - mid_reference
    broad_landform = heights - broad_reference
    fractured_detail = detail - detail_reference

    slope = _slope_grid(heights, east_grid, north_grid)
    mountain_mask = _smoothstep((heights - 120.0) / 620.0)
    # 平缓山脊仍保留一半强化量，陡坡则完整表现岩壁；低于 120m 的航迹低地严格不动。
    wall_weight = 0.52 + 0.48 * _smoothstep((slope - 0.06) / 0.72)
    summit_guard = 1.0 - 0.22 * _smoothstep((heights - 2240.0) / 560.0)
    displacement = (
        0.48 * mid_landform
        + 0.16 * broad_landform
        + 0.28 * fractured_detail
    ) * mountain_mask * wall_weight * summit_guard
    displacement += _peak_rock_relief(heights, east_grid, north_grid, layout) * mountain_mask
    # 沟槽略深于凸脊，使峡谷在斜俯视角下形成清晰暗线，同时限制总位移保护原轮廓。
    displacement = np.where(displacement < 0.0, displacement * 1.10, displacement)
    displacement = np.clip(displacement, -220.0, 220.0).astype(np.float32)
    display = np.maximum(0.0, heights + displacement).astype(np.float32)
    if not np.isfinite(display).all():
        raise ValueError("显示层高度包含非有限值")
    return display


def _peak_rock_relief(
    heights: np.ndarray,
    east_grid: np.ndarray,
    north_grid: np.ndarray,
    layout: dict[str, Any],
) -> np.ndarray:
    """沿既有峰体生成径向岩脊和窄沟。注意：只提供显示位移，不改变峰心平面位置。"""

    relief_sum = np.zeros_like(east_grid, dtype=np.float32)
    weight_sum = np.zeros_like(east_grid, dtype=np.float32)
    chains = layout.get("mountain_chains")
    if not isinstance(chains, list):
        return relief_sum
    for chain in chains:
        if not isinstance(chain, dict):
            continue
        chain_angle = _chain_angle_rad(chain)
        for peak in chain.get("peaks", []):
            if not isinstance(peak, dict):
                continue
            height_m = float(peak.get("height_m", 0.0))
            if height_m < 420.0:
                continue
            center_u, center_v = _peak_center_uv(chain, peak)
            radius = max(1.0, float(peak.get("base_radius_km", 1.0)) * 1000.0)
            aspect = max(0.35, float(peak.get("aspect_ratio", 1.3)))
            local_e, local_n = _rotated_offsets(
                east_grid - center_u * 1000.0,
                north_grid - center_v * 1000.0,
                chain_angle,
            )
            radial_x = local_e / (radius * aspect * 0.98)
            radial_y = local_n / (radius * 0.78)
            radial = np.sqrt(radial_x * radial_x + radial_y * radial_y)
            angle = np.arctan2(radial_y, radial_x)

            # 用尖锥剖面仅修正主导峰的圆钝峰肩：中心/峰高/山脚均不平移，过渡限制在既有峰体内。
            shoulder_long = np.abs(local_e / (radius * aspect * 0.84 * 1.38))
            shoulder_short = np.abs(local_n / (radius * 0.68 * 1.32))
            local_long = np.abs(local_e / (radius * aspect * 0.84))
            local_short = np.abs(local_n / (radius * 0.68))
            current_profile = (
                0.36 * np.exp(-0.82 * (shoulder_long ** 1.42 + shoulder_short ** 1.62))
                + 0.63 * np.exp(-1.42 * (local_long ** 1.58 + local_short ** 1.86))
                + 0.34 * np.exp(-3.65 * (local_long ** 2.1 + local_short ** 2.35))
            ) / 1.33
            target_profile = np.maximum(0.0, 1.0 - (radial / 1.35) ** 0.82)
            current_surface = height_m * current_profile
            dominance = _smoothstep((current_surface / np.maximum(heights, 1.0) - 0.58) / 0.32)
            profile_fade = 1.0 - _smoothstep((radial - 1.10) / 0.48)
            profile_correction = 0.68 * height_m * (target_profile - current_profile) * dominance * profile_fade

            peak_id = str(peak.get("id", "peak"))
            stable_code = sum((index + 1) * ord(char) for index, char in enumerate(peak_id))
            phase = (stable_code % 6283) / 1000.0
            rib_count = 7 + stable_code % 4
            secondary_count = 13 + stable_code % 6
            angular_warp = 0.38 * np.sin(angle * 3.0 + phase) + 0.18 * np.sin(angle * 5.0 - phase * 0.7)
            primary = np.cos(angle * rib_count + angular_warp + radial * 2.1 + phase)
            secondary = np.cos(angle * secondary_count - angular_warp * 0.7 - radial * 3.4 - phase * 0.4)
            # 有符号幂把凸脊收窄；负半区稍加权形成更清晰的下切沟槽。
            primary_ribs = np.sign(primary) * np.abs(primary) ** 1.75
            secondary_ribs = np.sign(secondary) * np.abs(secondary) ** 2.15
            rib_pattern = 0.72 * primary_ribs + 0.28 * secondary_ribs
            rib_pattern = np.where(rib_pattern < 0.0, rib_pattern * 1.18, rib_pattern)

            inner_fade = _smoothstep((radial - 0.015) / 0.11)
            outer_fade = 1.0 - _smoothstep((radial - 1.08) / 0.72)
            peak_weight = np.exp(-0.62 * radial ** 2.4) * outer_fade
            amplitude = min(185.0, max(46.0, height_m * 0.074))
            # 轻微抬峰、削肩把圆顶收成岩峰；量级小于沟脊总位移上限。
            summit_uplift = 0.48 * amplitude * (1.0 - _smoothstep(radial / 0.30))
            shoulder_cut = -0.28 * amplitude * _smoothstep((radial - 0.24) / 0.28) * (
                1.0 - _smoothstep((radial - 0.92) / 0.54)
            )
            candidate = amplitude * rib_pattern * inner_fade + summit_uplift + shoulder_cut + profile_correction
            relief_sum += candidate.astype(np.float32) * peak_weight.astype(np.float32)
            weight_sum += peak_weight.astype(np.float32)
    # 相邻峰重叠处取加权平均，避免简单相加制造超高接缝。
    return (relief_sum / np.maximum(weight_sum, 1.0)).astype(np.float32)


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
    """预计算低饱和数字孪生地形顶点色。注意：输出线性 RGB 供正式材质接收。"""

    # 3040m 是高度场软上限；颜色分层使用固定米制基准，避免同一布局因采样分辨率变色。
    h = np.clip(heights / 3040.0, 0.0, 1.0)
    # 颜色权重必须低通，不能直接追随细碎高度噪声，否则远景会重新出现斑驳卡通块。
    height_low = _box_blur(h, 7, passes=3)
    # 峰顶色带只做轻度低通，避免重度模糊把狭窄山脊的真实海拔完全抹掉。
    height_band = _box_blur(h, 2, passes=1)
    slope = _slope_grid(heights, east_grid, north_grid)
    # 坡度低通后只决定岩石分布；实际法线仍保留高频沟脊并交给 QML 光照。
    slope_low = _box_blur(slope, 5, passes=2)
    curvature_ao = _curvature_ao_grid(heights, east_grid, north_grid, slope)
    # 中尺度局部起伏直接来自显示高度：凸脊提亮、凹沟压暗，形成参考图中的岩壁切面。
    local_landform = heights - _box_blur(heights, 3, passes=2)
    mountain_samples = np.abs(local_landform[heights > 240.0])
    landform_scale = float(np.percentile(mountain_samples, 86)) if mountain_samples.size else 1.0
    landform_scale = max(landform_scale, 1.0)
    ridge_light = _smoothstep((local_landform - landform_scale * 0.08) / (landform_scale * 1.12))
    gully_shadow = _smoothstep((-local_landform - landform_scale * 0.06) / (landform_scale * 0.94))
    # AO 反相近似凸脊，让岩石灰同时跟随海拔、山脊和陡坡，而不是只画水平色带。
    ridge_source = _box_blur(np.clip(1.0 - curvature_ao, 0.0, 1.0), 6, passes=2)
    elevation_rock = _smoothstep((height_band - 0.18) / 0.38)
    ridge_rock = _smoothstep((ridge_source - 0.02) / 0.14) * _smoothstep((height_band - 0.10) / 0.32)
    wall_rock = _smoothstep((slope_low - 0.18) / 0.42) * _smoothstep((height_band - 0.08) / 0.32)
    rock_weight = np.clip(0.52 * elevation_rock + 0.34 * ridge_rock + 0.38 * wall_rock, 0.0, 1.0)
    # 先模糊再 smoothstep，让灰绿到岩灰的过渡保持大片连续坡面。
    rock_weight = _box_blur(rock_weight, 6, passes=1)
    rock_weight = _smoothstep((rock_weight - 0.04) / 0.64)
    summit_weight = _box_blur(_smoothstep((height_band - 0.50) / 0.18), 4, passes=1)

    # 所有调色值均为低饱和 sRGB：低处深绿灰，山腰灰绿，岩面中性偏冷。
    valley = _lerp_color((0.105, 0.145, 0.150), (0.155, 0.195, 0.180), _smoothstep(height_low / 0.20))
    shoulder = _lerp_color((0.155, 0.195, 0.180), (0.245, 0.265, 0.240), _smoothstep((height_low - 0.16) / 0.34))
    vegetation = valley * (1.0 - _smoothstep((height_low - 0.12) / 0.32)[..., None]) + shoulder * _smoothstep((height_low - 0.12) / 0.32)[..., None]
    rock_dark = _lerp_color((0.195, 0.215, 0.220), (0.325, 0.330, 0.315), _smoothstep((height_low - 0.24) / 0.44))
    # 只在最高脊线给少量暖灰，不使用雪白色，避免山顶变成摄影雪山。
    rock_light = _lerp_color((0.325, 0.330, 0.315), (0.515, 0.505, 0.475), summit_weight)
    rock = rock_dark * (1.0 - summit_weight[..., None]) + rock_light * summit_weight[..., None]
    # 岩石权重只替换坡面反照率，不改高度和法线，因此不会改变山体轮廓或避障语义。
    mixed = vegetation * (1.0 - rock_weight[..., None]) + rock * rock_weight[..., None]
    # 空气透视以地图中心为距离原点，避免配置平移后远景蓝化方向偏移。
    center_e = (float(extent["min_east_m"]) + float(extent["max_east_m"])) / 2.0
    center_n = (float(extent["min_north_m"]) + float(extent["max_north_m"])) / 2.0
    x_grid = east_grid - center_e
    z_grid = north_grid - center_n
    half_extent = min(float(extent["max_east_m"] - extent["min_east_m"]), float(extent["max_north_m"] - extent["min_north_m"])) / 2.0
    # 25km 基准让同一配色在不同地图范围下保持相近的远近层次比例。
    distance_scale = max(half_extent / 25000.0, 1e-6)
    far_distance = np.sqrt(x_grid * x_grid + z_grid * z_grid)
    # 远景轻微蓝化，但不再推向明亮青灰，防止整块地表被雾洗成浅绿色。
    far_mix = _smoothstep((far_distance - 8800.0 * distance_scale) / (12800.0 * distance_scale))[..., None]
    far_blue = np.array([0.125, 0.155, 0.185], dtype=np.float32)
    mixed = mixed * (1.0 - far_mix * 0.42) + far_blue * (far_mix * 0.42)
    edge_distance = half_extent - np.maximum(np.abs(x_grid), np.abs(z_grid))
    # 边缘仍融入天际雾，但使用深蓝灰而不是高亮蓝绿。
    edge_visibility = _smoothstep(edge_distance / (18000.0 * distance_scale))[..., None]
    mixed = mixed * edge_visibility + np.array([0.145, 0.175, 0.205], dtype=np.float32) * (1.0 - edge_visibility)
    light_dir = np.array([-0.58, 0.66, 0.48], dtype=np.float32)
    light_dir /= np.linalg.norm(light_dir)
    # 同一坡向关系烘焙柔和暖光/冷影，QML 法线光再补充近景细节。
    lambert = np.clip(np.sum(normals * light_dir, axis=-1), 0.0, 1.0)
    rock_channel = rock_weight[..., None]
    warm_light = np.array([1.10, 1.02, 0.90], dtype=np.float32)
    cool_shadow = np.array([0.72, 0.84, 1.08], dtype=np.float32)
    temperature = cool_shadow * (1.0 - lambert[..., None]) + warm_light * lambert[..., None]
    # 受光岩面比深绿灰低地拥有更大明暗跨度，突出山脊、陡坡和峡谷体积。
    veg_relief = 0.48 + 1.08 * (lambert[..., None] ** 0.82)
    # 岩面扩大光照动态范围，但后续仍用冷灰底线保护背光坡细节。
    rock_relief = 0.34 + 1.68 * (lambert[..., None] ** 0.76)
    relief = veg_relief * (1.0 - rock_channel) + rock_relief * rock_channel
    ridge_shadow = np.clip(1.02 - slope[..., None] * 0.12, 0.72, 1.04)
    ao = 0.58 + 0.42 * curvature_ao[..., None]
    color = mixed * temperature * relief * ridge_shadow * ao
    structure_weight = _smoothstep((height_band - 0.07) / 0.28) * (0.32 + 0.68 * rock_weight)
    geomorphic_contrast = 1.0 + structure_weight * (0.23 * ridge_light - 0.38 * gully_shadow)
    color *= geomorphic_contrast[..., None]
    color += (
        np.array([0.075, 0.064, 0.048], dtype=np.float32)
        * (ridge_light * structure_weight)[..., None]
    )
    # 160m 等高暗线只落在山坡，保持数字孪生感并帮助读出陡坡，不覆盖低地航迹区。
    contour_period_m = 160.0
    contour_wave = 1.0 - np.abs(np.sin(np.pi * heights / contour_period_m))
    contour_line = contour_wave ** 4.5
    contour_weight = contour_line * _smoothstep((heights - 240.0) / 720.0) * (0.30 + 0.70 * _smoothstep(slope / 0.75))
    color *= (1.0 - 0.15 * contour_weight)[..., None]
    # 冷灰蓝环境底线只补最暗面，不把整个地形提亮成雾灰色。
    shadow_amount = (1.0 - lambert[..., None]) * (0.22 + 0.28 * rock_channel)
    color += np.array([0.060, 0.082, 0.115], dtype=np.float32) * shadow_amount
    highlight = _smoothstep((lambert - 0.60) / 0.34)[..., None] * _smoothstep((height_band - 0.46) / 0.22)[..., None]
    color += np.array([0.072, 0.058, 0.034], dtype=np.float32) * highlight
    color *= 1.0 - 0.10 * far_mix
    color = np.maximum(color, np.array([0.065, 0.085, 0.110], dtype=np.float32))
    color = np.minimum(color, np.array([0.68, 0.66, 0.62], dtype=np.float32))
    # 上传前转线性 RGB；QML 基色保持白色，避免再次染色破坏冷暖坡向关系。
    return srgb_to_linear(np.clip(color, 0.0, 1.0)).astype(np.float32)


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


def _finite(value: float, label: str) -> float:
    """校验数值有限性。注意：NaN/Inf 一律视为坏配置抛 ValueError,由上层回退。"""

    if not math.isfinite(value):
        raise ValueError(f"{label} 非有限值: {value}")
    return value


def _finite_positive(value: float, label: str, *, allow_zero: bool = False) -> float:
    """校验数值为有限正数。注意：风险区半径/高度等语义量不允许非正值。"""

    _finite(value, label)
    if value < 0.0 or (value == 0.0 and not allow_zero):
        raise ValueError(f"{label} 必须为正数: {value}")
    return value


def _normalized_resolution(value: int) -> int:
    """规范化地形网格分辨率。注意：低配可用 384，默认 641。"""

    try:
        resolution = int(value)
    except (TypeError, ValueError):
        resolution = DEFAULT_TERRAIN_RESOLUTION
    return max(_MIN_RESOLUTION, min(_MAX_RESOLUTION, resolution))
