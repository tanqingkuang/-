#!/usr/bin/env python3
"""从山地布局高度场提取二维避障多边形障碍。"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.data.geo import GeoOrigin, enu_to_geodetic
from src.data.geo_config import format_geodetic_degree
from src.ui.gui.situation3d.terrain_field import TerrainField, get_terrain_field


_MOORE_NEIGHBORS = [(-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1)]


def label_components(mask: np.ndarray) -> tuple[np.ndarray, int]:
    """以 8 邻域标记布尔掩膜中的连通域，避免递归填充深度限制。"""

    labels = np.zeros(mask.shape, dtype=np.int32)
    component_count = 0
    rows, cols = mask.shape
    for seed_row, seed_col in zip(*np.nonzero(mask)):
        if labels[seed_row, seed_col]:
            continue
        component_count += 1
        stack = [(int(seed_row), int(seed_col))]
        labels[seed_row, seed_col] = component_count
        while stack:
            row, col = stack.pop()
            for delta_row, delta_col in _MOORE_NEIGHBORS:
                next_row, next_col = row + delta_row, col + delta_col
                if (
                    0 <= next_row < rows
                    and 0 <= next_col < cols
                    and mask[next_row, next_col]
                    and not labels[next_row, next_col]
                ):
                    labels[next_row, next_col] = component_count
                    stack.append((next_row, next_col))
    return labels, component_count


def trace_boundary(component: np.ndarray) -> list[tuple[int, int]]:
    """以 Moore 邻域追踪单个连通域边界，返回不重复首点的有序格点环。"""

    row_indices, col_indices = np.nonzero(component)
    if len(row_indices) == 0:
        return []
    top_row = int(row_indices.min())
    start = (top_row, int(col_indices[row_indices == top_row].min()))
    boundary = [start]
    rows, cols = component.shape

    def is_component_cell(cell: tuple[int, int]) -> bool:
        """判断候选格点是否仍在当前连通域中。"""

        return 0 <= cell[0] < rows and 0 <= cell[1] < cols and bool(component[cell])

    current = start
    backtrack_direction = 6
    # 上限与连通域大小成正比，坏输入不会导致离线工具无限循环。
    for _ in range(int(component.sum()) * 8 + 8):
        for offset in range(8):
            direction = (backtrack_direction + 1 + offset) % 8
            candidate = (
                current[0] + _MOORE_NEIGHBORS[direction][0],
                current[1] + _MOORE_NEIGHBORS[direction][1],
            )
            if not is_component_cell(candidate):
                continue
            if candidate == start and len(boundary) > 2:
                return boundary
            boundary.append(candidate)
            backtrack_direction = (direction + 4) % 8
            current = candidate
            break
        else:
            return boundary
    return boundary


def douglas_peucker(points: list[tuple[float, float]], tolerance: float) -> list[tuple[float, float]]:
    """用迭代 Douglas-Peucker 算法简化折线，输入闭合环时首尾不重复。"""

    if len(points) < 4:
        return list(points)
    keep = np.zeros(len(points), dtype=bool)
    keep[0] = keep[-1] = True
    pending_segments = [(0, len(points) - 1)]
    point_array = np.asarray(points, dtype=float)
    while pending_segments:
        start_index, end_index = pending_segments.pop()
        if end_index - start_index < 2:
            continue
        segment = point_array[end_index] - point_array[start_index]
        segment_length = float(np.hypot(segment[0], segment[1]))
        if segment_length < 1e-9:
            distances = np.hypot(*(point_array[start_index + 1:end_index] - point_array[start_index]).T)
        else:
            offsets = point_array[start_index + 1:end_index] - point_array[start_index]
            # 显式二维叉积，避免 NumPy 2.0 对二维 np.cross 的弃用警告。
            cross_products = segment[0] * offsets[:, 1] - segment[1] * offsets[:, 0]
            distances = np.abs(cross_products) / segment_length
        farthest_offset = int(np.argmax(distances))
        if distances[farthest_offset] > tolerance:
            split_index = start_index + 1 + farthest_offset
            keep[split_index] = True
            pending_segments.append((start_index, split_index))
            pending_segments.append((split_index, end_index))
    return [tuple(point) for point, retained in zip(points, keep) if retained]


def extract_obstacles_from_field(
    field: TerrainField,
    layout: dict[str, Any],
    *,
    threshold_m: float | None = None,
    min_area_km2: float = 0.15,
    simplify_tolerance_m: float = 60.0,
    corridor_u_min_km: float = -1.0,
    corridor_u_max_km: float = 28.0,
    corridor_v_half_width_km: float = 3.5,
) -> list[dict[str, Any]]:
    """从已有高度场提取满足净空阈值的避障多边形，不读取或写入文件。"""

    flight = layout.get("flight")
    if not isinstance(flight, dict):
        raise ValueError("布局缺少 flight 配置")
    obstacle_threshold = (
        float(threshold_m)
        if threshold_m is not None
        else float(flight["cruise_altitude_m"]) - float(flight["clearance_m"])
    )
    if field.resolution < 2:
        raise ValueError("高度场分辨率必须至少为 2")
    min_east = field.center_east_m - field.width_m / 2.0
    min_north = field.center_north_m - field.depth_m / 2.0
    step_east = field.width_m / (field.resolution - 1)
    step_north = field.depth_m / (field.resolution - 1)
    cell_area_km2 = step_east * step_north / 1_000_000.0
    origin_raw = layout.get("geo_reference")
    if not isinstance(origin_raw, dict):
        raise ValueError("布局缺少 geo_reference 配置")
    origin = GeoOrigin(float(origin_raw["latitude_deg"]), float(origin_raw["longitude_deg"]))
    corridor_min_east = corridor_u_min_km * 1000.0
    corridor_max_east = corridor_u_max_km * 1000.0
    corridor_min_north = -corridor_v_half_width_km * 1000.0
    corridor_max_north = corridor_v_half_width_km * 1000.0

    labels, component_count = label_components(field.heights_m >= obstacle_threshold)
    extracted: list[dict[str, Any]] = []
    for component_id in range(1, component_count + 1):
        component = labels == component_id
        row_indices, col_indices = np.nonzero(component)
        if len(row_indices) * cell_area_km2 < min_area_km2:
            continue
        component_min_east = min_east + int(col_indices.min()) * step_east
        component_max_east = min_east + int(col_indices.max()) * step_east
        component_min_north = min_north + int(row_indices.min()) * step_north
        component_max_north = min_north + int(row_indices.max()) * step_north
        # 仅保留与任务走廊 bbox 相交的山体，走廊外远山无需进入 A*。
        if (
            component_max_east < corridor_min_east
            or component_min_east > corridor_max_east
            or component_max_north < corridor_min_north
            or component_min_north > corridor_max_north
        ):
            continue
        boundary_cells = trace_boundary(component)
        boundary_enu = [
            (min_east + col * step_east, min_north + row * step_north)
            for row, col in boundary_cells
        ]
        simplified = douglas_peucker(boundary_enu, simplify_tolerance_m)
        if len(simplified) < 3:
            continue
        points = []
        for east_m, north_m in simplified:
            latitude_deg, longitude_deg = enu_to_geodetic(east_m, north_m, origin)
            points.append(
                {
                    "latitude_deg": format_geodetic_degree(latitude_deg),
                    "longitude_deg": format_geodetic_degree(longitude_deg),
                }
            )
        peak_height_m = float(field.heights_m[component].max())
        obstacle_id = f"T{len(extracted) + 1:02d}"
        extracted.append(
            {
                "id": obstacle_id,
                "type": "polygon",
                "enabled": True,
                "points": points,
                "height_m": round(peak_height_m, 1),
                "label": f"地形障碍 {obstacle_id} 峰值{round(peak_height_m):.0f}m",
            }
        )
    return extracted


def generate_obstacles(
    layout_path: str | Path,
    *,
    threshold_m: float | None = None,
    min_area_km2: float = 0.15,
    simplify_tolerance_m: float = 60.0,
    corridor_u_min_km: float = -1.0,
    corridor_u_max_km: float = 28.0,
    corridor_v_half_width_km: float = 3.5,
    resolution: int | None = None,
) -> list[dict[str, Any]]:
    """读取布局并复用共享高度场生成器，返回可写入障碍文件的对象数组。"""

    path = Path(layout_path)
    layout = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(layout, dict):
        raise ValueError("地形布局根节点必须是对象")
    detail = layout.get("detail")
    selected_resolution = resolution if resolution is not None else int(detail["grid_resolution"])
    field = get_terrain_field(path, resolution=selected_resolution)
    return extract_obstacles_from_field(
        field,
        layout,
        threshold_m=threshold_m,
        min_area_km2=min_area_km2,
        simplify_tolerance_m=simplify_tolerance_m,
        corridor_u_min_km=corridor_u_min_km,
        corridor_u_max_km=corridor_u_max_km,
        corridor_v_half_width_km=corridor_v_half_width_km,
    )


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """解析命令行参数，默认值与 mountain_demo 的已验证规划参数一致。"""

    parser = argparse.ArgumentParser(description="从山地高度场生成二维避障障碍库")
    parser.add_argument("--layout", required=True, help="山地布局 JSON 文件")
    parser.add_argument("--output", required=True, help="输出障碍 JSON 文件")
    parser.add_argument("--threshold-m", type=float, help="障碍高度阈值；缺省时由 flight 净空推导")
    parser.add_argument("--min-area-km2", type=float, default=0.15, help="保留连通域的最小面积")
    parser.add_argument("--simplify-tolerance-m", type=float, default=60.0, help="边界简化容差（米）")
    parser.add_argument("--corridor-u-min-km", type=float, default=-1.0, help="走廊 east 下界（km）")
    parser.add_argument("--corridor-u-max-km", type=float, default=28.0, help="走廊 east 上界（km）")
    parser.add_argument("--corridor-v-half-width-km", type=float, default=3.5, help="走廊 north 半宽（km）")
    parser.add_argument("--resolution", type=int, help="覆盖布局默认高度场分辨率")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """运行 CLI 并把障碍数组按项目 JSON 格式写入目标文件。"""

    args = _parse_args(argv)
    obstacles = generate_obstacles(
        args.layout,
        threshold_m=args.threshold_m,
        min_area_km2=args.min_area_km2,
        simplify_tolerance_m=args.simplify_tolerance_m,
        corridor_u_min_km=args.corridor_u_min_km,
        corridor_u_max_km=args.corridor_u_max_km,
        corridor_v_half_width_km=args.corridor_v_half_width_km,
        resolution=args.resolution,
    )
    output_path = Path(args.output)
    output_path.write_text(json.dumps(obstacles, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"已生成 {len(obstacles)} 个地形障碍：{output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
