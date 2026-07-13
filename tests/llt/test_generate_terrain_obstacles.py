"""山地高度场障碍提取的纯几何回归测试。"""

from __future__ import annotations

import unittest

import numpy as np

from scripts.generate_terrain_obstacles import douglas_peucker, extract_obstacles_from_field
from src.data.geo import GeoOrigin, geodetic_to_enu
from src.ui.gui.situation3d.terrain_field import TerrainField


class GenerateTerrainObstaclesTests(unittest.TestCase):
    """验证高度阈值连通域能稳定转为避障多边形。"""

    def _layout(self, *, cruise_altitude_m: float = 900.0, clearance_m: float = 200.0) -> dict[str, object]:
        """构造满足纯函数最小输入约束的布局。"""

        return {
            "flight": {"cruise_altitude_m": cruise_altitude_m, "clearance_m": clearance_m},
            "geo_reference": {"latitude_deg": 39.0, "longitude_deg": 116.0},
        }

    def _field(self, heights_m: np.ndarray) -> TerrainField:
        """把小型高度数组包装成测试所需的 TerrainField。"""

        resolution = heights_m.shape[0]
        return TerrainField(
            resolution=resolution,
            center_east_m=0.0,
            center_north_m=0.0,
            width_m=3900.0,
            depth_m=3900.0,
            heights_m=heights_m.astype(np.float32),
            normals=np.zeros((resolution, resolution, 3), dtype=np.float32),
            colors=np.zeros((resolution, resolution, 3), dtype=np.float32),
            risk_zones=(),
            generation_time_ms=0.0,
        )

    def _extract(self, heights_m: np.ndarray, **kwargs: object) -> list[dict[str, object]]:
        """以宽走廊调用提取函数，避免默认走廊干扰合成地形测试。"""

        options: dict[str, object] = {
            "min_area_km2": 0.0,
            "simplify_tolerance_m": 0.0,
            "corridor_u_min_km": -3.0,
            "corridor_u_max_km": 3.0,
            "corridor_v_half_width_km": 3.0,
        }
        options.update(kwargs)
        return extract_obstacles_from_field(self._field(heights_m), self._layout(), **options)

    def test_isolated_hill_extracts_one_polygon(self) -> None:
        """单个矩形高地应形成至少三顶点的单一多边形。"""

        heights = np.zeros((40, 40), dtype=float)
        heights[12:25, 14:27] = 800.0
        obstacles = self._extract(heights)
        self.assertEqual(len(obstacles), 1)
        self.assertEqual(obstacles[0]["type"], "polygon")
        self.assertGreaterEqual(len(obstacles[0]["points"]), 3)

    def test_low_saddle_keeps_two_peaks_as_independent_components(self) -> None:
        """两座高峰之间保留低鞍部时，不能被错误合并为一个障碍。"""

        heights = np.zeros((40, 40), dtype=float)
        rows, cols = np.ogrid[:40, :40]
        west_peak = (rows - 20) ** 2 + (cols - 12) ** 2 <= 25
        east_peak = (rows - 20) ** 2 + (cols - 28) ** 2 <= 25
        heights[west_peak | east_peak] = 900.0
        obstacles = self._extract(heights)
        self.assertEqual(len(obstacles), 2)

    def test_small_component_is_filtered_by_area(self) -> None:
        """面积小于阈值的噪声格块不应进入障碍文件。"""

        heights = np.zeros((40, 40), dtype=float)
        heights[20, 20] = 900.0
        obstacles = self._extract(heights, min_area_km2=0.02)
        self.assertEqual(obstacles, [])

    def test_component_outside_corridor_is_filtered(self) -> None:
        """走廊外高地被丢弃，走廊内高地仍被保留。"""

        heights = np.zeros((40, 40), dtype=float)
        heights[17:23, 17:23] = 900.0
        heights[17:23, 34:39] = 900.0
        obstacles = self._extract(
            heights,
            corridor_u_min_km=-0.5,
            corridor_u_max_km=0.5,
            corridor_v_half_width_km=0.8,
        )
        self.assertEqual(len(obstacles), 1)

    def test_simplification_reduces_vertices_without_leaving_original_bounds(self) -> None:
        """简化应减少锯齿边界顶点，且结果仍落在容差扩展后的原始范围内。"""

        points = [(float(index), float((index % 2) * 2)) for index in range(20)]
        points.extend([(20.0, 20.0), (0.0, 20.0)])
        simplified = douglas_peucker(points, tolerance=3.0)
        self.assertGreaterEqual(len(simplified), 3)
        self.assertLess(len(simplified), len(points))
        for east, north in simplified:
            self.assertGreaterEqual(east, -3.0)
            self.assertLessEqual(east, 23.0)
            self.assertGreaterEqual(north, -3.0)
            self.assertLessEqual(north, 23.0)

    def test_default_threshold_comes_from_flight_clearance(self) -> None:
        """未传阈值时应使用巡航高度减垂直净空。"""

        heights = np.zeros((40, 40), dtype=float)
        heights[10:20, 10:20] = 750.0
        field = self._field(heights)
        layout = self._layout(cruise_altitude_m=900.0, clearance_m=200.0)
        obstacles = extract_obstacles_from_field(
            field,
            layout,
            min_area_km2=0.0,
            simplify_tolerance_m=0.0,
            corridor_u_min_km=-3.0,
            corridor_u_max_km=3.0,
            corridor_v_half_width_km=3.0,
        )
        self.assertEqual(len(obstacles), 1)

    def test_geodetic_output_round_trips_to_enu(self) -> None:
        """写出的七位小数经纬度反投影后应仍贴合原网格边界。"""

        heights = np.zeros((40, 40), dtype=float)
        heights[12:25, 14:27] = 800.0
        obstacle = self._extract(heights)[0]
        origin = GeoOrigin(latitude_deg=39.0, longitude_deg=116.0)
        for point in obstacle["points"]:
            east, north = geodetic_to_enu(point["latitude_deg"], point["longitude_deg"], origin)
            expected_east = -1950.0 + round((east + 1950.0) / 100.0) * 100.0
            expected_north = -1950.0 + round((north + 1950.0) / 100.0) * 100.0
            self.assertAlmostEqual(east, expected_east, delta=0.02)
            self.assertAlmostEqual(north, expected_north, delta=0.02)


if __name__ == "__main__":
    unittest.main()
