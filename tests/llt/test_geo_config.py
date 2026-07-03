"""经纬高配置转换回归测试。"""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from src.algorithm.units.process.tra_plan.avoidance.obstacle import inside, make_polygon
from src.data.config_loader import resolve_config_references
from src.data.geo import GeoOrigin, enu_to_geodetic, geodetic_to_enu
from src.data.geo_config import route_to_internal


class GeoConversionTests(unittest.TestCase):
    """覆盖 WGS84 经纬度与 ENU 转换精度。"""

    def test_round_trip_keeps_100km_offsets_sub_millimeter(self) -> None:
        origin = GeoOrigin(latitude_deg=39.0, longitude_deg=116.0)
        for east, north in ((100000.0, 0.0), (0.0, 100000.0), (70000.0, 70000.0), (-60000.0, 80000.0)):
            lat, lon = enu_to_geodetic(east, north, origin)
            actual_east, actual_north = geodetic_to_enu(lat, lon, origin)
            self.assertAlmostEqual(actual_east, east, places=3)
            self.assertAlmostEqual(actual_north, north, places=3)


class GeoConfigTests(unittest.TestCase):
    """覆盖经纬高航线和障碍在配置加载边界转为 ENU。"""

    def test_loader_converts_route_and_rotated_rect_obstacle(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            element = root / "element"
            element.mkdir()
            origin = GeoOrigin(latitude_deg=39.0, longitude_deg=116.0)
            p0 = enu_to_geodetic(0.0, 0.0, origin)
            p1 = enu_to_geodetic(2000.0, 0.0, origin)
            rect = [enu_to_geodetic(east, north, origin) for east, north in ((0, 0), (100, 100), (0, 200), (-100, 100))]
            (element / "line.json").write_text(
                json.dumps(
                    {
                        "speed_mps": 20.0,
                        "waypoints": [
                            {"latitude_deg": p0[0], "longitude_deg": p0[1], "altitude_m": 1000.0},
                            {"latitude_deg": p1[0], "longitude_deg": p1[1], "altitude_m": 1000.0},
                        ],
                    }
                ),
                encoding="utf-8",
            )
            (element / "obstacles.json").write_text(
                json.dumps(
                    [
                        {
                            "id": "R1",
                            "type": "rect",
                            "points": [
                                {"latitude_deg": lat, "longitude_deg": lon}
                                for lat, lon in rect
                            ],
                        }
                    ]
                ),
                encoding="utf-8",
            )

            config = resolve_config_references(
                {"route_file": "element/line.json", "avoidance": {"obstacles_file": "element/obstacles.json"}},
                root / "base.json",
            )

        waypoints = config["route"]["waypoints"]
        self.assertAlmostEqual(waypoints[0]["x_m"], 0.0, places=6)
        self.assertAlmostEqual(waypoints[1]["x_m"], 2000.0, places=3)
        obstacle = config["avoidance"]["obstacles"][0]
        self.assertEqual(obstacle["type"], "polygon")
        self.assertEqual(len(obstacle["vertices"]), 4)

    def test_route_to_internal_rejects_enu_waypoints(self) -> None:
        """反例：非经纬(ENU x_m/y_m)航线在转换期直接报错——JSON 不再支持非经纬航线。"""
        with self.assertRaisesRegex(ValueError, "geodetic"):
            route_to_internal(
                {"waypoints": [{"x_m": 0.0, "y_m": 0.0}, {"x_m": 1000.0, "y_m": 0.0}]}
            )

    def test_route_to_internal_rejects_mixed_geodetic_and_enu_waypoints(self) -> None:
        """反例：首点是经纬但后续混入 ENU 航点时也必须报错，不能把缺省经纬当作 0°/0° 转换。"""
        with self.assertRaisesRegex(ValueError, r"waypoints\[1\].*geodetic"):
            route_to_internal(
                {
                    "waypoints": [
                        {"latitude_deg": 39.0, "longitude_deg": 116.0, "altitude_m": 1000.0},
                        {"x_m": 1000.0, "y_m": 0.0, "altitude_m": 1000.0},
                    ]
                }
            )

    def test_route_to_internal_rejects_enu_arc_center(self) -> None:
        """反例：圆弧中心点属于航线几何，JSON 中也必须用经纬度，不能混入 ENU center。"""
        origin = GeoOrigin(latitude_deg=39.0, longitude_deg=116.0)
        p1 = enu_to_geodetic(1000.0, 0.0, origin)
        with self.assertRaisesRegex(ValueError, r"center.*geodetic"):
            route_to_internal(
                {
                    "waypoints": [
                        {"latitude_deg": 39.0, "longitude_deg": 116.0, "altitude_m": 1000.0},
                        {
                            "latitude_deg": p1[0],
                            "longitude_deg": p1[1],
                            "altitude_m": 1000.0,
                            "turn_sign": 1.0,
                            "center": {"x_m": 500.0, "y_m": 500.0, "altitude_m": 1000.0},
                        },
                    ]
                }
            )

    def test_route_to_internal_accepts_geodetic_waypoints(self) -> None:
        """正例：经纬航线正常转成内部 ENU，首航点落在 ENU 原点。"""
        origin = GeoOrigin(latitude_deg=39.0, longitude_deg=116.0)
        p1 = enu_to_geodetic(1000.0, 0.0, origin)
        resolved, route_origin = route_to_internal(
            {
                "waypoints": [
                    {"latitude_deg": 39.0, "longitude_deg": 116.0, "altitude_m": 1000.0},
                    {"latitude_deg": p1[0], "longitude_deg": p1[1], "altitude_m": 1000.0},
                ]
            }
        )
        self.assertIsNotNone(route_origin)
        self.assertAlmostEqual(resolved["waypoints"][0]["x_m"], 0.0, places=3)
        self.assertAlmostEqual(resolved["waypoints"][1]["x_m"], 1000.0, places=3)

    def test_polygon_obstacle_preserves_rotation(self) -> None:
        obstacle = make_polygon("R1", [(0.0, 0.0), (100.0, 100.0), (0.0, 200.0), (-100.0, 100.0)])

        self.assertTrue(inside(obstacle, 0.0, 100.0))
        self.assertFalse(inside(obstacle, 70.0, 50.0))
        self.assertTrue(inside(obstacle, 70.0, 50.0, clearance=20.0))


if __name__ == "__main__":
    unittest.main()
