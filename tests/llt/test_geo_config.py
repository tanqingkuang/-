"""经纬高配置转换回归测试。"""

from __future__ import annotations

import json
import math
import tempfile
import unittest
from pathlib import Path

from src.algorithm.units.process.tra_plan.avoidance.obstacle import inside, make_polygon
from src.data.config_loader import resolve_config_references
from src.data.geo import GeoOrigin, enu_to_geodetic, geodetic_to_enu
from src.data.geo_config import obstacles_to_internal, route_to_external, route_to_internal


class GeoConversionTests(unittest.TestCase):
    """覆盖 WGS84 经纬度与 ENU 转换精度。"""

    def test_round_trip_keeps_100km_offsets_sub_millimeter(self) -> None:
        origin = GeoOrigin(latitude_deg=39.0, longitude_deg=116.0)
        for east, north in ((100000.0, 0.0), (0.0, 100000.0), (70000.0, 70000.0), (-60000.0, 80000.0)):
            lat, lon = enu_to_geodetic(east, north, origin)
            actual_east, actual_north = geodetic_to_enu(lat, lon, origin)
            self.assertAlmostEqual(actual_east, east, places=3)
            self.assertAlmostEqual(actual_north, north, places=3)

    def test_cardinal_geodetic_increments_keep_east_north_signs(self) -> None:
        """增大经度必须向东，增大纬度必须向北，避免正反变换同时翻号仍通过往返测试。"""
        origin = GeoOrigin(latitude_deg=39.0, longitude_deg=116.0)

        east, east_north = geodetic_to_enu(39.0, 116.001, origin)
        north_east, north = geodetic_to_enu(39.001, 116.0, origin)

        self.assertGreater(east, 0.0)
        self.assertGreater(north, 0.0)
        self.assertLess(abs(east_north), east)
        self.assertLess(abs(north_east), north)


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

    def test_route_to_internal_rejects_mixed_coordinates_within_one_waypoint(self) -> None:
        """反例：同一航点不得同时携带经纬度和任一套 ENU 别名。"""
        enu_representations = (
            {"x_m": 100.0, "y_m": 200.0},
            {"east_m": 100.0, "north_m": 200.0},
            {"east": 100.0, "north": 200.0},
        )
        for enu in enu_representations:
            with self.subTest(enu=enu), self.assertRaisesRegex(
                ValueError,
                r"waypoints\[0\].*mixes geodetic and ENU",
            ):
                route_to_internal(
                    {
                        "waypoints": [
                            {
                                "latitude_deg": 39.0,
                                "longitude_deg": 116.0,
                                "altitude_m": 1000.0,
                                **enu,
                            }
                        ]
                    }
                )

    def test_route_to_internal_rejects_mixed_coordinates_in_arc_center(self) -> None:
        """反例：圆弧中心同样只能使用经纬度，不能夹带 ENU 别名。"""
        with self.assertRaisesRegex(
            ValueError,
            r"center.*mixes geodetic and ENU",
        ):
            route_to_internal(
                {
                    "waypoints": [
                        {
                            "latitude_deg": 39.0,
                            "longitude_deg": 116.0,
                            "altitude_m": 1000.0,
                            "center": {
                                "latitude_deg": 39.001,
                                "longitude_deg": 116.001,
                                "east": 100.0,
                                "north": 200.0,
                            },
                        }
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

    def test_mixed_geodetic_and_enu_obstacle_points_are_rejected(self) -> None:
        """同一个多边形不得混用经纬度与 ENU，避免 ENU 点被静默按 0°/0° 投影。"""
        with self.assertRaisesRegex(ValueError, "mixes geodetic and ENU"):
            obstacles_to_internal(
                [
                    {
                        "type": "polygon",
                        "points": [
                            {"latitude_deg": 39.0, "longitude_deg": 116.0},
                            {"east_m": 100.0, "north_m": 200.0},
                        ],
                    }
                ],
                GeoOrigin(latitude_deg=39.0, longitude_deg=116.0),
            )

    def test_partial_geodetic_obstacle_point_is_rejected(self) -> None:
        """经纬度缺少任一半边时直接报错，不得回退为 ENU 或坐标原点。"""
        with self.assertRaisesRegex(ValueError, "both latitude and longitude"):
            obstacles_to_internal(
                [{"type": "polygon", "points": [{"latitude_deg": 39.0}]}],
                GeoOrigin(latitude_deg=39.0, longitude_deg=116.0),
            )

    def test_invalid_geodetic_obstacle_coordinates_are_rejected(self) -> None:
        """非有限及越界经纬度均应在投影前报错。"""
        invalid_points = (
            {"latitude_deg": math.nan, "longitude_deg": 116.0},
            {"latitude_deg": math.inf, "longitude_deg": 116.0},
            {"latitude_deg": 91.0, "longitude_deg": 116.0},
            {"latitude_deg": 39.0, "longitude_deg": 181.0},
        )
        for point in invalid_points:
            with self.subTest(point=point), self.assertRaises(ValueError):
                obstacles_to_internal(
                    [{"type": "circle", "center": point}],
                    GeoOrigin(latitude_deg=39.0, longitude_deg=116.0),
                )

    def test_route_external_supports_east_north_and_round_trips(self) -> None:
        """east_m/north_m 应正确输出经纬度，读回后保持方向和数值。"""
        origin = GeoOrigin(latitude_deg=39.0, longitude_deg=116.0)
        external = route_to_external(
            {
                "waypoints": [
                    {"east_m": 1200.0, "north_m": -800.0, "altitude_m": 1000.0},
                ]
            },
            origin,
        )

        point = external["waypoints"][0]
        self.assertNotIn("east_m", point)
        self.assertNotIn("north_m", point)
        internal, _ = route_to_internal(external, origin)
        actual = internal["waypoints"][0]
        self.assertAlmostEqual(actual["x_m"], 1200.0, delta=0.02)
        self.assertAlmostEqual(actual["y_m"], -800.0, delta=0.02)

    def test_route_external_missing_enu_coordinate_is_rejected(self) -> None:
        """内部航点缺少水平坐标时不得静默输出 origin 经纬度。"""
        with self.assertRaisesRegex(ValueError, "both east_m and north_m"):
            route_to_external(
                {"waypoints": [{"east_m": 100.0, "altitude_m": 1000.0}]},
                GeoOrigin(latitude_deg=39.0, longitude_deg=116.0),
            )


if __name__ == "__main__":
    unittest.main()
