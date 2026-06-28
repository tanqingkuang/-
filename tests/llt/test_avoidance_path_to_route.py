"""Tests for the avoidance exit translation: LOS simplification + arc route building."""

from __future__ import annotations

import math
import unittest

from src.algorithm.units.algo.arc_path import arc_radius, segment_length
from src.algorithm.entity.leader_follower_hold.leader import waypoint_inputs_to_waylines
from src.algorithm.units.process.tra_plan.avoidance.astar import plan_path
from src.algorithm.units.process.tra_plan.avoidance.obstacle import (
    blocked,
    make_circle,
    make_rect,
)
from src.algorithm.units.process.tra_plan.avoidance.path_to_route import (
    line_of_sight_clear,
    points_to_route,
    simplify_path,
)


class LineOfSightTests(unittest.TestCase):
    def test_clear_when_no_obstacles(self) -> None:
        self.assertTrue(line_of_sight_clear((0.0, 0.0), (100.0, 0.0), []))

    def test_blocked_when_segment_crosses_obstacle(self) -> None:
        obstacles = [make_circle("C", 50.0, 0.0, 20.0)]
        self.assertFalse(line_of_sight_clear((0.0, 0.0), (100.0, 0.0), obstacles))

    def test_clear_when_segment_passes_beside_obstacle(self) -> None:
        obstacles = [make_circle("C", 50.0, 0.0, 20.0)]
        self.assertTrue(line_of_sight_clear((0.0, 100.0), (100.0, 100.0), obstacles))

    def test_clearance_widens_blocking(self) -> None:
        obstacles = [make_circle("C", 50.0, 40.0, 20.0)]
        self.assertTrue(line_of_sight_clear((0.0, 0.0), (100.0, 0.0), obstacles))
        self.assertFalse(line_of_sight_clear((0.0, 0.0), (100.0, 0.0), obstacles, clearance=30.0))

    def test_sample_step_is_spacing_upper_bound_not_floored(self) -> None:
        # length=19、sample_step=10：int 向下取整只采两端点会漏掉中间细障碍；ceil 必须采到 (9.5,0)。
        obstacles = [make_circle("C", 9.5, 0.0, 2.0)]
        self.assertFalse(line_of_sight_clear((0.0, 0.0), (19.0, 0.0), obstacles, sample_step=10.0))

    def test_invalid_sample_step_raises(self) -> None:
        obstacles = [make_circle("C", 5.0, 0.0, 1.0)]
        with self.assertRaises(ValueError):
            line_of_sight_clear((0.0, 0.0), (10.0, 0.0), obstacles, sample_step=0.0)
        with self.assertRaises(ValueError):
            line_of_sight_clear((0.0, 0.0), (10.0, 0.0), obstacles, sample_step=-1.0)


class SimplifyPathTests(unittest.TestCase):
    def test_collinear_points_reduced_to_endpoints(self) -> None:
        points = [(0.0, 0.0), (10.0, 0.0), (20.0, 0.0), (30.0, 0.0), (40.0, 0.0)]
        self.assertEqual(simplify_path(points, []), [(0.0, 0.0), (40.0, 0.0)])

    def test_short_path_returned_as_is(self) -> None:
        self.assertEqual(simplify_path([(0.0, 0.0), (10.0, 0.0)], []), [(0.0, 0.0), (10.0, 0.0)])

    def test_staircase_without_obstacles_straightens(self) -> None:
        # 无障碍时锯齿应被拉直为两端点。
        points = [(0.0, 0.0), (10.0, 10.0), (20.0, 20.0), (30.0, 30.0)]
        self.assertEqual(simplify_path(points, []), [(0.0, 0.0), (30.0, 30.0)])

    def test_simplified_segments_do_not_cross_obstacle(self) -> None:
        obstacles = [make_circle("C", 500.0, 0.0, 150.0)]
        clearance = 50.0
        raw = plan_path((0.0, 0.0), (1000.0, 0.0), obstacles, resolution_m=25.0, clearance_m=clearance, margin_m=300.0)
        self.assertIsNotNone(raw)
        simplified = simplify_path(raw, obstacles, clearance=clearance)
        # 去冗余后拐点数显著少于原始格点。
        self.assertLess(len(simplified), len(raw))
        self.assertGreaterEqual(len(simplified), 2)
        # 端点保持不变。
        self.assertEqual(simplified[0], raw[0])
        self.assertEqual(simplified[-1], raw[-1])
        # 每条拉直后的腿都不得穿过障碍。
        for a, b in zip(simplified, simplified[1:]):
            self.assertTrue(line_of_sight_clear(a, b, obstacles, clearance=clearance), f"leg {a}-{b} crosses obstacle")


class PointsToRouteTests(unittest.TestCase):
    def test_two_points_make_wpi_list(self) -> None:
        wpi = points_to_route([(0.0, 0.0), (100.0, 0.0)], turn_radius_m=20.0, speed_mps=20.0)
        self.assertEqual(len(wpi), 2)
        self.assertEqual(wpi[0].r, 0.0)
        self.assertEqual(wpi[1].r, 0.0)
        self.assertEqual((wpi[0].pos.east, wpi[0].pos.north), (0.0, 0.0))
        self.assertEqual((wpi[1].pos.east, wpi[1].pos.north), (100.0, 0.0))
        self.assertEqual(wpi[0].vdCmd, 20.0)

    def test_interior_point_gets_radius(self) -> None:
        wpi = points_to_route(
            [(0.0, 0.0), (1000.0, 0.0), (1000.0, 1000.0)], turn_radius_m=200.0, speed_mps=20.0
        )
        self.assertEqual(len(wpi), 3)
        self.assertEqual(wpi[0].r, 0.0)  # 首点 r=0
        self.assertEqual(wpi[1].r, 200.0)  # 内部拐点 r=R
        self.assertEqual(wpi[2].r, 0.0)  # 末点 r=0

    def test_corner_arc_geometry_via_conversion(self) -> None:
        # L 形拐点，R 足够小可装进两腿 → 转换后得到直线+圆弧+直线段。
        wpi = points_to_route(
            [(0.0, 0.0), (1000.0, 0.0), (1000.0, 1000.0)], turn_radius_m=200.0, speed_mps=20.0
        )
        lines = waypoint_inputs_to_waylines(wpi)
        arcs = [ln for ln in lines if ln.start.turnSign != 0.0]
        self.assertEqual(len(arcs), 1)
        arc = arcs[0]
        # 左转（东→北）turnSign 应为 +1，且圆心到两切点距离均为 R。
        self.assertEqual(arc.start.turnSign, 1.0)
        r = arc_radius(arc)
        self.assertAlmostEqual(r, 200.0)
        r_end = math.hypot(arc.end.pos.east - arc.start.center.east, arc.end.pos.north - arc.start.center.north)
        self.assertAlmostEqual(r_end, 200.0, places=6)
        # 圆弧段长 = R·|扫掠角|，90° 转弯 ≈ R·pi/2。
        self.assertAlmostEqual(segment_length(arc), 200.0 * math.pi / 2.0, places=3)

    def test_oversized_corner_radius_falls_back_to_straight_lines(self) -> None:
        """圆弧切点超出相邻腿长时，应退回原始折线而不是插入腿外圆弧。"""

        wpi = points_to_route(
            [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)], turn_radius_m=100.0, speed_mps=20.0
        )
        lines = waypoint_inputs_to_waylines(wpi)

        self.assertEqual(len(lines), 2)
        self.assertTrue(all(line.start.turnSign == 0.0 for line in lines))
        self.assertAlmostEqual(lines[0].end.pos.east, 10.0)
        self.assertAlmostEqual(lines[0].end.pos.north, 0.0)
        self.assertAlmostEqual(lines[1].start.pos.east, 10.0)
        self.assertAlmostEqual(lines[1].start.pos.north, 0.0)

    def test_insert_arcs_false_all_r_zero(self) -> None:
        wpi = points_to_route(
            [(0.0, 0.0), (1000.0, 0.0), (1000.0, 1000.0)], turn_radius_m=200.0, speed_mps=20.0, insert_arcs=False
        )
        self.assertTrue(all(w.r == 0.0 for w in wpi))
        lines = waypoint_inputs_to_waylines(wpi)
        # n 点 → n-1 段直线
        self.assertEqual(len(lines), 2)
        self.assertTrue(all(ln.start.turnSign == 0.0 for ln in lines))

    def test_insert_arcs_true_default_inserts_arc_via_conversion(self) -> None:
        wpi = points_to_route([(0.0, 0.0), (1000.0, 0.0), (1000.0, 1000.0)], turn_radius_m=200.0, speed_mps=20.0)
        lines = waypoint_inputs_to_waylines(wpi)
        self.assertTrue(any(ln.start.turnSign != 0.0 for ln in lines))

    def test_endpoints_have_correct_pos(self) -> None:
        wpi = points_to_route(
            [(0.0, 0.0), (500.0, 0.0), (500.0, 500.0), (1000.0, 500.0)], turn_radius_m=120.0, speed_mps=18.0
        )
        self.assertEqual((wpi[0].pos.east, wpi[0].pos.north), (0.0, 0.0))
        self.assertEqual((wpi[-1].pos.east, wpi[-1].pos.north), (1000.0, 500.0))

    def test_altitude_applied(self) -> None:
        wpi = points_to_route([(0.0, 0.0), (100.0, 0.0)], turn_radius_m=0.0, speed_mps=20.0, altitude_m=1000.0)
        self.assertEqual(wpi[0].pos.h, 1000.0)
        self.assertEqual(wpi[1].pos.h, 1000.0)

    def test_per_point_altitudes_applied(self) -> None:
        wpi = points_to_route(
            [(0.0, 0.0), (100.0, 0.0)], turn_radius_m=0.0, speed_mps=20.0, altitudes=[1000.0, 1200.0]
        )
        self.assertEqual(wpi[0].pos.h, 1000.0)
        self.assertEqual(wpi[1].pos.h, 1200.0)

    def test_altitudes_length_mismatch_raises(self) -> None:
        with self.assertRaises(ValueError):
            points_to_route([(0.0, 0.0), (100.0, 0.0)], turn_radius_m=0.0, speed_mps=20.0, altitudes=[1000.0])

    def test_invalid_inputs_raise(self) -> None:
        with self.assertRaises(ValueError):
            points_to_route([(0.0, 0.0)], turn_radius_m=20.0, speed_mps=20.0)
        with self.assertRaises(ValueError):
            points_to_route([(0.0, 0.0), (1.0, 0.0)], turn_radius_m=-1.0, speed_mps=20.0)
        with self.assertRaises(ValueError):
            points_to_route([(0.0, 0.0), (1.0, 0.0)], turn_radius_m=20.0, speed_mps=-1.0)


class AStarToRouteIntegrationTests(unittest.TestCase):
    def test_full_chain_plan_simplify_route(self) -> None:
        obstacles = [make_circle("C", 500.0, 0.0, 150.0)]
        clearance = 50.0
        raw = plan_path((0.0, 0.0), (1000.0, 0.0), obstacles, resolution_m=20.0, clearance_m=clearance, margin_m=300.0)
        self.assertIsNotNone(raw)
        simplified = simplify_path(raw, obstacles, clearance=clearance)
        wpi = points_to_route(simplified, turn_radius_m=80.0, speed_mps=20.0, altitude_m=1000.0)
        lines = waypoint_inputs_to_waylines(wpi)
        self.assertGreaterEqual(len(lines), 1)
        # 起终点与规划一致。
        self.assertAlmostEqual(lines[0].start.pos.east, 0.0)
        self.assertAlmostEqual(lines[-1].end.pos.east, 1000.0)
        # 直线段端点应在障碍外（圆弧外凸是否触障由步骤4 校验，这里只查直线骨架）。
        for line in lines:
            if line.start.turnSign == 0.0:
                self.assertFalse(blocked(obstacles, line.start.pos.east, line.start.pos.north, clearance))
                self.assertFalse(blocked(obstacles, line.end.pos.east, line.end.pos.north, clearance))


if __name__ == "__main__":
    unittest.main()
