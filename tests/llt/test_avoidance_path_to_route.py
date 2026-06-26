"""Tests for the avoidance exit translation: LOS simplification + arc route building."""

from __future__ import annotations

import math
import unittest

from src.algorithm.units.algo.arc_path import segment_length
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
    def test_two_points_make_single_straight_segment(self) -> None:
        route = points_to_route([(0.0, 0.0), (100.0, 0.0)], turn_radius_m=20.0, speed_mps=20.0)
        self.assertEqual(len(route.lines), 1)
        line = route.lines[0]
        self.assertEqual(line.radius, 0.0)
        self.assertEqual((line.start.pos.east, line.start.pos.north), (0.0, 0.0))
        self.assertEqual((line.end.pos.east, line.end.pos.north), (100.0, 0.0))
        self.assertEqual(line.vdCmd, 20.0)

    def test_corner_inserts_tangent_arc(self) -> None:
        # L 形拐点，R 足够小可装进两腿 → 直线+圆弧+直线。
        route = points_to_route(
            [(0.0, 0.0), (1000.0, 0.0), (1000.0, 1000.0)], turn_radius_m=200.0, speed_mps=20.0
        )
        arcs = [ln for ln in route.lines if ln.radius > 0.0]
        self.assertEqual(len(arcs), 1)
        arc = arcs[0]
        # 左转（东→北）turnSign 应为 +1，且圆心到两切点距离均为 R。
        self.assertEqual(arc.turnSign, 1.0)
        self.assertAlmostEqual(arc.radius, 200.0)
        r_start = math.hypot(arc.start.pos.east - arc.center.east, arc.start.pos.north - arc.center.north)
        r_end = math.hypot(arc.end.pos.east - arc.center.east, arc.end.pos.north - arc.center.north)
        self.assertAlmostEqual(r_start, 200.0, places=6)
        self.assertAlmostEqual(r_end, 200.0, places=6)
        # 圆弧段长 = R·|扫掠角|，90° 转弯 ≈ R·pi/2。
        self.assertAlmostEqual(segment_length(arc), 200.0 * math.pi / 2.0, places=3)

    def test_insert_arcs_false_keeps_all_straight_through_corners(self) -> None:
        # 外切线交付：拐点不插圆弧，全直线段且穿过原拐点（含中间拐点）。
        corners = [(0.0, 0.0), (1000.0, 0.0), (1000.0, 1000.0)]
        route = points_to_route(corners, turn_radius_m=200.0, speed_mps=20.0, insert_arcs=False)
        self.assertTrue(all(ln.radius == 0.0 for ln in route.lines))
        # n 个拐点 → n-1 段直线，逐段端点正好是原拐点序列。
        self.assertEqual(len(route.lines), len(corners) - 1)
        for i, line in enumerate(route.lines):
            self.assertEqual((line.start.pos.east, line.start.pos.north), corners[i])
            self.assertEqual((line.end.pos.east, line.end.pos.north), corners[i + 1])

    def test_insert_arcs_true_default_inserts_arc(self) -> None:
        # 默认 insert_arcs=True 仍插圆弧（与 false 形成对照）。
        route = points_to_route([(0.0, 0.0), (1000.0, 0.0), (1000.0, 1000.0)], turn_radius_m=200.0, speed_mps=20.0)
        self.assertTrue(any(ln.radius > 0.0 for ln in route.lines))

    def test_corner_radius_too_large_falls_back_to_straight(self) -> None:
        # R 远大于腿长 → 切点超出腿，退化为直线，无圆弧。
        route = points_to_route(
            [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0)], turn_radius_m=5000.0, speed_mps=20.0
        )
        self.assertTrue(all(ln.radius == 0.0 for ln in route.lines))

    def test_endpoints_have_zero_radius(self) -> None:
        route = points_to_route(
            [(0.0, 0.0), (500.0, 0.0), (500.0, 500.0), (1000.0, 500.0)], turn_radius_m=120.0, speed_mps=18.0
        )
        # 首段起点与末段终点即整条航线端点。
        self.assertEqual((route.lines[0].start.pos.east, route.lines[0].start.pos.north), (0.0, 0.0))
        self.assertEqual((route.lines[-1].end.pos.east, route.lines[-1].end.pos.north), (1000.0, 500.0))

    def test_altitude_applied(self) -> None:
        route = points_to_route([(0.0, 0.0), (100.0, 0.0)], turn_radius_m=0.0, speed_mps=20.0, altitude_m=1000.0)
        self.assertEqual(route.lines[0].start.pos.h, 1000.0)
        self.assertEqual(route.lines[0].end.pos.h, 1000.0)

    def test_per_point_altitudes_applied(self) -> None:
        route = points_to_route(
            [(0.0, 0.0), (100.0, 0.0)], turn_radius_m=0.0, speed_mps=20.0, altitudes=[1000.0, 1200.0]
        )
        self.assertEqual(route.lines[0].start.pos.h, 1000.0)
        self.assertEqual(route.lines[0].end.pos.h, 1200.0)

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
        route = points_to_route(simplified, turn_radius_m=80.0, speed_mps=20.0, altitude_m=1000.0)
        self.assertGreaterEqual(len(route.lines), 1)
        # 起终点与规划一致。
        self.assertAlmostEqual(route.lines[0].start.pos.east, 0.0)
        self.assertAlmostEqual(route.lines[-1].end.pos.east, 1000.0)
        # 直线段端点应在障碍外（圆弧外凸是否触障由步骤4 校验，这里只查直线骨架）。
        for line in route.lines:
            if line.radius == 0.0:
                self.assertFalse(blocked(obstacles, line.start.pos.east, line.start.pos.north, clearance))
                self.assertFalse(blocked(obstacles, line.end.pos.east, line.end.pos.north, clearance))


if __name__ == "__main__":
    unittest.main()
