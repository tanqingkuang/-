"""Tests for avoidance feasibility checks: sharp turn, leg length, arc-vs-obstacle."""

from __future__ import annotations

import unittest

from src.algorithm.units.process.tra_plan.avoidance.feasibility import (
    ERR_ARC_HITS_OBSTACLE,
    ERR_LEG_TOO_SHORT,
    ERR_TURN_TOO_SHARP,
    check_feasibility,
)
from src.algorithm.units.process.tra_plan.avoidance.obstacle import make_circle


class FeasibilityTests(unittest.TestCase):
    def test_straight_two_points_is_feasible(self) -> None:
        result = check_feasibility([(0.0, 0.0), (1000.0, 0.0)], [], turn_radius_m=200.0, leg_margin_m=50.0)
        self.assertTrue(result.ok)
        self.assertEqual(result.code, "OK")

    def test_gentle_corner_with_room_is_feasible(self) -> None:
        # 90° 拐点，R=200 → d=200；两腿各 1000m，余度 50m 充足。
        points = [(0.0, 0.0), (1000.0, 0.0), (1000.0, 1000.0)]
        result = check_feasibility(points, [], turn_radius_m=200.0, leg_margin_m=50.0)
        self.assertTrue(result.ok, result.detail)

    def test_collinear_points_are_feasible(self) -> None:
        points = [(0.0, 0.0), (500.0, 0.0), (1000.0, 0.0)]
        self.assertTrue(check_feasibility(points, [], turn_radius_m=200.0, leg_margin_m=50.0).ok)

    def test_near_u_turn_flagged_too_sharp(self) -> None:
        points = [(0.0, 0.0), (1000.0, 0.0), (0.0, 10.0)]  # 拐点处近乎掉头
        result = check_feasibility(points, [], turn_radius_m=200.0, leg_margin_m=50.0)
        self.assertFalse(result.ok)
        self.assertEqual(result.code, ERR_TURN_TOO_SHARP)
        self.assertEqual(result.waypoint_index, 1)

    def test_short_leg_between_two_corners_flagged(self) -> None:
        # 两个 90° 拐点（各 d=200），中间腿仅 300m < 200+200+50=450m → 腿太短。
        points = [(0.0, 0.0), (500.0, 0.0), (500.0, 300.0), (0.0, 300.0)]
        result = check_feasibility(points, [], turn_radius_m=200.0, leg_margin_m=50.0)
        self.assertFalse(result.ok)
        self.assertEqual(result.code, ERR_LEG_TOO_SHORT)
        self.assertEqual(result.leg_index, 1)
        self.assertIn("缺", result.detail)

    def test_leg_margin_L_is_enforced(self) -> None:
        # 同一几何：L 小则可飞、L 大则腿太短。
        points = [(0.0, 0.0), (500.0, 0.0), (500.0, 500.0)]  # 单拐点 d=200，腿 500
        self.assertTrue(check_feasibility(points, [], turn_radius_m=200.0, leg_margin_m=250.0).ok)
        result = check_feasibility(points, [], turn_radius_m=200.0, leg_margin_m=350.0)
        self.assertFalse(result.ok)
        self.assertEqual(result.code, ERR_LEG_TOO_SHORT)

    def test_arc_hitting_obstacle_flagged(self) -> None:
        # 左转 90° 圆弧中点约 (441,59)，在此放小障碍 → 圆弧触障。
        points = [(0.0, 0.0), (500.0, 0.0), (500.0, 500.0)]
        obstacles = [make_circle("X", 441.0, 59.0, 25.0)]
        result = check_feasibility(points, obstacles, turn_radius_m=200.0, leg_margin_m=50.0)
        self.assertFalse(result.ok)
        self.assertEqual(result.code, ERR_ARC_HITS_OBSTACLE)
        self.assertEqual(result.waypoint_index, 1)
        self.assertEqual(result.obstacle_id, "X")

    def test_arc_clear_of_obstacle_is_feasible(self) -> None:
        # 障碍在转弯外侧远处，圆弧（内凸）不触及。
        points = [(0.0, 0.0), (500.0, 0.0), (500.0, 500.0)]
        obstacles = [make_circle("X", 800.0, -300.0, 50.0)]
        self.assertTrue(check_feasibility(points, obstacles, turn_radius_m=200.0, leg_margin_m=50.0).ok)

    def test_sharp_turn_checked_before_leg(self) -> None:
        # 急拐点应优先于腿长报出（更贴近根因）。
        points = [(0.0, 0.0), (100.0, 0.0), (0.0, 5.0)]
        result = check_feasibility(points, [], turn_radius_m=200.0, leg_margin_m=50.0)
        self.assertEqual(result.code, ERR_TURN_TOO_SHARP)

    def test_invalid_inputs_raise(self) -> None:
        with self.assertRaises(ValueError):
            check_feasibility([(0.0, 0.0)], [], turn_radius_m=200.0, leg_margin_m=50.0)
        with self.assertRaises(ValueError):
            check_feasibility([(0.0, 0.0), (1.0, 0.0)], [], turn_radius_m=-1.0, leg_margin_m=50.0)
        with self.assertRaises(ValueError):
            check_feasibility([(0.0, 0.0), (1.0, 0.0)], [], turn_radius_m=200.0, leg_margin_m=-1.0)

    def test_invalid_sample_step_raises(self) -> None:
        # sample_step<=0 必须直接报错：=0 会除零，<0 会被 max(1,..) 压成只采两端而漏检触障圆弧。
        points = [(0.0, 0.0), (500.0, 0.0), (500.0, 500.0)]
        obstacles = [make_circle("X", 441.0, 59.0, 25.0)]
        with self.assertRaises(ValueError):
            check_feasibility(points, obstacles, turn_radius_m=200.0, leg_margin_m=50.0, sample_step=0.0)
        with self.assertRaises(ValueError):
            check_feasibility(points, obstacles, turn_radius_m=200.0, leg_margin_m=50.0, sample_step=-1.0)


if __name__ == "__main__":
    unittest.main()
