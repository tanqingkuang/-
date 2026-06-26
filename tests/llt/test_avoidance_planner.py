"""Tests for the avoidance orchestration: plan_avoidance_route (A*→simplify→arc→feasibility)."""

from __future__ import annotations

import unittest

from src.algorithm.units.process.tra_plan.avoidance.feasibility import ERR_LEG_TOO_SHORT
from src.algorithm.units.process.tra_plan.avoidance.obstacle import blocked, make_circle, make_rect
from src.algorithm.units.process.tra_plan.avoidance.planner import (
    ERR_ENDPOINT_IN_OBSTACLE,
    ERR_NO_PATH,
    plan_avoidance_route,
)

CLEARANCE = 120.0
COMMON = dict(
    turn_radius_m=150.0,
    leg_margin_m=50.0,
    clearance_m=CLEARANCE,
    simplify_clearance_m=CLEARANCE,
    speed_mps=20.0,
    resolution_m=20.0,
    margin_m=300.0,
)


def _straights_collision_free(route, obstacles, clearance) -> bool:
    for line in route.lines:
        if line.radius == 0.0:
            if blocked(obstacles, line.start.pos.east, line.start.pos.north, clearance):
                return False
            if blocked(obstacles, line.end.pos.east, line.end.pos.north, clearance):
                return False
    return True


class PlanAvoidanceRouteTests(unittest.TestCase):
    def test_no_obstacles_follows_original_route(self) -> None:
        wps = [(0.0, 0.0, 1000.0), (2000.0, 0.0, 1000.0)]
        result = plan_avoidance_route(wps, [], **COMMON)
        self.assertTrue(result.ok, result.detail)
        self.assertEqual(result.route.lines[0].start.pos.east, 0.0)
        self.assertEqual(result.route.lines[-1].end.pos.east, 2000.0)

    def test_single_circle_on_leg_is_detoured(self) -> None:
        obstacles = [make_circle("C1", 900.0, 0.0, 180.0)]
        result = plan_avoidance_route([(0.0, 0.0, 1000.0), (2000.0, 0.0, 1000.0)], obstacles, **COMMON)
        self.assertTrue(result.ok, result.detail)
        self.assertTrue(_straights_collision_free(result.route, obstacles, CLEARANCE))
        # 绕行必须偏出原直线（north 抬到膨胀半径量级）。
        self.assertGreaterEqual(max(abs(p[1]) for p in result.simplified_points), 180.0)

    def test_simplify_clearance_independent_from_search_clearance(self) -> None:
        obstacles = [make_circle("C1", 900.0, 0.0, 180.0)]
        conservative = dict(COMMON)
        conservative["simplify_clearance_m"] = CLEARANCE
        relaxed = dict(COMMON)
        relaxed["simplify_clearance_m"] = 0.0

        conservative_result = plan_avoidance_route(
            [(0.0, 0.0, 1000.0), (2000.0, 0.0, 1000.0)], obstacles, **conservative
        )
        relaxed_result = plan_avoidance_route(
            [(0.0, 0.0, 1000.0), (2000.0, 0.0, 1000.0)], obstacles, **relaxed
        )

        self.assertTrue(conservative_result.ok, conservative_result.detail)
        self.assertTrue(relaxed_result.ok, relaxed_result.detail)
        self.assertLess(len(relaxed_result.simplified_points), len(conservative_result.simplified_points))

    def test_missing_simplify_clearance_defaults_to_search_clearance(self) -> None:
        obstacles = [make_circle("C1", 900.0, 0.0, 180.0)]
        explicit = plan_avoidance_route(
            [(0.0, 0.0, 1000.0), (2000.0, 0.0, 1000.0)], obstacles, **COMMON
        )
        implicit_params = dict(COMMON)
        implicit_params.pop("simplify_clearance_m")
        implicit = plan_avoidance_route(
            [(0.0, 0.0, 1000.0), (2000.0, 0.0, 1000.0)], obstacles, **implicit_params
        )

        self.assertTrue(explicit.ok, explicit.detail)
        self.assertTrue(implicit.ok, implicit.detail)
        self.assertEqual(implicit.simplified_points, explicit.simplified_points)

    def test_multi_leg_route_avoids_both_obstacles(self) -> None:
        wps = [(0.0, 0.0, 1000.0), (2000.0, 0.0, 1000.0), (2000.0, 2000.0, 1000.0)]
        obstacles = [make_circle("C1", 900.0, 0.0, 180.0), make_circle("C2", 2000.0, 1200.0, 180.0)]
        result = plan_avoidance_route(wps, obstacles, **COMMON)
        self.assertTrue(result.ok, result.detail)
        self.assertTrue(any(line.radius > 0.0 for line in result.route.lines))
        self.assertTrue(_straights_collision_free(result.route, obstacles, CLEARANCE))

    def test_altitude_profile_preserved(self) -> None:
        wps = [(0.0, 0.0, 1000.0), (2000.0, 0.0, 1400.0)]
        result = plan_avoidance_route(wps, [make_circle("C1", 900.0, 0.0, 180.0)], **COMMON)
        self.assertTrue(result.ok, result.detail)
        self.assertAlmostEqual(result.route.lines[0].start.pos.h, 1000.0)
        self.assertAlmostEqual(result.route.lines[-1].end.pos.h, 1400.0)
        # 高度沿航线单调上升。
        heights = [line.start.pos.h for line in result.route.lines] + [result.route.lines[-1].end.pos.h]
        self.assertTrue(all(b >= a - 1e-6 for a, b in zip(heights, heights[1:])))

    def test_endpoint_in_obstacle_reports_code(self) -> None:
        obstacles = [make_circle("C1", 900.0, 0.0, 180.0)]
        result = plan_avoidance_route([(0.0, 0.0, 1000.0), (900.0, 0.0, 1000.0)], obstacles, **COMMON)
        self.assertFalse(result.ok)
        self.assertEqual(result.code, ERR_ENDPOINT_IN_OBSTACLE)
        self.assertEqual(result.leg_index, 0)

    def test_fully_enclosed_goal_reports_no_path(self) -> None:
        # 厚墙（>分辨率）把终点围成闭合方框 → 无可达通道。
        box = [
            make_rect("L", 380.0, -160.0, 440.0, 160.0),
            make_rect("R", 560.0, -160.0, 620.0, 160.0),
            make_rect("T", 380.0, 100.0, 620.0, 160.0),
            make_rect("B", 380.0, -160.0, 620.0, -100.0),
        ]
        result = plan_avoidance_route(
            [(0.0, 0.0, 1000.0), (500.0, 0.0, 1000.0)], box,
            turn_radius_m=100.0, leg_margin_m=20.0, clearance_m=0.0, speed_mps=20.0, resolution_m=20.0, margin_m=100.0,
        )
        self.assertFalse(result.ok)
        self.assertEqual(result.code, ERR_NO_PATH)
        self.assertEqual(result.leg_index, 0)

    def test_feasibility_failure_propagates(self) -> None:
        # L 过大 → 可飞性校验判腿太短，原因码透传，并附诊断点。
        obstacles = [make_circle("C1", 900.0, 0.0, 180.0)]
        params = dict(COMMON)
        params["leg_margin_m"] = 5000.0
        result = plan_avoidance_route([(0.0, 0.0, 1000.0), (2000.0, 0.0, 1000.0)], obstacles, **params)
        self.assertFalse(result.ok)
        self.assertEqual(result.code, ERR_LEG_TOO_SHORT)
        self.assertIsNotNone(result.feasibility)
        self.assertGreaterEqual(len(result.simplified_points), 2)

    def test_allow_arc_false_outputs_only_straight_but_still_avoids(self) -> None:
        # 外切线交付：无圆弧段，但绕障与避障一致；直线段仍在障碍外。
        obstacles = [make_circle("C1", 900.0, 0.0, 180.0)]
        params = dict(COMMON)
        params["allow_arc"] = False
        result = plan_avoidance_route([(0.0, 0.0, 1000.0), (2000.0, 0.0, 1000.0)], obstacles, **params)
        self.assertTrue(result.ok, result.detail)
        self.assertTrue(all(line.radius == 0.0 for line in result.route.lines))
        self.assertTrue(_straights_collision_free(result.route, obstacles, CLEARANCE))

    def test_allow_arc_false_still_rejects_infeasible(self) -> None:
        # §3.2 始终按真实 R 校验：外切线交付下不可飞场景照样拒（不被编码绕过）。
        obstacles = [make_circle("C1", 900.0, 0.0, 180.0)]
        params = dict(COMMON)
        params["allow_arc"] = False
        params["leg_margin_m"] = 5000.0
        result = plan_avoidance_route([(0.0, 0.0, 1000.0), (2000.0, 0.0, 1000.0)], obstacles, **params)
        self.assertFalse(result.ok)
        self.assertEqual(result.code, ERR_LEG_TOO_SHORT)

    def test_too_few_waypoints_raises(self) -> None:
        with self.assertRaises(ValueError):
            plan_avoidance_route([(0.0, 0.0, 1000.0)], [], **COMMON)


if __name__ == "__main__":
    unittest.main()
