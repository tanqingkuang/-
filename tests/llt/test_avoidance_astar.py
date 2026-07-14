"""Tests for the avoidance backend: obstacle primitive (inside) and the A* kernel."""

from __future__ import annotations

import math
import unittest

from src.algorithm.units.process.tra_plan.avoidance.astar import (
    _DIRECTION_DELTA_DEG,
    _heading_transition_penalty,
    compute_bounds,
    plan_path,
)
from src.algorithm.units.process.tra_plan.avoidance.obstacle import (
    ObstacleS,
    blocked,
    inside,
    make_circle,
    make_rect,
    obstacle_bounds,
)


def _segment_lengths(path: list[tuple[float, float]]) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))


def _direction_metrics(path: list[tuple[float, float]]) -> tuple[int, float]:
    directions = [
        (round((b[0] - a[0]) / 10.0), round((b[1] - a[1]) / 10.0))
        for a, b in zip(path, path[1:])
    ]
    switches = 0
    total_angle = 0.0
    for previous, current in zip(directions, directions[1:]):
        if previous == current:
            continue
        switches += 1
        dot = previous[0] * current[0] + previous[1] * current[1]
        prev_len = math.hypot(*previous)
        cur_len = math.hypot(*current)
        total_angle += math.degrees(math.acos(max(-1.0, min(1.0, dot / (prev_len * cur_len)))))
    return switches, total_angle


def _gapped_wall_obstacles(gap_north: float) -> list[ObstacleS]:
    return [
        make_rect(f"W{north}", 45.0, north - 5.0, 55.0, north + 5.0)
        for north in range(-50, 60, 10)
        if north != gap_north
    ]


class ObstaclePrimitiveTests(unittest.TestCase):
    """The single shape primitive inside() must back both A* and feasibility checks."""

    def test_inside_circle_core_boundary_outside(self) -> None:
        circle = make_circle("C", 0.0, 0.0, 100.0)
        self.assertTrue(inside(circle, 0.0, 0.0))
        self.assertTrue(inside(circle, 100.0, 0.0))  # 边界算内
        self.assertFalse(inside(circle, 100.1, 0.0))

    def test_inside_circle_with_clearance(self) -> None:
        circle = make_circle("C", 0.0, 0.0, 100.0)
        self.assertFalse(inside(circle, 130.0, 0.0))
        self.assertTrue(inside(circle, 130.0, 0.0, clearance=40.0))

    def test_inside_rect_core_boundary_outside(self) -> None:
        rect = make_rect("R", 0.0, 0.0, 200.0, 100.0)
        self.assertTrue(inside(rect, 100.0, 50.0))
        self.assertTrue(inside(rect, 0.0, 0.0))  # 角点算内
        self.assertFalse(inside(rect, 201.0, 50.0))

    def test_inside_rect_with_clearance(self) -> None:
        rect = make_rect("R", 0.0, 0.0, 200.0, 100.0)
        self.assertFalse(inside(rect, 230.0, 50.0))
        self.assertTrue(inside(rect, 230.0, 50.0, clearance=40.0))

    def test_make_rect_normalizes_min_max(self) -> None:
        rect = make_rect("R", 200.0, 100.0, 0.0, 0.0)
        self.assertEqual((rect.min_e, rect.min_n, rect.max_e, rect.max_n), (0.0, 0.0, 200.0, 100.0))

    def test_blocked_over_list_and_empty(self) -> None:
        obstacles = [make_circle("C", 0.0, 0.0, 50.0), make_rect("R", 300.0, 0.0, 400.0, 100.0)]
        self.assertTrue(blocked(obstacles, 0.0, 0.0))
        self.assertTrue(blocked(obstacles, 350.0, 50.0))
        self.assertFalse(blocked(obstacles, 1000.0, 1000.0))
        self.assertFalse(blocked([], 0.0, 0.0))

    def test_obstacle_bounds(self) -> None:
        self.assertEqual(obstacle_bounds(make_circle("C", 10.0, 20.0, 5.0)), (5.0, 15.0, 15.0, 25.0))
        self.assertEqual(obstacle_bounds(make_rect("R", 0.0, 0.0, 30.0, 40.0)), (0.0, 0.0, 30.0, 40.0))


class AStarPlanPathTests(unittest.TestCase):
    """A* must find collision-free grid paths or report no-solution."""

    def _assert_collision_free(self, path, obstacles, clearance) -> None:
        for east, north in path:
            self.assertFalse(blocked(obstacles, east, north, clearance), f"point ({east},{north}) inside obstacle")

    def test_no_obstacles_returns_near_straight_path(self) -> None:
        path = plan_path((0.0, 0.0), (300.0, 0.0), [], resolution_m=25.0)
        self.assertIsNotNone(path)
        self.assertEqual(path[0], (0.0, 0.0))
        self.assertEqual(path[-1], (300.0, 0.0))
        # 无障碍时栅格最短路应接近直线距离。
        self.assertLessEqual(_segment_lengths(path), 300.0 * 1.1)

    def test_circle_on_straight_line_is_detoured(self) -> None:
        obstacles = [make_circle("C", 500.0, 0.0, 150.0)]
        clearance = 50.0
        path = plan_path((0.0, 0.0), (1000.0, 0.0), obstacles, resolution_m=25.0, clearance_m=clearance, margin_m=300.0)
        self.assertIsNotNone(path)
        self.assertEqual(path[0], (0.0, 0.0))
        self.assertEqual(path[-1], (1000.0, 0.0))
        self._assert_collision_free(path, obstacles, clearance)
        # 绕行必须偏出直线：至少一点的 |north| 达到膨胀半径量级。
        self.assertGreaterEqual(max(abs(n) for _, n in path), 150.0)

    def test_rect_on_straight_line_is_detoured(self) -> None:
        obstacles = [make_rect("R", 400.0, -100.0, 600.0, 100.0)]
        clearance = 40.0
        path = plan_path((0.0, 0.0), (1000.0, 0.0), obstacles, resolution_m=25.0, clearance_m=clearance, margin_m=300.0)
        self.assertIsNotNone(path)
        self._assert_collision_free(path, obstacles, clearance)
        self.assertGreaterEqual(max(abs(n) for _, n in path), 100.0)

    def test_goal_inside_obstacle_returns_none(self) -> None:
        obstacles = [make_circle("C", 1000.0, 0.0, 150.0)]
        self.assertIsNone(
            plan_path((0.0, 0.0), (1000.0, 0.0), obstacles, resolution_m=25.0, clearance_m=20.0, margin_m=200.0)
        )

    def test_start_inside_obstacle_returns_none(self) -> None:
        obstacles = [make_circle("C", 0.0, 0.0, 150.0)]
        self.assertIsNone(
            plan_path((0.0, 0.0), (1000.0, 0.0), obstacles, resolution_m=25.0, clearance_m=20.0, margin_m=200.0)
        )

    def test_exact_start_in_small_obstacle_returns_none(self) -> None:
        # 小障碍：精确起点在障碍内，但最近格心在障碍外，吸附检查会漏 → 必须按精确坐标拦截。
        obstacles = [make_circle("S", 0.0, 0.0, 1.0)]
        self.assertIsNone(
            plan_path((0.0, 0.0), (100.0, 0.0), obstacles, resolution_m=10.0, clearance_m=0.0, margin_m=30.0)
        )

    def test_exact_goal_in_small_obstacle_returns_none(self) -> None:
        obstacles = [make_circle("G", 100.0, 0.0, 1.0)]
        self.assertIsNone(
            plan_path(
                (0.0, 0.0), (100.0, 0.0), obstacles,
                resolution_m=10.0, clearance_m=0.0, bounds=(-10.0, -6.0, 106.0, 30.0),
            )
        )

    def test_exact_endpoint_in_clearance_band_returns_none(self) -> None:
        # 障碍本体不含起点，但加上 clearance 后精确起点落入膨胀带 → 无解。
        obstacles = [make_circle("S", 0.0, 0.0, 1.0)]
        self.assertIsNone(
            plan_path((5.0, 0.0), (100.0, 0.0), obstacles, resolution_m=10.0, clearance_m=8.0, margin_m=30.0)
        )

    def test_returned_path_endpoints_are_collision_free(self) -> None:
        # 与上述反例对照：合法场景下返回路径的首尾必须 collision-free。
        obstacles = [make_circle("C", 500.0, 0.0, 150.0)]
        clearance = 50.0
        path = plan_path((0.0, 0.0), (1000.0, 0.0), obstacles, resolution_m=25.0, clearance_m=clearance, margin_m=300.0)
        self.assertIsNotNone(path)
        self.assertFalse(blocked(obstacles, *path[0], clearance))
        self.assertFalse(blocked(obstacles, *path[-1], clearance))

    def test_sealed_corridor_returns_none(self) -> None:
        # 一道贯穿上下边界的矩形墙把起终点隔开 → 无解。
        bounds = (-50.0, -300.0, 1050.0, 300.0)
        wall = make_rect("W", 480.0, -300.0, 520.0, 300.0)
        self.assertIsNone(
            plan_path((0.0, 0.0), (1000.0, 0.0), [wall], resolution_m=25.0, clearance_m=0.0, bounds=bounds)
        )

    def test_larger_clearance_pushes_path_further(self) -> None:
        obstacles = [make_circle("C", 500.0, 0.0, 150.0)]
        center = (500.0, 0.0)

        def min_gap(clearance: float) -> float:
            path = plan_path(
                (0.0, 0.0), (1000.0, 0.0), obstacles, resolution_m=20.0, clearance_m=clearance, margin_m=400.0
            )
            self.assertIsNotNone(path)
            return min(math.hypot(e - center[0], n - center[1]) for e, n in path)

        self.assertGreater(min_gap(120.0), min_gap(20.0))

    def test_collision_free_invariant_with_two_obstacles(self) -> None:
        obstacles = [make_circle("C1", 350.0, 0.0, 120.0), make_circle("C2", 700.0, 120.0, 120.0)]
        clearance = 30.0
        path = plan_path((0.0, 0.0), (1000.0, 0.0), obstacles, resolution_m=20.0, clearance_m=clearance, margin_m=300.0)
        self.assertIsNotNone(path)
        self._assert_collision_free(path, obstacles, clearance)

    def test_direction_delta_table_matches_expected_octant_angles(self) -> None:
        self.assertEqual(len(_DIRECTION_DELTA_DEG), 8)
        self.assertAlmostEqual(_DIRECTION_DELTA_DEG[0][0], 0.0)
        self.assertAlmostEqual(_DIRECTION_DELTA_DEG[0][4], 45.0)
        self.assertAlmostEqual(_DIRECTION_DELTA_DEG[0][2], 90.0)
        self.assertAlmostEqual(_DIRECTION_DELTA_DEG[0][1], 180.0)
        for previous in range(8):
            for current in range(8):
                self.assertAlmostEqual(_DIRECTION_DELTA_DEG[previous][current], _DIRECTION_DELTA_DEG[current][previous])

    def test_heading_transition_penalty_uses_precomputed_angle_table(self) -> None:
        self.assertEqual(_heading_transition_penalty(None, 0, 30.0, 20.0), 0.0)
        self.assertEqual(_heading_transition_penalty(0, 0, 30.0, 20.0), 0.0)
        self.assertAlmostEqual(_heading_transition_penalty(0, 4, 30.0, 20.0), 50.0)
        self.assertAlmostEqual(_heading_transition_penalty(0, 2, 30.0, 20.0), 70.0)
        self.assertAlmostEqual(_heading_transition_penalty(0, 1, 30.0, 20.0), 110.0)

    def test_zero_heading_penalties_match_legacy_path(self) -> None:
        obstacles = _gapped_wall_obstacles(-30.0)
        common = dict(
            resolution_m=10.0,
            clearance_m=0.0,
            bounds=(-10.0, -60.0, 110.0, 60.0),
        )
        legacy = plan_path((0.0, -30.0), (100.0, 0.0), obstacles, **common)
        zero_penalty = plan_path(
            (0.0, -30.0), (100.0, 0.0), obstacles,
            turn_switch_penalty_m=0.0,
            turn_angle_weight_m=0.0,
            **common,
        )

        self.assertIsNotNone(legacy)
        self.assertEqual(zero_penalty, legacy)

    def test_turn_switch_penalty_reduces_direction_switches(self) -> None:
        obstacles = _gapped_wall_obstacles(-30.0)
        common = dict(
            resolution_m=10.0,
            clearance_m=0.0,
            bounds=(-10.0, -60.0, 110.0, 60.0),
        )
        baseline = plan_path((0.0, -30.0), (100.0, 0.0), obstacles, **common)
        penalized = plan_path(
            (0.0, -30.0), (100.0, 0.0), obstacles,
            turn_switch_penalty_m=50.0,
            turn_angle_weight_m=0.0,
            **common,
        )

        self.assertIsNotNone(baseline)
        self.assertIsNotNone(penalized)
        self.assertLess(_direction_metrics(penalized)[0], _direction_metrics(baseline)[0])
        self._assert_collision_free(penalized, obstacles, 0.0)

    def test_turn_angle_weight_reduces_total_heading_change(self) -> None:
        obstacles = _gapped_wall_obstacles(-30.0)
        common = dict(
            resolution_m=10.0,
            clearance_m=0.0,
            bounds=(-10.0, -60.0, 110.0, 60.0),
        )
        baseline = plan_path((0.0, -30.0), (100.0, 0.0), obstacles, **common)
        penalized = plan_path(
            (0.0, -30.0), (100.0, 0.0), obstacles,
            turn_switch_penalty_m=0.0,
            turn_angle_weight_m=50.0,
            **common,
        )

        self.assertIsNotNone(baseline)
        self.assertIsNotNone(penalized)
        self.assertLess(_direction_metrics(penalized)[1], _direction_metrics(baseline)[1])
        self._assert_collision_free(penalized, obstacles, 0.0)

    def test_invalid_resolution_raises(self) -> None:
        with self.assertRaises(ValueError):
            plan_path((0.0, 0.0), (10.0, 0.0), [], resolution_m=0.0)

    def test_grid_too_large_raises(self) -> None:
        with self.assertRaises(ValueError):
            plan_path((0.0, 0.0), (10.0, 0.0), [], resolution_m=1.0, bounds=(0.0, 0.0, 3000.0, 3000.0))

    def test_compute_bounds_covers_endpoints_and_obstacle(self) -> None:
        obstacles = [make_circle("C", 500.0, 0.0, 150.0)]
        min_e, min_n, max_e, max_n = compute_bounds((0.0, 0.0), (1000.0, 0.0), obstacles, clearance=50.0, pad=10.0)
        self.assertLessEqual(min_e, 0.0)
        self.assertGreaterEqual(max_e, 1000.0)
        self.assertLessEqual(min_n, -(150.0 + 50.0))
        self.assertGreaterEqual(max_n, 150.0 + 50.0)


if __name__ == "__main__":
    unittest.main()
