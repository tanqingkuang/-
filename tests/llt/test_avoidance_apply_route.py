"""Tests for adopting an avoidance route into the controller (apply/clear/override)."""

from __future__ import annotations

import unittest
from pathlib import Path

from src.algorithm.context.leaf_types import RouteS
from src.algorithm.units.process.tra_plan.avoidance.path_to_route import points_to_route
from src.runner.sim_control import SimulationController

CONFIG = str(Path(__file__).resolve().parents[2] / "configs" / "base.json")


def _sample_route() -> RouteS:
    # 一条带一个拐点的简单航线（直线+圆弧+直线）。
    return points_to_route(
        [(0.0, 0.0), (1000.0, 0.0), (1000.0, 1000.0)], turn_radius_m=150.0, speed_mps=20.0, altitude_m=1000.0
    )


def _route_end(controller: SimulationController) -> tuple[float, float]:
    pos = controller._leader_route.lines[-1].end.pos
    return (round(pos.east, 3), round(pos.north, 3))


# 原 base.json 航线终点 (2000,2000)；样例航线终点 (1000,1000)。
ORIGINAL_END = (2000.0, 2000.0)
SAMPLE_END = (1000.0, 1000.0)


class ApplyAvoidanceRouteTests(unittest.TestCase):
    def setUp(self) -> None:
        self.controller = SimulationController()
        self.addCleanup(self.controller.close)
        self.assertEqual(self.controller.load_config(CONFIG).code, "OK")
        self.assertEqual(_route_end(self.controller), ORIGINAL_END)

    def test_apply_replaces_leader_route(self) -> None:
        route = _sample_route()
        result = self.controller.apply_avoidance_route(route)
        self.assertEqual(result.code, "OK")
        self.assertEqual(len(self.controller._leader_route.lines), len(route.lines))
        self.assertEqual(_route_end(self.controller), SAMPLE_END)
        self.assertEqual(self.controller.get_snapshot().run_state, "READY")

    def test_applied_route_survives_reset(self) -> None:
        self.controller.apply_avoidance_route(_sample_route())
        self.assertEqual(self.controller.reset().code, "OK")
        self.assertEqual(_route_end(self.controller), SAMPLE_END)

    def test_clear_reverts_to_config_route(self) -> None:
        self.controller.apply_avoidance_route(_sample_route())
        self.assertEqual(self.controller.clear_avoidance_route().code, "OK")
        self.assertEqual(_route_end(self.controller), ORIGINAL_END)

    def test_reload_config_clears_override(self) -> None:
        self.controller.apply_avoidance_route(_sample_route())
        self.assertEqual(self.controller.load_config(CONFIG).code, "OK")
        self.assertEqual(_route_end(self.controller), ORIGINAL_END)

    def test_apply_while_running_is_rejected(self) -> None:
        self.controller.start()
        result = self.controller.apply_avoidance_route(_sample_route())
        self.assertEqual(result.code, "ERR_BUSY")

    def test_apply_empty_route_rejected(self) -> None:
        self.assertEqual(self.controller.apply_avoidance_route(RouteS(lines=[])).code, "ERR_CONFIG_INVALID")

    def test_apply_runs_after_adopt(self) -> None:
        self.controller.apply_avoidance_route(_sample_route())
        self.assertEqual(self.controller.start().code, "OK")
        self.controller.step(3)
        self.assertIn(self.controller.get_snapshot().run_state, {"RUNNING", "PAUSED"})


class ApplyWithoutConfigTests(unittest.TestCase):
    def test_apply_before_config_rejected(self) -> None:
        controller = SimulationController()
        self.addCleanup(controller.close)
        self.assertEqual(controller.apply_avoidance_route(_sample_route()).code, "ERR_NO_CONFIG")


if __name__ == "__main__":
    unittest.main()
