"""Tests for adopting an avoidance route into the controller (apply/clear/override)."""

from __future__ import annotations

import unittest
from pathlib import Path

from src.algorithm.context.leaf_types import PosInEarthS, WayPointInputS
from src.algorithm.units.process.tra_plan.avoidance.path_to_route import (
    assign_transition_radius,
    bake_transition_arcs,
    points_to_route,
)
from src.runner.sim_control import SimulationController

CONFIG = str(Path(__file__).resolve().parent / "fixtures" / "test.json")


def _sample_route() -> list[WayPointInputS]:
    # 一条带一个拐点的简单航线。
    return points_to_route(
        [(0.0, 0.0), (1000.0, 0.0), (1000.0, 1000.0)], speed_mps=20.0, altitude_m=1000.0
    )


def _route_end(controller: SimulationController) -> tuple[float, float]:
    route = controller._leader_route
    assert route is not None
    pos = route[-1].pos
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
        self.assertEqual(len(self.controller._leader_route), len(route))
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
        self.assertEqual(self.controller.apply_avoidance_route([]).code, "ERR_CONFIG_INVALID")

    def test_apply_single_point_rejected(self) -> None:
        single = [WayPointInputS(idx=0, pos=PosInEarthS(0.0, 0.0, 1000.0), vdCmd=8.0)]
        self.assertEqual(self.controller.apply_avoidance_route(single).code, "ERR_CONFIG_INVALID")

    def test_adopted_route_display_has_no_transition_arcs(self) -> None:
        # 显示航线只画航段几何(直线)，交接圆弧(r)是转弯信息不画 → _display_route 全是直线段；
        # 但飞行航线(_leader_route)仍带 r，长机照样平滑过弯。
        route = points_to_route([(0.0, 0.0), (1000.0, 0.0), (1000.0, 1000.0)], speed_mps=20.0, altitude_m=1000.0)
        assign_transition_radius(route, 300.0)
        self.assertEqual(self.controller.apply_avoidance_route(route).code, "OK")
        display = self.controller._display_route
        assert display is not None
        self.assertTrue(all(line.start.turnSign == 0.0 for line in display))
        self.assertTrue(any(wpi.r > 0.0 for wpi in self.controller._leader_route))

    def test_adopted_baked_arc_route_carries_radius_to_snapshot(self) -> None:
        # allow_arc=True 烘焙的圆弧航段(turnSign!=0)采用后，committed 快照航段须带 radius_m，
        # 俯视图才能按弧采样(而非切弦)，与预览一致。
        route = points_to_route([(0.0, 0.0), (1000.0, 0.0), (1000.0, 1000.0)], speed_mps=20.0, altitude_m=1000.0)
        assign_transition_radius(route, 200.0)
        route = bake_transition_arcs(route)
        self.assertEqual(self.controller.apply_avoidance_route(route).code, "OK")
        segments = self.controller.get_snapshot().route_segments
        self.assertTrue(any(seg.radius_m > 0.0 for seg in segments))

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
