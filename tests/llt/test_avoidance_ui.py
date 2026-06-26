"""Tests for the avoidance UI integration: param parsing, polyline, generate/adopt flow."""

from __future__ import annotations

import json
import os
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from src.algorithm.units.process.tra_plan.avoidance.path_to_route import points_to_route
from src.ui.gui.main_window import (
    MainWindow,
    parse_avoidance_params,
    route_to_polyline,
)

CONFIG = str(Path(__file__).resolve().parents[2] / "configs" / "base.json")


class ParseAvoidanceParamsTests(unittest.TestCase):
    def _write(self, payload: object) -> str:
        handle = tempfile.NamedTemporaryFile(mode="w", suffix=".json", encoding="utf-8", delete=False)
        json.dump(payload, handle)
        handle.close()
        self.addCleanup(lambda: Path(handle.name).unlink(missing_ok=True))
        return handle.name

    def test_base_config_parsed(self) -> None:
        params = parse_avoidance_params(CONFIG)
        self.assertIsNotNone(params)
        self.assertEqual(len(params.waypoints), 3)
        self.assertEqual(params.waypoints[0], (0.0, 0.0, 1000.0))
        self.assertGreater(params.turn_radius_m, 0.0)

    def test_allow_arc_defaults_true_when_absent(self) -> None:
        path = self._write({"avoidance": {"enabled": True}, "route": {"waypoints": [
            {"x_m": 0, "y_m": 0}, {"x_m": 1000, "y_m": 0}]}})
        params = parse_avoidance_params(path)
        self.assertIsNotNone(params)
        self.assertTrue(params.allow_arc)

    def test_allow_arc_false_parsed(self) -> None:
        path = self._write({"avoidance": {"enabled": True, "allow_arc": False}, "route": {"waypoints": [
            {"x_m": 0, "y_m": 0}, {"x_m": 1000, "y_m": 0}]}})
        params = parse_avoidance_params(path)
        self.assertIsNotNone(params)
        self.assertFalse(params.allow_arc)

    def test_simplify_clearance_parsed(self) -> None:
        path = self._write({"avoidance": {"enabled": True, "simplify_clearance_m": 12.5}, "route": {"waypoints": [
            {"x_m": 0, "y_m": 0}, {"x_m": 1000, "y_m": 0}]}})
        params = parse_avoidance_params(path)
        self.assertIsNotNone(params)
        self.assertAlmostEqual(params.simplify_clearance_m, 12.5)
        self.assertTrue(params.simplify_clearance_explicit)

    def test_simplify_clearance_defaults_to_clearance_when_absent(self) -> None:
        path = self._write({"avoidance": {"enabled": True, "clearance_m": 80.0}, "route": {"waypoints": [
            {"x_m": 0, "y_m": 0}, {"x_m": 1000, "y_m": 0}]}})
        params = parse_avoidance_params(path)
        self.assertIsNotNone(params)
        self.assertAlmostEqual(params.simplify_clearance_m, 80.0)
        self.assertFalse(params.simplify_clearance_explicit)

    def test_heading_penalties_parsed(self) -> None:
        path = self._write({"avoidance": {
            "enabled": True,
            "turn_switch_penalty_m": 40.0,
            "turn_angle_weight_m": 20.0,
        }, "route": {"waypoints": [
            {"x_m": 0, "y_m": 0}, {"x_m": 1000, "y_m": 0}]}})
        params = parse_avoidance_params(path)
        self.assertIsNotNone(params)
        self.assertAlmostEqual(params.turn_switch_penalty_m, 40.0)
        self.assertAlmostEqual(params.turn_angle_weight_m, 20.0)

    def test_missing_avoidance_returns_none(self) -> None:
        self.assertIsNone(parse_avoidance_params(self._write({"route": {"waypoints": []}})))

    def test_disabled_returns_none(self) -> None:
        path = self._write({"avoidance": {"enabled": False}, "route": {"waypoints": [
            {"x_m": 0, "y_m": 0}, {"x_m": 1, "y_m": 0}]}})
        self.assertIsNone(parse_avoidance_params(path))

    def test_too_few_waypoints_returns_none(self) -> None:
        path = self._write({"avoidance": {"enabled": True}, "route": {"waypoints": [{"x_m": 0, "y_m": 0}]}})
        self.assertIsNone(parse_avoidance_params(path))

    def test_east_north_aliases_parsed(self) -> None:
        # 控制器兼容 east/north/h 字段名，UI 解析也应一致，否则会得到全零航点。
        path = self._write({"avoidance": {"enabled": True}, "route": {"waypoints": [
            {"east": 100.0, "north": 200.0, "h": 1000.0},
            {"east": 300.0, "north": 400.0, "h": 1200.0},
        ]}})
        params = parse_avoidance_params(path)
        self.assertIsNotNone(params)
        self.assertEqual(params.waypoints[0], (100.0, 200.0, 1000.0))
        self.assertEqual(params.waypoints[1], (300.0, 400.0, 1200.0))


class RouteToPolylineTests(unittest.TestCase):
    def test_straight_route(self) -> None:
        route = points_to_route([(0.0, 0.0), (100.0, 0.0)], turn_radius_m=0.0, speed_mps=20.0)
        poly = route_to_polyline(route)
        self.assertEqual(poly[0], (0.0, 0.0))
        self.assertEqual(poly[-1], (100.0, 0.0))

    def test_arc_route_is_sampled(self) -> None:
        route = points_to_route([(0.0, 0.0), (1000.0, 0.0), (1000.0, 1000.0)], turn_radius_m=200.0, speed_mps=20.0)
        poly = route_to_polyline(route)
        # 圆弧被采样为多点，折线点数应明显多于航段数。
        self.assertGreater(len(poly), len(route.lines))


class AvoidanceUiFlowTests(unittest.TestCase):
    app: QApplication

    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def _write(self, payload: object) -> str:
        handle = tempfile.NamedTemporaryFile(mode="w", suffix=".json", encoding="utf-8", delete=False)
        json.dump(payload, handle)
        handle.close()
        self.addCleanup(lambda: Path(handle.name).unlink(missing_ok=True))
        return handle.name

    def _window(self) -> MainWindow:
        window = MainWindow(auto_load_config=False)
        self.addCleanup(window.close)
        window._apply_config_path(CONFIG)
        return window

    @staticmethod
    def _set_feasible_params(window: MainWindow) -> None:
        # 用一组已知可飞的参数覆盖控件值，使“生成成功”相关用例不依赖 base.json 的具体 R/L。
        window.turn_radius_spin.setValue(150.0)
        window.clearance_spin.setValue(120.0)
        window.leg_margin_spin.setValue(50.0)
        window.allow_arc_check.setChecked(True)

    def test_generate_then_adopt_replaces_route(self) -> None:
        window = self._window()
        self._set_feasible_params(window)
        original = len(window.sim.controller._leader_route.lines)
        self.assertFalse(window.adopt_route_button.isEnabled())
        window._generate_route()
        self.assertIsNotNone(window._preview_route)
        self.assertTrue(window.adopt_route_button.isEnabled())
        self.assertIsNotNone(window.top_view.preview_route_polyline)
        window._adopt_route()
        self.assertEqual(window.sim.last_result_code, "OK")
        self.assertNotEqual(len(window.sim.controller._leader_route.lines), original)

    def test_toggle_obstacle_invalidates_preview(self) -> None:
        window = self._window()
        self._set_feasible_params(window)
        window._generate_route()
        self.assertIsNotNone(window._preview_route)
        window._on_obstacle_toggled(window.obstacles[0], not window.obstacles[0].enabled)
        self.assertIsNone(window._preview_route)
        self.assertIsNone(window.top_view.preview_route_polyline)
        self.assertFalse(window.adopt_route_button.isEnabled())

    def test_no_enabled_obstacles_skips_generation(self) -> None:
        # 取消勾选所有障碍后生成航线应维持原状：无预览、采用按钮禁用。
        window = self._window()
        for obstacle in window.obstacles:
            if obstacle.enabled:
                window._on_obstacle_toggled(obstacle, False)
        window._generate_route()
        self.assertIsNone(window._preview_route)
        self.assertIsNone(window.top_view.preview_route_polyline)
        self.assertFalse(window.adopt_route_button.isEnabled())
        self.assertIn("未选择障碍", window.avoidance_status.text())

    def test_param_widgets_synced_from_config(self) -> None:
        # 加载配置后参数控件应反映 base.json 的值。
        window = self._window()
        params = window._avoidance_params
        self.assertAlmostEqual(window.turn_radius_spin.value(), params.turn_radius_m)
        self.assertAlmostEqual(window.clearance_spin.value(), params.clearance_m)
        self.assertAlmostEqual(window.leg_margin_spin.value(), params.leg_margin_m)
        self.assertEqual(window.allow_arc_check.isChecked(), params.allow_arc)

    def test_changing_param_invalidates_preview(self) -> None:
        window = self._window()
        self._set_feasible_params(window)
        window._generate_route()
        self.assertIsNotNone(window._preview_route)
        window.turn_radius_spin.setValue(window.turn_radius_spin.value() + 50.0)
        self.assertIsNone(window._preview_route)
        self.assertFalse(window.adopt_route_button.isEnabled())

    def test_allow_arc_unchecked_generates_straight_only(self) -> None:
        # 取消“航段带圆弧”后生成的预览应无圆弧段（外切线交付）。
        window = self._window()
        self._set_feasible_params(window)
        window.allow_arc_check.setChecked(False)
        window._generate_route()
        self.assertIsNotNone(window._preview_route)
        self.assertTrue(all(line.radius == 0.0 for line in window._preview_route.lines))

    def test_widget_value_overrides_config_at_generate(self) -> None:
        # 界面调大 L 到不可飞 → 生成失败，证明用的是控件值而非配置值。
        window = self._window()
        window.leg_margin_spin.setValue(5000.0)
        window._generate_route()
        self.assertIsNone(window._preview_route)
        self.assertIn("ERR_AVOID", window.avoidance_status.text())

    def test_generate_uses_current_clearance_for_implicit_simplify_clearance(self) -> None:
        # 旧配置没有 simplify_clearance_m 时，界面调整安全间距后，去冗余安全间距应同步跟随。
        path = self._write({
            "route": {
                "speed_mps": 20.0,
                "waypoints": [{"x_m": 0, "y_m": 0}, {"x_m": 1000, "y_m": 0}],
            },
            "avoidance": {
                "enabled": True,
                "clearance_m": 80.0,
                "turn_radius_m": 150.0,
                "leg_length_margin_m": 50.0,
                "grid": {"resolution_m": 20.0, "margin_m": 100.0},
                "obstacles": [{
                    "id": "C1",
                    "type": "circle",
                    "enabled": True,
                    "center": {"east_m": 500.0, "north_m": 0.0},
                    "radius_m": 120.0,
                }],
            },
        })
        window = MainWindow(auto_load_config=False)
        self.addCleanup(window.close)
        window._apply_config_path(path)
        window.clearance_spin.setValue(110.0)

        with patch(
            "src.ui.gui.main_window.plan_avoidance_route",
            return_value=SimpleNamespace(ok=False, route=None, code="ERR_TEST", detail="captured"),
        ) as planner:
            window._generate_route()

        self.assertAlmostEqual(planner.call_args.kwargs["clearance_m"], 110.0)
        self.assertAlmostEqual(planner.call_args.kwargs["simplify_clearance_m"], 110.0)

    def test_adopt_without_preview_is_noop(self) -> None:
        window = self._window()
        original = len(window.sim.controller._leader_route.lines)
        window._adopt_route()  # 无预览
        self.assertEqual(len(window.sim.controller._leader_route.lines), original)


if __name__ == "__main__":
    unittest.main()
