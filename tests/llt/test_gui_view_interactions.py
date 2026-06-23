"""Regression tests for PySide6 realtime view interactions."""

from __future__ import annotations

import json
import os
import sys
import tempfile
import time
import unittest
from configparser import ConfigParser
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QPointF, QRect
from PySide6.QtWidgets import QApplication

from src.ui.gui.main_window import (
    ControllerSimulationAdapter,
    MainWindow,
    LinkState,
    NodeState,
    Snapshot,
    TOP_VIEW_ORIGIN_MARGIN,
    default_project_root,
)


class GuiViewInteractionTests(unittest.TestCase):
    """Exercise view-state synchronization without starting the Qt event loop."""

    app: QApplication

    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self) -> None:
        self.window = MainWindow(auto_load_config=False)
        self.window.resize(1440, 900)
        self.window.show()
        self.app.processEvents()

    def tearDown(self) -> None:
        self.window.close()
        self.app.processEvents()

    def test_stage_fullscreen_reparents_only_realtime_display(self) -> None:
        self.window._enter_stage_fullscreen()
        self.app.processEvents()

        self.assertFalse(self.window.isFullScreen())
        self.assertIsNotNone(self.window._stage_fullscreen_dialog)
        self.assertIs(self.window.stage.parentWidget(), self.window._stage_fullscreen_dialog)
        self.assertEqual(self.window.fullscreen_button.text(), "↙")

        self.window._exit_stage_fullscreen()
        self.app.processEvents()

        self.assertIsNone(self.window._stage_fullscreen_dialog)
        self.assertEqual(self.window.main_layout.indexOf(self.window.stage), 1)
        self.assertEqual(self.window.fullscreen_button.text(), "⛶")

    def test_grid_toggle_controls_top_and_side_views(self) -> None:
        self.assertTrue(self.window.grid_toggle.isChecked())
        self.assertTrue(self.window.top_view.show_grid)
        self.assertTrue(self.window.side_view.show_grid)

        self.window.grid_toggle.setChecked(False)
        self.app.processEvents()

        self.assertFalse(self.window.top_view.show_grid)
        self.assertFalse(self.window.side_view.show_grid)

    def test_side_grid_uses_shared_world_x_mapping(self) -> None:
        self.window.side_view.snapshot = None
        self.window.top_view.offset = QPointF(73.0, 0.0)
        self.window.top_view.scale_value = 1.0
        self.window.side_view.update()
        self.app.processEvents()

        image = self.window.side_view.grab().toImage()
        canvas = self.window.theme.canvas.name()
        grid_x = round(self.window.side_view._map_x(0.0))

        self.assertGreater(grid_x, 0)
        self.assertLess(grid_x, image.width())
        self.assertNotEqual(image.pixelColor(grid_x, 10).name(), canvas)

    def test_world_grid_keeps_readable_screen_spacing_during_zoom(self) -> None:
        for scale in (0.45, 1.0, 3.5):
            self.window.top_view.scale_value = scale

            screen_spacing = self.window.top_view._grid_world_spacing() * scale

            self.assertGreaterEqual(screen_spacing, 36.0)
            self.assertLessEqual(screen_spacing, 96.0)
            self.assertEqual(
                self.window.side_view._grid_world_spacing(),
                self.window.top_view._grid_world_spacing(),
            )

    def test_top_view_aircraft_marker_keeps_screen_size_during_zoom(self) -> None:
        small = self._leader_marker_bounds_at_scale(0.45)
        large = self._leader_marker_bounds_at_scale(3.5)

        self.assertAlmostEqual(small.width(), large.width(), delta=4)
        self.assertAlmostEqual(small.height(), large.height(), delta=4)
        self.assertLessEqual(large.width(), 14)
        self.assertLessEqual(large.height(), 10)

    def test_top_view_link_keeps_screen_width_during_zoom(self) -> None:
        thin = self._link_stroke_height_at_scale(0.45)
        thick = self._link_stroke_height_at_scale(3.5)

        self.assertAlmostEqual(thin, thick, delta=2)

    def test_top_view_reset_restores_side_altitude_axis(self) -> None:
        self._load_ui_config()
        self.window.side_view.altitude_min = 1180.0
        self.window.side_view.altitude_max = 1240.0

        self.window.top_view.reset_view()
        self.app.processEvents()

        self.assert_route_and_aircraft_fit_viewport()
        self.assertEqual(self.window.side_view.altitude_min, self.window.side_view.ALTITUDE_MIN_DEFAULT)
        self.assertEqual(self.window.side_view.altitude_max, self.window.side_view.ALTITUDE_MAX_DEFAULT)

    def test_route_and_aircraft_fit_centered_viewport_after_load(self) -> None:
        self._load_ui_config(
            nodes=[
                {"node_id": "A01", "role": "leader", "x_m": 900.0, "y_m": 300.0, "altitude_m": 1200.0},
                {"node_id": "A02", "role": "wingman", "x_m": 880.0, "y_m": 340.0, "altitude_m": 1215.0},
            ],
            links=[{"link_id": "A01-A02", "latency_ms": 18.0, "loss_rate": 0.01}],
            route={
                "speed_mps": 12.0,
                "waypoints": [
                    {"x_m": 0.0, "y_m": 0.0, "altitude_m": 1200.0},
                    {"x_m": 100.0, "y_m": 0.0, "altitude_m": 1200.0},
                ],
            },
        )

        self.assert_route_and_aircraft_fit_viewport()

    def test_top_view_does_not_refit_during_running_snapshot_updates(self) -> None:
        self._load_ui_config()
        view = self.window.top_view
        initial_scale = view.scale_value
        initial_offset = QPointF(view.offset)
        snapshot = view.snapshot
        self.assertIsNotNone(snapshot)
        assert snapshot is not None
        moved_nodes = [replace(node, x=node.x + 400.0, y=node.y + 120.0) for node in snapshot.nodes]
        running_snapshot = replace(
            snapshot,
            time=snapshot.time + snapshot.step,
            run_state="RUNNING",
            nodes=moved_nodes,
        )

        self.window._update_snapshot(running_snapshot)
        self.app.processEvents()

        self.assertAlmostEqual(view.scale_value, initial_scale)
        self.assertAlmostEqual(view.offset.x(), initial_offset.x(), delta=0.01)
        self.assertAlmostEqual(view.offset.y(), initial_offset.y(), delta=0.01)

    def test_reset_view_refits_route_and_aircraft_after_manual_view_change(self) -> None:
        self._load_ui_config(
            nodes=[
                {"node_id": "A01", "role": "leader", "x_m": 900.0, "y_m": 300.0, "altitude_m": 1200.0},
                {"node_id": "A02", "role": "wingman", "x_m": 880.0, "y_m": 340.0, "altitude_m": 1215.0},
            ],
            links=[{"link_id": "A01-A02", "latency_ms": 18.0, "loss_rate": 0.01}],
            route={
                "speed_mps": 12.0,
                "waypoints": [
                    {"x_m": 0.0, "y_m": 0.0, "altitude_m": 1200.0},
                    {"x_m": 100.0, "y_m": 0.0, "altitude_m": 1200.0},
                ],
            },
        )
        self.window.top_view.scale_value = 3.0
        self.window.top_view.offset = QPointF(-500.0, -400.0)

        self.window.top_view.reset_view()
        self.app.processEvents()

        self.assert_route_and_aircraft_fit_viewport()

    def test_route_endpoints_are_visible_after_load(self) -> None:
        self._load_ui_config()
        route = self.window.top_view.snapshot.route
        self.assertIsNotNone(route)
        assert route is not None

        start_x = route.start_x * self.window.top_view.scale_value + self.window.top_view.offset.x()
        start_y = route.start_y * self.window.top_view.scale_value + self.window.top_view.offset.y()
        end_x = route.end_x * self.window.top_view.scale_value + self.window.top_view.offset.x()
        end_y = route.end_y * self.window.top_view.scale_value + self.window.top_view.offset.y()
        rect = self.window.top_view.viewport().rect()

        self.assertGreaterEqual(start_x, 5.0)
        self.assertGreaterEqual(start_y, 5.0)
        self.assertLessEqual(end_x, rect.width() - 5.0)
        self.assertGreaterEqual(end_y, 5.0)

    def test_multi_segment_route_is_available_to_views_after_load(self) -> None:
        route_config = {
            "speed_mps": 12.0,
            "waypoints": [
                {"x_m": 0.0, "y_m": 0.0, "altitude_m": 1000.0},
                {"x_m": 6000.0, "y_m": 0.0, "altitude_m": 1000.0},
                {"x_m": 6000.0, "y_m": 3000.0, "altitude_m": 1000.0},
            ],
        }

        self._load_ui_config(route=route_config)
        snapshot = self.window.top_view.snapshot

        self.assertIsNotNone(snapshot)
        assert snapshot is not None
        self.assertEqual(len(snapshot.route_segments), 2)
        self.assertAlmostEqual(snapshot.route_segments[0].end_x, 6000.0)
        self.assertAlmostEqual(snapshot.route_segments[1].end_y, 3000.0)
        second_end_x = snapshot.route_segments[1].end_x * self.window.top_view.scale_value + self.window.top_view.offset.x()
        second_end_y = snapshot.route_segments[1].end_y * self.window.top_view.scale_value + self.window.top_view.offset.y()
        rect = self.window.top_view.viewport().rect()
        self.assertGreaterEqual(second_end_x, 5.0)
        self.assertGreaterEqual(second_end_y, 5.0)
        self.assertLessEqual(second_end_x, rect.width() - 5.0)
        self.assertLessEqual(second_end_y, rect.height() - 5.0)

    def test_side_height_selection_does_not_move_top_view_aircraft(self) -> None:
        old_offset = QPointF(self.window.top_view.offset)
        old_scale = self.window.top_view.scale_value
        old_span = self.window.side_view.altitude_max - self.window.side_view.altitude_min

        self.window.side_view._selection_origin = QPointF(100.0, 44.0)
        self.window.side_view._selection_current = QPointF(128.0, 84.0)
        self.window.side_view._zoom_to_selection()
        self.app.processEvents()

        new_span = self.window.side_view.altitude_max - self.window.side_view.altitude_min
        self.assertEqual(self.window.top_view.offset, old_offset)
        self.assertEqual(self.window.top_view.scale_value, old_scale)
        self.assertLess(new_span, old_span)

    def test_side_horizontal_selection_updates_shared_x_view_only(self) -> None:
        old_scale = self.window.top_view.scale_value
        old_center_y = (
            self.window.top_view.viewport().height() / 2.0 - self.window.top_view.offset.y()
        ) / old_scale

        self.window.side_view._selection_origin = QPointF(100.0, 44.0)
        self.window.side_view._selection_current = QPointF(520.0, 84.0)
        self.window.side_view._zoom_to_selection()
        self.app.processEvents()

        new_center_y = (
            self.window.top_view.viewport().height() / 2.0 - self.window.top_view.offset.y()
        ) / self.window.top_view.scale_value
        self.assertGreater(self.window.top_view.scale_value, old_scale)
        self.assertAlmostEqual(new_center_y, old_center_y)

    def test_main_window_uses_simulation_controller_adapter(self) -> None:
        self.assertIsInstance(self.window.sim, ControllerSimulationAdapter)
        self.assertEqual(self.window.sim.controller.get_snapshot().run_state, "UNLOADED")
        self.assertEqual(self.window.node_table.rowCount(), 0)
        self.assertEqual(self.window.link_table.rowCount(), 0)
        self.assertFalse(self.window.start_button.isEnabled())
        self.assertFalse(self.window.pause_button.isEnabled())
        self.assertFalse(self.window.step_button.isEnabled())
        self.assertFalse(self.window.reset_button.isEnabled())
        self.assertTrue(all(not button.isEnabled() for button in self.window.disturbance_buttons))

    def test_unloaded_window_does_not_draw_reference_route(self) -> None:
        route_color = self.window.theme.route.name()

        self.assertEqual(self._count_pixels(self.window.top_view, route_color), 0)
        self.assertEqual(self._count_pixels(self.window.side_view, route_color), 0)

    def test_start_pause_drives_real_controller_snapshot(self) -> None:
        self._load_ui_config()

        self.window._start()
        running_snapshot = self._wait_for_controller_time()

        self.assertEqual(running_snapshot.run_state, "RUNNING")
        self.assertGreater(running_snapshot.time_s, 0.0)

        self.window._pause()
        paused_snapshot = self.window.sim.controller.get_snapshot()

        self.assertEqual(paused_snapshot.run_state, "PAUSED")

    def test_disturbance_label_clears_after_duration(self) -> None:
        self._load_ui_config()

        self.window.sim.controller.inject_disturbance(
            {"type": "link_loss", "target": "A01-A02", "duration_s": 0.005, "params": {"loss_rate": 0.3}}
        )
        disturbed = self.window.sim.snapshot()

        self.assertEqual(disturbed.disturbance, "链路丢包")

        self.window.sim.controller.step(3)
        cleared = self.window.sim.snapshot()

        self.assertEqual(cleared.disturbance, "无")

    def test_config_load_failure_is_reported_without_replacing_label(self) -> None:
        old_label = self.window.config_name.text()
        with tempfile.TemporaryDirectory() as tmp:
            bad_config = Path(tmp) / "bad.json"
            bad_config.write_text("{", encoding="utf-8")

            self.window._apply_config_path(str(bad_config))

        self.assertEqual(self.window.config_name.text(), old_label)
        self.assertIn("加载配置失败", self.window.log_dialog.text.toPlainText())

    def test_startup_loads_last_relative_config_from_ini(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            project_root = Path(tmp)
            config_path = self._write_config_file(project_root / "configs" / "startup.json")
            state_path = project_root / "config.ini"
            state_path.write_text("[config]\nlast_config = configs/startup.json\n", encoding="utf-8")

            window = MainWindow(project_root=project_root, config_state_path=state_path)
            window.show()
            self.app.processEvents()
            try:
                self.assertEqual(window.sim.controller.get_snapshot().run_state, "READY")
                self.assertEqual(window.config_name.text(), "configs/startup.json")
                self.assertEqual(window.node_table.rowCount(), 3)
                initial_scale = window.top_view.scale_value
                initial_offset = QPointF(window.top_view.offset)

                window.top_view.reset_view()
                self.app.processEvents()

                self.assertAlmostEqual(window.top_view.scale_value, initial_scale)
                self.assertAlmostEqual(window.top_view.offset.x(), initial_offset.x(), delta=1.0)
                self.assertAlmostEqual(window.top_view.offset.y(), initial_offset.y(), delta=1.0)
            finally:
                window.close()
                self.app.processEvents()

    def test_packaged_startup_loads_config_ini_next_to_exe(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            app_dir = Path(tmp)
            config_path = self._write_config_file(app_dir / "configs" / "packaged.json")
            state_path = app_dir / "config.ini"
            state_path.write_text("[config]\nlast_config = configs/packaged.json\n", encoding="utf-8")

            with (
                patch.object(sys, "frozen", True, create=True),
                patch.object(sys, "executable", str(app_dir / "编队仿真.exe")),
            ):
                self.assertEqual(default_project_root(), app_dir)
                window = MainWindow()
            window.show()
            self.app.processEvents()
            try:
                self.assertEqual(window.config_state_path, state_path)
                self.assertEqual(window.sim.controller.get_snapshot().run_state, "READY")
                self.assertEqual(window.config_name.text(), "configs/packaged.json")
            finally:
                window.close()
                self.app.processEvents()

    def test_packaged_config_write_is_relative_to_exe_directory(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            app_dir = Path(tmp)
            selected_config = self._write_config_file(app_dir / "configs" / "selected.json")
            state_path = app_dir / "config.ini"

            with (
                patch.object(sys, "frozen", True, create=True),
                patch.object(sys, "executable", str(app_dir / "编队仿真.exe")),
            ):
                window = MainWindow(auto_load_config=False)
            window._apply_config_path(str(selected_config))
            try:
                parser = ConfigParser()
                parser.read(state_path, encoding="utf-8")
                saved_path = parser["config"]["last_config"]

                self.assertEqual(window.config_state_path, state_path)
                self.assertEqual(saved_path, "configs/selected.json")
                self.assertFalse(Path(saved_path).is_absolute())
                self.assertNotIn(str(app_dir), saved_path)
                self.assertEqual(window.config_name.text(), "configs/selected.json")
            finally:
                window.close()
                self.app.processEvents()

    def test_packaged_config_write_can_reference_config_next_to_exe_parent(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            app_dir = root / "dist"
            app_dir.mkdir()
            selected_config = self._write_config_file(root / "configs" / "selected.json")
            state_path = app_dir / "config.ini"

            with (
                patch.object(sys, "frozen", True, create=True),
                patch.object(sys, "executable", str(app_dir / "编队仿真.exe")),
            ):
                window = MainWindow(auto_load_config=False)
            window._apply_config_path(str(selected_config))
            try:
                parser = ConfigParser()
                parser.read(state_path, encoding="utf-8")
                saved_path = parser["config"]["last_config"]

                self.assertEqual(saved_path, "../configs/selected.json")
                self.assertFalse(Path(saved_path).is_absolute())
            finally:
                window.close()
                self.app.processEvents()

            with (
                patch.object(sys, "frozen", True, create=True),
                patch.object(sys, "executable", str(app_dir / "编队仿真.exe")),
            ):
                reloaded = MainWindow()
            try:
                self.assertEqual(reloaded.sim.controller.get_snapshot().run_state, "READY")
                self.assertEqual(reloaded.config_name.text(), "../configs/selected.json")
            finally:
                reloaded.close()
                self.app.processEvents()

    def test_choose_config_starts_from_last_config_directory(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            project_root = Path(tmp)
            self._write_config_file(project_root / "configs" / "last.json")
            new_config = self._write_config_file(project_root / "configs" / "new.json")
            state_path = project_root / "config.ini"
            state_path.write_text("[config]\nlast_config = configs/last.json\n", encoding="utf-8")

            window = MainWindow(
                project_root=project_root,
                config_state_path=state_path,
                auto_load_config=False,
            )
            try:
                with patch(
                    "src.ui.gui.main_window.QFileDialog.getOpenFileName",
                    return_value=(str(new_config), "Config (*.json)"),
                ) as get_open_file_name:
                    window._choose_config()

                args = get_open_file_name.call_args.args
                self.assertEqual(Path(args[2]), (project_root / "configs").resolve())
                self.assertEqual(window.config_name.text(), "configs/new.json")
            finally:
                window.close()
                self.app.processEvents()

    def test_successful_config_load_updates_ini_with_relative_path(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            project_root = Path(tmp)
            first_config = self._write_config_file(project_root / "configs" / "first.json")
            second_config = self._write_config_file(project_root / "configs" / "second.json")
            state_path = project_root / "config.ini"
            state_path.write_text("[config]\nlast_config = configs/first.json\n", encoding="utf-8")

            window = MainWindow(
                project_root=project_root,
                config_state_path=state_path,
                auto_load_config=False,
            )
            window._apply_config_path(str(second_config))
            try:
                parser = ConfigParser()
                parser.read(state_path, encoding="utf-8")
                saved_path = parser["config"]["last_config"]

                self.assertEqual(saved_path, "configs/second.json")
                self.assertFalse(Path(saved_path).is_absolute())
                self.assertNotIn(str(project_root), saved_path)
                self.assertEqual(window.config_name.text(), "configs/second.json")
            finally:
                window.close()
                self.app.processEvents()

    def test_failed_config_load_does_not_update_ini(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            project_root = Path(tmp)
            self._write_config_file(project_root / "configs" / "first.json")
            bad_config = project_root / "configs" / "bad.json"
            bad_config.write_text("{", encoding="utf-8")
            state_path = project_root / "config.ini"
            state_path.write_text("[config]\nlast_config = configs/first.json\n", encoding="utf-8")

            window = MainWindow(
                project_root=project_root,
                config_state_path=state_path,
                auto_load_config=False,
            )
            window._apply_config_path(str(bad_config))
            try:
                parser = ConfigParser()
                parser.read(state_path, encoding="utf-8")

                self.assertEqual(parser["config"]["last_config"], "configs/first.json")
                self.assertIn("加载配置失败", window.log_dialog.text.toPlainText())
            finally:
                window.close()
                self.app.processEvents()

    def test_node_health_drives_table_status_and_warning_color_target(self) -> None:
        self._load_ui_config()

        self.window.sim.controller.inject_disturbance(
            {"type": "node_fault", "target": "A03", "duration_s": 1.0, "params": {"mode": "fault"}}
        )
        self.window._update_snapshot(self.window.sim.snapshot())

        statuses = {
            self.window.node_table.item(row, 0).text(): self.window.node_table.item(row, 5).text()
            for row in range(self.window.node_table.rowCount())
        }

        self.assertEqual(statuses["A03"], "故障")
        self.assertEqual(statuses["A02"], "正常")

    def test_link_table_displays_direction_from_controller_snapshot(self) -> None:
        self._load_ui_config(
            links=[
                {"link_id": "A01-A02", "direction": "duplex", "latency_ms": 18.0, "loss_rate": 0.01},
                {"link_id": "A02-A03", "direction": "simplex", "latency_ms": 30.0, "loss_rate": 0.02},
            ]
        )

        directions = {
            self.window.link_table.item(row, 0).text(): self.window.link_table.item(row, 1).text()
            for row in range(self.window.link_table.rowCount())
        }

        self.assertEqual(self.window.link_table.columnCount(), 5)
        self.assertEqual(self.window.link_table.horizontalHeaderItem(1).text(), "方向")
        self.assertEqual(directions["A01-A02"], "双向")
        self.assertEqual(directions["A02-A03"], "单向")
        self.assertEqual(self.window.link_table.horizontalScrollBar().maximum(), 0)

    def test_duration_input_syncs_loaded_config_duration(self) -> None:
        self._load_ui_config(duration_s=2400.0)

        self.assertEqual(self.window.duration_input.text(), "2400")
        self.assertEqual(self.window.timeline_label.text(), "0.0 / 2400s")

    def test_duration_input_updates_controller_duration_on_edit_finished(self) -> None:
        self._load_ui_config(duration_s=2400.0)

        self.window.duration_input.setText("120")
        self.window.duration_input.editingFinished.emit()
        self.app.processEvents()
        snapshot = self.window.sim.controller.get_snapshot()

        self.assertAlmostEqual(snapshot.duration_s, 120.0)
        self.assertEqual(self.window.timeline_label.text(), "0.0 / 120s")

    def test_duration_input_persists_json_config_duration(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            config_path = self._write_config_file(Path(tmp) / "case.json")
            self.window._apply_config_path(str(config_path))

            self.window.duration_input.setText("1200")
            self.window.duration_input.editingFinished.emit()
            self.app.processEvents()
            config = json.loads(config_path.read_text(encoding="utf-8"))

            self.assertAlmostEqual(config["duration_s"], 1200.0)

    def _load_ui_config(
        self,
        *,
        duration_s: float = 0.05,
        step_s: float = 0.005,
        nodes: list[dict[str, object]] | None = None,
        links: list[dict[str, object]] | None = None,
        route: dict[str, object] | None = None,
    ) -> None:
        config = {
            "duration_s": duration_s,
            "step_s": step_s,
            "playback_rate": 10.0,
            "nodes": nodes or [
                {"node_id": "A01", "role": "leader", "x_m": 140.0, "y_m": 260.0, "altitude_m": 1200.0},
                {"node_id": "A02", "role": "wingman", "x_m": 92.0, "y_m": 318.0, "altitude_m": 1215.0},
                {"node_id": "A03", "role": "wingman", "x_m": 88.0, "y_m": 202.0, "altitude_m": 1230.0},
            ],
            "links": links or [
                {"link_id": "A01-A02", "latency_ms": 18.0, "loss_rate": 0.01},
                {"link_id": "A01-A03", "latency_ms": 21.0, "loss_rate": 0.01},
                {"link_id": "A02-A03", "latency_ms": 30.0, "loss_rate": 0.02},
            ],
        }
        if route is not None:
            config["route"] = route
        with tempfile.NamedTemporaryFile("w", suffix=".json", encoding="utf-8", delete=False) as handle:
            json.dump(config, handle)
            config_path = handle.name
        try:
            self.window._apply_config_path(config_path)
            self.app.processEvents()
        finally:
            Path(config_path).unlink(missing_ok=True)

    def assert_route_and_aircraft_fit_viewport(self) -> None:
        snapshot = self.window.top_view.snapshot
        self.assertIsNotNone(snapshot)
        assert snapshot is not None
        xs = [node.x for node in snapshot.nodes]
        ys = [node.y for node in snapshot.nodes]
        for route in snapshot.route_segments or ([snapshot.route] if snapshot.route is not None else []):
            xs.extend([route.start_x, route.end_x])
            ys.extend([route.start_y, route.end_y])
        self.assertTrue(xs)
        self.assertTrue(ys)

        view = self.window.top_view
        rect = view.viewport().rect()
        left = min(xs) * view.scale_value + view.offset.x()
        right = max(xs) * view.scale_value + view.offset.x()
        top = min(ys) * view.scale_value + view.offset.y()
        bottom = max(ys) * view.scale_value + view.offset.y()
        fitted_width = right - left
        fitted_height = bottom - top

        self.assertAlmostEqual((left + right) / 2.0, rect.width() / 2.0, delta=1.0)
        self.assertAlmostEqual((top + bottom) / 2.0, rect.height() / 2.0, delta=1.0)
        self.assertLessEqual(fitted_width, rect.width() * 0.80 + 1.0)
        self.assertLessEqual(fitted_height, rect.height() * 0.80 + 1.0)
        self.assertTrue(
            abs(fitted_width - rect.width() * 0.80) <= 1.0
            or abs(fitted_height - rect.height() * 0.80) <= 1.0
        )

    def _write_config_file(self, path: Path) -> Path:
        path.parent.mkdir(parents=True, exist_ok=True)
        config = {
            "duration_s": 0.05,
            "step_s": 0.005,
            "playback_rate": 10.0,
            "nodes": [
                {"node_id": "A01", "role": "leader", "x_m": 140.0, "y_m": 260.0, "altitude_m": 1200.0},
                {"node_id": "A02", "role": "wingman", "x_m": 92.0, "y_m": 318.0, "altitude_m": 1215.0},
                {"node_id": "A03", "role": "wingman", "x_m": 88.0, "y_m": 202.0, "altitude_m": 1230.0},
            ],
            "links": [
                {"link_id": "A01-A02", "direction": "duplex", "latency_ms": 18.0, "loss_rate": 0.01},
            ],
        }
        path.write_text(json.dumps(config), encoding="utf-8")
        return path

    def _count_pixels(self, widget, color_name: str) -> int:  # noqa: ANN001
        image = widget.grab().toImage()
        count = 0
        for y in range(image.height()):
            for x in range(image.width()):
                if image.pixelColor(x, y).name() == color_name:
                    count += 1
        return count

    def _leader_marker_bounds_at_scale(self, scale: float) -> QRect:
        view = self.window.top_view
        view.show_grid = False
        view.snapshot = Snapshot(
            time=0.0,
            duration=1.0,
            step=0.1,
            run_state="READY",
            control_report="",
            disturbance="无",
            nodes=[NodeState("A01", "leader", 0.0, 0.0, 1.0, 0.0)],
            links=[],
        )
        view.scale_value = scale
        view.offset = QPointF(180.0, 180.0)
        view.viewport().update()
        self.app.processEvents()

        image = view.grab().toImage()
        marker_color = self.window.theme.leader.name()
        left = image.width()
        right = -1
        top = image.height()
        bottom = -1
        for y in range(image.height()):
            for x in range(image.width()):
                if image.pixelColor(x, y).name() == marker_color:
                    left = min(left, x)
                    right = max(right, x)
                    top = min(top, y)
                    bottom = max(bottom, y)
        self.assertGreaterEqual(right, left)
        self.assertGreaterEqual(bottom, top)
        return QRect(left, top, right - left + 1, bottom - top + 1)

    def _link_stroke_height_at_scale(self, scale: float) -> int:
        view = self.window.top_view
        view.show_grid = False
        view.snapshot = Snapshot(
            time=0.0,
            duration=1.0,
            step=0.1,
            run_state="READY",
            control_report="",
            disturbance="无",
            nodes=[
                NodeState("A01", "leader", -80.0, 0.0, 1.0, 0.0),
                NodeState("A02", "wingman", 80.0, 0.0, 1.0, 0.0),
            ],
            links=[LinkState("A01", "A02", "duplex", 18, 0.01)],
        )
        view.scale_value = scale
        view.offset = QPointF(180.0, 180.0)
        view.viewport().update()
        self.app.processEvents()

        image = view.grab().toImage()
        canvas = self.window.theme.canvas.name()
        x = round(view.offset.x())
        touched = [y for y in range(image.height()) if image.pixelColor(x, y).name() != canvas]
        self.assertTrue(touched)
        return max(touched) - min(touched) + 1

    def _wait_for_controller_time(self, timeout_s: float = 1.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.app.processEvents()
            self.window._on_tick()
            snapshot = self.window.sim.controller.get_snapshot()
            if snapshot.time_s > 0.0:
                return snapshot
            time.sleep(0.01)
        return self.window.sim.controller.get_snapshot()


if __name__ == "__main__":
    unittest.main()
