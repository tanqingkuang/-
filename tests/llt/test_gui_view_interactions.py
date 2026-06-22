"""Regression tests for PySide6 realtime view interactions."""

from __future__ import annotations

import json
import os
import sys
import tempfile
import time
import unittest
from configparser import ConfigParser
from pathlib import Path
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QPointF
from PySide6.QtWidgets import QApplication

from src.ui.gui.main_window import ControllerSimulationAdapter, MainWindow, TOP_VIEW_ORIGIN_MARGIN, default_project_root


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

    def test_top_view_reset_restores_side_altitude_axis(self) -> None:
        self.window.side_view.altitude_min = 1180.0
        self.window.side_view.altitude_max = 1240.0

        self.window.top_view.reset_view()
        self.app.processEvents()

        self.assertEqual(self.window.top_view.scale_value, 1.0)
        self.assertEqual(self.window.top_view.offset, QPointF(TOP_VIEW_ORIGIN_MARGIN, TOP_VIEW_ORIGIN_MARGIN))
        self.assertEqual(self.window.side_view.altitude_min, self.window.side_view.ALTITUDE_MIN_DEFAULT)
        self.assertEqual(self.window.side_view.altitude_max, self.window.side_view.ALTITUDE_MAX_DEFAULT)

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

    def _load_ui_config(
        self,
        *,
        duration_s: float = 0.05,
        step_s: float = 0.005,
        links: list[dict[str, object]] | None = None,
    ) -> None:
        config = {
            "duration_s": duration_s,
            "step_s": step_s,
            "playback_rate": 10.0,
            "nodes": [
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
        with tempfile.NamedTemporaryFile("w", suffix=".json", encoding="utf-8", delete=False) as handle:
            json.dump(config, handle)
            config_path = handle.name
        try:
            self.window._apply_config_path(config_path)
            self.app.processEvents()
        finally:
            Path(config_path).unlink(missing_ok=True)

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
