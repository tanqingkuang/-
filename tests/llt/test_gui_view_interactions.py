"""Regression tests for PySide6 realtime view interactions."""

from __future__ import annotations

import os
import tempfile
import time
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QPointF
from PySide6.QtWidgets import QApplication

from src.ui.gui.main_window import ControllerSimulationAdapter, MainWindow


class GuiViewInteractionTests(unittest.TestCase):
    """Exercise view-state synchronization without starting the Qt event loop."""

    app: QApplication

    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self) -> None:
        self.window = MainWindow()
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
        self.assertEqual(self.window.top_view.offset, QPointF(0.0, 0.0))
        self.assertEqual(self.window.side_view.altitude_min, self.window.side_view.ALTITUDE_MIN_DEFAULT)
        self.assertEqual(self.window.side_view.altitude_max, self.window.side_view.ALTITUDE_MAX_DEFAULT)

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
        self.assertEqual(self.window.sim.controller.get_snapshot().run_state, "READY")

    def test_start_pause_drives_real_controller_snapshot(self) -> None:
        self.window._start()
        running_snapshot = self._wait_for_controller_time()

        self.assertEqual(running_snapshot.run_state, "RUNNING")
        self.assertGreater(running_snapshot.time_s, 0.0)

        self.window._pause()
        paused_snapshot = self.window.sim.controller.get_snapshot()

        self.assertEqual(paused_snapshot.run_state, "PAUSED")

    def test_disturbance_label_clears_after_duration(self) -> None:
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

    def test_node_health_drives_table_status_and_warning_color_target(self) -> None:
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
