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

from PySide6.QtCore import QPointF, QRect, Qt
from PySide6.QtWidgets import QApplication, QFrame, QSplitter, QTableWidget

from src.data.geo import GeoOrigin
from src.runner.sim_control import (
    NodeState as ControllerNodeState,
    SimulationSnapshot as ControllerSnapshot,
)
from src.ui.gui.main_window import (
    ControllerSimulationAdapter,
    MainWindow,
    LinkState,
    NodeState,
    ReferenceRoute,
    Snapshot,
    TOP_VIEW_ORIGIN_MARGIN,
    TrailPoint,
    default_project_root,
    run_gui,
)
from src.ui.gui.view_models import ObstacleView, PLAYBACK_RATE_SLIDER_MAX


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

    def test_top_status_bar_moves_to_sidebar_and_help_menu(self) -> None:
        root_layout = self.window.centralWidget().layout()
        self.assertEqual(root_layout.count(), 1)
        self.assertFalse(self.window.findChildren(QFrame, "header"))

        self.assertIs(self.window.run_state_label.parentWidget(), self.window.status_group)
        self.assertIs(self.window.report_label.parentWidget(), self.window.status_group)
        snapshot = self.window.sim.snapshot()
        self.assertEqual(self.window.run_state_label.text(), snapshot.run_state)
        self.assertEqual(self.window.report_label.text(), f"回报：{snapshot.control_report}")

        menu_titles = [action.text() for action in self.window.menuBar().actions()]
        self.assertIn("控制监控(&V)", menu_titles)
        self.assertIn("数据分析(&D)", menu_titles)
        self.assertIn("帮助(&H)", menu_titles)
        self.assertEqual([action.text() for action in self.window.monitor_menu.actions()], ["数据监控(&M)", "离线分析(&A)"])
        self.assertEqual([action.text() for action in self.window.data_analysis_menu.actions()], ["控制效果分析(&A)"])
        self.assertIn("3D态势(&3)", menu_titles)
        help_menu = self.window.help_menu
        self.assertEqual([action.text() for action in help_menu.actions()], ["浅色模式", "深色模式", "", "日志"])

        self.window.dark_theme_action.trigger()
        self.app.processEvents()

        self.assertEqual(self.window.theme_key, "dark")
        self.assertTrue(self.window.dark_theme_action.isChecked())
        self.assertFalse(self.window.light_theme_action.isChecked())

    def test_situation3d_menu_opens_independent_window(self) -> None:
        self.assertIsNone(self.window._situation3d_window)
        menu_titles = [action.text() for action in self.window.menuBar().actions()]
        self.assertIn("3D态势(&3)", menu_titles)

        self.window.situation3d_action.trigger()
        self.app.processEvents()

        first_window = self.window._situation3d_window
        self.assertIsNotNone(first_window)
        self.assertTrue(first_window.isVisible())
        self.assertTrue(first_window.isWindow())
        self.assertTrue(first_window.isFullScreen())
        self.assertEqual(first_window.windowTitle(), "3D态势")
        self.assertEqual(first_window.quick_view.errors(), [])

        self.window.obstacles = [
            ObstacleView("OBS1", "circle", center_x=80.0, center_y=70.0, radius=25.0),
        ]
        self.window.clearance_spin.setValue(5.0)
        self.window._update_snapshot(
            Snapshot(
                time=1.0,
                duration=10.0,
                step=0.1,
                run_state="RUNNING",
                control_report="保持",
                disturbance="无",
                nodes=[
                    NodeState(
                        "A01",
                        "leader",
                        10.0,
                        20.0,
                        3.0,
                        4.0,
                        altitude=30.0,
                        trail=[TrailPoint(1.0, 2.0, 3.0, 0.0), TrailPoint(4.0, 5.0, 6.0, 1.0)],
                    ),
                    NodeState("A02", "wing", -30.0, 12.0, 2.0, 1.0, altitude=40.0),
                    NodeState("A03", "wing", -34.0, 28.0, 2.0, -1.0, altitude=42.0),
                ],
                links=[],
                route_segments=[ReferenceRoute(0.0, 0.0, 100.0, 100.0, 50.0, 120.0)],
            )
        )
        self.app.processEvents()
        scene_data = json.loads(first_window.bridge.sceneData())
        self.assertEqual(scene_data["counts"]["aircraft"], 3)
        self.assertGreaterEqual(scene_data["counts"]["trailPoints"], 2)
        self.assertGreaterEqual(scene_data["counts"]["routePoints"], 2)
        self.assertEqual(scene_data["counts"]["obstacles"], 1)
        self.assertEqual(scene_data["obstacles"][0]["radius"], 30.0)

        self.window.situation3d_action.trigger()
        self.app.processEvents()

        self.assertIs(self.window._situation3d_window, first_window)
        self.assertTrue(first_window.isVisible())
        self.assertTrue(first_window.isFullScreen())

    def test_data_analysis_menu_opens_independent_window(self) -> None:
        self.window._open_data_analysis_window()
        self.app.processEvents()

        self.assertIsNotNone(self.window._data_analysis_window)
        self.assertEqual(self.window._data_analysis_window.windowTitle(), "离线控制效果分析")

    def test_grid_toggle_controls_top_and_side_views(self) -> None:
        self.assertTrue(self.window.grid_toggle.isChecked())
        self.assertTrue(self.window.top_view.show_grid)
        self.assertTrue(self.window.side_view.show_grid)

        self.window.grid_toggle.setChecked(False)
        self.app.processEvents()

        self.assertFalse(self.window.top_view.show_grid)
        self.assertFalse(self.window.side_view.show_grid)

    def test_top_and_side_views_are_resizable_with_splitter(self) -> None:
        splitter = self.window.view_splitter

        self.assertIsInstance(splitter, QSplitter)
        self.assertEqual(splitter.orientation(), Qt.Orientation.Vertical)
        self.assertEqual(splitter.count(), 2)
        self.assertFalse(splitter.childrenCollapsible())
        self.assertIs(splitter.widget(0), self.window.top_view)
        self.assertIs(splitter.widget(1), self.window.side_view)

        splitter.setSizes([460, 260])
        self.app.processEvents()
        sizes = splitter.sizes()

        self.assertGreater(sizes[0], sizes[1])
        self.assertGreaterEqual(self.window.top_view.height(), 360)
        self.assertGreaterEqual(self.window.side_view.height(), 150)

    def test_playback_slider_uses_segmented_rate_steps(self) -> None:
        self.assertEqual(self.window.speed_slider.minimum(), 1)
        self.assertEqual(self.window.speed_slider.maximum(), PLAYBACK_RATE_SLIDER_MAX)

        cases = [
            (1, 0.1),
            (10, 1.0),
            (20, 2.0),
            (21, 3.0),
            (28, 10.0),
            (29, 12.0),
            (33, 20.0),
            (34, 23.0),
            (PLAYBACK_RATE_SLIDER_MAX, 50.0),
        ]
        for slider_value, expected_speed in cases:
            with self.subTest(slider_value=slider_value):
                self.window.speed_slider.setValue(slider_value)
                self.app.processEvents()

                self.assertEqual(self.window.speed_label.text(), f"{expected_speed:.1f}x")
                self.assertAlmostEqual(self.window.sim.speed, expected_speed)
                self.assertAlmostEqual(self.window.sim.controller.playback_rate, expected_speed)

    def test_cpu_utilization_label_updates_from_snapshot(self) -> None:
        snapshot = Snapshot(
            time=0.0,
            duration=10.0,
            step=0.1,
            run_state="RUNNING",
            control_report="保持",
            disturbance="无",
            nodes=[],
            links=[],
            cpu_utilization=0.8,
        )

        self.window._update_snapshot(snapshot)
        self.app.processEvents()

        self.assertEqual(self.window.cpu_label.text(), "CPU 80%")

    def test_load_config_syncs_playback_rate_to_slider(self) -> None:
        self._load_ui_config(playback_rate=2.0)

        self.assertEqual(self.window.speed_slider.value(), 20)
        self.assertEqual(self.window.speed_label.text(), "2.0x")
        self.assertAlmostEqual(self.window.sim.speed, 2.0)
        self.assertAlmostEqual(self.window.sim.controller.playback_rate, 2.0)

    def test_reset_keeps_current_playback_rate_after_slider_change(self) -> None:
        self._load_ui_config()
        self.window.speed_slider.setValue(PLAYBACK_RATE_SLIDER_MAX)
        self.app.processEvents()

        self.window._start()
        self._wait_for_controller_time()
        self.window._reset()
        self.app.processEvents()

        self.assertEqual(self.window.speed_label.text(), "50.0x")
        self.assertAlmostEqual(self.window.sim.speed, 50.0)
        self.assertAlmostEqual(self.window.sim.controller.playback_rate, 50.0)

    def test_adapter_pause_does_not_resume_from_paused_state(self) -> None:
        self._load_ui_config()
        self.window.sim.start()
        self._wait_for_controller_time()

        paused = self.window.sim.pause()
        paused_again = self.window.sim.pause()

        self.assertEqual(paused.run_state, "PAUSED")
        self.assertEqual(paused_again.run_state, "PAUSED")

    def test_repeated_snapshot_time_keeps_controller_velocity(self) -> None:
        snapshot = ControllerSnapshot(
            time_s=1.0,
            duration_s=10.0,
            step_s=0.1,
            run_state="RUNNING",
            control_report="保持",
            cpu_utilization=0.42,
            nodes=[
                ControllerNodeState(
                    node_id="A01",
                    role="leader",
                    health="normal",
                    x_m=100.0,
                    y_m=200.0,
                    altitude_m=1200.0,
                    psi_v_deg=90.0,
                    theta_deg=0.0,
                    speed_mps=8.0,
                    vx_mps=0.0,
                    vy_mps=8.0,
                    vz_mps=0.0,
                    nx=0.0,
                    nz=1.0,
                    phi_deg=0.0,
                    psi_dot_deg_s=0.0,
                )
            ],
            links=[],
        )

        first = self.window.sim._convert_snapshot(snapshot)
        repeated = self.window.sim._convert_snapshot(replace(snapshot, run_state="PAUSED"))

        self.assertAlmostEqual(first.nodes[0].vx, 0.0)
        self.assertAlmostEqual(first.nodes[0].vy, 8.0)
        self.assertAlmostEqual(first.cpu_utilization, 0.42)
        self.assertAlmostEqual(repeated.nodes[0].vx, 0.0)
        self.assertAlmostEqual(repeated.nodes[0].vy, 8.0)
        self.assertAlmostEqual(repeated.cpu_utilization, 0.42)

    def test_side_grid_uses_side_horizontal_mapping(self) -> None:
        self.window.side_view.snapshot = None
        self.window.side_view.horizontal_offset = 73.0
        self.window.side_view.horizontal_scale = 1.0
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
            self.window.side_view.horizontal_scale = scale

            screen_spacing = self.window.top_view._grid_world_spacing() * scale
            side_screen_spacing = self.window.side_view._grid_world_spacing() * scale

            self.assertGreaterEqual(screen_spacing, 36.0)
            self.assertLessEqual(screen_spacing, 96.0)
            self.assertGreaterEqual(side_screen_spacing, 36.0)
            self.assertLessEqual(side_screen_spacing, 96.0)

    def test_top_view_aircraft_marker_keeps_screen_size_during_zoom(self) -> None:
        small = self._leader_marker_bounds_at_scale(0.45)
        large = self._leader_marker_bounds_at_scale(3.5)

        self.assertAlmostEqual(small.width(), large.width(), delta=4)
        self.assertAlmostEqual(small.height(), large.height(), delta=4)
        self.assertLessEqual(large.width(), 14)
        self.assertLessEqual(large.height(), 10)

    def test_top_view_marks_role_leader_when_leader_is_not_first_node(self) -> None:
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
                NodeState("A01", "wingman", 0.0, 0.0, 1.0, 0.0),
                NodeState("A05", "leader", 80.0, 0.0, 1.0, 0.0),
            ],
            links=[],
        )
        view.scale_value = 1.0
        view.offset = QPointF(180.0, 180.0)
        view.viewport().update()
        self.app.processEvents()

        image = view.grab().toImage()
        leader_color = self.window.theme.leader.name()

        self.assertEqual(self._count_pixels_near(image, 180, 180, leader_color), 0)
        self.assertGreater(self._count_pixels_near(image, 260, 180, leader_color), 0)

    def test_top_view_maps_positive_north_upward(self) -> None:
        view = self.window.top_view
        view.scale_value = 1.0
        view.offset = QPointF(180.0, 180.0)

        origin = view._world_to_viewport(QPointF(0.0, 0.0))
        north = view._world_to_viewport(QPointF(0.0, 80.0))
        round_trip = view._viewport_to_world(north)

        self.assertAlmostEqual(origin.y(), 180.0)
        self.assertLess(north.y(), origin.y())
        self.assertAlmostEqual(round_trip.x(), 0.0)
        self.assertAlmostEqual(round_trip.y(), 80.0)

    def test_top_view_click_updates_copyable_geodetic_text(self) -> None:
        self.window._top_view_geo_origin = GeoOrigin(39.0, 116.0)

        self.window._on_top_view_point_clicked(0.0, 0.0)

        self.assertEqual(self.window.top_view_coordinate.text(), "116.0000000, 39.0000000")
        self.assertEqual(self.window.top_view_coordinate.selectedText(), "116.0000000, 39.0000000")
        self.assertEqual(self.window.top_view_coordinate_hint.text(), "(lon, lat)")

    def test_top_view_link_keeps_screen_width_during_zoom(self) -> None:
        thin = self._link_stroke_height_at_scale(0.45)
        thick = self._link_stroke_height_at_scale(3.5)

        self.assertAlmostEqual(thin, thick, delta=2)

    def test_top_view_reset_refits_side_altitude_axis(self) -> None:
        self._load_ui_config()
        self.window.side_view.altitude_min = 1180.0
        self.window.side_view.altitude_max = 1240.0

        self.window.top_view.reset_view()
        self.app.processEvents()

        self.assert_route_and_aircraft_fit_viewport()
        snapshot = self.window.side_view.snapshot
        self.assertIsNotNone(snapshot)
        assert snapshot is not None
        altitudes = [node.altitude for node in snapshot.nodes]
        for route in snapshot.route_segments:
            altitudes.extend([route.start_altitude, route.end_altitude])
        self.assertLessEqual(self.window.side_view.altitude_min, min(altitudes))
        self.assertGreaterEqual(self.window.side_view.altitude_max, max(altitudes))

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

    def test_reset_view_fits_large_80km_route(self) -> None:
        self._load_ui_config(
            nodes=[
                {"node_id": "S01", "role": "leader", "x_m": 0.0, "y_m": 0.0, "altitude_m": 1200.0, "speed_mps": 45.0},
            ],
            links=[],
            route={
                "speed_mps": 45.0,
                "waypoints": [
                    {"x_m": 0.0, "y_m": 0.0, "altitude_m": 1200.0},
                    {"x_m": 76000.0, "y_m": 0.0, "altitude_m": 1200.0},
                    {"x_m": 76000.0, "y_m": 70000.0, "altitude_m": 1200.0},
                    {"x_m": 20000.0, "y_m": 70000.0, "altitude_m": 1200.0},
                ],
            },
        )

        self.window.top_view.reset_view()
        self.app.processEvents()

        self.assertLess(self.window.top_view.scale_value, 0.05)
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

    def test_auto_center_moves_top_and_side_views_without_rescaling(self) -> None:
        self._load_ui_config()
        self.window.top_view.scale_value = 1.7
        self.window.side_view.horizontal_scale = 1.4
        self.window.side_view.altitude_min = 1180.0
        self.window.side_view.altitude_max = 1260.0
        self.window.auto_center.setChecked(True)
        self.app.processEvents()

        top_scale = self.window.top_view.scale_value
        side_scale = self.window.side_view.horizontal_scale
        altitude_span = self.window.side_view.altitude_max - self.window.side_view.altitude_min
        snapshot = self.window.top_view.snapshot
        self.assertIsNotNone(snapshot)
        assert snapshot is not None
        moved_nodes = [
            replace(node, x=node.x + 500.0, y=node.y + 80.0, altitude=node.altitude + 60.0)
            for node in snapshot.nodes
        ]
        running_snapshot = replace(
            snapshot,
            time=snapshot.time + snapshot.step,
            run_state="RUNNING",
            nodes=moved_nodes,
        )

        self.window._update_snapshot(running_snapshot)
        self.app.processEvents()

        self.assertAlmostEqual(self.window.top_view.scale_value, top_scale)
        self.assertAlmostEqual(self.window.side_view.horizontal_scale, side_scale)
        self.assertAlmostEqual(self.window.side_view.altitude_max - self.window.side_view.altitude_min, altitude_span)
        active = [node for node in moved_nodes if node.health == "normal"]
        top_center_x = sum(node.x for node in active) / len(active)
        top_center_y = sum(node.y for node in active) / len(active)
        self.assertAlmostEqual(
            self.window.top_view.offset.x(),
            self.window.top_view.viewport().rect().width() / 2.0 - top_center_x * top_scale,
            delta=0.01,
        )
        self.assertAlmostEqual(
            self.window.top_view.offset.y(),
            self.window.top_view.viewport().rect().height() / 2.0 + top_center_y * top_scale,
            delta=0.01,
        )
        side_center_x = sum(
            self.window.side_view._horizontal_for_point(node.x, node.y)
            for node in active
        ) / len(active)
        side_center_altitude = sum(node.altitude for node in active) / len(active)
        self.assertAlmostEqual(
            self.window.side_view.horizontal_offset,
            self.window.side_view.width() / 2.0 - side_center_x * side_scale,
            delta=0.01,
        )
        self.assertAlmostEqual(
            (self.window.side_view.altitude_min + self.window.side_view.altitude_max) / 2.0,
            side_center_altitude,
            delta=0.01,
        )

    def test_auto_center_survives_top_view_selection_zoom(self) -> None:
        self._load_ui_config()
        self.window.auto_center.setChecked(True)
        self.app.processEvents()
        view = self.window.top_view
        old_scale = view.scale_value
        view._selection_origin = QPointF(100.0, 100.0)
        view._selection_current = QPointF(300.0, 220.0)

        view._zoom_to_selection()
        self.app.processEvents()

        self.assertTrue(self.window.auto_center.isChecked())
        self.assertTrue(self.window.top_view.auto_center)
        self.assertTrue(self.window.side_view.auto_center)
        self.assertNotAlmostEqual(view.scale_value, old_scale)
        snapshot = view.snapshot
        self.assertIsNotNone(snapshot)
        assert snapshot is not None
        active = [node for node in snapshot.nodes if node.health == "normal"]
        center_x = sum(node.x for node in active) / len(active)
        center_y = sum(node.y for node in active) / len(active)
        self.assertAlmostEqual(
            view.offset.x(),
            view.viewport().rect().width() / 2.0 - center_x * view.scale_value,
            delta=0.01,
        )
        self.assertAlmostEqual(
            view.offset.y(),
            view.viewport().rect().height() / 2.0 + center_y * view.scale_value,
            delta=0.01,
        )

    def test_auto_center_survives_side_view_selection_zoom(self) -> None:
        self._load_ui_config()
        self.window.auto_center.setChecked(True)
        self.app.processEvents()
        view = self.window.side_view
        old_scale = view.horizontal_scale
        old_span = view.altitude_max - view.altitude_min
        view._selection_origin = QPointF(100.0, 44.0)
        view._selection_current = QPointF(420.0, 84.0)

        view._zoom_to_selection()
        self.app.processEvents()

        self.assertTrue(self.window.auto_center.isChecked())
        self.assertTrue(self.window.top_view.auto_center)
        self.assertTrue(self.window.side_view.auto_center)
        self.assertNotAlmostEqual(view.horizontal_scale, old_scale)
        self.assertNotAlmostEqual(view.altitude_max - view.altitude_min, old_span)
        snapshot = view.snapshot
        self.assertIsNotNone(snapshot)
        assert snapshot is not None
        active = [node for node in snapshot.nodes if node.health == "normal"]
        center_x = sum(view._horizontal_for_point(node.x, node.y) for node in active) / len(active)
        center_altitude = sum(node.altitude for node in active) / len(active)
        self.assertAlmostEqual(
            view.horizontal_offset,
            view.width() / 2.0 - center_x * view.horizontal_scale,
            delta=0.01,
        )
        self.assertAlmostEqual(
            (view.altitude_min + view.altitude_max) / 2.0,
            center_altitude,
            delta=0.01,
        )

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

    def test_side_reset_view_refits_route_altitude_and_aircraft(self) -> None:
        self._load_ui_config(
            nodes=[
                {"node_id": "A01", "role": "leader", "x_m": 900.0, "y_m": 300.0, "altitude_m": 1200.0},
                {"node_id": "A02", "role": "wingman", "x_m": 880.0, "y_m": 340.0, "altitude_m": 1215.0},
            ],
            links=[{"link_id": "A01-A02", "latency_ms": 18.0, "loss_rate": 0.01}],
            route={
                "speed_mps": 12.0,
                "waypoints": [
                    {"x_m": 0.0, "y_m": 0.0, "altitude_m": 1000.0},
                    {"x_m": 1000.0, "y_m": 0.0, "altitude_m": 1000.0},
                ],
            },
        )
        self.window.side_view.altitude_min = 1180.0
        self.window.side_view.altitude_max = 1220.0

        self.window.side_view.reset_view()
        self.app.processEvents()

        snapshot = self.window.side_view.snapshot
        self.assertIsNotNone(snapshot)
        assert snapshot is not None
        self.assertLessEqual(self.window.side_view.altitude_min, 1000.0)
        self.assertGreaterEqual(self.window.side_view.altitude_max, 1215.0)
        for route in snapshot.route_segments:
            for altitude in (route.start_altitude, route.end_altitude):
                y = self.window.side_view._map_y(altitude)
                self.assertGreaterEqual(y, 5.0)
                self.assertLessEqual(y, self.window.side_view.height() - 5.0)

    def test_route_endpoints_are_visible_after_load(self) -> None:
        self._load_ui_config()
        route = self.window.top_view.snapshot.route
        self.assertIsNotNone(route)
        assert route is not None

        start = self.window.top_view._world_to_viewport(QPointF(route.start_x, route.start_y))
        end = self.window.top_view._world_to_viewport(QPointF(route.end_x, route.end_y))
        rect = self.window.top_view.viewport().rect()

        self.assertGreaterEqual(start.x(), 5.0)
        self.assertGreaterEqual(start.y(), 5.0)
        self.assertLessEqual(end.x(), rect.width() - 5.0)
        self.assertGreaterEqual(end.y(), 5.0)

    def test_low_zoom_two_point_trail_remains_visible(self) -> None:
        """低缩放长航线下，刚开始运行产生的两点尾迹也应清晰可见。"""

        view = self.window.top_view
        view.show_grid = False
        view.snapshot = Snapshot(
            time=0.2,
            duration=10.0,
            step=0.1,
            run_state="RUNNING",
            control_report="保持",
            disturbance="无",
            nodes=[
                NodeState(
                    "A01",
                    "leader",
                    160.0,
                    220.0,
                    20.0,
                    0.0,
                    trail=[
                        TrailPoint(120.0, 20.0, 1200.0, 0.1),
                        TrailPoint(160.0, 20.0, 1200.0, 0.2),
                    ],
                )
            ],
            links=[],
        )
        view.scale_value = 0.2
        view.offset = QPointF(120.0, 120.0)
        view.viewport().update()
        self.app.processEvents()

        image = view.grab().toImage()
        canvas = self.window.theme.canvas.name()
        y = round(view._world_to_viewport(QPointF(0.0, 20.0)).y())
        touched = [
            x
            for x in range(image.width())
            if image.pixelColor(x, y).name() != canvas
        ]

        self.assertGreaterEqual(len(touched), 6)

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
        second_end = self.window.top_view._world_to_viewport(
            QPointF(snapshot.route_segments[1].end_x, snapshot.route_segments[1].end_y)
        )
        rect = self.window.top_view.viewport().rect()
        self.assertGreaterEqual(second_end.x(), 5.0)
        self.assertGreaterEqual(second_end.y(), 5.0)
        self.assertLessEqual(second_end.x(), rect.width() - 5.0)
        self.assertLessEqual(second_end.y(), rect.height() - 5.0)

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

    def test_side_horizontal_selection_updates_side_axis_only(self) -> None:
        old_top_scale = self.window.top_view.scale_value
        old_top_offset = QPointF(self.window.top_view.offset)
        old_side_scale = self.window.side_view.horizontal_scale

        self.window.side_view._selection_origin = QPointF(100.0, 44.0)
        self.window.side_view._selection_current = QPointF(520.0, 84.0)
        self.window.side_view._zoom_to_selection()
        self.app.processEvents()

        self.assertEqual(self.window.top_view.offset, old_top_offset)
        self.assertEqual(self.window.top_view.scale_value, old_top_scale)
        self.assertGreater(self.window.side_view.horizontal_scale, old_side_scale)

    def test_segment_lock_projects_current_north_south_route_to_station_axis(self) -> None:
        route = ReferenceRoute(1000.0, 100.0, 1200.0, 1000.0, 500.0, 1200.0)
        snapshot = Snapshot(
            time=0.0,
            duration=10.0,
            step=0.1,
            run_state="READY",
            control_report="待命",
            disturbance="无",
            nodes=[NodeState("A01", "leader", 1000.0, 300.0, 0.0, 20.0, 1200.0)],
            links=[],
            route=route,
            route_segments=[route],
        )

        self.window._update_snapshot(snapshot, fit_top_view=True)
        self.app.processEvents()

        self.assertTrue(self.window.segment_lock.isChecked())
        self.assertFalse(self.window.view_angle_slider.isEnabled())
        self.assertFalse(self.window.view_angle_input.isEnabled())
        self.assertEqual(self.window.view_angle_slider.value(), 270)
        self.assertEqual(self.window.view_angle_input.value(), 270)
        self.assertAlmostEqual(self.window.side_view._horizontal_for_point(1000.0, 300.0), 200.0)

        self.window.segment_lock.setChecked(False)
        self.app.processEvents()

        self.assertTrue(self.window.view_angle_slider.isEnabled())
        self.assertTrue(self.window.view_angle_input.isEnabled())
        self.assertEqual(self.window.view_angle_slider.value(), 270)
        self.assertEqual(self.window.view_angle_input.value(), 270)
        self.assertAlmostEqual(self.window.side_view.view_angle_deg, 270.0)

    def test_unlocked_view_angle_slider_projects_by_direction(self) -> None:
        snapshot = Snapshot(
            time=0.0,
            duration=10.0,
            step=0.1,
            run_state="READY",
            control_report="待命",
            disturbance="无",
            nodes=[NodeState("A01", "leader", 100.0, 100.0, 0.0, 20.0, 1200.0)],
            links=[],
        )
        self.window._update_snapshot(snapshot, fit_top_view=True)
        self.app.processEvents()

        self.assertFalse(self.window.segment_lock.isEnabled())
        self.assertTrue(self.window.view_angle_slider.isEnabled())
        self.assertTrue(self.window.view_angle_input.isEnabled())

        self.window.view_angle_input.setValue(90)
        self.app.processEvents()

        self.assertEqual(self.window.view_angle_slider.value(), 90)
        self.assertAlmostEqual(self.window.side_view.view_angle_deg, 90.0)
        self.assertAlmostEqual(self.window.side_view._horizontal_for_point(0.0, 100.0), -100.0)
        self.assertAlmostEqual(self.window.side_view._horizontal_for_point(100.0, 0.0), 0.0, delta=1e-6)

    def test_main_window_uses_simulation_controller_adapter(self) -> None:
        self.assertIsInstance(self.window.sim, ControllerSimulationAdapter)
        self.assertEqual(self.window.sim.controller.get_snapshot().run_state, "UNLOADED")
        self.assertEqual(self.window.node_table.rowCount(), 0)
        self.assertEqual(self.window.overall_table.rowCount(), 0)
        self.assertEqual(self.window.link_table.rowCount(), 0)
        self.assertFalse(self.window.play_button.isEnabled())
        self.assertFalse(self.window.step_button.isEnabled())
        self.assertFalse(self.window.reset_button.isEnabled())
        self.assertTrue(all(not button.isEnabled() for button in self.window.disturbance_buttons))

    def test_run_gui_opens_main_window_maximized(self) -> None:
        """验证正式启动入口默认以最大化方式显示主窗口。"""
        calls: list[str] = []

        class FakeApp:
            """替代 QApplication，避免测试进入真实事件循环。"""

            def __init__(self, argv: list[str]) -> None:
                self.argv = argv

            def exec(self) -> int:
                calls.append("exec")
                return 7

        class FakeMainWindow:
            """记录主窗口显示方式，锁定 exe 启动行为。"""

            def show(self) -> None:
                calls.append("show")

            def showMaximized(self) -> None:
                calls.append("showMaximized")

        with (
            patch("src.ui.gui.main_window.QApplication", FakeApp),
            patch("src.ui.gui.main_window.MainWindow", FakeMainWindow),
        ):
            exit_code = run_gui(["--smoke"])

        self.assertEqual(exit_code, 7)
        self.assertEqual(calls, ["showMaximized", "exec"])

    def test_unloaded_window_does_not_draw_reference_route(self) -> None:
        route_color = self.window.theme.route.name()

        self.assertEqual(self._count_pixels(self.window.top_view, route_color), 0)
        self.assertEqual(self._count_pixels(self.window.side_view, route_color), 0)

    def test_play_pause_button_toggles_real_controller_snapshot(self) -> None:
        self._load_ui_config(duration_s=1.0)
        self.assertEqual(self.window.play_button.text(), "开始")

        self.window.play_button.click()
        self.app.processEvents()
        running_snapshot = self._wait_for_controller_time()

        self.assertEqual(running_snapshot.run_state, "RUNNING")
        self.assertGreater(running_snapshot.time_s, 0.0)
        self.assertEqual(self.window.play_button.text(), "暂停")

        self.window.play_button.click()
        self.app.processEvents()
        paused_snapshot = self.window.sim.controller.get_snapshot()

        self.assertEqual(paused_snapshot.run_state, "PAUSED")
        self.assertEqual(self.window.play_button.text(), "继续")

        self.window.play_button.click()
        self.app.processEvents()
        resumed_snapshot = self.window.sim.controller.get_snapshot()

        self.assertEqual(resumed_snapshot.run_state, "RUNNING")
        self.assertEqual(self.window.play_button.text(), "暂停")

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
                self.assertEqual(window.overall_table.rowCount(), 1)
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

    def test_startup_load_refits_side_view_route_altitude(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            project_root = Path(tmp)
            config_path = self._write_config_file(project_root / "configs" / "startup.json")
            config = json.loads(config_path.read_text(encoding="utf-8"))
            config["route"] = {
                "speed_mps": 12.0,
                "waypoints": [
                    {"x_m": 0.0, "y_m": 0.0, "altitude_m": 1000.0},
                    {"x_m": 1000.0, "y_m": 0.0, "altitude_m": 1000.0},
                ],
            }
            config_path.write_text(json.dumps(config), encoding="utf-8")
            state_path = project_root / "config.ini"
            state_path.write_text("[config]\nlast_config = configs/startup.json\n", encoding="utf-8")

            window = MainWindow(project_root=project_root, config_state_path=state_path)
            window.show()
            self.app.processEvents()
            try:
                self.assertLessEqual(window.side_view.altitude_min, 1000.0)
                self.assertGreaterEqual(window.side_view.altitude_max, 1215.0)
                self.assertGreaterEqual(window.side_view._map_y(1000.0), 5.0)
                self.assertLessEqual(window.side_view._map_y(1000.0), window.side_view.height() - 5.0)
            finally:
                window.close()
                self.app.processEvents()

    def test_packaged_startup_loads_config_ini_next_to_exe(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            app_dir = Path(tmp).resolve()  # macOS: /var → /private/var，与 default_project_root() 的真实路径对齐
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
            app_dir = Path(tmp).resolve()  # macOS: /var → /private/var，与 default_project_root() 的真实路径对齐
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
            self.window.node_table.item(row, 0).text(): self.window.node_table.item(row, 4).text()
            for row in range(self.window.node_table.rowCount())
        }

        self.assertEqual(statuses["A03"], "故障")
        self.assertEqual(statuses["A02"], "正常")

    def test_status_tables_expand_to_use_panel_height_before_scrolling(self) -> None:
        self.app.processEvents()

        self.assertGreater(self.window.node_table.height(), 180)
        self.assertGreater(self.window.link_table.height(), 180)

    def test_node_table_shows_track_errors_and_overall_table_uses_leader_route_metrics(self) -> None:
        snapshot = Snapshot(
            time=0.0,
            duration=10.0,
            step=0.1,
            run_state="READY",
            control_report="待命",
            disturbance="无",
            nodes=[
                NodeState(
                    "A01",
                    "leader",
                    100.0,
                    120.0,
                    20.0,
                    0.0,
                    1200.0,
                    vertical_speed=3.2,
                    cross_track_error=12.4,
                    distance_to_go=345.6,
                    track_pos_err_x=1.2,
                    track_pos_err_y=-3.4,
                    track_pos_err_z=5.6,
                ),
                NodeState(
                    "A02",
                    "wingman",
                    80.0,
                    90.0,
                    20.0,
                    0.0,
                    1210.0,
                    track_pos_err_x=-7.8,
                    track_pos_err_y=9.1,
                    track_pos_err_z=-2.3,
                ),
            ],
            links=[],
        )

        self.window._update_snapshot(snapshot)

        self.assertEqual(self.window.node_table.columnCount(), 5)
        self.assertEqual(self.window.node_table.horizontalHeaderItem(1).text(), "前向(m)")
        self.assertEqual(self.window.node_table.horizontalHeaderItem(2).text(), "垂向(m)")
        self.assertEqual(self.window.node_table.horizontalHeaderItem(3).text(), "侧向(m)")
        self.assertEqual(self.window.node_table.item(0, 1).text(), "1.2")
        self.assertEqual(self.window.node_table.item(0, 1).textAlignment(), int(Qt.AlignmentFlag.AlignCenter))
        self.assertEqual(self.window.node_table.item(0, 2).text(), "-3.4")
        self.assertEqual(self.window.node_table.item(0, 3).text(), "5.6")
        self.assertEqual(self.window.node_table.item(1, 1).text(), "-7.8")
        self.assertEqual(self.window.node_table.item(1, 2).text(), "9.1")
        self.assertEqual(self.window.node_table.item(1, 3).text(), "-2.3")
        self.assertEqual(self.window.overall_table.rowCount(), 1)
        self.assertEqual(self.window.overall_table.columnCount(), 5)
        self.assertEqual(self.window.overall_table.horizontalHeaderItem(3).text(), "地速(m/s)")
        self.assertEqual(self.window.overall_table.horizontalHeaderItem(4).text(), "天向速度(m/s)")
        self.assertEqual(self.window.overall_table.item(0, 0).text(), "12")
        self.assertEqual(self.window.overall_table.item(0, 1).text(), "346")
        self.assertEqual(self.window.overall_table.item(0, 2).text(), "1200")
        self.assertEqual(self.window.overall_table.item(0, 3).text(), "20")
        self.assertEqual(self.window.overall_table.item(0, 3).textAlignment(), int(Qt.AlignmentFlag.AlignCenter))
        self.assertEqual(self.window.overall_table.item(0, 4).text(), "3")
        self.assertLessEqual(self.window.overall_table.height(), 66)
        self.assertEqual(self.window.node_table.horizontalScrollBar().maximum(), 0)
        self.assertEqual(self.window.overall_table.horizontalScrollBar().maximum(), 0)
        self.assert_table_uses_full_width(self.window.node_table)
        self.assert_table_uses_full_width(self.window.overall_table)

    def test_node_table_update_does_not_write_beyond_five_columns(self) -> None:
        """验证节点表保持五列时，集结阶段不会触发越界写入。"""
        snapshot = Snapshot(
            time=0.0,
            duration=10.0,
            step=0.1,
            run_state="READY",
            control_report="待命",
            disturbance="无",
            nodes=[
                NodeState(
                    "R01",
                    "rally_leader",
                    100.0,
                    120.0,
                    20.0,
                    0.0,
                    rally_phase="JN·FLY",
                ),
            ],
            links=[],
        )
        original_set_item = QTableWidget.setItem
        written_columns: list[int] = []

        def record_set_item(table: QTableWidget, row: int, column: int, item) -> None:
            if table is self.window.node_table:
                written_columns.append(column)
            original_set_item(table, row, column, item)

        with patch.object(QTableWidget, "setItem", new=record_set_item):
            self.window._update_tables(snapshot)

        self.assertEqual(self.window.node_table.columnCount(), 5)
        self.assertTrue(written_columns)
        self.assertLess(max(written_columns), self.window.node_table.columnCount())

    def test_overall_table_uses_role_leader_when_leader_is_not_first_node(self) -> None:
        snapshot = Snapshot(
            time=0.0,
            duration=10.0,
            step=0.1,
            run_state="READY",
            control_report="待命",
            disturbance="无",
            nodes=[
                NodeState(
                    "A01",
                    "wingman",
                    100.0,
                    120.0,
                    20.0,
                    0.0,
                    1300.0,
                    cross_track_error=99.4,
                    distance_to_go=888.6,
                ),
                NodeState(
                    "A05",
                    "leader",
                    80.0,
                    90.0,
                    20.0,
                    0.0,
                    1200.0,
                    vertical_speed=-4.4,
                    cross_track_error=12.4,
                    distance_to_go=345.6,
                ),
            ],
            links=[],
        )

        self.window._update_snapshot(snapshot)

        self.assertEqual(self.window.overall_table.item(0, 0).text(), "12")
        self.assertEqual(self.window.overall_table.item(0, 1).text(), "346")
        self.assertEqual(self.window.overall_table.item(0, 2).text(), "1200")
        self.assertEqual(self.window.overall_table.item(0, 3).text(), "20")
        self.assertEqual(self.window.overall_table.item(0, 4).text(), "-4")


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
        self.assertEqual(self.window.link_table.item(0, 0).textAlignment(), int(Qt.AlignmentFlag.AlignCenter))
        self.assertEqual(self.window.link_table.item(0, 4).textAlignment(), int(Qt.AlignmentFlag.AlignCenter))
        self.assertEqual(self.window.link_table.horizontalScrollBar().maximum(), 0)
        self.assert_table_uses_full_width(self.window.link_table)

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

    def test_duration_input_rejects_value_before_current_time(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            config_path = self._write_config_file(Path(tmp) / "case.json")
            config = json.loads(config_path.read_text(encoding="utf-8"))
            config["duration_s"] = 1.0
            config_path.write_text(json.dumps(config), encoding="utf-8")
            self.window._apply_config_path(str(config_path))
            self.window.sim.controller.step(100)
            self.window._update_snapshot(self.window.sim.snapshot())

            self.window.duration_input.setText("0.2")
            self.window.duration_input.editingFinished.emit()
            self.app.processEvents()
            snapshot = self.window.sim.controller.get_snapshot()
            saved_config = json.loads(config_path.read_text(encoding="utf-8"))

            self.assertEqual(self.window.sim.last_result_code, "ERR_INVALID_ARGUMENT")
            self.assertEqual(self.window.duration_input.text(), "1")
            self.assertAlmostEqual(saved_config["duration_s"], 1.0)
            self.assertEqual(snapshot.run_state, "PAUSED")
            self.assertAlmostEqual(snapshot.time_s, 0.5)
            self.assertAlmostEqual(snapshot.duration_s, 1.0)

    def test_open_live_monitor_in_ready_state_binds_controller(self) -> None:
        """Regression: READY 曾被排除在 follow() 之外，导致监控窗口 _ctrl 始终为 None。"""
        try:
            from src.ui.gui.live_monitor import LiveMonitorWindow  # noqa: F401
        except ModuleNotFoundError:
            self.skipTest("PySide6.QtCharts not available")

        self._load_ui_config()
        self.assertEqual(self.window.sim.controller.get_snapshot().run_state, "READY")

        self.window._open_live_monitor()
        self.app.processEvents()

        self.assertIsNotNone(self.window._live_monitor)
        self.assertIsNotNone(self.window._live_monitor._ctrl)
        self.assertIs(self.window._live_monitor._ctrl, self.window.sim.controller)

    def test_step_binds_live_monitor_controller(self) -> None:
        """Regression: _step() 未调用 follow()，单步后监控窗口 _ctrl 仍为 None。"""
        try:
            from src.ui.gui.live_monitor import LiveMonitorWindow
        except ModuleNotFoundError:
            self.skipTest("PySide6.QtCharts not available")

        self._load_ui_config()
        self.window._live_monitor = LiveMonitorWindow(self.window)
        # _ctrl 未绑定，模拟旧代码中打开监控但未 follow 的状态
        self.assertIsNone(self.window._live_monitor._ctrl)

        self.window._step()
        self.app.processEvents()

        self.assertIsNotNone(self.window._live_monitor._ctrl)
        self.assertIs(self.window._live_monitor._ctrl, self.window.sim.controller)

    def _load_ui_config(
        self,
        *,
        duration_s: float = 0.05,
        step_s: float = 0.005,
        playback_rate: float = 10.0,
        nodes: list[dict[str, object]] | None = None,
        links: list[dict[str, object]] | None = None,
        route: dict[str, object] | None = None,
    ) -> None:
        config = {
            "duration_s": duration_s,
            "step_s": step_s,
            "playback_rate": playback_rate,
            "nodes": nodes if nodes is not None else [
                {"node_id": "A01", "role": "leader", "x_m": 140.0, "y_m": 260.0, "altitude_m": 1200.0},
                {"node_id": "A02", "role": "wingman", "x_m": 92.0, "y_m": 318.0, "altitude_m": 1215.0},
                {"node_id": "A03", "role": "wingman", "x_m": 88.0, "y_m": 202.0, "altitude_m": 1230.0},
            ],
            "links": links if links is not None else [
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
        top = view.offset.y() - max(ys) * view.scale_value
        bottom = view.offset.y() - min(ys) * view.scale_value
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

    def assert_table_uses_full_width(self, table) -> None:  # noqa: ANN001
        header_width = sum(table.columnWidth(column) for column in range(table.columnCount()))
        self.assertGreaterEqual(header_width, table.viewport().width() - 1)

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

    def _count_pixels_near(self, image, center_x: int, center_y: int, color_name: str) -> int:  # noqa: ANN001
        count = 0
        for y in range(max(0, center_y - 18), min(image.height(), center_y + 19)):
            for x in range(max(0, center_x - 18), min(image.width(), center_x + 19)):
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
