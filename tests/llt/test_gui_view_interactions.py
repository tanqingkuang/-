"""Regression tests for PySide6 realtime view interactions."""

from __future__ import annotations

import json
import math
import os
import sys
import tempfile
import time
import unittest
from configparser import ConfigParser
from dataclasses import replace
from pathlib import Path
from unittest.mock import Mock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QMetaObject, QPoint, QPointF, QRect, Qt
from PySide6.QtWidgets import QApplication, QFrame, QGroupBox, QLabel, QSplitter, QTableWidget

from src.data.geo import GeoOrigin
from tests.llt._geo_route import geodetic_config
from src.runner.sim_control import (
    NodeState as ControllerNodeState,
    RallyPlanGeometryState,
    SimulationSnapshot as ControllerSnapshot,
)
from src.ui.gui.main_window import (
    ControllerSimulationAdapter,
    MainWindow,
    LinkState,
    NodeState,
    ReferenceRoute,
    Snapshot,
    TrailPoint,
    default_project_root,
    run_gui,
)
from src.ui.gui.view_models import (
    ObstacleView,
    PLAYBACK_RATE_SLIDER_MAX,
    RallyGeometryView,
    is_major_grid_line,
    trail_seconds_for_duration,
)
from src.ui.gui.node_card_view_model import CardLayoutConfig, card_rect_for
from src.ui.gui.side_view import SideView
from src.ui.gui.top_view import NODE_CARD_GAP_X, NODE_CARD_GAP_Y, NODE_CARD_HEIGHT, NODE_CARD_WIDTH, TopView


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

    def test_main_window_uses_project_application_icon(self) -> None:
        """主窗口标题栏应显示项目图标，而不是 Qt 默认图标。"""
        self.assertFalse(self.window.windowIcon().isNull())

    def test_main_window_defaults_to_dark_theme_and_platform_color_scheme(self) -> None:
        """主窗口默认使用深色主题，并同步请求原生窗口采用深色外观。"""
        self.assertEqual(self.window.theme_key, "dark")
        self.assertTrue(self.window.dark_theme_action.isChecked())
        self.assertFalse(self.window.light_theme_action.isChecked())

        with patch("src.ui.gui.main_window_style.QGuiApplication.styleHints") as style_hints:
            self.window._apply_theme()
        style_hints.return_value.setColorScheme.assert_called_once_with(Qt.ColorScheme.Dark)

        with patch("src.ui.gui.main_window_style.QGuiApplication.styleHints") as style_hints:
            self.window.light_theme_action.trigger()
        self.app.processEvents()

        self.assertEqual(self.window.theme_key, "light")
        style_hints.return_value.setColorScheme.assert_called_once_with(Qt.ColorScheme.Light)

    def test_plain_labels_blend_into_panel_in_both_themes(self) -> None:
        """普通文字标签应透出面板底色，显式状态胶囊仍保留自己的字段底色。"""
        config_group = next(group for group in self.window.findChildren(QGroupBox) if group.title() == "配置")
        duration_label = next(label for label in config_group.findChildren(QLabel) if label.text() == "时长(s)")

        for theme_key in ("dark", "light"):
            with self.subTest(theme_key=theme_key):
                self.window._set_theme(theme_key)
                self.app.processEvents()

                group_image = config_group.grab().toImage()
                label_corner_pos = duration_label.mapTo(
                    config_group, QPoint(duration_label.width() - 1, duration_label.height() - 1)
                )
                label_corner = group_image.pixelColor(label_corner_pos)
                self.assertEqual(label_corner.name(), self.window.theme.panel.name())

                pill_image = self.window.run_state_label.grab().toImage()
                pill_corner = pill_image.pixelColor(pill_image.width() - 1, pill_image.height() - 1)
                self.assertEqual(pill_corner.name(), self.window.theme.field.name())

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

    def test_views_read_blocked_route_segments_from_snapshot(self) -> None:
        """验证俯视图和侧视图只读取快照中的封锁航段，空列表保持为空。"""

        snapshot = Snapshot(
            time=0.0,
            duration=1.0,
            step=0.1,
            run_state="READY",
            control_report="待命",
            disturbance="无",
            nodes=[],
            links=[],
        )
        top_view = TopView.__new__(TopView)
        side_view = SideView.__new__(SideView)
        top_view.snapshot = snapshot
        side_view.snapshot = snapshot
        self.assertEqual(top_view._blocked_route_segments(), [])
        self.assertEqual(side_view._blocked_route_segments(), [])

        blocked = ReferenceRoute(0.0, 0.0, 100.0, 80.0, 0.0, 100.0)
        snapshot.blocked_route_segments = [blocked]
        self.assertEqual(top_view._blocked_route_segments(), [blocked])
        self.assertEqual(side_view._blocked_route_segments(), [blocked])

    def test_node_card_switches_anchor_near_viewport_edge_in_real_top_view(self) -> None:
        """在真实 TopView 渲染路径上复现评审场景一：贴边节点的卡片不得覆盖属主机体。

        固定 scale/offset（不用 fit_view 自动铺满）以精确控制屏幕坐标，
        使节点落在 640x420 视口的 (620, 30) 附近，与评审给出的最小复现一致。
        """

        top_view = TopView()
        top_view.resize(640, 420)
        top_view.scale_value = 1.0
        top_view.offset = QPointF(620.0, 30.0)
        snapshot = Snapshot(
            time=0.0,
            duration=1.0,
            step=0.1,
            run_state="READY",
            control_report="待命",
            disturbance="无",
            nodes=[NodeState("A01", "leader", 0.0, 0.0, 60.0, 0.0)],
            links=[],
        )
        top_view.set_snapshot(snapshot, fit_view=False)
        self.app.processEvents()

        point = next(p for p in top_view._node_screen_points() if p.node_id == "A01")
        self.assertAlmostEqual(point.x, 620.0)
        self.assertAlmostEqual(point.y, 30.0)
        self.assertTrue(top_view.cards.is_card_shown("A01"))

        rect = card_rect_for(
            point,
            CardLayoutConfig(
                NODE_CARD_WIDTH,
                NODE_CARD_HEIGHT,
                NODE_CARD_GAP_X,
                NODE_CARD_GAP_Y,
                viewport_width=640.0,
                viewport_height=420.0,
            ),
        )
        contains_owner = rect.x <= point.x <= rect.x + rect.w and rect.y <= point.y <= rect.y + rect.h
        self.assertFalse(contains_owner, "贴边节点的卡片矩形不得覆盖属主飞机本体")
        self.assertGreaterEqual(rect.x, 0.0)
        self.assertGreaterEqual(rect.y, 0.0)
        self.assertLessEqual(rect.x + rect.w, 640.0)
        self.assertLessEqual(rect.y + rect.h, 420.0)

        top_view.close()

    def test_node_card_hidden_and_not_blocking_when_owner_leaves_viewport(self) -> None:
        """在真实 TopView 渲染路径上复现评审场景二：离屏节点不产生孤立卡片、不挡屏内卡片。"""

        top_view = TopView()
        top_view.resize(640, 420)
        top_view.scale_value = 1.0
        # 偏移使 A01 落在视口左侧之外 (-120, 210)，A02 留在屏内。
        top_view.offset = QPointF(0.0, 0.0)
        snapshot = Snapshot(
            time=0.0,
            duration=1.0,
            step=0.1,
            run_state="READY",
            control_report="待命",
            disturbance="无",
            nodes=[
                NodeState("A01", "wingman", -120.0, -210.0, 60.0, 0.0),
                NodeState("A02", "leader", 40.0, -210.0, 60.0, 0.0),
            ],
            links=[],
        )
        top_view.set_snapshot(snapshot, fit_view=False)
        self.app.processEvents()

        self.assertFalse(top_view.cards.is_card_shown("A01"))
        self.assertTrue(top_view.cards.is_card_shown("A02"))

        top_view.close()

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

    def test_macos_wraps_avoidance_and_3d_entries_in_top_level_menus(self) -> None:
        """macOS 原生菜单栏必须显示避障和 3D 顶层入口。"""

        # 裸 QAction 在 macOS 原生菜单栏会被忽略，两个入口都应改由 QMenu 承载。
        with (
            patch("src.ui.gui.main_window_layout.sys.platform", "darwin"),
            patch("src.ui.gui.features.full.situation3d.sys.platform", "darwin"),
        ):
            window = MainWindow(auto_load_config=False)

        try:
            self.assertEqual(window.avoidance_menu.title(), "避障规划(&O)")
            self.assertEqual(window.situation3d_menu.title(), "3D态势(&3)")
            self.assertIn(window.avoidance_action, window.avoidance_menu.actions())
            self.assertIn(window.situation3d_action, window.situation3d_menu.actions())
        finally:
            window.close()

    def test_situation3d_menu_opens_independent_window(self) -> None:
        self.assertIsNone(self.window.features.situation3d.window)
        menu_titles = [action.text() for action in self.window.menuBar().actions()]
        self.assertIn("3D态势(&3)", menu_titles)

        self.window.situation3d_action.trigger()
        self.app.processEvents()

        first_window = self.window.features.situation3d.window
        self.assertIsNotNone(first_window)
        self.assertTrue(first_window.isVisible())
        self.assertTrue(first_window.isWindow())
        self.assertTrue(first_window.isMaximized())
        self.assertFalse(first_window.isFullScreen())
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
        self.assertGreaterEqual(scene_data["counts"]["trailRibbons"], 1)
        self.assertGreaterEqual(scene_data["counts"]["routePoints"], 2)
        self.assertEqual(scene_data["counts"]["obstacles"], 1)
        self.assertEqual(
            scene_data["obstacles"][0],
            {
                "id": "OBS1",
                "minX": 50.0,
                "maxX": 110.0,
                "minZ": -100.0,
                "maxZ": -40.0,
                "boundsHeight": 720.0,
            },
        )

        self.window._update_snapshot(
            Snapshot(
                time=1.1,
                duration=10.0,
                step=0.1,
                run_state="RUNNING",
                control_report="保持",
                disturbance="无",
                nodes=[
                    NodeState("A01", "leader", 18.0, 27.0, 3.0, 4.0, altitude=36.0),
                    NodeState("A02", "wing", -24.0, 16.0, 2.0, 1.0, altitude=44.0),
                    NodeState("A03", "wing", -28.0, 31.0, 2.0, -1.0, altitude=45.0),
                ],
                links=[],
                route_segments=[ReferenceRoute(0.0, 0.0, 100.0, 100.0, 50.0, 120.0)],
            )
        )
        self.app.processEvents()
        moved_scene_data = json.loads(first_window.bridge.sceneData())
        moved_aircraft = {item["nodeId"]: item for item in moved_scene_data["aircraft"]}
        self.assertEqual(moved_scene_data["time"], 1.1)
        self.assertEqual(moved_aircraft["A01"]["x"], 18.0)
        self.assertEqual(moved_aircraft["A01"]["y"], 36.0)
        self.assertEqual(moved_aircraft["A01"]["z"], -27.0)

        root_object = first_window.quick_view.rootObject()
        self.assertIsNotNone(root_object)
        root_object.setProperty("yaw", 12.0)
        root_object.setProperty("pitch", -20.0)
        root_object.setProperty("distance", 3456.0)
        root_object.setProperty("followEnabled", True)
        root_object.setProperty("followNodeId", "A01")
        self.window._update_snapshot(self.window.sim.snapshot())
        self.app.processEvents()

        self.assertAlmostEqual(float(root_object.property("yaw")), 12.0)
        self.assertAlmostEqual(float(root_object.property("pitch")), -20.0)
        self.assertAlmostEqual(float(root_object.property("distance")), 3456.0)

        self.assertTrue(QMetaObject.invokeMethod(root_object, "resetCamera"))
        self.app.processEvents()
        self.assertAlmostEqual(float(root_object.property("yaw")), scene_data["camera"]["yaw"])
        self.assertAlmostEqual(float(root_object.property("pitch")), scene_data["camera"]["pitch"])
        self.assertAlmostEqual(float(root_object.property("distance")), scene_data["camera"]["distance"])
        self.assertFalse(bool(root_object.property("followEnabled")))
        self.assertEqual(str(root_object.property("followNodeId")), "")

        self.window.situation3d_action.trigger()
        self.app.processEvents()

        self.assertIs(self.window.features.situation3d.window, first_window)
        self.assertTrue(first_window.isVisible())
        self.assertTrue(first_window.isMaximized())
        self.assertFalse(first_window.isFullScreen())

    def test_situation3d_aircraft_follows_real_controller_snapshot(self) -> None:
        """3D 态势应消费真实控制器快照，而不是停留在打开窗口时的初始点。"""

        self._load_ui_config(duration_s=0.2, step_s=0.005, playback_rate=10.0)
        self.window.situation3d_action.trigger()
        self.app.processEvents()

        situation_window = self.window.features.situation3d.window
        self.assertIsNotNone(situation_window)
        initial_scene_data = json.loads(situation_window.bridge.sceneData())
        initial_aircraft = {item["nodeId"]: item for item in initial_scene_data["aircraft"]}

        self.window._start()
        advanced_snapshot = self._wait_for_controller_time(timeout_s=1.0)
        self.window._on_tick()
        self.app.processEvents()

        moved_scene_data = json.loads(situation_window.bridge.sceneData())
        moved_aircraft = {item["nodeId"]: item for item in moved_scene_data["aircraft"]}
        self.assertGreater(advanced_snapshot.time_s, initial_scene_data["time"])
        self.assertNotEqual(moved_aircraft["A01"]["x"], initial_aircraft["A01"]["x"])
        self.assertNotEqual(moved_aircraft["A01"]["z"], initial_aircraft["A01"]["z"])

    def test_situation3d_drag_helpers_use_grabbed_scene_direction(self) -> None:
        """验证 3D 旋转和平移按抓住场景拖动的方向更新。"""

        self.window.situation3d_action.trigger()
        self.app.processEvents()
        situation_window = self.window.features.situation3d.window
        self.assertIsNotNone(situation_window)
        root_object = situation_window.quick_view.rootObject()
        self.assertIsNotNone(root_object)

        root_object.setProperty("distance", 1800.0)
        root_object.setProperty("focusX", 0.0)
        root_object.setProperty("focusZ", 0.0)
        root_object.setProperty("yaw", 0.0)
        root_object.applyGroundPan(120.0, 0.0)
        self.assertAlmostEqual(float(root_object.property("focusX")), -120.0, delta=1e-6)
        self.assertAlmostEqual(float(root_object.property("focusZ")), 0.0, delta=1e-6)

        root_object.setProperty("focusX", 0.0)
        root_object.setProperty("focusZ", 0.0)
        root_object.applyGroundPan(0.0, 120.0)
        self.assertAlmostEqual(float(root_object.property("focusX")), 0.0, delta=1e-6)
        self.assertAlmostEqual(float(root_object.property("focusZ")), -120.0, delta=1e-6)

        root_object.setProperty("focusX", 0.0)
        root_object.setProperty("focusZ", 0.0)
        root_object.setProperty("yaw", -90.0)
        root_object.applyGroundPan(120.0, 0.0)
        self.assertAlmostEqual(float(root_object.property("focusX")), 0.0, delta=1e-6)
        self.assertAlmostEqual(float(root_object.property("focusZ")), -120.0, delta=1e-6)

        root_object.setProperty("focusX", 0.0)
        root_object.setProperty("focusZ", 0.0)
        root_object.setProperty("yaw", -38.0)
        root_object.applyGroundPan(100.0, 0.0)
        self.assertAlmostEqual(float(root_object.property("focusX")), -100.0 * math.cos(math.radians(-38.0)), delta=1e-6)
        self.assertAlmostEqual(float(root_object.property("focusZ")), 100.0 * math.sin(math.radians(-38.0)), delta=1e-6)

        root_object.setProperty("yaw", -38.0)
        root_object.setProperty("pitch", -34.0)
        root_object.applyCameraDrag(40.0, 10.0, 500.0)
        self.assertAlmostEqual(float(root_object.property("yaw")), -48.0, delta=1e-6)
        self.assertAlmostEqual(float(root_object.property("pitch")), -35.8, delta=1e-6)

        root_object.setProperty("yaw", -38.0)
        root_object.setProperty("pitch", -34.0)
        root_object.applyCameraDrag(40.0, 10.0, 120.0)
        self.assertAlmostEqual(float(root_object.property("yaw")), -28.0, delta=1e-6)
        self.assertAlmostEqual(float(root_object.property("pitch")), -35.8, delta=1e-6)

        root_object.applyCameraDrag(0.0, 1000.0, 500.0)
        self.assertAlmostEqual(float(root_object.property("pitch")), -88.0, delta=1e-6)
        root_object.applyCameraDrag(0.0, -1000.0, 500.0)
        self.assertAlmostEqual(float(root_object.property("pitch")), -6.0, delta=1e-6)

    def test_data_analysis_menu_opens_independent_window(self) -> None:
        self.window._open_data_analysis_window()
        self.app.processEvents()

        self.assertIsNotNone(self.window.features.data_analysis.window)
        self.assertEqual(self.window.features.data_analysis.window.windowTitle(), "离线控制效果分析")

    def test_lite_profile_hides_trimmed_feature_menus_and_keeps_modules_unloaded(self) -> None:
        """裁剪版应隐藏整块功能入口，并且构造阶段不能导入被裁剪模块。"""
        trimmed_modules = [
            "src.ui.gui.live_monitor",
            "src.ui.gui.offline_plot",
            "src.ui.gui.data_analysis_window",
            "src.data.control_effect_analysis",
            "src.ui.gui.situation3d",
        ]
        for module_name in list(sys.modules):
            if any(module_name == trimmed or module_name.startswith(f"{trimmed}.") for trimmed in trimmed_modules):
                sys.modules.pop(module_name)

        with patch.dict(os.environ, {"SIMU_GUI_FEATURE_PROFILE": "lite"}):
            window = MainWindow(auto_load_config=False)

        try:
            menu_titles = [action.text() for action in window.menuBar().actions()]
            self.assertEqual(window.features.profile, "lite")
            self.assertNotIn("控制监控(&V)", menu_titles)
            self.assertNotIn("数据分析(&D)", menu_titles)
            self.assertNotIn("3D态势(&3)", menu_titles)
            self.assertIn("避障规划(&O)", menu_titles)
            self.assertIn("帮助(&H)", menu_titles)
            for trimmed in trimmed_modules:
                self.assertNotIn(trimmed, sys.modules)

            window._open_live_monitor()
            window._open_data_analysis_window()
            window._open_situation3d_window()
            self.app.processEvents()

            self.assertIsNone(window.features.control_monitor.live_monitor)
            self.assertIsNone(window.features.control_monitor.offline_plot)
            self.assertIsNone(window.features.data_analysis.window)
            self.assertIsNone(window.features.situation3d.window)
        finally:
            window.close()

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

    def test_playback_slider_binding_applies_view_model_output(self) -> None:
        self.assertEqual(self.window.speed_slider.minimum(), 1)
        self.assertEqual(self.window.speed_slider.maximum(), PLAYBACK_RATE_SLIDER_MAX)

        with patch.object(self.window.sim, "set_speed", wraps=self.window.sim.set_speed) as set_speed:
            self.window.speed_slider.setValue(34)
            self.app.processEvents()

        set_speed.assert_called_once_with(23.0)
        self.assertEqual(self.window.speed_label.text(), "23.0x")
        self.assertAlmostEqual(self.window.sim.speed, 23.0)
        self.assertAlmostEqual(self.window.sim.controller.playback_rate, 23.0)

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

    def test_load_config_syncs_playback_rate_to_slider_without_reapplying_rate(self) -> None:
        with patch.object(self.window.sim, "set_speed", wraps=self.window.sim.set_speed) as set_speed:
            self._load_ui_config(playback_rate=2.0)

        set_speed.assert_not_called()
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
                    ground_speed_mps=8.0,
                    vx_mps=0.0,
                    vy_mps=8.0,
                    vz_mps=0.0,
                    nx=0.0,
                    ny=1.0,
                    nz=0.0,
                    n_normal=1.0,
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

    def test_trail_seconds_input_applies_view_model_output_to_controls_and_views(self) -> None:
        self._load_ui_config(duration_s=2400.0)
        expected_seconds = trail_seconds_for_duration(2400.0)

        self.assertAlmostEqual(self.window.trail_seconds_input.value(), expected_seconds)
        self.assertGreaterEqual(self.window.trail_seconds_input.maximum(), expected_seconds)
        self.assertAlmostEqual(self.window.top_view.trail_seconds, expected_seconds)
        self.assertAlmostEqual(self.window.side_view.trail_seconds, expected_seconds)
        self.assertAlmostEqual(self.window.sim.trail_seconds, expected_seconds)

        self.window.trail_seconds_input.setValue(6.5)
        self.app.processEvents()

        self.assertAlmostEqual(self.window.trail_seconds_input.value(), 6.5)
        # 手动改小尾迹不应把长航时放宽后的范围上限缩回默认 600。
        self.assertGreaterEqual(self.window.trail_seconds_input.maximum(), expected_seconds)
        self.assertAlmostEqual(self.window.top_view.trail_seconds, 6.5)
        self.assertAlmostEqual(self.window.side_view.trail_seconds, 6.5)
        self.assertAlmostEqual(self.window.sim.trail_seconds, 6.5)

    def test_loading_same_duration_config_reapplies_trail_view_model_output(self) -> None:
        self._load_ui_config(duration_s=120.0)
        self.window.trail_seconds_input.setValue(6.5)
        self.app.processEvents()

        self._load_ui_config(duration_s=120.0)
        expected_seconds = trail_seconds_for_duration(120.0)

        self.assertAlmostEqual(self.window.trail_seconds_input.value(), expected_seconds)
        self.assertAlmostEqual(self.window.sim.trail_seconds, expected_seconds)

    def test_manual_trail_seconds_change_refreshes_situation3d_snapshot(self) -> None:
        class SpyFeatures:
            """记录快照刷新调用。注意：只替代本用例需要的 feature 接口。"""

            def __init__(self) -> None:
                """初始化 SpyFeatures 实例，建立后续断言所需状态。"""

                self.snapshots: list[Snapshot] = []

            def on_snapshot_updated(self, window: MainWindow, snapshot: Snapshot) -> None:
                """记录 3D 态势快照刷新。注意：window 参数保留接口一致性。"""

                self.snapshots.append(snapshot)

            def close(self) -> None:
                """兼容主窗口关闭流程。注意：测试替身没有真实资源需要释放。"""

        self._load_ui_config(duration_s=120.0)
        spy = SpyFeatures()
        self.window.features = spy

        self.window.trail_seconds_input.setValue(6.5)
        self.app.processEvents()

        self.assertEqual(len(spy.snapshots), 1)
        self.assertAlmostEqual(self.window.sim.trail_seconds, 6.5)
        self.assertAlmostEqual(spy.snapshots[0].duration, 120.0)

    def test_adapter_trail_seconds_zero_outputs_no_trail(self) -> None:
        node = ControllerNodeState(
            node_id="A01",
            role="leader",
            health="normal",
            x_m=100.0,
            y_m=200.0,
            altitude_m=1200.0,
            psi_v_deg=90.0,
            theta_deg=0.0,
            speed_mps=8.0,
            ground_speed_mps=8.0,
            vx_mps=0.0,
            vy_mps=8.0,
            vz_mps=0.0,
            nx=0.0,
            ny=1.0,
            nz=0.0,
            n_normal=1.0,
            phi_deg=0.0,
            psi_dot_deg_s=0.0,
        )
        snapshot = ControllerSnapshot(
            time_s=1.0,
            duration_s=10.0,
            step_s=0.1,
            run_state="RUNNING",
            control_report="保持",
            cpu_utilization=0.42,
            nodes=[node],
            links=[],
        )
        self.window.sim._convert_snapshot(snapshot)
        self.window.sim._convert_snapshot(replace(snapshot, time_s=2.0, nodes=[replace(node, x_m=108.0)]))

        self.window.sim.set_trail_seconds(0.0)
        closed = self.window.sim._convert_snapshot(replace(snapshot, time_s=3.0, nodes=[replace(node, x_m=116.0)]))

        self.assertEqual(closed.nodes[0].trail, [])
        self.assertEqual(self.window.sim._trail_by_node, {})

    def test_adapter_keeps_monotonic_trail_distance_after_old_points_are_pruned(self) -> None:
        node = ControllerNodeState(
            node_id="A02",
            role="wingman",
            health="normal",
            x_m=100.0,
            y_m=200.0,
            altitude_m=1200.0,
            psi_v_deg=90.0,
            theta_deg=0.0,
            speed_mps=12.0,
            ground_speed_mps=12.0,
            vx_mps=12.0,
            vy_mps=0.0,
            vz_mps=0.0,
            nx=0.0,
            ny=1.0,
            nz=0.0,
            n_normal=1.0,
            phi_deg=0.0,
            psi_dot_deg_s=0.0,
        )
        snapshot = ControllerSnapshot(
            time_s=1.0,
            duration_s=10.0,
            step_s=0.1,
            run_state="RUNNING",
            control_report="保持",
            cpu_utilization=0.42,
            nodes=[node],
            links=[],
        )
        self.window.sim.set_trail_seconds(1.5)
        # 本用例直接构造控制器固定时钟样本；_convert_snapshot 只读队列，不再按 GUI 帧入队。
        self.window.sim._append_trail_sample(snapshot)
        self.window.sim._append_trail_sample(replace(snapshot, time_s=2.0, nodes=[replace(node, x_m=112.0)]))
        self.window.sim._append_trail_sample(
            replace(snapshot, time_s=3.0, nodes=[replace(node, x_m=124.0)])
        )
        self.window.sim._trail_by_node["A02"].expire(3.0, 1.5)
        converted = self.window.sim._convert_snapshot(replace(snapshot, time_s=3.0, nodes=[replace(node, x_m=124.0)]))

        self.assertEqual([point.x for point in converted.nodes[0].trail], [112.0, 124.0])
        self.assertEqual([point.path_distance for point in converted.nodes[0].trail], [12.0, 24.0])

    def test_adapter_trail_sampling_is_independent_of_gui_poll_frequency(self) -> None:
        """相同仿真区间必须得到相同 10 Hz 尾迹，不能把 GUI 轮询频率当采样时钟。"""

        self._load_ui_config(duration_s=2.0, playback_rate=7.0)
        for _ in range(7):
            self.assertEqual(self.window.sim.controller.step(20).code, "OK")
            frequent = self.window.sim.snapshot()
        frequent_times = [round(point.time, 6) for point in frequent.nodes[0].trail]

        reset = self.window.sim.reset()
        self.assertEqual(self.window.sim.last_result_code, "OK")
        self.assertEqual([point.time for point in reset.nodes[0].trail], [0.0])
        self.assertEqual(self.window.sim.controller.step(140).code, "OK")
        delayed = self.window.sim.snapshot()
        delayed_times = [round(point.time, 6) for point in delayed.nodes[0].trail]

        self.assertEqual(frequent_times, [round(index * 0.1, 6) for index in range(8)])
        self.assertEqual(delayed_times, frequent_times)

    def test_adapter_keeps_current_aircraft_position_outside_stable_trail_queue(self) -> None:
        """固定采样间隙内只更新飞机实时端点，稳定队列不得追加墙钟轮询点。"""

        self._load_ui_config(duration_s=1.0, playback_rate=7.0)
        self.assertEqual(self.window.sim.controller.step(1).code, "OK")

        converted = self.window.sim.snapshot()

        self.assertAlmostEqual(converted.time, 0.005)
        self.assertEqual([point.time for point in converted.nodes[0].trail], [0.0])

    def test_adapter_does_not_backfill_samples_recorded_while_trail_is_disabled(self) -> None:
        """关闭期间只推进固定样本游标，重新开启必须从当前机位建立新尾迹。"""

        self._load_ui_config(duration_s=1.0, playback_rate=7.0)
        self.assertEqual(self.window.sim.controller.step(40).code, "OK")
        before_close = self.window.sim.snapshot()
        self.assertEqual([round(point.time, 6) for point in before_close.nodes[0].trail], [0.0, 0.1, 0.2])

        self.window.sim.set_trail_seconds(0.0)
        self.assertEqual(self.window.sim.controller.step(40).code, "OK")
        self.window.sim.set_trail_seconds(1.0)
        reopened = self.window.sim.snapshot()

        self.assertEqual([round(point.time, 6) for point in reopened.nodes[0].trail], [0.4])

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

    def test_top_grid_uses_dashed_major_lines_every_five_cells(self) -> None:
        view = self.window.top_view
        view.snapshot = None
        view.scale_value = 1.0
        view.offset = QPointF(40.0, 40.0)
        view.viewport().update()
        self.app.processEvents()

        image = view.viewport().grab().toImage()
        spacing = view._grid_world_spacing()
        canvas = self.window.theme.canvas
        major_distances = [
            self._color_distance(image.pixelColor(round(view.offset.x()), y), canvas) for y in range(8, 32)
        ]
        minor_distances = [
            self._color_distance(image.pixelColor(round(view.offset.x() + spacing), y), canvas) for y in range(8, 32)
        ]

        self.assertGreater(max(major_distances), max(minor_distances))
        self.assertLess(min(major_distances), max(major_distances))
        self.assertGreater(min(minor_distances), 0)

    def test_side_grid_uses_dashed_major_lines_every_five_cells(self) -> None:
        view = self.window.side_view
        view.snapshot = None
        view.horizontal_offset = 73.0
        view.horizontal_scale = 1.0
        view.update()
        self.app.processEvents()

        image = view.grab().toImage()
        spacing = view._grid_world_spacing()
        canvas = self.window.theme.canvas
        major_distances = [
            self._color_distance(image.pixelColor(round(view._map_x(0.0)), y), canvas) for y in range(8, 32)
        ]
        minor_distances = [
            self._color_distance(image.pixelColor(round(view._map_x(float(spacing))), y), canvas)
            for y in range(8, 32)
        ]

        self.assertGreater(max(major_distances), max(minor_distances))
        self.assertLess(min(major_distances), max(major_distances))
        self.assertGreater(min(minor_distances), 0)

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

    def test_major_grid_line_repeats_every_five_cells_for_positive_and_negative_coordinates(self) -> None:
        spacing = 48
        cases = (
            (-6, False),
            (-5, True),
            (-4, False),
            (4, False),
            (5, True),
            (6, False),
            (10, True),
        )

        for multiple, expected in cases:
            with self.subTest(multiple=multiple):
                self.assertEqual(is_major_grid_line(multiple * spacing, spacing), expected)

    @staticmethod
    def _color_distance(first, second) -> int:  # noqa: ANN001
        """返回两种渲染颜色的 RGB 曼哈顿距离。注意：仅比较可见色差。"""

        return abs(first.red() - second.red()) + abs(first.green() - second.green()) + abs(first.blue() - second.blue())

    def test_top_view_aircraft_marker_keeps_screen_size_during_zoom(self) -> None:
        small = self._leader_marker_bounds_at_scale(0.45)
        large = self._leader_marker_bounds_at_scale(3.5)

        self.assertAlmostEqual(small.width(), large.width(), delta=4)
        self.assertAlmostEqual(small.height(), large.height(), delta=4)
        # 绝对上限对应改进后的俯视图机型图标(实测 31×22，旧小三角为 14×10)；核心是上面两条"尺寸不随缩放变"。
        self.assertLessEqual(large.width(), 40)
        self.assertLessEqual(large.height(), 30)

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

    def test_top_view_uses_thin_cyan_dashes_for_normal_communication_links(self) -> None:
        view = self.window.top_view
        view.scale_value = 2.0
        snapshot = Snapshot(
            time=0.0,
            duration=1.0,
            step=0.1,
            run_state="READY",
            control_report="",
            disturbance="无",
            nodes=[
                NodeState("A01", "leader", 0.0, 0.0, 1.0, 0.0),
                NodeState("A02", "wingman", 80.0, 0.0, 1.0, 0.0),
            ],
            links=[
                LinkState("A01", "A02", "duplex", 18, 0.01, True),
                LinkState("A02", "A01", "duplex", 80, 0.30, False),
            ],
        )
        painter = Mock()

        view._draw_links(painter, snapshot)

        normal_pen, abnormal_pen = [call.args[0] for call in painter.setPen.call_args_list]
        self.assertEqual(normal_pen.color().name(), self.window.theme.link.name())
        self.assertAlmostEqual(normal_pen.widthF(), 0.5)
        self.assertEqual(normal_pen.style(), Qt.PenStyle.CustomDashLine)
        self.assertEqual(abnormal_pen.color().name(), self.window.theme.warn.name())
        self.assertAlmostEqual(abnormal_pen.widthF(), 1.5)
        self.assertEqual(abnormal_pen.style(), Qt.PenStyle.SolidLine)

    def test_top_view_uses_thin_gray_blue_dashes_for_unflown_reference_route(self) -> None:
        view = self.window.top_view
        view.scale_value = 2.0
        view.snapshot = Snapshot(
            time=0.0,
            duration=1.0,
            step=0.1,
            run_state="READY",
            control_report="",
            disturbance="无",
            nodes=[],
            links=[],
            route=ReferenceRoute(0.0, 0.0, 1200.0, 100.0, 0.0, 1200.0),
        )
        painter = Mock()

        view._draw_route(painter)

        route_pen = painter.setPen.call_args_list[0].args[0]
        self.assertEqual(route_pen.color().name(), self.window.theme.formation_reference.name())
        self.assertAlmostEqual(route_pen.widthF(), 0.5)
        self.assertEqual(route_pen.style(), Qt.PenStyle.CustomDashLine)

    def test_link_visibility_checkbox_hides_top_view_links(self) -> None:
        self.assertTrue(self.window.legend_link.isChecked())
        self.assertTrue(self.window.top_view.show_links)
        self.assertTrue(self._link_stroke_rows_at_scale(1.0))

        self.window.legend_link.setChecked(False)
        self.app.processEvents()

        self.assertFalse(self.window.top_view.show_links)
        self.assertEqual(self._link_stroke_rows_at_scale(1.0), [])

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

    def test_side_view_owns_manual_view_signal_and_main_window_bridges_it(self) -> None:
        """侧视图手动操作信号由自身声明，主窗口负责关闭自动居中。"""

        self.window.auto_center.setChecked(True)
        self.app.processEvents()

        self.assertFalse(hasattr(self.window.side_view, "top_view"))
        self.window.side_view.manualViewChanged.emit()
        self.app.processEvents()

        self.assertFalse(self.window.auto_center.isChecked())
        self.assertFalse(self.window.top_view.auto_center)
        self.assertFalse(self.window.side_view.auto_center)

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

    def test_auto_center_survives_top_view_wheel_zoom(self) -> None:
        """俯视图滚轮缩放只改变倍率，不应关闭自动居中或移走编队中心。"""
        self._load_ui_config()
        self.window.auto_center.setChecked(True)
        self.app.processEvents()
        view = self.window.top_view
        old_scale = view.scale_value
        event = Mock()
        event.pixelDelta.return_value = QPointF(0.0, 0.0)
        event.angleDelta.return_value = QPointF(0.0, 120.0)
        event.position.return_value = QPointF(120.0, 160.0)

        view.wheelEvent(event)
        self.app.processEvents()

        self.assertTrue(self.window.auto_center.isChecked())
        self.assertTrue(self.window.top_view.auto_center)
        self.assertTrue(self.window.side_view.auto_center)
        self.assertGreater(view.scale_value, old_scale)
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
        event.accept.assert_called_once_with()

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

    def test_auto_center_survives_side_view_wheel_zoom(self) -> None:
        """侧视图滚轮缩放只改变横轴倍率，不应关闭自动居中或移走编队中心。"""
        self._load_ui_config()
        self.window.auto_center.setChecked(True)
        self.app.processEvents()
        view = self.window.side_view
        old_scale = view.horizontal_scale
        altitude_span = view.altitude_max - view.altitude_min
        event = Mock()
        event.pixelDelta.return_value = QPointF(0.0, 0.0)
        event.angleDelta.return_value = QPointF(0.0, 120.0)
        event.position.return_value = QPointF(180.0, 80.0)

        view.wheelEvent(event)
        self.app.processEvents()

        self.assertTrue(self.window.auto_center.isChecked())
        self.assertTrue(self.window.top_view.auto_center)
        self.assertTrue(self.window.side_view.auto_center)
        self.assertGreater(view.horizontal_scale, old_scale)
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
        self.assertAlmostEqual(view.altitude_max - view.altitude_min, altitude_span, delta=0.01)
        self.assertAlmostEqual(
            (view.altitude_min + view.altitude_max) / 2.0,
            center_altitude,
            delta=0.01,
        )
        event.accept.assert_called_once_with()

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
        view.trail_seconds = 5.0
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

    def test_top_view_keeps_leader_trail_solid_and_draws_wingman_trail_dashed(self) -> None:
        view = self.window.top_view
        view.trail_seconds = 10.0
        trail = [
            TrailPoint(0.0, 0.0, 1200.0, 0.0, 0.0),
            TrailPoint(12.0, 0.0, 1200.0, 1.0, 12.0),
            TrailPoint(24.0, 0.0, 1200.0, 2.0, 24.0),
        ]

        leader_painter = Mock()
        view._draw_trail(leader_painter, NodeState("A01", "leader", 24.0, 0.0, 1.0, 0.0, trail=trail), True, 2.0)
        leader_pens = [call.args[0] for call in leader_painter.setPen.call_args_list]

        wingman_painter = Mock()
        view._draw_trail(wingman_painter, NodeState("A02", "wingman", 24.0, 0.0, 1.0, 0.0, trail=trail), False, 2.0)
        wingman_pens = [call.args[0] for call in wingman_painter.setPen.call_args_list]

        self.assertTrue(leader_pens)
        self.assertTrue(wingman_pens)
        self.assertTrue(all(pen.style() == Qt.PenStyle.SolidLine for pen in leader_pens))
        self.assertTrue(all(pen.style() == Qt.PenStyle.CustomDashLine for pen in wingman_pens))
        self.assertEqual(leader_painter.drawPath.call_count, 1)
        self.assertEqual(wingman_painter.drawPath.call_count, 1)
        self.assertEqual(wingman_pens[0].capStyle(), Qt.PenCapStyle.RoundCap)
        self.assertEqual(wingman_pens[0].joinStyle(), Qt.PenJoinStyle.RoundJoin)

    def test_top_view_rally_geometry_draws_only_circles_without_static_line_or_markers(self) -> None:
        """集结辅助几何只绘制两个盘旋圆，不绘制静态线、点标记及标签。"""

        view = self.window.top_view
        snapshot = Snapshot(
            time=1.0,
            duration=10.0,
            step=0.1,
            run_state="RUNNING",
            control_report="集结",
            disturbance="无",
            nodes=[NodeState("A01", "rally_leader", 0.0, 0.0, 20.0, 0.0, rally_phase="RALLY_TRANSIT")],
            links=[],
            rally_geometry=[
                RallyGeometryView(
                    node_id="A01",
                    center_x=100.0,
                    center_y=50.0,
                    radius=50.0,
                    local_center_x=0.0,
                    local_center_y=50.0,
                    local_radius=50.0,
                )
            ],
        )
        painter = Mock()

        with patch.object(view, "_draw_screen_text") as draw_screen_text:
            view._draw_rally_geometry(painter, snapshot)

        self.assertEqual(painter.drawEllipse.call_count, 2)
        painter.drawLine.assert_not_called()
        painter.drawRect.assert_not_called()
        painter.drawPath.assert_not_called()
        draw_screen_text.assert_not_called()

    def test_top_view_wingman_dash_offset_stays_stable_after_oldest_trail_point_is_pruned(self) -> None:
        view = self.window.top_view
        view.trail_seconds = 10.0
        full_trail = [
            TrailPoint(0.0, 0.0, 1200.0, 0.0, 0.0),
            TrailPoint(12.0, 0.0, 1200.0, 1.0, 12.0),
            TrailPoint(24.0, 0.0, 1200.0, 2.0, 24.0),
        ]

        full_painter = Mock()
        view._draw_trail(full_painter, NodeState("A02", "wingman", 24.0, 0.0, 1.0, 0.0, trail=full_trail), False, 2.0)
        full_pens = [call.args[0] for call in full_painter.setPen.call_args_list]

        pruned_painter = Mock()
        view._draw_trail(
            pruned_painter,
            NodeState("A02", "wingman", 24.0, 0.0, 1.0, 0.0, trail=full_trail[1:]),
            False,
            2.0,
        )
        pruned_pens = [call.args[0] for call in pruned_painter.setPen.call_args_list]

        self.assertAlmostEqual(full_pens[0].dashOffset(), 0.0)
        self.assertAlmostEqual(pruned_pens[0].dashOffset(), 12.0 * view.scale_value / 2.4)

    def test_top_view_wingman_dash_offset_stays_stable_after_new_trail_point_is_appended(self) -> None:
        view = self.window.top_view
        view.trail_seconds = 10.0
        full_trail = [
            TrailPoint(0.0, 0.0, 1200.0, 0.0, 0.0),
            TrailPoint(12.0, 0.0, 1200.0, 1.0, 12.0),
            TrailPoint(24.0, 0.0, 1200.0, 2.0, 24.0),
        ]

        before_painter = Mock()
        view._draw_trail(before_painter, NodeState("A02", "wingman", 24.0, 0.0, 1.0, 0.0, trail=full_trail), False, 2.0)
        before_pens = [call.args[0] for call in before_painter.setPen.call_args_list]

        after_painter = Mock()
        view._draw_trail(
            after_painter,
            NodeState(
                "A02",
                "wingman",
                36.0,
                0.0,
                1.0,
                0.0,
                trail=[*full_trail, TrailPoint(36.0, 0.0, 1200.0, 3.0, 36.0)],
            ),
            False,
            3.0,
        )
        after_pens = [call.args[0] for call in after_painter.setPen.call_args_list]

        self.assertEqual([pen.dashOffset() for pen in before_pens], [pen.dashOffset() for pen in after_pens[:2]])

    def test_top_view_trail_seconds_zero_hides_trail(self) -> None:
        view = self.window.top_view
        view.show_grid = False
        view.trail_seconds = 0.0
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

        self.assertEqual(touched, [])

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
        self.assertFalse(self.window.rally_button.isEnabled())
        self.assertFalse(self.window.step_button.isEnabled())
        self.assertFalse(self.window.reset_button.isEnabled())
        self.assertTrue(all(not button.isEnabled() for button in self.window.disturbance_buttons))

    def test_adapter_preserves_only_visible_rally_circle_geometry_fields(self) -> None:
        """验证 GUI 适配层只保留本地待命圆和集结圆字段。"""

        adapter = ControllerSimulationAdapter()
        self.addCleanup(adapter.close)
        snapshot = ControllerSnapshot(
            time_s=0.0,
            duration_s=1.0,
            step_s=0.1,
            run_state="READY",
            control_report="待命",
            nodes=[],
            links=[],
            rally_geometry={
                "R01": RallyPlanGeometryState(
                    node_id="R01",
                    local_center_east_m=1.0,
                    local_center_north_m=2.0,
                    local_radius_m=3.0,
                    rally_center_east_m=4.0,
                    rally_center_north_m=5.0,
                    rally_radius_m=6.0,
                )
            },
        )

        converted = adapter._convert_snapshot(snapshot)
        geometry = converted.rally_geometry[0]

        self.assertEqual(geometry.local_center_x, 1.0)
        self.assertEqual(geometry.center_x, 4.0)
        for removed_field in ("entry_x", "entry_y", "local_tangent_x", "local_tangent_y", "fallback_used"):
            self.assertFalse(hasattr(geometry, removed_field))

    def test_run_gui_opens_main_window_maximized(self) -> None:
        """验证正式启动入口默认以最大化方式显示主窗口。"""
        calls: list[str] = []

        class FakeApp:
            """替代 QApplication，避免测试进入真实事件循环。"""

            def __init__(self, argv: list[str]) -> None:
                self.argv = argv

            def setWindowIcon(self, icon) -> None:  # noqa: ANN001, N802
                """记录应用图标设置，并验证加载到的项目图标有效。"""
                self.icon = icon
                self.assert_icon_is_valid(icon)
                calls.append("setWindowIcon")

            @staticmethod
            def assert_icon_is_valid(icon) -> None:  # noqa: ANN001
                """确保正式启动入口没有传入空图标。"""
                if icon.isNull():
                    raise AssertionError("应用图标不能为空")

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
        self.assertEqual(calls, ["setWindowIcon", "showMaximized", "exec"])

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

    def test_rally_button_starts_rally_from_local_loiter(self) -> None:
        """验证集结按钮只在本地待命盘旋阶段可用，点击后切入集结流程。"""

        config_path = Path("configs/rally_demo_5_aircraft.json").resolve()
        self.window._apply_config_path(str(config_path))
        self.app.processEvents()

        self.assertEqual(self.window.rally_button.text(), "集结")
        self.assertFalse(self.window.rally_button.isEnabled())

        self.window._step()
        self.app.processEvents()
        standby_snapshot = self.window.sim.snapshot()

        self.assertEqual(standby_snapshot.run_state, "PAUSED")
        self.assertEqual(standby_snapshot.control_report, "待命")
        self.assertTrue(self.window.rally_button.isEnabled())

        self.window.rally_button.click()
        self.app.processEvents()
        rally_snapshot = self.window.sim.snapshot()

        self.assertEqual(self.window.sim.last_result_code, "OK")
        self.assertEqual(rally_snapshot.control_report, "集结")
        self.assertTrue(self.window.rally_button.isEnabled())

        self.window.rally_button.click()
        self.app.processEvents()

        self.assertEqual(self.window.sim.last_result_code, "ERR_INVALID_STATE")
        self.assertIn("已在集结中", self.window.sim.last_result_message)

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
            self._write_config_file(project_root / "configs" / "startup.json")
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
            config_path.write_text(json.dumps(geodetic_config(config)), encoding="utf-8")
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
            self._write_config_file(app_dir / "configs" / "packaged.json")
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
                    "src.ui.gui.main_window_actions.QFileDialog.getOpenFileName",
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
            self._write_config_file(project_root / "configs" / "first.json")
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

    def test_node_table_shows_slot_deviation_and_overall_table_uses_leader_route_metrics(self) -> None:
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
                NodeState("A02", "wingman", 80.0, 90.0, 20.0, 0.0, 1210.0),
            ],
            links=[],
        )

        self.window._update_snapshot(snapshot)

        self.assertEqual(self.window.node_table.columnCount(), 5)
        # 表头文案与悬浮提示属于布局层契约，函数级 VM 测试覆盖不到，保留在 GUI 层验证。
        self.assertEqual(self.window.node_table.horizontalHeaderItem(1).text(), "待飞距(m)")
        self.assertEqual(self.window.node_table.horizontalHeaderItem(2).text(), "高飘(m)")
        self.assertEqual(self.window.node_table.horizontalHeaderItem(3).text(), "右偏(m)")
        self.assertIn("目标-本机", self.window.node_table.horizontalHeaderItem(1).toolTip())
        self.assertIn("本机-目标", self.window.node_table.horizontalHeaderItem(2).toolTip())
        self.assertIn("本机-目标", self.window.node_table.horizontalHeaderItem(3).toolTip())
        self.assertEqual(self.window.node_table.item(0, 1).text(), "1.2")
        self.assertEqual(self.window.node_table.item(0, 1).textAlignment(), int(Qt.AlignmentFlag.AlignCenter))
        self.assertEqual(self.window.overall_table.rowCount(), 1)
        self.assertEqual(self.window.overall_table.columnCount(), 5)
        self.assertEqual(self.window.overall_table.item(0, 0).text(), "12")
        self.assertEqual(self.window.overall_table.item(0, 0).textAlignment(), int(Qt.AlignmentFlag.AlignCenter))
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
                    rally_phase="RALLY_TRANSIT",
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
                NodeState("A01", "wingman", 100.0, 120.0, 20.0, 0.0, 1300.0, cross_track_error=99.4, distance_to_go=888.6),
                NodeState("A05", "leader", 80.0, 90.0, 20.0, 0.0, 1200.0, cross_track_error=12.4, distance_to_go=345.6),
            ],
            links=[],
        )

        self.window._update_snapshot(snapshot)

        self.assertEqual(self.window.overall_table.rowCount(), 1)
        self.assertEqual(self.window.overall_table.item(0, 0).text(), "12")

    def test_link_table_displays_direction_from_controller_snapshot(self) -> None:
        self._load_ui_config(
            links=[
                {"link_id": "A01-A02", "direction": "duplex", "latency_ms": 18.0, "loss_rate": 0.01},
                {"link_id": "A02-A03", "direction": "simplex", "latency_ms": 30.0, "loss_rate": 0.02},
            ]
        )

        self.assertEqual(self.window.link_table.columnCount(), 5)
        self.assertEqual(self.window.link_table.horizontalHeaderItem(1).text(), "方向")
        self.assertEqual(self.window.link_table.item(0, 1).text(), "双向")
        self.assertEqual(self.window.link_table.item(0, 1).textAlignment(), int(Qt.AlignmentFlag.AlignCenter))
        self.assertEqual(self.window.link_table.horizontalScrollBar().maximum(), 0)
        self.assert_table_uses_full_width(self.window.link_table)

    def test_duration_input_syncs_loaded_config_duration(self) -> None:
        self._load_ui_config(duration_s=2400.0)

        self.assertEqual(self.window.duration_input.text(), "2400")
        self.assertAlmostEqual(self.window.trail_seconds_input.value(), trail_seconds_for_duration(2400.0))
        self.assertAlmostEqual(self.window.sim.trail_seconds, trail_seconds_for_duration(2400.0))

    def test_duration_input_updates_controller_duration_on_edit_finished(self) -> None:
        self._load_ui_config(duration_s=2400.0)

        self.window.duration_input.setText("120")
        self.window.duration_input.editingFinished.emit()
        self.app.processEvents()
        snapshot = self.window.sim.controller.get_snapshot()

        self.assertAlmostEqual(snapshot.duration_s, 120.0)
        self.assertAlmostEqual(self.window.trail_seconds_input.value(), trail_seconds_for_duration(120.0))
        self.assertAlmostEqual(self.window.top_view.trail_seconds, trail_seconds_for_duration(120.0))
        self.assertAlmostEqual(self.window.side_view.trail_seconds, trail_seconds_for_duration(120.0))
        self.assertAlmostEqual(self.window.sim.trail_seconds, trail_seconds_for_duration(120.0))

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

        self.assertIsNotNone(self.window.features.control_monitor.live_monitor)
        self.assertIsNotNone(self.window.features.control_monitor.live_monitor._ctrl)
        self.assertIs(self.window.features.control_monitor.live_monitor._ctrl, self.window.sim.controller)

    def test_step_binds_live_monitor_controller(self) -> None:
        """Regression: _step() 未调用 follow()，单步后监控窗口 _ctrl 仍为 None。"""
        try:
            from src.ui.gui.live_monitor import LiveMonitorWindow
        except ModuleNotFoundError:
            self.skipTest("PySide6.QtCharts not available")

        self._load_ui_config()
        self.window.features.control_monitor.live_monitor = LiveMonitorWindow(self.window)
        # _ctrl 未绑定，模拟旧代码中打开监控但未 follow 的状态
        self.assertIsNone(self.window.features.control_monitor.live_monitor._ctrl)

        self.window._step()
        self.app.processEvents()

        self.assertIsNotNone(self.window.features.control_monitor.live_monitor._ctrl)
        self.assertIs(self.window.features.control_monitor.live_monitor._ctrl, self.window.sim.controller)

    def test_reset_keeps_live_monitor_nodes_visible(self) -> None:
        """Regression: 主窗口重置不应让实时监控节点列表消失。"""
        try:
            from src.ui.gui.live_monitor import LiveMonitorWindow  # noqa: F401
        except ModuleNotFoundError:
            self.skipTest("PySide6.QtCharts not available")

        self._load_ui_config()
        self.window._open_live_monitor()
        self.assertIsNotNone(self.window.features.control_monitor.live_monitor)
        monitor = self.window.features.control_monitor.live_monitor
        monitor._poll()
        self.app.processEvents()

        self.assertEqual(sorted(monitor._nodes.keys()), ["A01", "A02", "A03"])

        self.window._reset()
        self.app.processEvents()

        self.assertIs(monitor._ctrl, self.window.sim.controller)
        self.assertEqual(sorted(monitor._nodes.keys()), ["A01", "A02", "A03"])

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
            # 产品约定航线只支持经纬度；把 ENU 航线等价转成经纬后写盘。
            json.dump(geodetic_config(config), handle)
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
        touched = self._link_stroke_rows_at_scale(scale)
        self.assertTrue(touched)
        return max(touched) - min(touched) + 1

    def _link_stroke_rows_at_scale(self, scale: float) -> list[int]:
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
        center_x = round(view.offset.x())
        # 虚线中心可能落在空隙，采样邻近列以稳定识别线宽与显示开关。
        return sorted(
            {
                y
                for x in range(max(0, center_x - 6), min(image.width(), center_x + 7))
                for y in range(image.height())
                if image.pixelColor(x, y).name() != canvas
            }
        )

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
