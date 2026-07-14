"""实时与离线控制图表公共契约测试。"""

from __future__ import annotations

import ast
import os
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication, QGroupBox, QVBoxLayout, QWidget

from src.ui.gui.chart_common import (
    CONTROL_ERROR_CHANNELS,
    build_chart_sidebar,
    heading_deviation,
    refresh_chart_node_panel,
)


class ChartCommonTests(unittest.TestCase):
    """锁定两种图表窗口共用的通道与计算口径。"""

    @classmethod
    def setUpClass(cls) -> None:
        """复用 Qt 应用实例。"""

        cls.app = QApplication.instance() or QApplication([])

    def test_control_error_channels_have_one_stable_definition(self) -> None:
        """七个通道的键、分组与默认勾选状态只由公共表定义。"""

        self.assertEqual(
            tuple(channel.key for channel in CONTROL_ERROR_CHANNELS),
            (
                "perr_x",
                "verr_x",
                "perr_y",
                "verr_y",
                "perr_z",
                "verr_z",
                "hdg_dev",
            ),
        )
        self.assertEqual(
            tuple(channel.key for channel in CONTROL_ERROR_CHANNELS if channel.on),
            ("perr_x", "perr_y", "perr_z"),
        )

    def test_heading_deviation_keeps_enu_wrap_contract(self) -> None:
        """航向偏差按 ENU 口径包络到负 180 至正 180 度。"""

        node = SimpleNamespace(
            cmd_vel_east_mps=0.0,
            cmd_vel_north_mps=10.0,
            psi_v_deg=-170.0,
        )

        self.assertEqual(heading_deviation(node), 100.0)
        node.cmd_vel_north_mps = 0.0
        self.assertIsNone(heading_deviation(node))

    def test_offline_window_does_not_import_live_window_private_symbols(self) -> None:
        """离线窗口只能依赖公共图表模块，不能反向耦合实时窗口私有实现。"""

        path = Path("src/ui/gui/offline_plot.py")
        tree = ast.parse(path.read_text(encoding="utf-8"))
        imported_modules = {
            node.module
            for node in ast.walk(tree)
            if isinstance(node, ast.ImportFrom) and node.module is not None
        }

        self.assertNotIn("src.ui.gui.live_monitor", imported_modules)

    def test_shared_sidebar_builds_channels_and_accepts_window_specific_group(self) -> None:
        """公共侧栏构造器统一节点和通道结构，同时允许实时窗口追加时间组。"""

        time_group = QGroupBox("时间窗口")
        rebuild = Mock()

        sidebar = build_chart_sidebar(
            empty_text="（等待数据）",
            rebuild_charts=rebuild,
            extra_widgets=(time_group,),
        )

        self.assertEqual(sidebar.widget.width(), 170)
        self.assertEqual(
            tuple(sidebar.channel_checkboxes),
            tuple(channel.key for channel in CONTROL_ERROR_CHANNELS),
        )
        self.assertIn(time_group, sidebar.widget.findChildren(QGroupBox))
        sidebar.channel_checkboxes["perr_x"].setChecked(False)
        rebuild.assert_called_once()
        sidebar.widget.deleteLater()

    def test_shared_node_panel_updates_visibility_and_rebuilds_charts(self) -> None:
        """两个窗口共用节点复选框重建和显隐切换语义。"""

        owner = QWidget()
        node_layout = QVBoxLayout(owner)
        nodes = {
            "A01": {"color": "#123456", "visible": True, "cb": None},
        }
        rebuild = Mock()

        refresh_chart_node_panel(
            node_layout,
            nodes,
            empty_text="（等待数据）",
            rebuild_charts=rebuild,
        )
        checkbox = nodes["A01"]["cb"]
        self.assertEqual(checkbox.text(), "A01")
        self.assertIn("#123456", checkbox.styleSheet())

        checkbox.setChecked(False)

        self.assertFalse(nodes["A01"]["visible"])
        rebuild.assert_called_once()
        owner.deleteLater()


if __name__ == "__main__":
    unittest.main()
