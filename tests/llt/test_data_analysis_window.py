"""数据分析窗口回归测试。"""

from __future__ import annotations

import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCharts import QChartView
from PySide6.QtCore import QPoint, Qt
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication, QPushButton

from src.data.control_effect_analysis import GUI_CHANNELS
from src.ui.gui import data_analysis_window as data_analysis_window_module
from src.ui.gui.data_analysis_window import DataAnalysisWindow


class DataAnalysisWindowTests(unittest.TestCase):
    """验证离线分析 UI 的核心联动，不启动 Qt 主事件循环。"""

    app: QApplication

    @classmethod
    def setUpClass(cls) -> None:
        """创建测试进程内共享的 QApplication。"""
        cls.app = QApplication.instance() or QApplication([])

    def tearDown(self) -> None:
        """关闭测试遗留的顶层窗口。"""
        for widget in QApplication.topLevelWidgets():
            widget.close()
        self.app.processEvents()

    def test_load_a_b_sources_refreshes_summary_and_popup_charts(self) -> None:
        """A/B 文件加载和启用状态应同步影响表格与滑动窗口图。"""
        with tempfile.TemporaryDirectory() as tmp:
            a_path = self._write_snapshots(Path(tmp) / "a.snapshots.jsonl", offset=0.0)
            b_path = self._write_snapshots(Path(tmp) / "b.snapshots.jsonl", offset=1.0)
            window = DataAnalysisWindow()
            window.show()
            self.app.processEvents()

            window._load_file("A", a_path)
            window._load_file("B", b_path)
            self.app.processEvents()

            # 通道扩容后表格覆盖全部 GUI 通道，超出可视行数部分转为内部滚动。
            self.assertEqual(window.summary_table.rowCount(), len(GUI_CHANNELS))
            self.assertEqual(window.summary_table.horizontalHeaderItem(1).text(), "均值")
            self.assertEqual(window.summary_table.horizontalHeaderItem(3).text(), "标准差")
            header_texts = [
                window.summary_table.horizontalHeaderItem(i).text()
                for i in range(window.summary_table.columnCount())
            ]
            self.assertIn("P95绝对值", header_texts)
            self.assertIn("总变差", header_texts)
            self.assertIn("时间积分", header_texts)
            self.assertIn("收敛时刻(s)", header_texts)
            self.assertEqual(window._target_combo.itemText(0), "all")
            self.assertIn("A01", [window._target_combo.itemText(i) for i in range(window._target_combo.count())])
            self.assertEqual(window.summary_table.item(0, 1).text(), "4.00 |")
            self.assertIn("pos_y", window._channel_buttons)
            self.assertIn("vel_y", window._channel_buttons)
            # 扩展通道（几何裁判/过载/指令/耗时）进入绘图通道单选列表。
            self.assertIn("e_perp", window._channel_buttons)
            self.assertIn("n_tot", window._channel_buttons)
            self.assertEqual(len(window._channel_buttons), len(GUI_CHANNELS))
            self.assertEqual(window._chart_layers["mean"].x_axis.titleText(), "")

            window._source_checks["B"].setChecked(True)
            self.app.processEvents()

            self.assertEqual(window.summary_table.item(0, 1).text(), "4.00 | 5.00")
            self.assertTrue(window.findChildren(type(window._status_label), "offlineLegendLabelA"))
            self.assertTrue(window.findChildren(type(window._status_label), "offlineLegendLabelB"))
            self.assertTrue(all(not view.chart().legend().isVisible() for view in window.findChildren(QChartView)))
            window._target_combo.setCurrentText("A01")
            self.app.processEvents()
            self.assertEqual(window.summary_table.item(0, 1).text(), "3.00 | 4.00")

            window._open_chart_popup()
            self.app.processEvents()

            self.assertIsNotNone(window._popup)
            assert window._popup is not None
            self.assertEqual(len(window._popup.findChildren(QChartView)), 4)
            self.assertTrue(window._popup.findChildren(type(window._status_label), "offlineLegendLabelA"))
            self.assertTrue(window._popup.findChildren(type(window._status_label), "offlineLegendLabelB"))
            self.assertTrue(all(not view.chart().legend().isVisible() for view in window._popup.findChildren(QChartView)))
            self.assertTrue(
                all(
                    not view.chart().axes(Qt.Orientation.Horizontal)[0].titleText()
                    for view in window._popup.findChildren(QChartView)
                )
            )

    def test_chart_popup_button_uses_icon_instead_of_text_glyph(self) -> None:
        """滑动窗口弹出按钮应使用图标，避免字体缺字时显示成小方框。"""
        window = DataAnalysisWindow()
        window.show()
        self.app.processEvents()

        button = window.findChild(QPushButton, "offlineChartPopupButton")

        self.assertIsNotNone(button)
        assert button is not None
        self.assertEqual(button.text(), "")
        self.assertFalse(button.icon().isNull())
        self.assertEqual(button.toolTip(), "弹出图表窗口")

    def test_vertical_channel_buttons_switch_plot_channel(self) -> None:
        """垂向位置和垂向速度按钮应能切换当前绘图通道。"""
        with tempfile.TemporaryDirectory() as tmp:
            path = self._write_snapshots(Path(tmp) / "snapshots.jsonl", offset=0.0)
            window = DataAnalysisWindow()
            window.show()
            window._load_file("A", path)
            self.app.processEvents()

            window._channel_buttons["pos_y"].click()
            self.app.processEvents()
            self.assertEqual(window._selected_channel().key, "pos_y")
            self.assertEqual(window._status_label.text(), "垂向位置误差 y")
            self.assertIn(("A", "all", "pos_y", 0.0, 1.0, 5.0), window._window_curve_cache)

            window._channel_buttons["vel_y"].click()
            self.app.processEvents()
            self.assertEqual(window._selected_channel().key, "vel_y")
            self.assertEqual(window._status_label.text(), "垂向速度误差 y")
            self.assertIn(("A", "all", "vel_y", 0.0, 1.0, 5.0), window._window_curve_cache)

    def test_channel_click_dispatches_one_selection_change(self) -> None:
        """一次按钮点击只经选中态信号切换一次通道。"""

        window = DataAnalysisWindow()
        window.show()
        self.app.processEvents()

        with patch.object(window, "_set_plot_channel", wraps=window._set_plot_channel) as callback:
            window._channel_buttons["pos_y"].click()
            self.app.processEvents()

        callback.assert_called_once_with("pos_y")

    def test_channel_button_row_blank_area_is_clickable(self) -> None:
        """点击通道按钮右侧空白区域也应切换绘图通道。"""
        window = DataAnalysisWindow()
        window.show()
        self.app.processEvents()
        button = window._channel_buttons["pos_y"]

        QTest.mouseClick(
            button,
            Qt.MouseButton.LeftButton,
            Qt.KeyboardModifier.NoModifier,
            QPoint(button.width() - 2, button.height() // 2),
        )
        self.app.processEvents()

        self.assertEqual(window._selected_channel().key, "pos_y")
        self.assertTrue(button.isChecked())

    def test_channel_click_repairs_stale_chart_title(self) -> None:
        """通道 key 已变化但图表未刷新时，同通道点击仍应补刷新。"""
        window = DataAnalysisWindow()
        window.show()
        self.app.processEvents()

        window._selected_channel_key = "pos_y"
        window._status_label.setText("前向位置误差 x")
        window._set_plot_channel("pos_y")
        self.app.processEvents()

        self.assertEqual(window._selected_channel().key, "pos_y")
        self.assertEqual(window._status_label.text(), "垂向位置误差 y")

    def test_loaded_file_displays_relative_path_when_under_workspace(self) -> None:
        """工作区内文件应在顶栏显示相对路径，而不是只显示文件名。"""
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as tmp:
            path = self._write_snapshots(Path(tmp) / "snapshots.jsonl", offset=0.0)
            window = DataAnalysisWindow()
            window.show()
            self.app.processEvents()

            window._load_file("A", path)
            self.app.processEvents()

            label_text = window._path_labels["A"].text()
            self.assertNotEqual(label_text, "snapshots.jsonl")
            self.assertTrue(label_text.endswith("snapshots.jsonl"))
            self.assertIn("snapshots.jsonl", window._path_labels["A"].toolTip())

    def test_toggling_b_reuses_a_window_curve_cache(self) -> None:
        """B 图层显隐不应触发 A 图层的滑窗重复计算。"""
        with tempfile.TemporaryDirectory() as tmp:
            a_path = self._write_snapshots(Path(tmp) / "a.snapshots.jsonl", offset=0.0)
            b_path = self._write_snapshots(Path(tmp) / "b.snapshots.jsonl", offset=1.0)
            window = DataAnalysisWindow()
            window.show()
            self.app.processEvents()

            with patch.object(
                data_analysis_window_module,
                "sliding_window",
                wraps=data_analysis_window_module.sliding_window,
            ) as sliding_window_mock:
                window._load_file("A", a_path)
                self.app.processEvents()
                self.assertEqual(sliding_window_mock.call_count, 1)

                window._load_file("B", b_path)
                self.app.processEvents()
                self.assertEqual(sliding_window_mock.call_count, 1)

                window._source_checks["B"].setChecked(True)
                self.app.processEvents()
                self.assertEqual(sliding_window_mock.call_count, 2)

                window._source_checks["B"].setChecked(False)
                self.app.processEvents()
                self.assertEqual(sliding_window_mock.call_count, 2)

    def _write_snapshots(self, path: Path, *, offset: float) -> Path:
        """写入两帧两机的最小 snapshots.jsonl 测试数据。"""
        records = [
            {
                "time_s": 0.0,
                "nodes": [
                    self._node("A01", 1.0 + offset),
                    self._node("A02", 3.0 + offset),
                ],
            },
            {
                "time_s": 1.0,
                "nodes": [
                    self._node("A01", 5.0 + offset),
                    self._node("A02", 7.0 + offset),
                ],
            },
        ]
        path.write_text("\n".join(json.dumps(record) for record in records), encoding="utf-8")
        return path

    def _node(self, node_id: str, base: float) -> dict[str, object]:
        """构造包含六个误差通道的节点快照。"""
        return {
            "node_id": node_id,
            "track_pos_err_x_m": base,
            "track_pos_err_y_m": base + 0.1,
            "track_pos_err_z_m": base + 0.2,
            "track_vel_err_x_mps": base + 0.3,
            "track_vel_err_y_mps": base + 0.4,
            "track_vel_err_z_mps": base + 0.5,
        }


if __name__ == "__main__":
    unittest.main()
