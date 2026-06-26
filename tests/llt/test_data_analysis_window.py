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
from PySide6.QtWidgets import QApplication

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

            self.assertEqual(window.summary_table.rowCount(), 6)
            self.assertEqual(window.summary_table.horizontalHeaderItem(1).text(), "均值")
            self.assertEqual(window.summary_table.horizontalHeaderItem(3).text(), "标准差")
            self.assertEqual(window.summary_table.verticalScrollBar().maximum(), 0)
            self.assertEqual(window._target_combo.itemText(0), "all")
            self.assertIn("A01", [window._target_combo.itemText(i) for i in range(window._target_combo.count())])
            self.assertEqual(window.summary_table.item(0, 1).text(), "4.00 |")

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

            rows = window._metric_rows_for_source(window._sources["A"], 0.0, 1.0)
            self.assertEqual(len(rows), 18)
            self.assertEqual(rows[0]["input_label"], "A")
            self.assertEqual(rows[0]["scope"], "all")
            self.assertEqual(rows[0]["channel"], "前向位置误差 x")

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
