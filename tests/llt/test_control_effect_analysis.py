"""控制效果离线分析内核测试。"""

from __future__ import annotations

import csv
import json
import tempfile
import unittest
from pathlib import Path

from src.data.control_effect_analysis import (
    DEFAULT_CHANNELS,
    load_snapshot_samples,
    metric_rows_for_source,
    points_for,
    sliding_window,
    summary_for,
    write_metrics_csv,
)


class ControlEffectAnalysisTests(unittest.TestCase):
    """验证不依赖 PySide6 的读取、统计、滑窗和导出逻辑。"""

    def test_load_summary_filter_window_and_export(self) -> None:
        """完整路径应能读取 snapshots、过滤时间段、生成窗口和导出 CSV。"""
        with tempfile.TemporaryDirectory() as tmp:
            source_path = self._write_snapshots(Path(tmp) / "snapshots.jsonl")
            source = load_snapshot_samples(source_path, label="A")

            self.assertEqual(source.label, "A")
            self.assertEqual(sorted(source.samples), ["A01", "A02"])
            self.assertAlmostEqual(source.t_min, 0.0)
            self.assertAlmostEqual(source.t_max, 2.0)

            all_summary = summary_for(source, "all", "pos_x", 0.0, 2.0)
            self.assertIsNotNone(all_summary)
            assert all_summary is not None
            self.assertEqual(all_summary.count, 6)
            self.assertAlmostEqual(all_summary.mean, 23.0 / 6.0)
            self.assertAlmostEqual(all_summary.variance, 185.0 / 36.0)
            self.assertAlmostEqual(all_summary.std, (185.0 / 36.0) ** 0.5)
            self.assertAlmostEqual(all_summary.rms, (119.0 / 6.0) ** 0.5)
            self.assertAlmostEqual(all_summary.max_abs, 8.0)
            self.assertAlmostEqual(all_summary.max_abs_time_s, 2.0)

            node_points = points_for(source, "A01", "pos_x", 1.0, 2.0)
            self.assertEqual(node_points, [(1.0, 3.0), (2.0, 5.0)])

            narrow_window = sliding_window(node_points, 1.0, 2.0, 0.5)
            wide_window = sliding_window(node_points, 1.0, 2.0, 2.0)
            self.assertEqual([summary.count for _t, summary in narrow_window], [1, 1])
            self.assertEqual([summary.count for _t, summary in wide_window], [2, 1])

            rows = metric_rows_for_source(source, 0.0, 2.0)
            self.assertEqual(len(rows), 18)
            self.assertEqual(rows[0]["scope"], "all")
            self.assertEqual(rows[0]["node_id"], "all")
            self.assertEqual(rows[0]["channel"], "前向位置误差 x")

            export_path = Path(tmp) / "metrics.csv"
            write_metrics_csv(export_path, [source], 0.0, 2.0)
            with export_path.open(encoding="utf-8-sig", newline="") as handle:
                exported = list(csv.DictReader(handle))
            self.assertEqual(len(exported), 18)
            self.assertIn("max_abs_time_s", exported[0])

    def test_load_rejects_missing_channel_field(self) -> None:
        """缺少默认误差通道字段时应给出明确错误，而不是按 0 兜底。"""
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "bad.snapshots.jsonl"
            node = self._node("A01", 1.0)
            del node["track_vel_err_z_mps"]
            path.write_text(json.dumps({"time_s": 0.0, "nodes": [node]}), encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "A01 缺少 track_vel_err_z_mps"):
                load_snapshot_samples(path, label="A")

    def test_load_rejects_empty_or_bad_line(self) -> None:
        """空文件和坏 JSON 行应报告可定位错误。"""
        with tempfile.TemporaryDirectory() as tmp:
            empty = Path(tmp) / "empty.snapshots.jsonl"
            empty.write_text("\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "文件为空"):
                load_snapshot_samples(empty, label="A")

            bad = Path(tmp) / "bad.snapshots.jsonl"
            bad.write_text("{", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "第 1 行不是合法 JSON"):
                load_snapshot_samples(bad, label="A")

    def _write_snapshots(self, path: Path) -> Path:
        """写入三帧两机样本文件。"""
        records = [
            {"time_s": 0.0, "nodes": [self._node("A01", 1.0), self._node("A02", 2.0)]},
            {"time_s": 1.0, "nodes": [self._node("A01", 3.0), self._node("A02", 4.0)]},
            {"time_s": 2.0, "nodes": [self._node("A01", 5.0), self._node("A02", 8.0)]},
        ]
        path.write_text("\n".join(json.dumps(record) for record in records), encoding="utf-8")
        return path

    def _node(self, node_id: str, base: float) -> dict[str, object]:
        """构造覆盖全部默认通道的节点数据。"""
        values = {
            "track_pos_err_x_m": base,
            "track_pos_err_y_m": base + 0.1,
            "track_pos_err_z_m": base + 0.2,
            "track_vel_err_x_mps": base + 0.3,
            "track_vel_err_y_mps": base + 0.4,
            "track_vel_err_z_mps": base + 0.5,
        }
        self.assertEqual(set(values), {channel.field_name for channel in DEFAULT_CHANNELS})
        return {"node_id": node_id, **values}


if __name__ == "__main__":
    unittest.main()
