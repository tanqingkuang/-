"""控制效果离线分析内核测试。"""

from __future__ import annotations

import csv
import json
import math
import tempfile
import unittest
from pathlib import Path

from src.data.control_effect_analysis import (
    DEFAULT_CHANNELS,
    GUI_CHANNELS,
    MAX_WINDOW_ANCHORS,
    analyze_source,
    calc_summary,
    convergence_time_s,
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
            self.assertEqual([summary.count for _t, summary in narrow_window], [1])
            self.assertEqual(wide_window, [])

            rows = metric_rows_for_source(source, 0.0, 2.0)
            self.assertEqual(len(rows), 18)
            self.assertEqual(rows[0]["scope"], "all")
            self.assertEqual(rows[0]["node_id"], "all")
            self.assertEqual(rows[0]["channel"], "track_pos_err_x_m")
            self.assertEqual(rows[0]["channel_label"], "前向位置误差 x")

            result = analyze_source(source, 2.0, 0.0)
            self.assertEqual(result.source, source)
            self.assertEqual(result.start_s, 0.0)
            self.assertEqual(result.end_s, 2.0)
            self.assertEqual(result.channels, DEFAULT_CHANNELS)
            self.assertEqual(len(result.metric_rows), 18)
            self.assertEqual(len(result.rows_for_scope("all")), 6)
            self.assertEqual(len(result.rows_for_scope("node")), 12)
            self.assertEqual(result.rows_for_scope("node")[0]["node_id"], "A01")

            export_path = Path(tmp) / "metrics.csv"
            write_metrics_csv(export_path, [source], 0.0, 2.0)
            with export_path.open(encoding="utf-8-sig", newline="") as handle:
                exported = list(csv.DictReader(handle))
            self.assertEqual(len(exported), 18)
            self.assertEqual(exported[0]["channel"], "track_pos_err_x_m")
            self.assertEqual(exported[0]["channel_label"], "前向位置误差 x")
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

    def test_sliding_window_uses_rolling_statistics_without_changing_results(self) -> None:
        """滑窗统计应正确处理重复时刻、无序输入和最大绝对值发生时刻。"""
        points = [(2.0, 2.0), (0.0, 1.0), (1.0, -4.0), (0.0, 3.0), (3.0, 5.0)]

        windows = sliding_window(points, 0.0, 3.0, 2.0)

        self.assertEqual([time_s for time_s, _summary in windows], [0.0, 1.0])
        self.assertEqual([summary.count for _time_s, summary in windows], [3, 2])
        self.assertEqual([summary.max_abs_time_s for _time_s, summary in windows], [1.0, 1.0])
        self.assertEqual([summary.max_abs for _time_s, summary in windows], [4.0, 4.0])
        self.assertAlmostEqual(windows[0][1].mean, 0.0)
        self.assertAlmostEqual(windows[0][1].rms, (26.0 / 3.0) ** 0.5)

    def test_sliding_window_rms_handles_negative_square_total_drift(self) -> None:
        """误差趋零的窗口里增量平方和会消差成极小负数，rms 不能因此抛 math domain error。"""
        # 前段是量级 O(1) 的有符号误差，后段是收敛到 0 的稳态；当大误差滑出窗口后，
        # 增量维护的 square_total 会留下约 -3.6e-15 的负残差，旧实现直接开方会崩溃。
        head = [3.68, -3.24, 2.34, 4.95, -3.41, 2.8, -4.03]
        points = [(round(index * 0.1, 4), value) for index, value in enumerate(head + [0.0] * 9)]

        windows = sliding_window(points, 0.0, 1.5, 0.8)

        self.assertTrue(windows)
        for _time_s, summary in windows:
            # 任何窗口的 rms 都必须是有限非负数，不能出现 nan 或异常。
            self.assertGreaterEqual(summary.rms, 0.0)
            self.assertTrue(math.isfinite(summary.rms))
        # 全零稳态窗口的真实 rms 就是 0，夹断负残差后应精确回到 0。
        steady = [summary for _time_s, summary in windows if abs(summary.mean) < 1e-9 and summary.count >= 8]
        self.assertTrue(steady)
        self.assertEqual(max(summary.rms for summary in steady), 0.0)

    def test_sliding_window_skips_tail_shorter_than_window(self) -> None:
        """滑窗不应输出超过 end_s 的尾部半窗口。"""
        points = [(0.0, 1.0), (1.0, 10.0), (2.0, 100.0)]

        windows = sliding_window(points, 0.0, 2.0, 2.0)

        self.assertEqual([time_s for time_s, _summary in windows], [0.0])
        self.assertEqual(windows[0][1].count, 2)
        self.assertAlmostEqual(windows[0][1].mean, 5.5)

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

    def test_extended_summary_metrics(self) -> None:
        """calc_summary 应给出 P95 绝对值、总变差和有符号时间积分。"""
        points = [(0.0, 0.0), (1.0, 2.0), (2.0, -2.0), (3.0, 0.0)]

        summary = calc_summary(points)

        self.assertIsNotNone(summary)
        assert summary is not None
        # 总变差 = |2-0| + |-2-2| + |0-(-2)| = 8。
        self.assertAlmostEqual(summary.tv, 8.0)
        # 梯形积分 = 1 + 0 + (-1) = 0（正负面积抵消）。
        self.assertAlmostEqual(summary.integral, 0.0)
        self.assertGreaterEqual(summary.p95_abs, 0.0)
        self.assertLessEqual(summary.p95_abs, summary.max_abs)

    def test_convergence_time_uses_hold_duration_not_first_crossing(self) -> None:
        """收敛时刻必须是入带并保持 hold_s 后的时刻，不能取第一次穿越阈值。"""
        # 0~1s 短暂入带后弹出（不算收敛），3s 起持续入带。
        points = [(0.0, 0.5), (1.0, 3.0), (2.0, 3.0), (3.0, 0.5), (4.0, 0.4), (5.0, 0.3), (6.0, 0.2)]

        self.assertAlmostEqual(convergence_time_s(points, band=1.0, hold_s=2.0), 5.0)
        # 覆盖不足 hold_s 时不得给出收敛结论。
        self.assertIsNone(convergence_time_s(points, band=1.0, hold_s=10.0))
        # 全程不入带返回 None。
        self.assertIsNone(convergence_time_s(points, band=0.01, hold_s=1.0))

    def test_optional_channel_missing_is_skipped_not_error(self) -> None:
        """可选扩展通道缺字段/为 null 时按无样本处理，基础通道仍正常加载。"""
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "old.snapshots.jsonl"
            # 旧日志：只有六个基础误差通道，没有任何扩展字段。
            path.write_text(json.dumps({"time_s": 0.0, "nodes": [self._node("A01", 1.0)]}), encoding="utf-8")

            source = load_snapshot_samples(path, label="A", channels=GUI_CHANNELS)

            self.assertIn("pos_x", source.samples["A01"])
            self.assertNotIn("phi", source.samples["A01"])
            self.assertNotIn("e_rigid_x", source.samples["A01"])
            # 无样本通道的 summary 为 None，GUI 显示空位。
            self.assertIsNone(summary_for(source, "A01", "phi", 0.0, 1.0))

    def test_derived_channels_sign_and_frame_conventions(self) -> None:
        """派生裁判量必须遵循文档口径：e_perp 左正、e_out 外侧正、e_rigid 用长机 FUR。"""
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "derived.snapshots.jsonl"
            leader = {
                **self._node("L01", 0.0),
                "role": "leader",
                # 长机朝正北（ENU 航向 90°）平飞于原点。
                "x_m": 0.0,
                "y_m": 0.0,
                "altitude_m": 1000.0,
                "psi_v_deg": 90.0,
                "theta_deg": 0.0,
                "nx": 0.0,
                "ny": 1.0,
                "nz": 0.0,
                "cross_track_error_m": 2.0,
                "slot_x_m": 0.0,
                "slot_y_m": 0.0,
                "slot_z_m": 0.0,
            }
            follower = {
                **self._node("W02", 0.0),
                "role": "wingman",
                # 朝北时右侧为正东：僚机在长机正东 40m，槽位要求右侧 30m。
                "x_m": 40.0,
                "y_m": 0.0,
                "altitude_m": 1000.0,
                "psi_v_deg": 90.0,
                "theta_deg": 0.0,
                "nx": 0.0,
                "ny": 1.0,
                "nz": 1.0,
                "cross_track_error_m": -3.0,
                "slot_x_m": 0.0,
                "slot_y_m": 0.0,
                "slot_z_m": 30.0,
            }
            record = {
                "time_s": 0.0,
                # 左转弯道（turn_sign=+1，κ>0）。
                "route": {"turn_sign": 1.0, "radius_m": 500.0},
                "nodes": [leader, follower],
            }
            path.write_text(json.dumps(record), encoding="utf-8")

            source = load_snapshot_samples(path, label="A", channels=GUI_CHANNELS)

            samples = source.samples
            # e_perp 左正 = -cross_track_error_m（快照右正）。
            self.assertAlmostEqual(samples["L01"]["e_perp"][0][1], -2.0)
            self.assertAlmostEqual(samples["W02"]["e_perp"][0][1], 3.0)
            # 左转时外侧在右：e_out = -sgn(+1)·e_perp。
            self.assertAlmostEqual(samples["L01"]["e_out"][0][1], 2.0)
            self.assertAlmostEqual(samples["W02"]["e_out"][0][1], -3.0)
            # n_tot = sqrt(0²+1²+1²)，n_over = (n_tot-1)²。
            self.assertAlmostEqual(samples["W02"]["n_tot"][0][1], math.sqrt(2.0))
            self.assertAlmostEqual(samples["W02"]["n_over"][0][1], (math.sqrt(2.0) - 1.0) ** 2)
            # 长机 FUR 下僚机实际在右 40m，槽位 30m，刚性误差 z=+10；长机自身无 e_rigid。
            self.assertAlmostEqual(samples["W02"]["e_rigid_x"][0][1], 0.0)
            self.assertAlmostEqual(samples["W02"]["e_rigid_y"][0][1], 0.0)
            self.assertAlmostEqual(samples["W02"]["e_rigid_z"][0][1], 10.0)
            self.assertNotIn("e_rigid_z", samples["L01"])

    def test_all_target_time_series_metrics_aggregate_per_node(self) -> None:
        """all 目标的总变差与时间积分必须逐机计算后求和，不得跨机差分。"""
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "two.snapshots.jsonl"
            # A01 的 pos_x 序列 0→100→0（TV=200、积分=100），A02 恒为 0（TV=0、积分=0）。
            records = []
            for t, a01 in ((0.0, 0.0), (1.0, 100.0), (2.0, 0.0)):
                node_a = {**self._node("A01", 0.0), "track_pos_err_x_m": a01}
                node_b = {**self._node("A02", 0.0), "track_pos_err_x_m": 0.0}
                records.append({"time_s": t, "nodes": [node_a, node_b]})
            path.write_text("\n".join(json.dumps(record) for record in records), encoding="utf-8")

            source = load_snapshot_samples(path, label="A")
            all_summary = summary_for(source, "all", "pos_x", 0.0, 2.0)
            a01_summary = summary_for(source, "A01", "pos_x", 0.0, 2.0)
            a02_summary = summary_for(source, "A02", "pos_x", 0.0, 2.0)

            assert all_summary is not None and a01_summary is not None and a02_summary is not None
            # 跨机拼接的错误实现会把"A01→A02"的机间差当成时间变化，TV 远大于逐机之和。
            self.assertAlmostEqual(all_summary.tv, a01_summary.tv + a02_summary.tv)
            self.assertAlmostEqual(all_summary.integral, a01_summary.integral + a02_summary.integral)
            self.assertAlmostEqual(all_summary.tv, 200.0)
            self.assertAlmostEqual(all_summary.integral, 100.0)
            # 分布类指标仍按合并样本统计，不受聚合口径调整影响。
            self.assertEqual(all_summary.count, 6)
            self.assertAlmostEqual(all_summary.max_abs, 100.0)

    def test_e_out_integral_clips_negative_part(self) -> None:
        """e_out 的时间积分是外甩面积 ∫max(e_out,0)dt，内侧偏差不得抵消外侧。"""
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "eout.snapshots.jsonl"
            records = []
            # 左转（turn_sign=+1）时 e_out = cross_track_error_m；构造 [2, -2] 正负交替。
            for t, cte in ((0.0, 2.0), (1.0, -2.0)):
                node = {**self._node("A01", 0.0), "cross_track_error_m": cte}
                records.append({"time_s": t, "route": {"turn_sign": 1.0}, "nodes": [node]})
            path.write_text("\n".join(json.dumps(record) for record in records), encoding="utf-8")

            source = load_snapshot_samples(path, label="A", channels=GUI_CHANNELS)
            e_out = summary_for(source, "A01", "e_out", 0.0, 1.0)
            e_perp = summary_for(source, "A01", "e_perp", 0.0, 1.0)

            assert e_out is not None and e_perp is not None
            # 截正部后梯形积分：max([2,-2],0)=[2,0] 在 [0,1] 上积分为 1。
            self.assertAlmostEqual(e_out.integral, 1.0)
            # 普通有符号通道保持原口径：e_perp=[-2,2] 的梯形积分为 0。
            self.assertAlmostEqual(e_perp.integral, 0.0)

    def test_sliding_window_decimates_anchors_over_limit(self) -> None:
        """锚点超过上限时应均匀抽稀，同时保证首尾锚点仍被覆盖。"""
        count = MAX_WINDOW_ANCHORS * 2 + 100
        points = [(index * 0.01, float(index % 7)) for index in range(count)]

        windows = sliding_window(points, 0.0, points[-1][0], 0.05)

        self.assertLessEqual(len(windows), MAX_WINDOW_ANCHORS)
        self.assertGreater(len(windows), MAX_WINDOW_ANCHORS // 2)
        self.assertAlmostEqual(windows[0][0], 0.0)

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
