"""稳定保持阶段编队精度分析测试。"""

from __future__ import annotations

import csv
import json
import tempfile
import unittest
from unittest.mock import patch
from pathlib import Path

from scripts.analyze_formation_accuracy import main as analyze_main
from src.data.formation_accuracy_analysis import analyze_formation_accuracy, write_accuracy_report


class FormationAccuracyAnalysisTests(unittest.TestCase):
    """验证任务 HOLD、稳定分段、中文指标和报告导出。"""

    @staticmethod
    def _node(
        node_id: str,
        role: str,
        *,
        x_m: float,
        slot_x_m: float,
        task_stage: str,
        track_error_m: float = 0.0,
    ) -> dict[str, object]:
        """构造满足分析日志契约的节点。"""

        return {
            "node_id": node_id,
            "role": role,
            "x_m": x_m,
            "y_m": 0.0,
            "altitude_m": 1000.0,
            "psi_v_deg": 0.0,
            "theta_deg": 0.0,
            "slot_x_m": slot_x_m,
            "slot_y_m": 0.0,
            "slot_z_m": 0.0,
            "task_stage": task_stage,
            "track_pos_err_x_m": track_error_m,
            "track_pos_err_y_m": 0.0,
            "track_pos_err_z_m": 0.0,
            "track_vel_err_x_mps": 0.0,
            "track_vel_err_y_mps": 0.0,
            "track_vel_err_z_mps": 0.0,
        }

    def _write_run(self, root: Path, errors: tuple[float, ...]) -> Path:
        """写入一份从 2 秒进入 HOLD 的最小仿真结果。"""

        run_dir = root / "run-test"
        run_dir.mkdir()
        config = {"rally_cfg": {"tight_radius_m": 1.0, "stable_hold_s": 2.0}}
        (run_dir / "config.json").write_text(json.dumps(config), encoding="utf-8")
        records = []
        for time_s, error_m in enumerate(errors):
            stage = "HOLD" if time_s >= 2 else "RALLY"
            records.append(
                {
                    "time_s": float(time_s),
                    "nodes": [
                        self._node("A01", "rally_leader", x_m=0.0, slot_x_m=0.0, task_stage=stage),
                        self._node(
                            "A02",
                            "rally_follower",
                            x_m=10.0 + error_m,
                            slot_x_m=10.0,
                            task_stage=stage,
                            track_error_m=error_m * 0.5,
                        ),
                    ],
                }
            )
        snapshots = "\n".join(json.dumps(record, ensure_ascii=False) for record in records)
        (run_dir / "snapshots.jsonl").write_text(snapshots, encoding="utf-8")
        return run_dir

    def test_report_only_uses_stable_hold_samples(self) -> None:
        """统计应从全队 HOLD 后连续满足门限的完成时刻开始。"""

        with tempfile.TemporaryDirectory() as tmp:
            run_dir = self._write_run(Path(tmp), (4.0, 3.0, 0.8, 0.7, 0.0, 1.0, 0.0))
            report = analyze_formation_accuracy(run_dir)

        self.assertEqual(report.hold_start_s, 2.0)
        self.assertEqual(report.stable_start_s, 4.0)
        self.assertEqual(report.status, "稳定保持")
        self.assertEqual(len(report.metric_rows), 1)
        rigid_norm = report.metric_rows[0]
        self.assertEqual(rigid_norm["飞机编号"], "A02")
        # 稳定段只包含 t=4,5,6 的 [0,1,0]，HOLD 初期的 0.8/0.7 不得混入。
        self.assertAlmostEqual(float(rigid_norm["编队三维位置误差均值(米)"]), 1.0 / 3.0)
        self.assertAlmostEqual(float(rigid_norm["编队三维位置误差方差(米²)"]), 2.0 / 9.0)
        self.assertAlmostEqual(float(rigid_norm["编队三维位置误差均方根(米)"]), (1.0 / 3.0) ** 0.5)
        self.assertEqual(rigid_norm["编队三维位置误差最大值时刻(秒)"], 5.0)

    def test_unstable_hold_does_not_export_performance_metrics(self) -> None:
        """进入 HOLD 但未稳定时不得输出伪造的稳态性能。"""

        with tempfile.TemporaryDirectory() as tmp:
            run_dir = self._write_run(Path(tmp), (4.0, 3.0, 2.0, 2.0, 2.0, 2.0))
            report = analyze_formation_accuracy(run_dir)

        self.assertEqual(report.status, "保持未稳定")
        self.assertIsNone(report.stable_start_s)
        self.assertEqual(report.metric_rows, ())

    def test_rally_config_without_tight_radius_uses_runtime_default(self) -> None:
        """集结配置省略紧队形半径时应沿用运行时 2 米缺省值。"""

        with tempfile.TemporaryDirectory() as tmp:
            run_dir = self._write_run(Path(tmp), (4.0, 3.0, 3.0, 3.0, 3.0, 3.0))
            config_path = run_dir / "config.json"
            config = json.loads(config_path.read_text(encoding="utf-8"))
            del config["rally_cfg"]["tight_radius_m"]
            config_path.write_text(json.dumps(config), encoding="utf-8")

            report = analyze_formation_accuracy(run_dir)

        self.assertEqual(report.status, "保持未稳定")
        self.assertIsNone(report.stable_start_s)
        self.assertEqual(report.metric_rows, ())

    def test_each_follower_has_exactly_one_metric_row(self) -> None:
        """多僚机场景应每机一行，不追加全队或分轴指标行。"""

        with tempfile.TemporaryDirectory() as tmp:
            run_dir = self._write_run(Path(tmp), (4.0, 3.0, 0.8, 0.7, 0.0, 1.0, 0.0))
            snapshot_path = run_dir / "snapshots.jsonl"
            records = [json.loads(line) for line in snapshot_path.read_text(encoding="utf-8").splitlines()]
            for record in records:
                follower = dict(record["nodes"][1])
                follower.update({"node_id": "A04", "x_m": float(follower["x_m"]) + 10.0, "slot_x_m": 20.0})
                record["nodes"].append(follower)
            snapshot_path.write_text(
                "\n".join(json.dumps(record, ensure_ascii=False) for record in records),
                encoding="utf-8",
            )

            report = analyze_formation_accuracy(run_dir)

        self.assertEqual([row["飞机编号"] for row in report.metric_rows], ["A02", "A04"])

    def test_export_only_keeps_chinese_follower_csv(self) -> None:
        """导出目录只应保留中文逐僚机指标 CSV，并清理旧附加报告。"""

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            report = analyze_formation_accuracy(
                self._write_run(root, (4.0, 3.0, 0.8, 0.7, 0.0, 1.0, 0.0))
            )
            output_dir = root / "analysis" / report.run_id
            output_dir.mkdir(parents=True)
            (output_dir / "formation_accuracy_summary.csv").write_text("旧汇总", encoding="utf-8")
            (output_dir / "formation_accuracy.json").write_text("{}", encoding="utf-8")

            written_dir = write_accuracy_report(report, root / "analysis")
            with (output_dir / "formation_accuracy_detail.csv").open(
                encoding="utf-8-sig", newline=""
            ) as handle:
                detail = next(csv.DictReader(handle))

        self.assertEqual(written_dir, output_dir)
        self.assertEqual(detail["飞机编号"], "A02")
        self.assertIn("编队三维位置误差均值(米)", detail)
        self.assertIn("跟踪三维位置误差方差(米²)", detail)
        self.assertFalse((output_dir / "formation_accuracy_summary.csv").exists())
        self.assertFalse((output_dir / "formation_accuracy.json").exists())

    def test_cli_analyzes_selected_snapshot_file(self) -> None:
        """命令行选择 snapshots.jsonl 后应分析其所在运行目录。"""

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            run_dir = self._write_run(root, (4.0, 3.0, 0.8, 0.7, 0.0, 1.0, 0.0))
            output_root = root / "analysis"

            exit_code = analyze_main(
                [
                    str(run_dir / "snapshots.jsonl"),
                    "--output-root",
                    str(output_root),
                ]
            )
            self.assertEqual(exit_code, 0)
            self.assertTrue((output_root / "run-test" / "formation_accuracy_detail.csv").is_file())

    def test_cli_opens_file_picker_when_path_is_omitted(self) -> None:
        """未传路径时应使用文件选择框返回的 snapshots.jsonl。"""

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            run_dir = self._write_run(root, (4.0, 3.0, 0.8, 0.7, 0.0, 1.0, 0.0))
            output_root = root / "analysis"
            with patch(
                "scripts.analyze_formation_accuracy._choose_snapshot_file",
                return_value=run_dir / "snapshots.jsonl",
            ):
                exit_code = analyze_main(["--output-root", str(output_root)])

            self.assertEqual(exit_code, 0)
            self.assertTrue((output_root / "run-test" / "formation_accuracy_detail.csv").is_file())

if __name__ == "__main__":
    unittest.main()
