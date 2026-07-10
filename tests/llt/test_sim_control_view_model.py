"""仿真控制 ViewModel 的函数级回归测试。注意：本文件不构造 Qt 对象。"""

from __future__ import annotations

import ast
import unittest
from pathlib import Path

from src.ui.gui.sim_control_view_model import (
    SimControlViewModel,
    format_duration_text,
    parse_duration_text,
    rally_button_enabled,
)
from src.ui.gui.view_models import NodeState, Snapshot


class SimControlViewModelTests(unittest.TestCase):
    """覆盖仿真控制区稳定显示规则，避免依赖完整 GUI 长链条。"""

    def test_view_model_and_tests_do_not_import_pyside6(self) -> None:
        """SimControlViewModel 及本测试文件不得在 import 中引入 PySide6。"""

        paths = [Path("src/ui/gui/sim_control_view_model.py"), Path(__file__)]
        for path in paths:
            with self.subTest(path=str(path)):
                tree = ast.parse(path.read_text(encoding="utf-8"))
                imported_roots = {
                    alias.name.split(".", maxsplit=1)[0]
                    for node in ast.walk(tree)
                    if isinstance(node, ast.Import)
                    for alias in node.names
                }
                imported_roots.update(
                    node.module.split(".", maxsplit=1)[0]
                    for node in ast.walk(tree)
                    if isinstance(node, ast.ImportFrom) and node.module is not None
                )
                self.assertNotIn("PySide6", imported_roots)

    def test_snapshot_display_formats_texts_progress_and_button_enables(self) -> None:
        """快照文本、千分进度和控制按钮使能由 ViewModel 一次性生成。"""

        snapshot = Snapshot(
            time=2.5,
            duration=10.0,
            step=0.1,
            run_state="RUNNING",
            control_report="保持",
            disturbance="无",
            nodes=[],
            links=[],
            cpu_utilization=0.876,
        )

        display = SimControlViewModel().on_snapshot(snapshot)

        self.assertEqual(display.report_text, "回报：保持")
        self.assertEqual(display.timeline_text, "2.5 / 10s")
        self.assertEqual(display.cpu_text, "CPU 88%")
        self.assertEqual(display.progress_permille, 250)
        self.assertTrue(display.play_enabled)
        self.assertEqual(display.play_text, "暂停")
        self.assertFalse(display.step_enabled)
        self.assertTrue(display.reset_enabled)
        self.assertTrue(display.disturbance_enabled)
        self.assertEqual(display.duration_text, "10")
        self.assertFalse(display.duration_enabled)

    def test_zero_duration_progress_falls_back_to_zero(self) -> None:
        """duration 为 0 时进度条置 0，避免除零。"""

        snapshot = Snapshot(1.0, 0.0, 0.1, "READY", "待命", "无", [], [])

        display = SimControlViewModel().on_snapshot(snapshot)

        self.assertEqual(display.progress_permille, 0)
        self.assertEqual(display.timeline_text, "1.0 / 0s")

    def test_play_text_and_enables_follow_run_state_display_contract(self) -> None:
        """播放按钮文案覆盖运行、暂停和默认三态，使能只排除未加载与结束。"""

        view_model = SimControlViewModel()
        cases = [
            ("RUNNING", "暂停", True, False, True, True),
            ("PAUSED", "继续", True, True, True, True),
            ("READY", "开始", True, True, True, True),
            ("UNLOADED", "开始", False, False, False, False),
            ("FINISHED", "开始", False, False, True, False),
        ]

        for run_state, play_text, play_enabled, step_enabled, reset_enabled, disturbance_enabled in cases:
            with self.subTest(run_state=run_state):
                snapshot = Snapshot(0.0, 10.0, 0.1, run_state, "待命", "无", [], [])
                display = view_model.on_snapshot(snapshot)

                self.assertEqual(display.play_text, play_text)
                self.assertEqual(display.play_enabled, play_enabled)
                self.assertEqual(display.step_enabled, step_enabled)
                self.assertEqual(display.reset_enabled, reset_enabled)
                self.assertEqual(display.disturbance_enabled, disturbance_enabled)

    def test_rally_button_state_machine_covers_terminal_and_active_phases(self) -> None:
        """集结按钮只对集结角色的本地待命和活动阶段开放，HOLD 为终态关闭。"""

        cases = [
            ("READY", [self._node("rally_leader", "LOCAL_LOITER")], False),
            ("PAUSED", [self._node("leader", "LOCAL_LOITER")], False),
            ("PAUSED", [self._node("rally_follower", "")], False),
            ("PAUSED", [self._node("rally_follower", "HOLD")], False),
            ("PAUSED", [self._node("rally_follower", "LOCAL_LOITER")], True),
            ("RUNNING", [self._node(" rally_leader ", "CATCHUP")], True),
            ("RUNNING", [self._node("rally_follower", "HOLD"), self._node("rally_leader", "CATCHUP")], False),
            ("FINISHED", [self._node("rally_follower", "CATCHUP")], False),
        ]

        for run_state, nodes, expected in cases:
            with self.subTest(run_state=run_state, phases=[node.rally_phase for node in nodes]):
                self.assertEqual(rally_button_enabled(run_state, nodes), expected)

    def test_duration_format_and_parse_keep_existing_text_contract(self) -> None:
        """时长格式化保留整数无小数、小数去尾零，解析失败返回 None。"""

        self.assertEqual(format_duration_text(2400.0), "2400")
        self.assertEqual(format_duration_text(12.34), "12.34")
        self.assertEqual(format_duration_text(12.3456), "12.346")
        self.assertEqual(parse_duration_text("120"), 120.0)
        self.assertEqual(parse_duration_text(" 3.5 "), 3.5)
        self.assertIsNone(parse_duration_text("abc"))
        self.assertIsNone(parse_duration_text(""))

    def test_unloaded_duration_input_disables_without_text_backfill(self) -> None:
        """未加载态禁用时长输入且不回填文本，READY/PAUSED 才允许编辑。"""

        view_model = SimControlViewModel()
        unloaded = view_model.on_snapshot(Snapshot(0.0, 10.0, 0.1, "UNLOADED", "", "无", [], []))
        ready = view_model.on_snapshot(Snapshot(0.0, 10.0, 0.1, "READY", "", "无", [], []))
        paused = view_model.on_snapshot(Snapshot(0.0, 10.5, 0.1, "PAUSED", "", "无", [], []))

        self.assertIsNone(unloaded.duration_text)
        self.assertFalse(unloaded.duration_enabled)
        self.assertEqual(ready.duration_text, "10")
        self.assertTrue(ready.duration_enabled)
        self.assertEqual(paused.duration_text, "10.5")
        self.assertTrue(paused.duration_enabled)

    @staticmethod
    def _node(role: str, rally_phase: str) -> NodeState:
        """构造集结状态测试节点。注意：只填状态机相关字段。"""

        return NodeState("R01", role, 0.0, 0.0, 0.0, 0.0, rally_phase=rally_phase)


if __name__ == "__main__":
    unittest.main()
