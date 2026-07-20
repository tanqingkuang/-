"""命令行与一键 BAT 入口测试。"""

from __future__ import annotations

import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from src.runner.sim_control_types import CommandResult, RunState


PROJECT_ROOT = Path(__file__).resolve().parents[2]


class MainEntrypointTests(unittest.TestCase):
    """验证无界面进程入口解析配置和倍率。"""

    def test_main_runs_config_with_default_rate(self) -> None:
        """只传配置路径时应按默认 10 倍速调用仿真入口。"""

        from src import main

        with patch.object(main, "run_simulation", return_value=0) as run_simulation:
            exit_code = main.main(["--config", "configs/base.json"])

        self.assertEqual(exit_code, 0)
        run_simulation.assert_called_once_with(Path("configs/base.json"), playback_rate=10.0)


class HeadlessRunnerTests(unittest.TestCase):
    """验证无界面单次仿真对控制器的调用契约。"""

    def test_headless_run_uses_paced_controller_and_closes_it(self) -> None:
        """无界面运行必须按倍率节流、自动集结并在结束后释放控制器。"""

        from src.main import run_simulation

        controller = MagicMock()
        controller.load_config.return_value = CommandResult("OK", "loaded")
        controller.set_playback_rate.return_value = CommandResult("OK", "rate updated")
        controller.start.return_value = CommandResult("OK", "started")
        controller.start_rally.return_value = CommandResult("OK", "开始集结")
        controller.get_snapshot.side_effect = [
            SimpleNamespace(run_state=RunState.READY, nodes=[SimpleNamespace(role="rally_leader")]),
            SimpleNamespace(run_state=RunState.RUNNING, nodes=[]),
            SimpleNamespace(run_state=RunState.FINISHED, nodes=[]),
        ]
        with (
            patch("src.main.SimulationController", return_value=controller),
            patch("src.main.time.sleep") as sleep,
        ):
            exit_code = run_simulation(Path("configs/base.json"), playback_rate=10.0)

        self.assertEqual(exit_code, 0)
        controller.set_file_log_enabled.assert_called_once_with(True)
        controller.load_config.assert_called_once_with(str(Path("configs/base.json")))
        controller.set_playback_rate.assert_called_once_with(10.0)
        controller.start.assert_called_once_with()
        controller.start_rally.assert_called_once_with()
        sleep.assert_called_once()
        controller.close.assert_called_once_with()


class BatchBatTests(unittest.TestCase):
    """验证用户可双击的一键运行脚本契约。"""

    def test_batch_bat_runs_headless_source_at_ten_times_rate(self) -> None:
        """BAT 应直接按 10 倍速无界面运行源码，并固定使用 simulation_data。"""

        script = (PROJECT_ROOT / "result" / "run_batch.bat").read_text(encoding="utf-8")

        self.assertIn(".venv\\Scripts\\python.exe", script)
        self.assertIn("src\\main.py", script)
        self.assertIn('"%SOURCE_PYTHON%" "%PROJECT_ROOT%\\src\\main.py"', script)
        self.assertNotIn("src\\ui\\gui\\main_window.py", script)
        self.assertIn("--config", script)
        self.assertIn('set "PLAYBACK_RATE=10"', script)
        self.assertIn('--rate "%PLAYBACK_RATE%"', script)
        self.assertNotIn("--auto-run", script)
        self.assertNotIn("PyInstaller", script)
        self.assertNotIn("build_windows_full_release.ps1", script)
        self.assertNotIn("analyze_formation_accuracy.py", script)
        self.assertIn("simulation_data", script)

    def test_batch_bat_uses_windows_line_endings(self) -> None:
        """BAT 必须使用 CRLF，避免 CMD 将下一行首字符吞掉后直接退出。"""

        content = (PROJECT_ROOT / "result" / "run_batch.bat").read_bytes()

        self.assertIn(b"\r\n", content)
        self.assertNotIn(b"\n", content.replace(b"\r\n", b""))


class AnalysisBatTests(unittest.TestCase):
    """验证编队精度分析使用独立 BAT，不与仿真脚本耦合。"""

    def test_accuracy_bat_supports_snapshot_selection(self) -> None:
        """分析 BAT 应弹出快照选择框，并允许直接传入快照文件。"""

        script = (PROJECT_ROOT / "result" / "analyze_accuracy.bat").read_text(encoding="utf-8")

        self.assertIn("analyze_formation_accuracy.py", script)
        self.assertIn("analysis", script)

    def test_accuracy_bat_uses_windows_line_endings(self) -> None:
        """独立分析 BAT 同样必须使用 CRLF，保证双击执行稳定。"""

        content = (PROJECT_ROOT / "result" / "analyze_accuracy.bat").read_bytes()

        self.assertIn(b"\r\n", content)
        self.assertNotIn(b"\n", content.replace(b"\r\n", b""))


if __name__ == "__main__":
    unittest.main()
