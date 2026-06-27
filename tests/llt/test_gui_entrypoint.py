"""验证 GUI 文件入口在不同启动方式下保持可导入。"""

from __future__ import annotations

import os
import subprocess
import sys
import unittest
from pathlib import Path


class GuiEntrypointTests(unittest.TestCase):
    """覆盖 VS Code 直接调试 Python 文件时的路径初始化行为。"""

    def test_main_window_file_loads_without_pythonpath(self) -> None:
        """清空 PYTHONPATH 后直接加载 GUI 文件，应仍能解析 src 包导入。"""

        project_root = Path(__file__).resolve().parents[2]
        env = os.environ.copy()
        env.pop("PYTHONPATH", None)
        env.setdefault("QT_QPA_PLATFORM", "offscreen")

        script = (
            "import runpy; "
            "ns = runpy.run_path('src/ui/gui/main_window.py', run_name='formation_sim_bootstrap_check'); "
            "print(ns['default_project_root']())"
        )
        result = subprocess.run(
            [sys.executable, "-c", script],
            cwd=project_root,
            env=env,
            check=True,
            capture_output=True,
            text=True,
        )

        self.assertEqual(result.stdout.strip(), str(project_root))


if __name__ == "__main__":
    unittest.main()
