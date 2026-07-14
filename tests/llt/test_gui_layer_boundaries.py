"""GUI 分层边界回归测试。注意：防止界面层重新越过 runner 应用层。"""

from __future__ import annotations

import ast
from pathlib import Path
import unittest


_PROJECT_ROOT = Path(__file__).resolve().parents[2]
_BOUNDARY_FILES = (
    "src/ui/gui/main_window.py",
    "src/ui/gui/main_window_actions.py",
    "src/ui/gui/main_window_avoidance.py",
    "src/ui/gui/avoidance_tools.py",
    "src/ui/gui/simulation_adapter.py",
    "src/ui/gui/view_models.py",
)
_FORBIDDEN_PREFIXES = ("src.algorithm", "src.data")


class GuiLayerBoundaryTests(unittest.TestCase):
    """锁定正式主界面到仿真控制应用层的单向依赖。"""

    def test_main_gui_does_not_import_algorithm_or_data_layers(self) -> None:
        """主界面相关模块不得直接导入算法层或数据层。"""

        violations: list[str] = []
        for relative_path in _BOUNDARY_FILES:
            path = _PROJECT_ROOT / relative_path
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=relative_path)
            for node in ast.walk(tree):
                if not isinstance(node, ast.ImportFrom) or node.module is None:
                    continue
                if node.module.startswith(_FORBIDDEN_PREFIXES):
                    violations.append(f"{relative_path}:{node.lineno} -> {node.module}")
        self.assertEqual(violations, [])


if __name__ == "__main__":
    unittest.main()
