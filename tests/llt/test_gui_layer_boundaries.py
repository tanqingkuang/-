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

    def test_fullscreen_actions_use_layout_owned_stage_interface(self) -> None:
        """全屏动作不得直接读写布局索引、拉伸系数和占位控件。"""

        path = _PROJECT_ROOT / "src/ui/gui/main_window_actions.py"
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        methods = {
            node.name: node
            for node in ast.walk(tree)
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        }
        action_nodes = (
            methods["_enter_stage_fullscreen"],
            methods["_exit_stage_fullscreen"],
        )
        attributes = {
            node.attr
            for method in action_nodes
            for node in ast.walk(method)
            if isinstance(node, ast.Attribute)
        }

        self.assertTrue(
            {
                "_take_stage_for_fullscreen",
                "_restore_stage_from_fullscreen",
            }.issubset(attributes)
        )
        self.assertTrue(
            {
                "main_layout",
                "_stage_layout_index",
                "_stage_layout_stretch",
                "_stage_placeholder",
            }.isdisjoint(attributes)
        )

    def test_demo_buttons_are_driven_by_one_action_spec(self) -> None:
        """演示按钮的文案、提示和配置文件必须由同一规格表循环生成。"""

        path = _PROJECT_ROOT / "src/ui/gui/main_window_layout.py"
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        spec_names = {
            target.id
            for node in tree.body
            if isinstance(node, (ast.Assign, ast.AnnAssign))
            for target in (
                node.targets if isinstance(node, ast.Assign) else (node.target,)
            )
            if isinstance(target, ast.Name)
        }
        demo_loops = [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.For)
            and isinstance(node.iter, ast.Name)
            and node.iter.id == "DEMO_CONFIG_ACTIONS"
        ]

        self.assertIn("DEMO_CONFIG_ACTIONS", spec_names)
        self.assertEqual(len(demo_loops), 1)


if __name__ == "__main__":
    unittest.main()
