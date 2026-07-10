"""避障面板 ViewModel 的函数级回归测试。注意：本文件不构造 Qt 对象。"""

from __future__ import annotations

import ast
import unittest
from pathlib import Path

from src.ui.gui.avoidance_panel_view_model import (
    adopt_enabled,
    avoidance_status_text,
    export_enabled,
    param_widgets_enabled,
    simplify_should_follow,
)


class AvoidancePanelViewModelTests(unittest.TestCase):
    """覆盖避障面板稳定显示规则，避免依赖完整 GUI 长链条。"""

    def test_view_model_and_tests_do_not_import_pyside6(self) -> None:
        """避障面板 ViewModel 及本测试文件不得在 import 中引入 PySide6。"""

        paths = [Path("src/ui/gui/avoidance_panel_view_model.py"), Path(__file__)]
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

    def test_simplify_clearance_follows_only_implicit_clearance_changes(self) -> None:
        """拉直间距只在安全间距来源、有参数且未显式设置时跟随。"""

        cases = [
            (False, True, False, False),
            (True, False, False, False),
            (True, True, True, False),
            (True, True, False, True),
        ]

        for source_is_clearance, params_present, explicit, expected in cases:
            with self.subTest(
                source_is_clearance=source_is_clearance,
                params_present=params_present,
                explicit=explicit,
            ):
                self.assertEqual(
                    simplify_should_follow(source_is_clearance, params_present, explicit),
                    expected,
                )

    def test_widget_enables_cover_params_and_preview_matrix(self) -> None:
        """参数与预览存在性共同决定参数控件、采用和导出按钮使能。"""

        cases = [
            (False, False, False, False, False),
            (True, False, True, False, False),
            (True, True, True, True, True),
        ]

        for has_params, has_preview, widgets, adopt, export in cases:
            with self.subTest(has_params=has_params, has_preview=has_preview):
                self.assertEqual(param_widgets_enabled(has_params), widgets)
                self.assertEqual(adopt_enabled(has_preview), adopt)
                self.assertEqual(export_enabled(has_params, has_preview), export)

    def test_status_text_preserves_empty_partial_and_all_enabled_wording(self) -> None:
        """状态文案覆盖无障碍、部分勾选和全勾选，并保持既有文字逐字一致。"""

        self.assertEqual(
            avoidance_status_text(0, 0),
            "未加载障碍：当前配置无 avoidance.obstacles。",
        )
        self.assertEqual(
            avoidance_status_text(2, 3),
            "已勾选 2/3 个障碍。\n设置参数后点「生成航线」预览，满意再「采用航线」。",
        )
        self.assertEqual(
            avoidance_status_text(3, 3),
            "已勾选 3/3 个障碍。\n设置参数后点「生成航线」预览，满意再「采用航线」。",
        )


if __name__ == "__main__":
    unittest.main()
