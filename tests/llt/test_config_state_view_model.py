"""配置状态 ViewModel 的函数级回归测试。注意：本文件不构造 Qt 对象。"""

from __future__ import annotations

import ast
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from src.ui.gui.config_state_view_model import (
    dialog_start_dir,
    display_config_path,
    parse_last_config_value,
    relative_config_path,
)


class ConfigStateViewModelTests(unittest.TestCase):
    """覆盖配置路径显示和记忆决策，避免依赖完整 GUI 长链条。"""

    def test_view_model_and_tests_do_not_import_pyside6(self) -> None:
        """ConfigStateViewModel 及本测试文件不得在 import 中引入 PySide6。"""

        paths = [Path("src/ui/gui/config_state_view_model.py"), Path(__file__)]
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

    def test_relative_path_preserves_child_and_parent_traversal_paths(self) -> None:
        """相对化保留普通子路径和上行路径，并统一使用正斜杠。"""

        with tempfile.TemporaryDirectory() as tmp:
            parent = Path(tmp).resolve()
            project_root = parent / "app"
            child = project_root / "configs" / "child.json"
            sibling = parent / "configs" / "sibling.json"

            child_relative = relative_config_path(child, project_root)
            sibling_relative = relative_config_path(sibling, project_root)

        self.assertEqual(child_relative, "configs/child.json")
        self.assertEqual(sibling_relative, "../configs/sibling.json")
        self.assertNotIn("\\", child_relative or "")
        self.assertNotIn("\\", sibling_relative or "")

    def test_relative_path_returns_none_when_relpath_raises_value_error(self) -> None:
        """跨盘符导致 relpath 抛出 ValueError 时返回 None。"""

        with patch("src.ui.gui.config_state_view_model.os.path.relpath", side_effect=ValueError):
            result = relative_config_path(Path("D:/configs/demo.json"), Path("C:/app"))

        self.assertIsNone(result)

    def test_relative_path_returns_none_when_relpath_stays_absolute(self) -> None:
        """relpath 异常返回绝对路径时按不可相对化处理。"""

        absolute_path = str(Path.cwd().resolve())
        with patch("src.ui.gui.config_state_view_model.os.path.relpath", return_value=absolute_path):
            result = relative_config_path(Path("config.json"), Path.cwd())

        self.assertIsNone(result)

    def test_display_path_prefers_relative_path_and_falls_back_to_name(self) -> None:
        """显示文本优先使用相对路径，不可相对化时只显示文件名。"""

        project_root = Path.cwd().resolve()
        relative = display_config_path(project_root / "configs" / "demo.json", project_root)
        with patch("src.ui.gui.config_state_view_model.relative_config_path", return_value=None):
            fallback = display_config_path(Path("D:/outside/demo.json"), project_root)

        self.assertEqual(relative, "configs/demo.json")
        self.assertEqual(fallback, "demo.json")

    def test_dialog_start_dir_uses_root_without_memory(self) -> None:
        """没有上次配置记录时对话框从项目根目录开始。"""

        project_root = Path.cwd().resolve()

        self.assertEqual(dialog_start_dir(None, project_root), project_root)

    def test_dialog_start_dir_uses_existing_parent_directory(self) -> None:
        """上次配置的父目录仍存在时从该目录打开对话框。"""

        with tempfile.TemporaryDirectory() as tmp:
            project_root = Path(tmp).resolve()
            config_dir = project_root / "configs"
            config_dir.mkdir()

            result = dialog_start_dir("configs/last.json", project_root)

        self.assertEqual(result, config_dir)

    def test_dialog_start_dir_falls_back_when_parent_directory_is_missing(self) -> None:
        """上次配置的父目录不存在时回退项目根目录。"""

        with tempfile.TemporaryDirectory() as tmp:
            project_root = Path(tmp).resolve()

            result = dialog_start_dir("missing/last.json", project_root)

        self.assertEqual(result, project_root)

    def test_parse_last_config_value_normalizes_blank_and_strips_text(self) -> None:
        """ini 空值和纯空白归一化为 None，正常路径去除首尾空白。"""

        self.assertIsNone(parse_last_config_value(""))
        self.assertIsNone(parse_last_config_value(" \t\r\n "))
        self.assertEqual(parse_last_config_value("  configs/demo.json  "), "configs/demo.json")


if __name__ == "__main__":
    unittest.main()
