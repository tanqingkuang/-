"""侧视图控制 ViewModel 的函数级回归测试。注意：本文件不构造 Qt 对象。"""

from __future__ import annotations

import ast
import unittest
from pathlib import Path

from src.ui.gui.side_view_control_view_model import (
    SideViewControlViewModel,
    geodetic_click_text,
    normalized_view_angle,
)


class SideViewControlViewModelTests(unittest.TestCase):
    """覆盖侧视图锁定、角度与坐标文案规则。注意：测试不依赖 GUI 控件。"""

    def test_view_model_and_tests_do_not_import_pyside6(self) -> None:
        """ViewModel 及本测试文件不得在 import 中引入 PySide6。"""

        paths = [Path("src/ui/gui/side_view_control_view_model.py"), Path(__file__)]
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

    def test_lock_toggle_remembers_preference_only_while_enabled(self) -> None:
        """禁用态信号不覆盖偏好，启用态切换才记忆用户选择。"""

        view_model = SideViewControlViewModel()
        disabled_update = view_model.on_lock_toggled(
            checked=False,
            lock_enabled=False,
            current_angle=123.5,
        )
        restored = view_model.on_sync(
            lock_available=True,
            side_view_locked=False,
            current_angle=123.5,
        )
        view_model.on_lock_toggled(checked=False, lock_enabled=True, current_angle=45.0)
        user_unlocked = view_model.on_sync(
            lock_available=True,
            side_view_locked=False,
            current_angle=45.0,
        )

        self.assertFalse(disabled_update.apply_locked)
        self.assertEqual(disabled_update.view_angle_deg, 123.5)
        self.assertTrue(restored.lock_checked)
        self.assertEqual(restored.apply_locked, True)
        self.assertFalse(user_unlocked.lock_checked)
        self.assertIsNone(user_unlocked.apply_locked)

    def test_unavailable_lock_forces_unchecked_without_losing_preference(self) -> None:
        """航段锁定暂不可用时强制取消，恢复可用后仍采用原偏好。"""

        view_model = SideViewControlViewModel()
        unavailable = view_model.on_sync(
            lock_available=False,
            side_view_locked=True,
            current_angle=270.0,
        )
        restored = view_model.on_sync(
            lock_available=True,
            side_view_locked=False,
            current_angle=270.0,
        )

        self.assertFalse(unavailable.lock_enabled)
        self.assertFalse(unavailable.lock_checked)
        self.assertEqual(unavailable.apply_locked, False)
        self.assertTrue(restored.lock_enabled)
        self.assertTrue(restored.lock_checked)
        self.assertEqual(restored.apply_locked, True)

    def test_normalized_view_angle_wraps_and_rounds_like_existing_ui(self) -> None:
        """角度先按 Python round 取整再折回 0 到 359。"""

        self.assertEqual(normalized_view_angle(-0.6), 359)
        self.assertEqual(normalized_view_angle(721.6), 2)
        self.assertEqual(normalized_view_angle(12.5), 12)
        self.assertEqual(normalized_view_angle(13.5), 14)

    def test_sync_disables_angle_controls_while_locked(self) -> None:
        """锁定生效时禁用角度控件，并回填归一化后的当前角度。"""

        update = SideViewControlViewModel().on_sync(
            lock_available=True,
            side_view_locked=True,
            current_angle=361.6,
        )

        self.assertTrue(update.lock_checked)
        self.assertIsNone(update.apply_locked)
        self.assertEqual(update.angle_value, 2)
        self.assertFalse(update.angle_controls_enabled)

    def test_geodetic_click_text_handles_missing_and_formats_longitude_first(self) -> None:
        """无 origin 显示提示，有坐标时经度在前且保留七位小数。"""

        self.assertEqual(geodetic_click_text(None), "当前配置无经纬 origin")
        self.assertEqual(
            geodetic_click_text((39.12345678, 116.98765432)),
            "116.9876543, 39.1234568",
        )


if __name__ == "__main__":
    unittest.main()
