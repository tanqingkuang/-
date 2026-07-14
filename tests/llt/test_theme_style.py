"""GUI 主题样式与复用控件测试。"""

from __future__ import annotations

import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QPoint
from PySide6.QtWidgets import QApplication

from src.ui.gui.main_window_style import MainWindowStyleMixin
from src.ui.gui.theme_widgets import (
    SELECT_POPUP_RIGHT_GAP_PX,
    THEMES,
    SelectButton,
)


class _CanvasStub:
    """记录样式下发的最小画布替身。"""

    def set_theme(self, theme) -> None:
        """保存最后一次主题。"""

        self.theme = theme


class _StyleHost(MainWindowStyleMixin):
    """为主题 Mixin 提供最小宿主。"""

    def __init__(self, theme_key: str) -> None:
        """初始化主题和样式捕获槽。"""

        self.theme_key = theme_key
        self.theme = THEMES[theme_key]
        self.top_view = _CanvasStub()
        self.side_view = _CanvasStub()
        self.stylesheet = ""

    def setStyleSheet(self, stylesheet: str) -> None:
        """捕获待应用的样式表。"""

        self.stylesheet = stylesheet


class ThemeStyleTests(unittest.TestCase):
    """锁定主题色引用和选择器弹出锚点。"""

    @classmethod
    def setUpClass(cls) -> None:
        """复用 Qt 应用实例。"""

        cls.app = QApplication.instance() or QApplication([])

    def test_report_pill_follows_current_theme_accent(self) -> None:
        """控制回报胶囊应随深浅主题切换，不能残留固定蓝色。"""

        for theme_key in THEMES:
            with self.subTest(theme_key=theme_key):
                host = _StyleHost(theme_key)
                host._apply_theme()
                report_rule = host.stylesheet.split("QLabel#reportPill", 1)[1].split("}", 1)[0]
                self.assertIn(f"color: {host.theme.accent.name()};", report_rule)
                self.assertNotIn("#175cd3", report_rule)

    def test_right_popup_anchor_uses_named_gap(self) -> None:
        """右侧菜单锚点以命名间距计算，尺寸变化时仍从按钮右缘起算。"""

        button = SelectButton(120, popup_side="right")
        button.resize(180, 32)
        self.assertEqual(
            button._popup_anchor(),
            QPoint(button.width() + SELECT_POPUP_RIGHT_GAP_PX, 0),
        )


if __name__ == "__main__":
    unittest.main()
