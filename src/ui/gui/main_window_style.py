"""MainWindow 主题样式应用方法。注意：集中维护样式表。"""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtGui import QGuiApplication


class MainWindowStyleMixin:
    """拆分主窗口主题渲染逻辑。注意：由 MainWindow 继承使用。"""

    def _apply_theme(self) -> None:
        """应用 theme 设置。注意：只修改对应显示或运行参数。"""
        # Qt 的平台配色请求会同步 Windows 原生标题栏，避免深色客户区上方仍保留亮白标题栏。
        color_scheme = Qt.ColorScheme.Dark if self.theme_key == "dark" else Qt.ColorScheme.Light
        QGuiApplication.styleHints().setColorScheme(color_scheme)
        # 由当前主题派生若干交互态颜色（悬停/按下/选中），统一注入 Qt 样式表。
        theme = self.theme
        button_hover = theme.line.lighter(108)
        button_pressed = theme.line.darker(108)
        button_border_hover = theme.accent
        menu_selected = theme.line.lighter(112)
        # 用 f-string 把主题色名填入 QSS；注意 QSS 的花括号在 f-string 中需写成 {{ }}。
        self.setStyleSheet(
            f"""
            QMainWindow, QWidget {{
                background: {theme.bg.name()};
                color: {theme.ink.name()};
                font-family: -apple-system, BlinkMacSystemFont, "Segoe UI";
                font-size: 13px;
            }}
            QLabel {{
                background: transparent;
            }}
            QFrame#panel, QGroupBox {{
                background: {theme.panel.name()};
                border: 1px solid {theme.line.name()};
                border-radius: 8px;
            }}
            QGroupBox {{
                margin-top: 10px;
                font-weight: 700;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 10px;
                padding: 0 4px;
                background: {theme.panel.name()};
            }}
            QLabel#stageTitle {{
                font-size: 15px;
                font-weight: 700;
            }}
            QLabel#sectionTitle {{
                background: transparent;
                color: {theme.ink.name()};
                font-size: 14px;
                font-weight: 700;
                padding: 0 0 2px 0;
            }}
            QLabel#statusPill {{
                color: {theme.accent.name()};
                background: {theme.field.name()};
                border-radius: 14px;
                padding: 5px 14px;
                font-weight: 700;
            }}
            QLabel#reportPill {{
                color: {theme.accent.name()};
                background: {theme.field.name()};
                border-radius: 14px;
                padding: 5px 14px;
                font-weight: 700;
            }}
            QLabel#legendLeader {{
                color: {theme.leader.name()};
                font-weight: 700;
            }}
            QLabel#legendWingman {{
                color: {theme.wingman.name()};
                font-weight: 700;
            }}
            QLabel#legendLink, QCheckBox#legendLink {{
                color: {theme.link.name()};
                font-weight: 700;
            }}
            QLabel#legendWarn {{
                color: {theme.warn.name()};
                font-weight: 700;
            }}
            QPushButton {{
                background: {theme.field.name()};
                color: {theme.ink.name()};
                border: 1px solid {theme.line.name()};
                border-radius: 6px;
                min-height: 28px;
                padding: 0 10px;
            }}
            QPushButton:hover {{
                background: {button_hover.name()};
                border-color: {button_border_hover.name()};
            }}
            QPushButton:pressed, QPushButton:down {{
                background: {button_pressed.name()};
                border-color: {theme.accent.name()};
                padding-top: 1px;
                padding-left: 11px;
            }}
            QPushButton:disabled {{
                color: {theme.muted.name()};
                background: {theme.line.name()};
                border-color: {theme.line.name()};
            }}
            QPushButton#selectButton {{
                text-align: left;
                padding-left: 10px;
                padding-right: 10px;
            }}
            QPushButton#selectButton:pressed, QPushButton#selectButton:down {{
                padding-left: 11px;
                padding-right: 9px;
            }}
            QLineEdit {{
                background: {theme.field.name()};
                color: {theme.ink.name()};
                selection-background-color: {theme.accent.name()};
                selection-color: #ffffff;
                border: 1px solid {theme.line.name()};
                border-radius: 6px;
                min-height: 30px;
                padding: 0 10px;
            }}
            QLineEdit:focus {{
                border-color: {theme.accent.name()};
            }}
            QLineEdit:disabled {{
                color: {theme.muted.name()};
                background: {theme.line.name()};
                border-color: {theme.line.name()};
            }}
            QDoubleSpinBox {{
                background: {theme.field.name()};
                color: {theme.ink.name()};
                border: 1px solid {theme.line.name()};
                border-radius: 6px;
                min-height: 26px;
                padding: 0 6px;
            }}
            QDoubleSpinBox:focus {{
                border-color: {theme.accent.name()};
            }}
            QDoubleSpinBox:disabled {{
                color: {theme.muted.name()};
                background: {theme.line.name()};
                border-color: {theme.line.name()};
            }}
            QLabel#paramLabel {{
                color: {theme.ink.name()};
            }}
            QLabel#avoidHint {{
                color: {theme.muted.name()};
                background: {theme.field.name()};
                border: 1px solid {theme.line.name()};
                border-radius: 6px;
                padding: 8px 10px;
            }}
            QMenu {{
                background: {theme.field.name()};
                color: {theme.ink.name()};
                border: 1px solid {theme.line.name()};
            }}
            QMenu::item {{
                padding: 4px;
            }}
            QMenu::item:selected {{
                background: {menu_selected.name()};
            }}
            QTableWidget {{
                background: {theme.field.name()};
                gridline-color: {theme.line.name()};
                border: 1px solid {theme.line.name()};
                alternate-background-color: {theme.field.name()};
                font-size: 13px;
            }}
            QHeaderView::section {{
                background: {theme.panel.name()};
                color: {theme.muted.name()};
                border: 0;
                border-bottom: 1px solid {theme.line.name()};
                padding: 6px 4px;
                font-weight: 700;
            }}
            QSplitter#viewSplitter {{
                background: transparent;
            }}
            QSplitter#viewSplitter::handle:vertical {{
                background: transparent;
                border-top: 1px solid {theme.line.name()};
            }}
            QSplitter#viewSplitter::handle:vertical:hover {{
                border-top-color: {theme.accent.name()};
            }}
            QSlider::groove:horizontal {{
                background: {theme.line.name()};
                height: 6px;
                border-radius: 3px;
            }}
            QSlider::sub-page:horizontal {{
                background: {theme.accent.name()};
                border-radius: 3px;
            }}
            QSlider::handle:horizontal {{
                background: {theme.accent.name()};
                border: 2px solid {theme.panel.name()};
                width: 16px;
                height: 16px;
                margin: -6px 0;
                border-radius: 8px;
            }}
            QProgressBar#progress {{
                background: {theme.line.name()};
                border: 0;
                border-radius: 3px;
                min-height: 6px;
                max-height: 6px;
            }}
            QProgressBar#progress::chunk {{
                background: {theme.accent.name()};
                border-radius: 3px;
            }}
            """
        )
        # 样式表只管控件；画布颜色需单独下发给两个自绘视图。
        self.top_view.set_theme(theme)
        self.side_view.set_theme(theme)
