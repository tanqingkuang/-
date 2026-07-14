"""主窗口使用的辅助弹窗。注意：不持有仿真业务状态。"""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QDialog, QPushButton, QTextEdit, QVBoxLayout, QWidget

LOG_MAX_BLOCKS = 1000


class LogDialog(QDialog):
    """仿真事件弹窗。注意：只展示日志文本。"""

    def __init__(self, parent: QWidget | None = None) -> None:
        """初始化 LogDialog 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        super().__init__(parent)
        self.setWindowTitle("日志")
        self.resize(720, 360)
        layout = QVBoxLayout(self)
        self.text = QTextEdit()
        self.text.setReadOnly(True)
        # 让 QTextDocument 在追加时自动淘汰最旧文本块，避免手工复制整段日志。
        self.text.document().setMaximumBlockCount(LOG_MAX_BLOCKS)
        clear_button = QPushButton("清空")
        clear_button.clicked.connect(self.text.clear)
        layout.addWidget(self.text)
        layout.addWidget(clear_button, alignment=Qt.AlignmentFlag.AlignRight)

    def append(self, time_value: float, source: str, message: str) -> None:
        """追加一条显示内容。注意：超出容量时需要裁剪旧记录。"""
        self.text.append(f"{time_value:05.1f}s  {source:<10} {message}")


class StageFullscreenDialog(QDialog):
    """只用于全屏实时显示区的顶层外壳。注意：退出时需归还原控件。"""

    def __init__(self, owner: "MainWindow") -> None:
        """初始化 StageFullscreenDialog 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        super().__init__(owner)
        self.owner = owner
        self.setWindowTitle("二维实时显示")
        self.setWindowFlag(Qt.WindowType.FramelessWindowHint, True)

    def keyPressEvent(self, event) -> None:  # type: ignore[override]
        """处理键盘事件。注意：快捷键只影响窗口交互状态。"""
        if event.key() == Qt.Key.Key_Escape:
            self.owner._exit_stage_fullscreen()
            return
        super().keyPressEvent(event)

    def closeEvent(self, event) -> None:  # type: ignore[override]
        """处理窗口关闭事件。注意：关闭前需要释放控制器资源。"""
        if self.owner._stage_fullscreen_dialog is self:
            self.owner._exit_stage_fullscreen()
        event.accept()
