"""三维态势子窗口。注意：当前只建立独立窗口外壳，后续再接 Qt Quick 3D 场景。"""

from __future__ import annotations

from PySide6.QtWidgets import QVBoxLayout, QWidget


class Situation3DWindow(QWidget):
    """3D 态势独立窗口。注意：窗口生命周期由 MainWindow 持有和复用。"""

    def __init__(self, parent: QWidget | None = None) -> None:
        """初始化 Situation3DWindow 实例，建立后续嵌入 Qt Quick 3D 的容器。"""
        super().__init__(parent)
        self.setWindowTitle("3D态势")
        self.resize(1120, 760)
        self.setMinimumSize(900, 620)
        # 先固定独立布局边界；后续 QQuickView container 只追加到这个布局里。
        self.root_layout = QVBoxLayout(self)
        self.root_layout.setContentsMargins(0, 0, 0, 0)
        self.root_layout.setSpacing(0)
