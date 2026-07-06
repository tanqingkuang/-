"""全量版数据分析功能。注意：真实分析窗口在用户打开入口时才导入。"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from PySide6.QtWidgets import QMenu

    from src.ui.gui.data_analysis_window import DataAnalysisWindow
    from src.ui.gui.main_window import MainWindow


class FullDataAnalysisFeature:
    """全量版数据分析入口。注意：该功能只消费离线日志，不参与仿真闭环。"""

    def __init__(self) -> None:
        """初始化窗口引用。注意：真实窗口保持懒创建。"""

        self.menu: QMenu | None = None
        self.window: DataAnalysisWindow | None = None

    def register_menu(self, window: MainWindow) -> None:
        """注册数据分析菜单。注意：保持既有菜单文案。"""

        # 菜单对象挂回主窗口，便于现有调试和测试继续定位菜单项。
        self.menu = window.menuBar().addMenu("数据分析(&D)")
        window.data_analysis_menu = self.menu
        # 数据分析只有一个入口，触发时再加载较重的 QtCharts 窗口。
        self.menu.addAction("控制效果分析(&A)").triggered.connect(lambda checked=False: self.open(window))

    def open(self, window: MainWindow) -> None:
        """打开离线控制效果分析窗口。注意：窗口实例复用。"""

        from src.ui.gui.data_analysis_window import DataAnalysisWindow

        # 分析窗口可复用，避免重复打开时丢失用户已加载的数据。
        if self.window is None:
            self.window = DataAnalysisWindow(window)
        self.window.show()
        self.window.raise_()

    def close(self) -> None:
        """关闭数据分析窗口。注意：未打开时保持空操作。"""

        # 主窗口关闭时统一收口，避免独立窗口留在后台。
        if self.window is not None:
            self.window.close()
