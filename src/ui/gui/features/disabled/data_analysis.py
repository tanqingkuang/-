"""裁剪版数据分析功能。注意：不注册菜单，也不导入分析窗口。"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from src.ui.gui.main_window import MainWindow


class DisabledDataAnalysisFeature:
    """裁剪版数据分析入口。注意：所有入口调用均为空操作。"""

    def __init__(self) -> None:
        """初始化空引用。注意：用于测试确认功能未启用。"""

        self.menu = None
        self.window = None

    def register_menu(self, window: MainWindow) -> None:
        """跳过数据分析菜单注册。注意：裁剪版顶部不出现该入口。"""

        # 给调用方留下明确的空引用，避免误判为尚未初始化。
        window.data_analysis_menu = None

    def open(self, window: MainWindow) -> None:
        """忽略数据分析窗口打开请求。注意：裁剪版没有该窗口。"""

    def close(self) -> None:
        """忽略窗口关闭请求。注意：裁剪版没有可关闭窗口。"""
