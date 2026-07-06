"""GUI 可选功能契约。注意：上层只依赖这些方法，不依赖具体窗口类。"""

from __future__ import annotations

from typing import TYPE_CHECKING, Protocol

if TYPE_CHECKING:
    from src.ui.gui.main_window import MainWindow
    from src.ui.gui.view_models import Snapshot


class ControlMonitorFeature(Protocol):
    """控制监控功能契约。注意：包含实时监控和旧离线回放两个入口。"""

    def register_menu(self, window: MainWindow) -> None:
        """注册控制监控菜单。裁剪版不注册任何可见入口。"""

    def open_live_monitor(self, window: MainWindow) -> None:
        """打开实时数据监控窗口。裁剪版保持空操作。"""

    def open_offline_plot(self, window: MainWindow) -> None:
        """打开旧离线回放窗口。裁剪版保持空操作。"""

    def follow_if_open(self, window: MainWindow) -> None:
        """在控制器可用时绑定已打开监控窗口。裁剪版保持空操作。"""

    def unfollow(self) -> None:
        """解除实时监控窗口与控制器的绑定。裁剪版保持空操作。"""

    def close(self) -> None:
        """关闭该功能已创建的窗口。裁剪版保持空操作。"""


class DataAnalysisFeature(Protocol):
    """数据分析功能契约。注意：只负责独立离线控制效果分析入口。"""

    def register_menu(self, window: MainWindow) -> None:
        """注册数据分析菜单。裁剪版不注册任何可见入口。"""

    def open(self, window: MainWindow) -> None:
        """打开控制效果分析窗口。裁剪版保持空操作。"""

    def close(self) -> None:
        """关闭该功能已创建的窗口。裁剪版保持空操作。"""


class Situation3DFeature(Protocol):
    """3D 态势功能契约。注意：只通过快照推送展示数据，不参与仿真闭环。"""

    def register_menu(self, window: MainWindow) -> None:
        """注册 3D 态势入口。裁剪版不注册任何可见入口。"""

    def open(self, window: MainWindow) -> None:
        """打开 3D 态势窗口。裁剪版保持空操作。"""

    def update_snapshot(self, window: MainWindow, snapshot: Snapshot) -> None:
        """向已打开的 3D 态势窗口同步快照。裁剪版保持空操作。"""

    def close(self) -> None:
        """关闭该功能已创建的窗口。裁剪版保持空操作。"""
