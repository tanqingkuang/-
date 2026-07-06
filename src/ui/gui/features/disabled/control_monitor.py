"""裁剪版控制监控功能。注意：不注册菜单，也不导入 QtCharts 窗口。"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from src.ui.gui.main_window import MainWindow


class DisabledControlMonitorFeature:
    """裁剪版控制监控入口。注意：所有生命周期调用均为空操作。"""

    def __init__(self) -> None:
        """初始化空引用。注意：用于测试和调试确认功能未启用。"""

        self.monitor_menu = None
        self.live_monitor = None
        self.offline_plot = None

    def register_menu(self, window: MainWindow) -> None:
        """跳过控制监控菜单注册。注意：裁剪版顶部不出现该入口。"""

        # 给调用方留下明确的空引用，避免误判为尚未初始化。
        window.monitor_menu = None

    def open_live_monitor(self, window: MainWindow) -> None:
        """忽略实时监控打开请求。注意：裁剪版没有该窗口。"""
        # 主窗口保留兼容方法，裁剪版调用到这里时必须稳定无副作用。

    def open_offline_plot(self, window: MainWindow) -> None:
        """忽略旧离线回放打开请求。注意：裁剪版没有该窗口。"""
        # 不抛异常，方便测试或旧快捷入口误触时保持主界面可用。

    def follow_if_open(self, window: MainWindow) -> None:
        """忽略控制器绑定请求。注意：裁剪版不会持有控制器。"""
        # 裁剪版不持有窗口对象，因此也不应持有控制器引用。

    def unfollow(self) -> None:
        """忽略控制器解绑请求。注意：裁剪版不会持有控制器。"""
        # 保持与 full provider 相同生命周期接口，调用层无需分支。

    def close(self) -> None:
        """忽略窗口关闭请求。注意：裁剪版没有可关闭窗口。"""
        # 裁剪版没有子窗口，关闭阶段不做任何额外动作。
