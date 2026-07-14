"""全量版控制监控功能。注意：真实窗口在用户打开入口时才导入。"""

from __future__ import annotations

from typing import TYPE_CHECKING

from src.runner.sim_control import RunState

if TYPE_CHECKING:
    from PySide6.QtWidgets import QMenu

    from src.ui.gui.live_monitor import LiveMonitorWindow
    from src.ui.gui.main_window import MainWindow
    from src.ui.gui.offline_plot import OfflinePlotWindow


class FullControlMonitorFeature:
    """全量版控制监控入口。注意：同时维护实时监控和旧离线回放窗口生命周期。"""

    def __init__(self) -> None:
        """初始化窗口引用。注意：真实窗口保持懒创建。"""

        self.monitor_menu: QMenu | None = None
        self.live_monitor: LiveMonitorWindow | None = None
        self.offline_plot: OfflinePlotWindow | None = None

    def register_menu(self, window: MainWindow) -> None:
        """注册控制监控菜单。注意：保持既有菜单文案和顺序。"""

        # 菜单对象同时挂到窗口上，保留既有测试和调试入口的可见引用。
        self.monitor_menu = window.menuBar().addMenu("控制监控(&V)")
        window.monitor_menu = self.monitor_menu
        # lambda 延迟传入主窗口实例，避免 QAction 触发时丢失 owner 上下文。
        self.monitor_menu.addAction("数据监控(&M)").triggered.connect(
            lambda checked=False: self.open_live_monitor(window)
        )
        # 旧离线回放仍归在控制监控菜单下，和历史 UI 保持一致。
        self.monitor_menu.addAction("离线分析(&A)").triggered.connect(
            lambda checked=False: self.open_offline_plot(window)
        )

    def open_live_monitor(self, window: MainWindow) -> None:
        """打开实时控制监控窗口。注意：READY 状态也要绑定控制器。"""

        from src.ui.gui.live_monitor import LiveMonitorWindow

        # 窗口懒创建，未使用该功能时 PyInstaller 不需要追踪运行期对象。
        if self.live_monitor is None:
            self.live_monitor = LiveMonitorWindow(window)
        # READY 状态已经有真实控制器，打开监控时应立即 follow。
        if window.sim.snapshot().run_state != RunState.UNLOADED:
            self.live_monitor.follow(window.sim.controller)
        self.live_monitor.show()
        self.live_monitor.raise_()

    def open_offline_plot(self, window: MainWindow) -> None:
        """打开旧离线控制误差回放窗口。注意：窗口实例复用。"""

        from src.ui.gui.offline_plot import OfflinePlotWindow

        # 离线回放不绑定控制器，复用窗口即可保留用户选择状态。
        if self.offline_plot is None:
            self.offline_plot = OfflinePlotWindow(window)
        self.offline_plot.show()
        self.offline_plot.raise_()

    def follow_if_open(self, window: MainWindow) -> None:
        """在实时监控窗口已打开时绑定控制器。注意：不开窗口不产生额外依赖。"""

        # 未打开监控窗口时不导入、不创建，也不占用刷新链路。
        if self.live_monitor is not None:
            self.live_monitor.follow(window.sim.controller)

    def reset_if_open(self, window: MainWindow) -> None:
        """在仿真重置后重置实时监控数据流。注意：保留当前配置节点列表。"""

        # reset 后控制器仍可用，只清曲线数据，不解绑实时窗口。
        if self.live_monitor is not None:
            self.live_monitor.reset_stream(window.sim.controller)

    def unfollow(self) -> None:
        """解除实时监控窗口绑定。注意：配置切换或控制器不可用时调用。"""

        # 只处理已打开窗口，裁剪版和未打开状态都不需要额外分支。
        if self.live_monitor is not None:
            self.live_monitor.unfollow()

    def close(self) -> None:
        """关闭控制监控相关窗口。注意：主窗口释放控制器前调用。"""

        # 实时窗口先关，避免主控制器关闭后定时器继续轮询。
        if self.live_monitor is not None:
            self.live_monitor.close()
        # 离线窗口后关，它只读文件，不依赖控制器生命周期。
        if self.offline_plot is not None:
            self.offline_plot.close()
