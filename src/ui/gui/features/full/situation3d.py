"""全量版 3D 态势功能。注意：Qt Quick 3D 依赖只在打开窗口时导入。"""

from __future__ import annotations

from typing import TYPE_CHECKING

from PySide6.QtGui import QAction

if TYPE_CHECKING:
    from src.ui.gui.main_window import MainWindow
    from src.ui.gui.situation3d import Situation3DWindow
    from src.ui.gui.view_models import Snapshot


class FullSituation3DFeature:
    """全量版 3D 态势入口。注意：窗口只展示主窗口推送的快照。"""

    def __init__(self) -> None:
        """初始化窗口和动作引用。注意：真实 3D 窗口保持懒创建。"""

        self.action: QAction | None = None
        self.window: Situation3DWindow | None = None

    def register_menu(self, window: MainWindow) -> None:
        """注册 3D 态势入口。注意：保持顶层菜单栏动作形态。"""

        # 3D 态势历史上是顶层 QAction，不是下拉菜单。
        self.action = QAction("3D态势(&3)", window)
        window.situation3d_action = self.action
        # 触发时才导入 Qt Quick 3D，源码 lite 档位构造时不会碰到 QML。
        self.action.triggered.connect(lambda checked=False: self.open(window))
        window.menuBar().addAction(self.action)

    def open(self, window: MainWindow) -> None:
        """打开 3D 态势窗口。注意：重复触发复用同一个窗口实例。"""

        from src.ui.gui.situation3d import Situation3DWindow

        # QQuickView 初始化成本较高，因此只在用户首次打开时创建。
        if self.window is None:
            self.window = Situation3DWindow(window)
        # 首次打开前先推送一帧快照，避免窗口空白等待下一次 tick。
        self.update_snapshot(window, window.sim.snapshot())
        self.window.showMaximized()
        self.window.raise_()
        self.window.activateWindow()

    def update_snapshot(self, window: MainWindow, snapshot: Snapshot) -> None:
        """同步 3D 态势窗口数据。注意：窗口未打开时不产生 QML 更新。"""

        if self.window is None:
            return
        # 避障安全间距会影响 3D 障碍膨胀展示，需随当前控件值同步。
        clearance = window.clearance_spin.value() if hasattr(window, "clearance_spin") else 0.0
        self.window.set_snapshot(snapshot, obstacles=window.obstacles, clearance_m=clearance)

    def close(self) -> None:
        """关闭 3D 态势窗口。注意：未打开时保持空操作。"""

        # 关闭 QML 容器必须早于主 QApplication 退出。
        if self.window is not None:
            self.window.close()
