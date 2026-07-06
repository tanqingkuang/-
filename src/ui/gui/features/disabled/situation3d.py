"""裁剪版 3D 态势功能。注意：不注册菜单，也不导入 Qt Quick 3D。"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from src.ui.gui.main_window import MainWindow
    from src.ui.gui.view_models import Snapshot


class DisabledSituation3DFeature:
    """裁剪版 3D 态势入口。注意：所有快照推送均为空操作。"""

    def __init__(self) -> None:
        """初始化空引用。注意：用于测试确认功能未启用。"""

        self.action = None
        self.window = None

    def register_menu(self, window: MainWindow) -> None:
        """跳过 3D 态势入口注册。注意：裁剪版顶部不出现该入口。"""

        # 给调用方留下明确的空引用，避免误判为尚未初始化。
        window.situation3d_action = None

    def open(self, window: MainWindow) -> None:
        """忽略 3D 窗口打开请求。注意：裁剪版没有该窗口。"""

    def update_snapshot(self, window: MainWindow, snapshot: Snapshot) -> None:
        """忽略快照更新请求。注意：裁剪版不消费展示数据。"""

    def close(self) -> None:
        """忽略窗口关闭请求。注意：裁剪版没有可关闭窗口。"""
