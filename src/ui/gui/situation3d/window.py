"""三维态势子窗口。注意：窗口内嵌 Qt Quick 3D 场景，数据由主窗口推送。"""

from __future__ import annotations

from pathlib import Path

# 显式导入 QtQuick3D，保证 PyInstaller 收集 QML 侧 QtQuick3D 运行插件。
from PySide6 import QtQuick3D  # noqa: F401
from PySide6.QtCore import Qt, QUrl
from PySide6.QtQml import qmlRegisterType
from PySide6.QtQuick import QQuickView
from PySide6.QtWidgets import QDialog, QLabel, QVBoxLayout, QWidget

from src.ui.gui.situation3d.bridge import Situation3DBridge
from src.ui.gui.situation3d.scene_data import build_scene_payload
from src.ui.gui.situation3d.terrain_geometry import TerrainGeometry
from src.ui.gui.view_models import ObstacleView, Snapshot

_QML_TYPES_REGISTERED = False


def _register_qml_types() -> None:
    """注册 3D 态势 QML 类型。注意：重复注册会触发 Qt 告警。"""

    global _QML_TYPES_REGISTERED
    if _QML_TYPES_REGISTERED:
        return
    qmlRegisterType(TerrainGeometry, "Simu3D", 1, 0, "TerrainGeometry")
    _QML_TYPES_REGISTERED = True


class Situation3DWindow(QDialog):
    """3D 态势独立窗口。注意：窗口生命周期由 MainWindow 持有和复用。"""

    def __init__(self, parent: QWidget | None = None) -> None:
        """初始化 Situation3DWindow 实例，创建 Qt Quick 3D 场景容器。"""
        super().__init__(parent)
        self.setModal(False)
        # QDialog 默认只给关闭按钮，显式补上最小化/最大化，方便全屏观察 3D 场景。
        self.setWindowFlags(
            self.windowFlags()
            | Qt.WindowMinimizeButtonHint
            | Qt.WindowMaximizeButtonHint
        )
        self.setWindowTitle("3D态势")
        self.resize(1120, 760)
        self.setMinimumSize(900, 620)
        self.root_layout = QVBoxLayout(self)
        self.root_layout.setContentsMargins(0, 0, 0, 0)
        self.root_layout.setSpacing(0)
        self.bridge = Situation3DBridge()
        _register_qml_types()
        self.quick_view = QQuickView()
        self.quick_view.setResizeMode(QQuickView.ResizeMode.SizeRootObjectToView)
        self.quick_view.rootContext().setContextProperty("sceneBridge", self.bridge)
        qml_path = Path(__file__).resolve().parent / "qml" / "Situation3DView.qml"
        self.quick_view.setSource(QUrl.fromLocalFile(str(qml_path)))
        # 通过 QWidget 容器嵌入 QQuickView，使主 GUI 仍保持 Widgets 架构。
        self.quick_container = QWidget.createWindowContainer(self.quick_view, self)
        self.quick_container.setMinimumSize(1, 1)
        self.root_layout.addWidget(self.quick_container)
        self._fallback_label: QLabel | None = None
        if self.quick_view.status() == QQuickView.Status.Error:
            self._show_fallback()

    def set_snapshot(
        self,
        snapshot: Snapshot,
        *,
        obstacles: list[ObstacleView] | None = None,
        clearance_m: float = 0.0,
    ) -> None:
        """刷新 3D 场景数据。注意：输入快照仍使用项目 ENU 坐标约定。"""

        payload = build_scene_payload(snapshot, obstacles or [], clearance_m=clearance_m)
        self.bridge.set_scene_payload(payload)

    def _show_fallback(self) -> None:
        """显示 QML 加载失败兜底信息。注意：避免窗口空白导致误判为正常。"""

        messages = ["Qt Quick 3D 场景加载失败："]
        messages.extend(error.toString() for error in self.quick_view.errors())
        self._fallback_label = QLabel("\n".join(messages), self)
        self._fallback_label.setWordWrap(True)
        self.root_layout.addWidget(self._fallback_label)
