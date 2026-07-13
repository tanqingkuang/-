"""三维态势子窗口。注意：窗口内嵌 Qt Quick 3D 场景，数据由主窗口推送。"""

from __future__ import annotations

from pathlib import Path

# 显式导入 QtQuick3D，保证 PyInstaller 收集 QML 侧 QtQuick3D 运行插件。
from PySide6 import QtQuick3D  # noqa: F401
from PySide6.QtCore import Qt, QTimer, QUrl
from PySide6.QtQml import qmlRegisterType
from PySide6.QtQuick import QQuickView
from PySide6.QtQuickControls2 import QQuickStyle
from PySide6.QtWidgets import QDialog, QLabel, QVBoxLayout, QWidget

from src.ui.gui.situation3d.aircraft_model_style import (
    DEFAULT_AIRCRAFT_MODEL_TYPE,
    AircraftModelType,
)
from src.ui.gui.situation3d.bridge import Situation3DBridge
from src.ui.gui.situation3d.risk_fill_geometry import RiskFillGeometry
from src.ui.gui.situation3d.scene_data import TrailPayloadState, build_scene_payload
from src.ui.gui.situation3d.terrain_geometry import TerrainGeometry
from src.ui.gui.situation3d.trail_ribbon_geometry import TrailRibbonGeometry
from src.ui.gui.situation3d.trail_tip_geometry import TrailTipGeometry
from src.ui.gui.view_models import ObstacleView, Snapshot

_QML_TYPES_REGISTERED = False


def _register_qml_types() -> None:
    """注册 3D 态势 QML 类型。注意：重复注册会触发 Qt 告警。"""

    global _QML_TYPES_REGISTERED
    if _QML_TYPES_REGISTERED:
        return
    # 机型下拉自定义了 background/contentItem，Windows 原生样式不支持自定义，须显式用 Basic。
    QQuickStyle.setStyle("Basic")
    qmlRegisterType(TerrainGeometry, "Simu3D", 1, 0, "TerrainGeometry")
    qmlRegisterType(TrailRibbonGeometry, "Simu3D", 1, 0, "TrailRibbonGeometry")
    qmlRegisterType(TrailTipGeometry, "Simu3D", 1, 0, "TrailTipGeometry")
    qmlRegisterType(RiskFillGeometry, "Simu3D", 1, 0, "RiskFillGeometry")
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
        self._trail_payload_state = TrailPayloadState()
        self._current_model_type = DEFAULT_AIRCRAFT_MODEL_TYPE
        self._cached_scene: tuple[Snapshot, list[ObstacleView], float] | None = None
        self.bridge.modelSelected.connect(self._on_model_selected)
        # 高度场后台生成期间的重推定时器:单次触发,推完再按需重新武装。
        self._terrain_refresh_timer = QTimer(self)
        self._terrain_refresh_timer.setInterval(500)
        self._terrain_refresh_timer.setSingleShot(True)
        self._terrain_refresh_timer.timeout.connect(self._on_terrain_refresh)
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

        obstacle_items = list(obstacles or [])
        self._cached_scene = (snapshot, obstacle_items, clearance_m)
        self._push_scene_payload(snapshot, obstacle_items, clearance_m)

    def _on_model_selected(self, value: str) -> None:
        """处理 QML 机型选择。注意：只重建显示 payload，不修改仿真快照。"""

        try:
            self._current_model_type = AircraftModelType(value)
        except ValueError:
            return
        if self._cached_scene is None:
            return
        snapshot, obstacles, clearance_m = self._cached_scene
        self._push_scene_payload(snapshot, obstacles, clearance_m)

    def _push_scene_payload(
        self,
        snapshot: Snapshot,
        obstacles: list[ObstacleView],
        clearance_m: float,
    ) -> None:
        """推送当前机型对应的场景 payload。注意：机型只影响 QML 渲染样式。"""

        payload = build_scene_payload(
            snapshot,
            obstacles,
            clearance_m=clearance_m,
            model_type=self._current_model_type,
            trail_state=self._trail_payload_state,
        )
        self.bridge.set_scene_payload(payload)
        self._schedule_terrain_refresh(payload)

    def _schedule_terrain_refresh(self, payload: dict) -> None:
        """高度场未就绪时定时重推快照。注意：READY 态主窗口 100ms 定时器未启动,
        没有这条重推链路的话,后台生成完成的山地永远替换不进 3D 场景(显示旧占位小图)。"""

        surface = payload.get("terrain", {}).get("surface", {}) if isinstance(payload, dict) else {}
        pending = surface.get("mode") == "layout" and not surface.get("fieldReady", True)
        if not pending or self._terrain_refresh_timer.isActive():
            return
        self._terrain_refresh_timer.start()

    def _on_terrain_refresh(self) -> None:
        """定时器回调:用缓存快照重推 payload,就绪后自然停止(fieldReady 翻转)。"""

        if self._cached_scene is None:
            return
        snapshot, obstacles, clearance_m = self._cached_scene
        self._push_scene_payload(snapshot, obstacles, clearance_m)

    def _show_fallback(self) -> None:
        """显示 QML 加载失败兜底信息。注意：避免窗口空白导致误判为正常。"""

        messages = ["Qt Quick 3D 场景加载失败："]
        messages.extend(error.toString() for error in self.quick_view.errors())
        self._fallback_label = QLabel("\n".join(messages), self)
        self._fallback_label.setWordWrap(True)
        self.root_layout.addWidget(self._fallback_label)
