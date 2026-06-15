"""PySide6 main window for the formation simulation UI."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from PySide6.QtCore import QPoint, QPointF, QRectF, Qt, QTimer, Signal
from PySide6.QtGui import QAction, QColor, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import (
    QAbstractItemView,
    QAbstractScrollArea,
    QApplication,
    QCheckBox,
    QDialog,
    QFileDialog,
    QFormLayout,
    QFrame,
    QGraphicsView,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMenu,
    QProgressBar,
    QPushButton,
    QHeaderView,
    QSlider,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


WORLD_WIDTH = 1600.0
WORLD_HEIGHT = 520.0
TRAIL_SECONDS = 18.0


@dataclass
class TrailPoint:
    """One sampled position in simulation time."""

    x: float
    y: float
    altitude: float
    time: float


@dataclass
class NodeState:
    """Display state for one aircraft node."""

    node_id: str
    role: str
    x: float
    y: float
    vx: float
    vy: float
    trail: list[TrailPoint] = field(default_factory=list)


@dataclass
class LinkState:
    """Display state for one communication link."""

    source: str
    target: str
    latency_ms: int
    loss: float
    ok: bool = True


@dataclass
class Snapshot:
    """UI-facing simulation snapshot."""

    time: float
    duration: float
    step: float
    run_state: str
    control_report: str
    disturbance: str
    nodes: list[NodeState]
    links: list[LinkState]


class MockSimulation:
    """Small UI-only simulation source until the real controller is connected."""

    def __init__(self) -> None:
        self.duration = 120.0
        self.step = 0.1
        self.speed = 1.0
        self.time = 0.0
        self.running = False
        self.paused = False
        self.disturbance = "无"
        self.disturbance_until = 0.0
        self.fault_node: str | None = None
        self.loss_until = 0.0
        self.nodes: list[NodeState] = []
        self.links: list[LinkState] = []
        self.reset()

    def reset(self) -> Snapshot:
        self.time = 0.0
        self.running = False
        self.paused = False
        self.disturbance = "无"
        self.disturbance_until = 0.0
        self.fault_node = None
        self.loss_until = 0.0
        self.nodes = [
            NodeState("A01", "leader", 140.0, 260.0, 5.2, -0.1),
            NodeState("A02", "wing", 92.0, 318.0, 5.0, 0.0),
            NodeState("A03", "wing", 88.0, 202.0, 5.0, 0.0),
        ]
        self.links = [
            LinkState("A01", "A02", 18, 0.01),
            LinkState("A01", "A03", 21, 0.01),
            LinkState("A02", "A03", 30, 0.02),
        ]
        return self.snapshot()

    def start(self) -> Snapshot:
        self.running = True
        self.paused = False
        return self.snapshot()

    def pause(self) -> Snapshot:
        if self.running:
            self.paused = not self.paused
        return self.snapshot()

    def single_step(self) -> Snapshot:
        self.running = True
        self.paused = True
        self.advance()
        return self.snapshot()

    def inject_disturbance(self, kind: str) -> Snapshot:
        if kind == "wind":
            self.disturbance = "风场"
            self.disturbance_until = self.time + 8.0
        elif kind == "fault":
            self.disturbance = "节点故障"
            self.fault_node = "A02"
            self.disturbance_until = self.time + 10.0
        elif kind == "loss":
            self.disturbance = "链路丢包"
            self.loss_until = self.time + 12.0
            self.disturbance_until = self.time + 12.0
        elif kind == "clear":
            self.disturbance = "无"
            self.disturbance_until = 0.0
            self.loss_until = 0.0
            self.fault_node = None
        return self.snapshot()

    def advance(self) -> Snapshot:
        if self.time >= self.duration:
            self.running = False
            self.paused = False
            return self.snapshot()

        self.time = min(self.duration, self.time + self.step * self.speed)
        wind = 1.8 if self.disturbance == "风场" else 0.0
        formation = [(0.0, 0.0), (-54.0, 58.0), (-54.0, -58.0)]
        leader = self.nodes[0]

        for index, node in enumerate(self.nodes):
            _, dy = formation[index]
            target_y = 238.0 + math.sin(self.time / 8.0) * 34.0 if index == 0 else leader.y + dy
            gain = 0.012 if self.fault_node == node.node_id else 0.04
            node.vx = 4.8 + index * 0.12
            node.vy += (target_y - node.y) * gain + wind * math.sin(self.time + index)
            node.x += node.vx * self.step * self.speed * 3.2
            node.y += node.vy * self.step * self.speed
            if node.x > WORLD_WIDTH + 60.0:
                node.x = -30.0
                node.trail.clear()
            node.trail.append(TrailPoint(node.x, node.y, node_altitude(index, self.time), self.time))
            node.trail = [point for point in node.trail if self.time - point.time <= TRAIL_SECONDS]

        if self.disturbance != "无" and self.time > self.disturbance_until:
            self.disturbance = "无"
            self.fault_node = None

        for index, link in enumerate(self.links):
            degraded = self.time < self.loss_until and index != 2
            link.loss = 0.26 + index * 0.05 if degraded else 0.01 + index * 0.006
            link.latency_ms = 76 + index * 8 if degraded else 18 + index * 5 + round(math.sin(self.time + index) * 3)
            link.ok = link.loss < 0.2

        return self.snapshot()

    def snapshot(self) -> Snapshot:
        if not self.running:
            run_state = "READY"
            report = "待命"
        elif self.paused:
            run_state = "PAUSED"
            report = "保持"
        elif self.disturbance == "风场":
            run_state = "RUNNING"
            report = "抗风"
        elif self.disturbance == "节点故障":
            run_state = "RUNNING"
            report = "重构"
        elif self.disturbance == "链路丢包":
            run_state = "RUNNING"
            report = "保链"
        else:
            run_state = "RUNNING"
            report = "集结"
        return Snapshot(
            time=self.time,
            duration=self.duration,
            step=self.step,
            run_state=run_state,
            control_report=report,
            disturbance=self.disturbance,
            nodes=self.nodes,
            links=self.links,
        )


def node_altitude(index: int, time_value: float) -> float:
    """Return a demo altitude for side-view rendering."""

    return 1200.0 + index * 35.0 + math.sin(time_value / 6.0 + index) * 12.0


class Theme:
    """Centralized colors for one UI theme."""

    def __init__(
        self,
        *,
        bg: str,
        panel: str,
        ink: str,
        muted: str,
        line: str,
        canvas: str,
        grid: str,
        route: str,
        leader: str,
        wingman: str,
        link: str,
        warn: str,
        accent: str,
        field: str,
    ) -> None:
        self.bg = QColor(bg)
        self.panel = QColor(panel)
        self.ink = QColor(ink)
        self.muted = QColor(muted)
        self.line = QColor(line)
        self.canvas = QColor(canvas)
        self.grid = QColor(grid)
        self.route = QColor(route)
        self.leader = QColor(leader)
        self.wingman = QColor(wingman)
        self.link = QColor(link)
        self.warn = QColor(warn)
        self.accent = QColor(accent)
        self.field = QColor(field)


THEMES = {
    "light": Theme(
        bg="#eaf2f8",
        panel="#edf6fd",
        ink="#17202a",
        muted="#667085",
        line="#cfdae6",
        canvas="#e2edf6",
        grid="#c5d4e2",
        route="#94a3b8",
        leader="#2563eb",
        wingman="#7c3aed",
        link="#0891b2",
        warn="#b45309",
        accent="#0f766e",
        field="#f4f9fe",
    ),
    "dark": Theme(
        bg="#0e141b",
        panel="#151d26",
        ink="#e7edf4",
        muted="#94a3b8",
        line="#2a3644",
        canvas="#101923",
        grid="#243141",
        route="#64748b",
        leader="#60a5fa",
        wingman="#c084fc",
        link="#22d3ee",
        warn="#f59e0b",
        accent="#14b8a6",
        field="#0f1720",
    ),
}


class SelectButton(QPushButton):
    """Push-button backed option selector with a controlled popup position."""

    currentIndexChanged = Signal()

    def __init__(self, min_width: int, popup_side: str = "below", parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._items: list[tuple[str, object | None]] = []
        self._index = -1
        self._menu = QMenu(self)
        self._popup_side = popup_side
        self.setObjectName("selectButton")
        self.setMinimumWidth(min_width)
        self.clicked.connect(self.show_menu)
        self._menu.aboutToHide.connect(lambda: self.setDown(False))

    def addItem(self, text: str, data: object | None = None) -> None:
        self._items.append((text, data))
        if self._index == -1:
            self.setCurrentIndex(0, emit=False)

    def addItems(self, texts: list[str]) -> None:
        for text in texts:
            self.addItem(text, text)

    def setCurrentIndex(self, index: int, *, emit: bool = True) -> None:
        if index < 0 or index >= len(self._items):
            return
        if index == self._index:
            return
        self._index = index
        self.setText(f"{self._items[index][0]}  ▾")
        if emit:
            self.currentIndexChanged.emit()

    def currentText(self) -> str:
        if self._index < 0:
            return ""
        return self._items[self._index][0]

    def currentData(self) -> object | None:
        if self._index < 0:
            return None
        return self._items[self._index][1]

    def show_menu(self) -> None:
        self.setDown(True)
        self._menu.clear()
        self._menu.setMinimumWidth(self.width())
        for index, (text, _) in enumerate(self._items):
            action = QAction(text, self._menu)
            action.setCheckable(True)
            action.setChecked(index == self._index)
            action.triggered.connect(lambda checked=False, row=index: self.setCurrentIndex(row))
            self._menu.addAction(action)
        if self._popup_side == "right":
            point = QPoint(self.width() + 34, 0)
        else:
            point = QPoint(0, self.height() + 2)
        self._menu.popup(self.mapToGlobal(point))


class TopView(QGraphicsView):
    """Top-down formation view with pan and zoom."""

    viewChanged = Signal()
    manualViewChanged = Signal()
    resetViewRequested = Signal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.snapshot: Snapshot | None = None
        self.theme = THEMES["light"]
        self.scale_value = 1.0
        self.offset = QPointF(0.0, 0.0)
        self.auto_center = False
        self.show_grid = True
        self._pan_origin: QPointF | None = None
        self._selection_origin: QPointF | None = None
        self._selection_current: QPointF | None = None
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setFrameShape(QFrame.Shape.NoFrame)
        self.setMinimumHeight(360)
        self.setMouseTracking(True)

    def set_theme(self, theme: Theme) -> None:
        self.theme = theme
        self.viewport().update()

    def set_snapshot(self, snapshot: Snapshot) -> None:
        self.snapshot = snapshot
        if self.auto_center:
            self._apply_auto_center()
        self.viewport().update()

    def reset_view(self) -> None:
        self.scale_value = 1.0
        self.offset = QPointF(0.0, 0.0)
        self.viewport().update()
        self.viewChanged.emit()
        self.manualViewChanged.emit()
        self.resetViewRequested.emit()

    def wheelEvent(self, event) -> None:  # noqa: ANN001
        delta = event.pixelDelta().y() or event.angleDelta().y()
        if delta == 0:
            return
        cursor = event.position()
        before = QPointF(
            (cursor.x() - self.offset.x()) / self.scale_value,
            (cursor.y() - self.offset.y()) / self.scale_value,
        )
        factor = math.pow(1.001, delta)
        self.scale_value = min(3.5, max(0.45, self.scale_value * factor))
        self.offset = QPointF(
            cursor.x() - before.x() * self.scale_value,
            cursor.y() - before.y() * self.scale_value,
        )
        self.viewport().update()
        self.viewChanged.emit()
        self.manualViewChanged.emit()
        event.accept()

    def mousePressEvent(self, event) -> None:  # noqa: ANN001
        if event.button() == Qt.MouseButton.MiddleButton:
            self._pan_origin = event.position()
            self.setCursor(Qt.CursorShape.ClosedHandCursor)
            event.accept()
        elif event.button() == Qt.MouseButton.LeftButton:
            self._selection_origin = event.position()
            self._selection_current = event.position()
            self.viewport().update()
            event.accept()

    def mouseMoveEvent(self, event) -> None:  # noqa: ANN001
        if self._pan_origin is not None:
            delta = event.position() - self._pan_origin
            self.offset += QPointF(delta.x(), delta.y())
            self._pan_origin = event.position()
            self.viewport().update()
            self.viewChanged.emit()
            self.manualViewChanged.emit()
            event.accept()
        elif self._selection_origin is not None:
            self._selection_current = event.position()
            self.viewport().update()
            event.accept()

    def mouseReleaseEvent(self, event) -> None:  # noqa: ANN001
        if event.button() == Qt.MouseButton.MiddleButton:
            self._pan_origin = None
            self.setCursor(Qt.CursorShape.ArrowCursor)
            event.accept()
        elif event.button() == Qt.MouseButton.LeftButton:
            self._zoom_to_selection()
            self._selection_origin = None
            self._selection_current = None
            self.viewport().update()
            event.accept()

    def mouseDoubleClickEvent(self, event) -> None:  # noqa: ANN001
        if event.button() == Qt.MouseButton.LeftButton:
            self.reset_view()
            event.accept()

    def paintEvent(self, event) -> None:  # noqa: ARG002, ANN001
        painter = QPainter(self.viewport())
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), self.theme.canvas)
        painter.translate(self.offset)
        painter.scale(self.scale_value, self.scale_value)
        if self.show_grid:
            self._draw_grid(painter)
        self._draw_route(painter)
        if self.snapshot:
            self._draw_links(painter, self.snapshot)
            self._draw_nodes(painter, self.snapshot)
        painter.resetTransform()
        self._draw_selection(painter)

    def _viewport_to_world(self, point: QPointF) -> QPointF:
        return QPointF(
            (point.x() - self.offset.x()) / self.scale_value,
            (point.y() - self.offset.y()) / self.scale_value,
        )

    def _zoom_to_selection(self) -> None:
        if self._selection_origin is None or self._selection_current is None:
            return
        left = min(self._selection_origin.x(), self._selection_current.x())
        right = max(self._selection_origin.x(), self._selection_current.x())
        top = min(self._selection_origin.y(), self._selection_current.y())
        bottom = max(self._selection_origin.y(), self._selection_current.y())
        if right - left < 8 or bottom - top < 8:
            return

        world_start = self._viewport_to_world(QPointF(left, top))
        world_end = self._viewport_to_world(QPointF(right, bottom))
        world_width = max(1.0, abs(world_end.x() - world_start.x()))
        world_height = max(1.0, abs(world_end.y() - world_start.y()))
        viewport = self.viewport().rect()
        margin = 0.94
        scale = min(viewport.width() / world_width, viewport.height() / world_height) * margin
        self.scale_value = min(3.5, max(0.45, scale))

        center_x = (world_start.x() + world_end.x()) / 2.0
        center_y = (world_start.y() + world_end.y()) / 2.0
        self.offset = QPointF(
            viewport.width() / 2.0 - center_x * self.scale_value,
            viewport.height() / 2.0 - center_y * self.scale_value,
        )
        self.viewChanged.emit()
        self.manualViewChanged.emit()

    def _draw_selection(self, painter: QPainter) -> None:
        if self._selection_origin is None or self._selection_current is None:
            return
        left = min(self._selection_origin.x(), self._selection_current.x())
        right = max(self._selection_origin.x(), self._selection_current.x())
        top = min(self._selection_origin.y(), self._selection_current.y())
        bottom = max(self._selection_origin.y(), self._selection_current.y())
        if right - left < 2 or bottom - top < 2:
            return
        selection = QRectF(left, top, right - left, bottom - top)
        pen = QPen(self.theme.accent, 1.4)
        pen.setDashPattern([5, 4])
        painter.setPen(pen)
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawRect(selection)

    def _apply_auto_center(self) -> None:
        if not self.snapshot or not self.snapshot.nodes:
            return
        active = [node for node in self.snapshot.nodes if self.snapshot.disturbance != "节点故障" or node.node_id != "A02"]
        if not active:
            active = self.snapshot.nodes
        center_x = sum(node.x for node in active) / len(active)
        center_y = sum(node.y for node in active) / len(active)
        rect = self.viewport().rect()
        self.offset = QPointF(
            rect.width() / 2.0 - center_x * self.scale_value,
            rect.height() / 2.0 - center_y * self.scale_value,
        )
        self.viewChanged.emit()

    def _draw_grid(self, painter: QPainter) -> None:
        rect = self.viewport().rect()
        left = (rect.left() - self.offset.x()) / self.scale_value
        right = (rect.right() - self.offset.x()) / self.scale_value
        top = (rect.top() - self.offset.y()) / self.scale_value
        bottom = (rect.bottom() - self.offset.y()) / self.scale_value
        spacing = 48
        start_x = math.floor(left / spacing) * spacing
        end_x = math.ceil(right / spacing) * spacing
        start_y = math.floor(top / spacing) * spacing
        end_y = math.ceil(bottom / spacing) * spacing

        painter.setPen(QPen(self.theme.grid, 1.0 / self.scale_value))
        for x in range(start_x, end_x + spacing, spacing):
            painter.drawLine(x, start_y, x, end_y)
        for y in range(start_y, end_y + spacing, spacing):
            painter.drawLine(start_x, y, end_x, y)

    def _draw_route(self, painter: QPainter) -> None:
        pen = QPen(self.theme.route, 2)
        pen.setDashPattern([8, 7])
        painter.setPen(pen)
        path = QPainterPath(QPointF(40, WORLD_HEIGHT / 2))
        path.cubicTo(WORLD_WIDTH * 0.35, WORLD_HEIGHT * 0.2, WORLD_WIDTH * 0.66, WORLD_HEIGHT * 0.8, WORLD_WIDTH - 40, WORLD_HEIGHT * 0.46)
        painter.drawPath(path)
        painter.setBrush(self.theme.ink)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(QPointF(WORLD_WIDTH - 42, WORLD_HEIGHT * 0.46), 5, 5)

    def _draw_links(self, painter: QPainter, snapshot: Snapshot) -> None:
        by_id = {node.node_id: node for node in snapshot.nodes}
        for link in snapshot.links:
            source = by_id[link.source]
            target = by_id[link.target]
            color = QColor(self.theme.link if link.ok else self.theme.warn)
            color.setAlphaF(0.58 if link.ok else 0.75)
            painter.setPen(QPen(color, 2 if link.ok else 3))
            painter.drawLine(QPointF(source.x, source.y), QPointF(target.x, target.y))

    def _draw_nodes(self, painter: QPainter, snapshot: Snapshot) -> None:
        for index, node in enumerate(snapshot.nodes):
            self._draw_trail(painter, node, index, snapshot.time)
            color = self.theme.warn if snapshot.disturbance == "节点故障" and node.node_id == "A02" else self.theme.leader if index == 0 else self.theme.wingman
            painter.save()
            painter.translate(node.x, node.y)
            painter.rotate(math.degrees(math.atan2(node.vy, node.vx)))
            painter.setBrush(color)
            painter.setPen(QPen(self.theme.panel, 2))
            path = QPainterPath(QPointF(16, 0))
            path.lineTo(-12, -9)
            path.lineTo(-6, 0)
            path.lineTo(-12, 9)
            path.closeSubpath()
            painter.drawPath(path)
            painter.restore()
            painter.setPen(QPen(self.theme.ink, 1))
            painter.drawText(QPointF(node.x - 13, node.y - 18), node.node_id)

    def _draw_trail(self, painter: QPainter, node: NodeState, index: int, current_time: float) -> None:
        if len(node.trail) <= 2:
            return
        base = self.theme.leader if index == 0 else self.theme.wingman
        for previous, current in zip(node.trail, node.trail[1:]):
            age = max(0.0, current_time - current.time)
            alpha = max(0.08, 1.0 - age / TRAIL_SECONDS)
            color = QColor(base)
            color.setAlphaF((0.52 if index == 0 else 0.44) * alpha)
            painter.setPen(QPen(color, 2))
            painter.drawLine(QPointF(previous.x, previous.y), QPointF(current.x, current.y))


class SideView(QWidget):
    """Altitude over distance side view."""

    def __init__(self, top_view: TopView, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.top_view = top_view
        self.snapshot: Snapshot | None = None
        self.theme = THEMES["light"]
        self.show_grid = True
        self.altitude_min = 1120.0
        self.altitude_max = 1320.0
        self._pan_origin: QPointF | None = None
        self._selection_origin: QPointF | None = None
        self._selection_current: QPointF | None = None
        self.setMinimumHeight(150)
        self.setMouseTracking(True)

    def set_theme(self, theme: Theme) -> None:
        self.theme = theme
        self.update()

    def set_snapshot(self, snapshot: Snapshot) -> None:
        self.snapshot = snapshot
        self.update()

    def paintEvent(self, event) -> None:  # noqa: ARG002, ANN001
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), self.theme.canvas)
        if self.show_grid:
            self._draw_grid(painter)
        self._draw_reference(painter)
        if self.snapshot:
            self._draw_trails(painter, self.snapshot)
            self._draw_nodes(painter, self.snapshot)
        painter.setPen(self.theme.muted)
        painter.drawText(QPointF(self.width() - 76, self.height() - 8), "待飞距")
        painter.drawText(QPointF(12, 20), "高度")
        self._draw_selection(painter)

    def wheelEvent(self, event) -> None:  # noqa: ANN001
        delta = event.pixelDelta().y() or event.angleDelta().y()
        if delta == 0:
            return
        before_x = self._screen_to_world_x(event.position().x())
        old_scale = self.top_view.scale_value
        factor = math.pow(1.001, delta)
        self.top_view.scale_value = min(3.5, max(0.45, old_scale * factor))
        self.top_view.offset.setX(event.position().x() - before_x * self.top_view.scale_value)
        self._preserve_top_view_vertical_center(old_scale)
        self._emit_shared_view_changed()
        event.accept()

    def mousePressEvent(self, event) -> None:  # noqa: ANN001
        if event.button() == Qt.MouseButton.MiddleButton:
            self._pan_origin = event.position()
            self.setCursor(Qt.CursorShape.ClosedHandCursor)
            event.accept()
        elif event.button() == Qt.MouseButton.LeftButton:
            self._selection_origin = event.position()
            self._selection_current = event.position()
            self.update()
            event.accept()

    def mouseMoveEvent(self, event) -> None:  # noqa: ANN001
        if self._pan_origin is not None:
            delta = event.position() - self._pan_origin
            self.top_view.offset.setX(self.top_view.offset.x() + delta.x())
            self._pan_altitude(delta.y())
            self._pan_origin = event.position()
            self._emit_shared_view_changed()
            event.accept()
        elif self._selection_origin is not None:
            self._selection_current = event.position()
            self.update()
            event.accept()

    def mouseReleaseEvent(self, event) -> None:  # noqa: ANN001
        if event.button() == Qt.MouseButton.MiddleButton:
            self._pan_origin = None
            self.setCursor(Qt.CursorShape.ArrowCursor)
            event.accept()
        elif event.button() == Qt.MouseButton.LeftButton:
            self._zoom_to_selection()
            self._selection_origin = None
            self._selection_current = None
            self.update()
            event.accept()

    def mouseDoubleClickEvent(self, event) -> None:  # noqa: ANN001
        if event.button() == Qt.MouseButton.LeftButton:
            self.reset_altitude_view()
            self.top_view.reset_view()
            event.accept()

    def reset_altitude_view(self) -> None:
        self.altitude_min = 1120.0
        self.altitude_max = 1320.0
        self.update()

    def _map_x(self, x: float) -> float:
        return x * self.top_view.scale_value + self.top_view.offset.x()

    def _screen_to_world_x(self, x: float) -> float:
        return (x - self.top_view.offset.x()) / self.top_view.scale_value

    def _screen_to_altitude(self, y: float) -> float:
        plot_height = max(1.0, self.height() - 52)
        ratio = (self.height() - 24 - y) / plot_height
        return self.altitude_min + ratio * (self.altitude_max - self.altitude_min)

    def _pan_altitude(self, delta_y: float) -> None:
        altitude_delta = delta_y / max(1.0, self.height() - 52) * (self.altitude_max - self.altitude_min)
        self.altitude_min += altitude_delta
        self.altitude_max += altitude_delta

    def _preserve_top_view_vertical_center(self, old_scale: float) -> None:
        viewport = self.top_view.viewport().rect()
        center_y = (viewport.height() / 2.0 - self.top_view.offset.y()) / old_scale
        self.top_view.offset.setY(viewport.height() / 2.0 - center_y * self.top_view.scale_value)

    def _emit_shared_view_changed(self) -> None:
        self.top_view.viewport().update()
        self.top_view.viewChanged.emit()
        self.top_view.manualViewChanged.emit()

    def _zoom_to_selection(self) -> None:
        if self._selection_origin is None or self._selection_current is None:
            return
        left = min(self._selection_origin.x(), self._selection_current.x())
        right = max(self._selection_origin.x(), self._selection_current.x())
        top = min(self._selection_origin.y(), self._selection_current.y())
        bottom = max(self._selection_origin.y(), self._selection_current.y())
        selection_width = right - left
        selection_height = bottom - top
        has_width = selection_width >= 80 and selection_width >= selection_height * 1.25
        has_height = selection_height >= 8
        if not has_width and not has_height:
            return

        if has_width:
            start_x = self._screen_to_world_x(left)
            end_x = self._screen_to_world_x(right)
            world_width = max(1.0, abs(end_x - start_x))
            old_scale = self.top_view.scale_value
            self.top_view.scale_value = min(3.5, max(0.45, self.width() / world_width * 0.94))
            center_x = (start_x + end_x) / 2.0
            self.top_view.offset.setX(self.width() / 2.0 - center_x * self.top_view.scale_value)
            self._preserve_top_view_vertical_center(old_scale)

        if has_height:
            altitude_top = self._screen_to_altitude(top)
            altitude_bottom = self._screen_to_altitude(bottom)
            center = (altitude_top + altitude_bottom) / 2.0
            span = max(8.0, abs(altitude_top - altitude_bottom) / 0.94)
            self.altitude_min = center - span / 2.0
            self.altitude_max = center + span / 2.0

        self._emit_shared_view_changed()

    def _draw_selection(self, painter: QPainter) -> None:
        if self._selection_origin is None or self._selection_current is None:
            return
        left = min(self._selection_origin.x(), self._selection_current.x())
        right = max(self._selection_origin.x(), self._selection_current.x())
        top = min(self._selection_origin.y(), self._selection_current.y())
        bottom = max(self._selection_origin.y(), self._selection_current.y())
        if right - left < 2 or bottom - top < 2:
            return
        selection = QRectF(left, top, right - left, bottom - top)
        pen = QPen(self.theme.accent, 1.4)
        pen.setDashPattern([5, 4])
        painter.setPen(pen)
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawRect(selection)

    def _map_y(self, altitude: float) -> float:
        return self.height() - 24 - ((altitude - self.altitude_min) / (self.altitude_max - self.altitude_min)) * (self.height() - 52)

    def _draw_grid(self, painter: QPainter) -> None:
        painter.setPen(QPen(self.theme.grid, 1))
        spacing = 48
        left = self._screen_to_world_x(0.0)
        right = self._screen_to_world_x(float(self.width()))
        start_x = math.floor(left / spacing) * spacing
        end_x = math.ceil(right / spacing) * spacing
        for world_x in range(start_x, end_x + spacing, spacing):
            x = self._map_x(float(world_x))
            painter.drawLine(QPointF(x, 0.0), QPointF(x, float(self.height())))

        altitude_spacing = 40
        start_altitude = math.floor(self.altitude_min / altitude_spacing) * altitude_spacing
        end_altitude = math.ceil(self.altitude_max / altitude_spacing) * altitude_spacing
        for altitude in range(start_altitude, end_altitude + altitude_spacing, altitude_spacing):
            y = self._map_y(float(altitude))
            painter.drawLine(QPointF(0.0, y), QPointF(float(self.width()), y))

    def _draw_reference(self, painter: QPainter) -> None:
        pen = QPen(self.theme.route, 2)
        pen.setDashPattern([7, 6])
        painter.setPen(pen)
        start_x = max(-40.0, self._map_x(0.0))
        end_x = min(self.width() + 40.0, self._map_x(WORLD_WIDTH))
        painter.drawLine(QPointF(start_x, self._map_y(1200.0)), QPointF(end_x, self._map_y(1260.0)))

    def _draw_trails(self, painter: QPainter, snapshot: Snapshot) -> None:
        for index, node in enumerate(snapshot.nodes):
            if len(node.trail) <= 2:
                continue
            base = self.theme.leader if index == 0 else self.theme.wingman
            for previous, current in zip(node.trail, node.trail[1:]):
                x1 = self._map_x(previous.x)
                x2 = self._map_x(current.x)
                if (x1 < -24 and x2 < -24) or (x1 > self.width() + 24 and x2 > self.width() + 24):
                    continue
                age = max(0.0, snapshot.time - current.time)
                alpha = max(0.08, 1.0 - age / TRAIL_SECONDS)
                color = QColor(base)
                color.setAlphaF((0.48 if index == 0 else 0.40) * alpha)
                painter.setPen(QPen(color, 2))
                painter.drawLine(QPointF(x1, self._map_y(previous.altitude)), QPointF(x2, self._map_y(current.altitude)))

    def _draw_nodes(self, painter: QPainter, snapshot: Snapshot) -> None:
        for index, node in enumerate(snapshot.nodes):
            x = self._map_x(node.x)
            if x < -24 or x > self.width() + 24:
                continue
            color = self.theme.warn if snapshot.disturbance == "节点故障" and node.node_id == "A02" else self.theme.leader if index == 0 else self.theme.wingman
            y = self._map_y(node_altitude(index, snapshot.time))
            painter.setBrush(color)
            painter.setPen(QPen(self.theme.panel, 2))
            painter.drawEllipse(QPointF(x, y), 8, 8)
            painter.setPen(self.theme.ink)
            painter.drawText(QPointF(x + 10, y + 4), node.node_id)


class LogDialog(QDialog):
    """Popup dialog for simulation events."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("日志")
        self.resize(720, 360)
        layout = QVBoxLayout(self)
        self.text = QTextEdit()
        self.text.setReadOnly(True)
        clear_button = QPushButton("清空")
        clear_button.clicked.connect(self.text.clear)
        layout.addWidget(self.text)
        layout.addWidget(clear_button, alignment=Qt.AlignmentFlag.AlignRight)

    def append(self, time_value: float, source: str, message: str) -> None:
        self.text.append(f"{time_value:05.1f}s  {source:<10} {message}")


class StageFullscreenDialog(QDialog):
    """Top-level shell used to fullscreen only the realtime display stage."""

    def __init__(self, owner: "MainWindow") -> None:
        super().__init__(owner)
        self.owner = owner
        self.setWindowTitle("二维实时显示")
        self.setWindowFlag(Qt.WindowType.FramelessWindowHint, True)

    def keyPressEvent(self, event) -> None:  # type: ignore[override]
        if event.key() == Qt.Key.Key_Escape:
            self.owner._exit_stage_fullscreen()
            return
        super().keyPressEvent(event)

    def closeEvent(self, event) -> None:  # type: ignore[override]
        if self.owner._stage_fullscreen_dialog is self:
            self.owner._exit_stage_fullscreen()
        event.accept()


class MainWindow(QMainWindow):
    """Main PySide6 UI shell."""

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("编队仿真")
        self.resize(1440, 900)
        self.setMinimumSize(1280, 780)
        self.sim = MockSimulation()
        self.theme_key = "light"
        self.theme = THEMES[self.theme_key]
        self.timer = QTimer(self)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self._on_tick)
        self.log_dialog = LogDialog(self)
        self.main_layout: QHBoxLayout | None = None
        self.stage: QWidget | None = None
        self.fullscreen_button: QPushButton | None = None
        self._stage_placeholder: QWidget | None = None
        self._stage_fullscreen_dialog: StageFullscreenDialog | None = None
        self._stage_layout_index = 1
        self._stage_layout_stretch = 1
        self._build_ui()
        self._install_button_cursors()
        self._apply_theme()
        self._update_snapshot(self.sim.snapshot())
        self._log("SimControl", "初始化场景，等待 start 命令")

    def _build_ui(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)
        outer.addWidget(self._build_header())

        main = QHBoxLayout()
        self.main_layout = main
        main.setContentsMargins(8, 8, 8, 8)
        main.setSpacing(8)
        outer.addLayout(main, 1)
        main.addWidget(self._build_left_panel(), 0)
        self.stage = self._build_stage()
        main.addWidget(self.stage, 1)
        main.addWidget(self._build_right_panel(), 0)

    def _build_header(self) -> QWidget:
        header = QFrame()
        header.setFixedHeight(42)
        layout = QHBoxLayout(header)
        layout.setContentsMargins(14, 6, 14, 6)
        layout.setSpacing(10)
        title = QLabel("编队仿真")
        title.setObjectName("title")
        title.setAlignment(Qt.AlignmentFlag.AlignVCenter)
        layout.addWidget(title)
        layout.addStretch(1)
        self.scenario_label = QLabel("场景：三机楔形队形")
        self.step_label = QLabel("步长：0.1s")
        self.run_state_label = QLabel("READY")
        self.run_state_label.setObjectName("statusPill")
        self.report_label = QLabel("回报：待命")
        self.report_label.setObjectName("reportPill")
        self.theme_select = SelectButton(126)
        self.theme_select.addItem("浅色模式", "light")
        self.theme_select.addItem("深色模式", "dark")
        self.theme_select.currentIndexChanged.connect(self._on_theme_changed)
        log_button = QPushButton("日志")
        log_button.clicked.connect(self.log_dialog.show)
        for widget in [self.scenario_label, self.step_label, self.run_state_label, self.report_label, self.theme_select, log_button]:
            layout.addWidget(widget)
        return header

    def _build_left_panel(self) -> QWidget:
        panel = QFrame()
        panel.setObjectName("panel")
        panel.setFixedWidth(216)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(10, 12, 10, 10)
        layout.setSpacing(10)
        config_group = QGroupBox("配置")
        form = QFormLayout(config_group)
        form.setContentsMargins(10, 18, 10, 10)
        form.setHorizontalSpacing(8)
        form.setVerticalSpacing(8)
        self.config_name = QLabel("未选择")
        choose_config = QPushButton("选择文件")
        choose_config.clicked.connect(self._choose_config)
        self.scenario_select = SelectButton(132, popup_side="right")
        self.scenario_select.addItems(["三机楔形", "五机纵队", "受限重构"])
        self.algorithm_select = SelectButton(128, popup_side="right")
        self.algorithm_select.addItems(["Follow", "Consensus", "RuleBased"])
        self.duration_select = SelectButton(96, popup_side="right")
        self.duration_select.addItems(["120", "180", "300"])
        form.addRow("配置", choose_config)
        form.addRow("", self.config_name)
        form.addRow("场景", self.scenario_select)
        form.addRow("算法", self.algorithm_select)
        form.addRow("时长(s)", self.duration_select)
        layout.addWidget(config_group)

        playback_group = QGroupBox("播放")
        playback_layout = QVBoxLayout(playback_group)
        playback_layout.setContentsMargins(10, 18, 10, 10)
        self.speed_label = QLabel("1.0x")
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(1, 100)
        self.speed_slider.setValue(10)
        self.speed_slider.valueChanged.connect(self._on_speed_changed)
        playback_layout.addWidget(self.speed_slider)
        playback_layout.addWidget(self.speed_label, alignment=Qt.AlignmentFlag.AlignRight)
        layout.addWidget(playback_group)

        disturb_group = QGroupBox("运行期扰动")
        grid = QGridLayout(disturb_group)
        grid.setContentsMargins(10, 18, 10, 10)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(8)
        actions: list[tuple[str, str]] = [
            ("风场脉冲", "wind"),
            ("节点故障", "fault"),
            ("链路丢包", "loss"),
            ("清除扰动", "clear"),
        ]
        for index, (text, kind) in enumerate(actions):
            button = QPushButton(text)
            button.clicked.connect(lambda checked=False, value=kind: self._inject_disturbance(value))
            grid.addWidget(button, index // 2, index % 2)
        layout.addWidget(disturb_group)
        layout.addStretch(1)
        return panel

    def _build_stage(self) -> QWidget:
        stage = QFrame()
        stage.setObjectName("panel")
        layout = QVBoxLayout(stage)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        toolbar = QHBoxLayout()
        toolbar.setContentsMargins(12, 10, 12, 10)
        toolbar.setSpacing(8)
        title = QLabel("二维实时显示")
        title.setObjectName("stageTitle")
        fullscreen = QPushButton("⛶")
        fullscreen.setFixedSize(30, 30)
        fullscreen.clicked.connect(self._toggle_fullscreen)
        self.fullscreen_button = fullscreen
        toolbar.addWidget(title)
        toolbar.addWidget(fullscreen)
        toolbar.addStretch(1)
        self.legend_leader = QLabel("● 长机")
        self.legend_leader.setObjectName("legendLeader")
        self.legend_wingman = QLabel("● 僚机")
        self.legend_wingman.setObjectName("legendWingman")
        self.legend_link = QLabel("● 通信链路")
        self.legend_link.setObjectName("legendLink")
        self.legend_warn = QLabel("● 异常状态")
        self.legend_warn.setObjectName("legendWarn")
        for label in [self.legend_leader, self.legend_wingman, self.legend_link, self.legend_warn]:
            label.setContentsMargins(0, 0, 2, 0)
            toolbar.addWidget(label)
        self.grid_toggle = QCheckBox("网格")
        self.grid_toggle.setChecked(True)
        self.grid_toggle.stateChanged.connect(self._on_grid_changed)
        self.auto_center = QCheckBox("自动居中")
        self.auto_center.stateChanged.connect(self._on_auto_center_changed)
        reset_view = QPushButton("重置视图")
        reset_view.clicked.connect(self._reset_view)
        toolbar.addWidget(self.grid_toggle)
        toolbar.addWidget(self.auto_center)
        toolbar.addWidget(reset_view)
        layout.addLayout(toolbar)

        self.top_view = TopView()
        self.side_view = SideView(self.top_view)
        self.top_view.viewChanged.connect(self.side_view.update)
        self.top_view.manualViewChanged.connect(self._disable_auto_center)
        self.top_view.resetViewRequested.connect(self.side_view.reset_altitude_view)
        layout.addWidget(self.top_view, 1)
        layout.addWidget(self.side_view, 0)

        timeline = QHBoxLayout()
        timeline.setContentsMargins(12, 6, 12, 6)
        self.timeline_label = QLabel("0.0 / 120s")
        self.start_button = QPushButton("开始")
        self.pause_button = QPushButton("暂停")
        self.step_button = QPushButton("单步")
        self.reset_button = QPushButton("重置")
        self.progress = QProgressBar()
        self.progress.setObjectName("progress")
        self.progress.setRange(0, 1000)
        self.progress.setTextVisible(False)
        self.start_button.clicked.connect(self._start)
        self.pause_button.clicked.connect(self._pause)
        self.step_button.clicked.connect(self._step)
        self.reset_button.clicked.connect(self._reset)
        for widget in [self.timeline_label, self.start_button, self.pause_button, self.step_button, self.reset_button, self.progress]:
            timeline.addWidget(widget)
        timeline.setStretchFactor(self.progress, 1)
        layout.addLayout(timeline)
        return stage

    def _build_right_panel(self) -> QWidget:
        panel = QFrame()
        panel.setObjectName("panel")
        panel.setFixedWidth(400)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(10, 12, 10, 10)
        layout.setSpacing(8)
        self.node_table = QTableWidget(0, 6)
        self.node_table.setHorizontalHeaderLabels(["ID", "侧偏(m)", "待飞距(m)", "高度(m)", "速度(m/s)", "状态"])
        self.link_table = QTableWidget(0, 4)
        self.link_table.setHorizontalHeaderLabels(["链路", "延迟", "丢包", "状态"])
        self._configure_table(self.node_table, [48, 58, 74, 58, 76, 50])
        self._configure_table(self.link_table, [92, 64, 58, 58])
        node_title = QLabel("节点状态")
        node_title.setObjectName("sectionTitle")
        link_title = QLabel("链路状态")
        link_title.setObjectName("sectionTitle")
        layout.addWidget(node_title)
        layout.addWidget(self.node_table)
        layout.addSpacing(8)
        layout.addWidget(link_title)
        layout.addWidget(self.link_table)
        layout.addStretch(1)
        return panel

    def _configure_table(self, table: QTableWidget, widths: list[int]) -> None:
        table.verticalHeader().setVisible(False)
        table.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        table.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        table.setSizeAdjustPolicy(QAbstractScrollArea.SizeAdjustPolicy.AdjustIgnored)
        table.setAlternatingRowColors(False)
        table.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        header = table.horizontalHeader()
        header.setStretchLastSection(True)
        header.setSectionResizeMode(QHeaderView.ResizeMode.Fixed)
        for index, width in enumerate(widths):
            table.setColumnWidth(index, width)
        table.verticalHeader().setDefaultSectionSize(30)
        table.verticalHeader().setMinimumSectionSize(30)
        table.setFixedHeight(138)

    def _install_button_cursors(self) -> None:
        for button in self.findChildren(QPushButton):
            button.setCursor(Qt.CursorShape.PointingHandCursor)

    def _apply_theme(self) -> None:
        theme = self.theme
        button_hover = theme.line.lighter(108)
        button_pressed = theme.line.darker(108)
        button_border_hover = theme.accent
        menu_selected = theme.line.lighter(112)
        self.setStyleSheet(
            f"""
            QMainWindow, QWidget {{
                background: {theme.bg.name()};
                color: {theme.ink.name()};
                font-family: -apple-system, BlinkMacSystemFont, "Segoe UI";
                font-size: 13px;
            }}
            QFrame#panel, QGroupBox {{
                background: {theme.panel.name()};
                border: 1px solid {theme.line.name()};
                border-radius: 8px;
            }}
            QGroupBox {{
                margin-top: 10px;
                font-weight: 700;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 10px;
                padding: 0 4px;
                background: {theme.panel.name()};
            }}
            QLabel#title {{
                font-size: 18px;
                font-weight: 700;
            }}
            QLabel#stageTitle {{
                font-size: 15px;
                font-weight: 700;
            }}
            QLabel#sectionTitle {{
                background: transparent;
                color: {theme.ink.name()};
                font-size: 14px;
                font-weight: 700;
                padding: 0 0 2px 0;
            }}
            QLabel#statusPill {{
                color: {theme.accent.name()};
                background: {theme.field.name()};
                border-radius: 14px;
                padding: 5px 14px;
                font-weight: 700;
            }}
            QLabel#reportPill {{
                color: #175cd3;
                background: {theme.field.name()};
                border-radius: 14px;
                padding: 5px 14px;
                font-weight: 700;
            }}
            QLabel#legendLeader {{
                color: {theme.leader.name()};
                font-weight: 700;
            }}
            QLabel#legendWingman {{
                color: {theme.wingman.name()};
                font-weight: 700;
            }}
            QLabel#legendLink {{
                color: {theme.link.name()};
                font-weight: 700;
            }}
            QLabel#legendWarn {{
                color: {theme.warn.name()};
                font-weight: 700;
            }}
            QPushButton {{
                background: {theme.field.name()};
                color: {theme.ink.name()};
                border: 1px solid {theme.line.name()};
                border-radius: 6px;
                min-height: 28px;
                padding: 0 10px;
            }}
            QPushButton:hover {{
                background: {button_hover.name()};
                border-color: {button_border_hover.name()};
            }}
            QPushButton:pressed, QPushButton:down {{
                background: {button_pressed.name()};
                border-color: {theme.accent.name()};
                padding-top: 1px;
                padding-left: 11px;
            }}
            QPushButton:disabled {{
                color: {theme.muted.name()};
                background: {theme.line.name()};
                border-color: {theme.line.name()};
            }}
            QPushButton#selectButton {{
                text-align: left;
                padding-left: 10px;
                padding-right: 10px;
            }}
            QPushButton#selectButton:pressed, QPushButton#selectButton:down {{
                padding-left: 11px;
                padding-right: 9px;
            }}
            QMenu {{
                background: {theme.field.name()};
                color: {theme.ink.name()};
                border: 1px solid {theme.line.name()};
            }}
            QMenu::item {{
                padding: 4px;
            }}
            QMenu::item:selected {{
                background: {menu_selected.name()};
            }}
            QTableWidget {{
                background: {theme.field.name()};
                gridline-color: {theme.line.name()};
                border: 1px solid {theme.line.name()};
                alternate-background-color: {theme.field.name()};
                font-size: 13px;
            }}
            QHeaderView::section {{
                background: {theme.panel.name()};
                color: {theme.muted.name()};
                border: 0;
                border-bottom: 1px solid {theme.line.name()};
                padding: 6px 4px;
                font-weight: 700;
            }}
            QSlider::groove:horizontal {{
                background: {theme.line.name()};
                height: 6px;
                border-radius: 3px;
            }}
            QSlider::sub-page:horizontal {{
                background: {theme.accent.name()};
                border-radius: 3px;
            }}
            QSlider::handle:horizontal {{
                background: {theme.accent.name()};
                border: 2px solid {theme.panel.name()};
                width: 16px;
                height: 16px;
                margin: -6px 0;
                border-radius: 8px;
            }}
            QProgressBar#progress {{
                background: {theme.line.name()};
                border: 0;
                border-radius: 3px;
                min-height: 6px;
                max-height: 6px;
            }}
            QProgressBar#progress::chunk {{
                background: {theme.accent.name()};
                border-radius: 3px;
            }}
            """
        )
        self.top_view.set_theme(theme)
        self.side_view.set_theme(theme)

    def _update_snapshot(self, snapshot: Snapshot) -> None:
        self.run_state_label.setText(snapshot.run_state)
        self.report_label.setText(f"回报：{snapshot.control_report}")
        self.timeline_label.setText(f"{snapshot.time:.1f} / {snapshot.duration:.0f}s")
        self.progress.setValue(round(snapshot.time / snapshot.duration * 1000) if snapshot.duration else 0)
        self.pause_button.setEnabled(snapshot.run_state != "READY")
        self.start_button.setText("继续" if snapshot.run_state == "PAUSED" else "开始")
        self.top_view.set_snapshot(snapshot)
        self.side_view.set_snapshot(snapshot)
        self._update_tables(snapshot)

    def _update_tables(self, snapshot: Snapshot) -> None:
        self.node_table.setRowCount(len(snapshot.nodes))
        for row, node in enumerate(snapshot.nodes):
            speed = math.hypot(node.vx, node.vy)
            side_offset = (node.y - WORLD_HEIGHT / 2) * 0.8
            distance_to_go = max(0.0, (WORLD_WIDTH - node.x) * 4)
            status = "降级" if snapshot.disturbance == "节点故障" and node.node_id == "A02" else "正常"
            values = [node.node_id, f"{side_offset:.0f}", f"{distance_to_go:.0f}", f"{node_altitude(row, snapshot.time):.0f}", f"{speed:.1f}", status]
            for column, value in enumerate(values):
                self.node_table.setItem(row, column, QTableWidgetItem(value))

        self.link_table.setRowCount(len(snapshot.links))
        for row, link in enumerate(snapshot.links):
            values = [f"{link.source}-{link.target}", f"{link.latency_ms}ms", f"{link.loss * 100:.0f}%", "正常" if link.ok else "丢包"]
            for column, value in enumerate(values):
                self.link_table.setItem(row, column, QTableWidgetItem(value))

    def _start(self) -> None:
        self._update_snapshot(self.sim.start())
        self.timer.start()
        self._log("UI", "发送 start/resume 命令")

    def _pause(self) -> None:
        snapshot = self.sim.pause()
        if snapshot.run_state == "PAUSED":
            self.timer.stop()
        elif snapshot.run_state == "RUNNING":
            self.timer.start()
        self._update_snapshot(snapshot)
        self._log("UI", "发送 pause/resume 命令")

    def _step(self) -> None:
        self.timer.stop()
        self._update_snapshot(self.sim.single_step())
        self._log("UI", "发送 step 命令，推进一个仿真步")

    def _reset(self) -> None:
        self.timer.stop()
        self._update_snapshot(self.sim.reset())
        self._log("SimControl", "重置仿真")

    def _on_tick(self) -> None:
        snapshot = self.sim.advance()
        self._update_snapshot(snapshot)
        if snapshot.run_state == "READY":
            self.timer.stop()

    def _inject_disturbance(self, kind: str) -> None:
        messages = {
            "wind": "注入风场脉冲",
            "fault": "注入 A02 控制效率下降",
            "loss": "注入链路丢包",
            "clear": "清除运行期扰动",
        }
        self._update_snapshot(self.sim.inject_disturbance(kind))
        self._log("Disturb", messages[kind])

    def _choose_config(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "选择配置文件", str(Path.cwd()), "Config (*.yaml *.yml *.json)")
        if not path:
            return
        self.config_name.setText(Path(path).name)
        self._log("Config", f"选择配置文件 {Path(path).name}")

    def _on_speed_changed(self, value: int) -> None:
        speed = value / 10.0
        self.sim.speed = speed
        self.speed_label.setText(f"{speed:.1f}x")

    def _on_theme_changed(self) -> None:
        self.theme_key = self.theme_select.currentData()
        self.theme = THEMES[self.theme_key]
        self._apply_theme()
        self._log("UI", f"切换主题：{self.theme_select.currentText()}")

    def _on_auto_center_changed(self) -> None:
        self.top_view.auto_center = self.auto_center.isChecked()
        self.top_view.set_snapshot(self.sim.snapshot())

    def _on_grid_changed(self) -> None:
        show_grid = self.grid_toggle.isChecked()
        self.top_view.show_grid = show_grid
        self.side_view.show_grid = show_grid
        self.top_view.viewport().update()
        self.side_view.update()

    def _disable_auto_center(self) -> None:
        if self.auto_center.isChecked():
            self.auto_center.setChecked(False)

    def _reset_view(self) -> None:
        self.top_view.reset_view()
        self.side_view.update()

    def _toggle_fullscreen(self) -> None:
        if self._stage_fullscreen_dialog is not None:
            self._exit_stage_fullscreen()
        else:
            self._enter_stage_fullscreen()

    def _enter_stage_fullscreen(self) -> None:
        if self.stage is None or self.main_layout is None or self._stage_fullscreen_dialog is not None:
            return

        index = self.main_layout.indexOf(self.stage)
        if index < 0:
            return

        self._stage_layout_index = index
        self._stage_layout_stretch = self.main_layout.stretch(index)
        self.main_layout.removeWidget(self.stage)

        self._stage_placeholder = QWidget()
        self.main_layout.insertWidget(self._stage_layout_index, self._stage_placeholder, self._stage_layout_stretch)

        dialog = StageFullscreenDialog(self)
        dialog_layout = QVBoxLayout(dialog)
        dialog_layout.setContentsMargins(0, 0, 0, 0)
        dialog_layout.setSpacing(0)
        dialog_layout.addWidget(self.stage)
        self._stage_fullscreen_dialog = dialog
        self._set_fullscreen_button_state(True)
        dialog.showFullScreen()

    def _exit_stage_fullscreen(self) -> None:
        if self.stage is None or self.main_layout is None or self._stage_fullscreen_dialog is None:
            return

        dialog = self._stage_fullscreen_dialog
        dialog.layout().removeWidget(self.stage)
        dialog.hide()
        dialog.deleteLater()
        self._stage_fullscreen_dialog = None

        if self._stage_placeholder is not None:
            placeholder_index = self.main_layout.indexOf(self._stage_placeholder)
            if placeholder_index >= 0:
                self.main_layout.removeWidget(self._stage_placeholder)
            self._stage_placeholder.deleteLater()
            self._stage_placeholder = None

        insert_index = min(self._stage_layout_index, self.main_layout.count())
        self.main_layout.insertWidget(insert_index, self.stage, self._stage_layout_stretch)
        self._set_fullscreen_button_state(False)
        self.stage.show()
        self.top_view.update()
        self.side_view.update()

    def _set_fullscreen_button_state(self, active: bool) -> None:
        if self.fullscreen_button is None:
            return
        self.fullscreen_button.setText("↙" if active else "⛶")
        self.fullscreen_button.setToolTip("退出全屏" if active else "全屏显示")
        self.fullscreen_button.setAccessibleName("退出全屏" if active else "全屏显示")

    def _log(self, source: str, message: str) -> None:
        self.log_dialog.append(self.sim.time, source, message)


def run_gui(argv: list[str] | None = None) -> int:
    """Run the PySide6 GUI."""

    app = QApplication(argv or [])
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(run_gui())
