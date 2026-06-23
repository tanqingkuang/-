"""编队仿真 PySide6 主窗口。注意：正式 GUI 入口在本模块。"""

from __future__ import annotations

from configparser import ConfigParser
import math
import os
import sys
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

from src.runner.sim_control import SimulationController
from src.runner.sim_control import SimulationSnapshot as ControllerSnapshot


WORLD_WIDTH = 1600.0
WORLD_HEIGHT = 520.0
TRAIL_SECONDS = 18.0
TOP_VIEW_ORIGIN_MARGIN = 40.0
VIEW_MIN_SCALE = 0.05
VIEW_MAX_SCALE = 3.5
WORLD_GRID_SPACING = 48
GRID_MIN_SCREEN_SPACING = 36.0
GRID_MAX_SCREEN_SPACING = 96.0
APP_CONFIG_SECTION = "config"
APP_CONFIG_KEY_LAST_CONFIG = "last_config"
APP_CONFIG_FILE_NAME = "config.ini"


def adaptive_world_grid_spacing(scale_value: float) -> int:
    spacing = WORLD_GRID_SPACING
    safe_scale = max(scale_value, 0.001)
    while spacing * safe_scale < GRID_MIN_SCREEN_SPACING:
        spacing *= 2
    while spacing > 1 and spacing * safe_scale > GRID_MAX_SCREEN_SPACING:
        spacing = max(1, spacing // 2)
    return spacing


def default_project_root() -> Path:
    """返回项目根目录。注意：打包后路径和源码运行路径不同。"""

    if getattr(sys, "frozen", False):
        return Path(sys.executable).resolve().parent

    source_root = Path(__file__).resolve().parents[3]
    cwd = Path.cwd().resolve()
    for candidate in (cwd, cwd.parent, source_root):
        if (candidate / "configs").exists():
            return candidate
    return cwd


@dataclass
class TrailPoint:
    """仿真时间中的一个轨迹采样点。注意：用于绘制历史尾迹。"""

    x: float
    y: float
    altitude: float
    time: float


@dataclass
class NodeState:
    """单个飞机节点的显示状态。注意：字段用于 GUI 绘图和表格。"""

    node_id: str
    role: str
    x: float
    y: float
    vx: float
    vy: float
    altitude: float = 1200.0
    health: str = "normal"
    trail: list[TrailPoint] = field(default_factory=list)
    cross_track_error: float | None = None
    distance_to_go: float | None = None


@dataclass
class LinkState:
    """单条通信链路的显示状态。注意：loss 为 0 到 1 的比例。"""

    source: str
    target: str
    direction: str
    latency_ms: int
    loss: float
    ok: bool = True


@dataclass
class ReferenceRoute:
    """俯视图和侧视图共用的参考航段。注意：坐标单位为米。"""

    start_x: float
    start_y: float
    start_altitude: float
    end_x: float
    end_y: float
    end_altitude: float


@dataclass
class Snapshot:
    """面向 UI 的仿真快照。注意：由真实控制器或 mock 数据适配得到。"""

    time: float
    duration: float
    step: float
    run_state: str
    control_report: str
    disturbance: str
    nodes: list[NodeState]
    links: list[LinkState]
    route: ReferenceRoute | None = None
    route_segments: list[ReferenceRoute] = field(default_factory=list)


class MockSimulation:
    """真实控制器接入前使用的小型 UI 演示数据源。注意：仅作为界面兜底。"""

    def __init__(self) -> None:
        """初始化 MockSimulation 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
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
        """复位 MockSimulation 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
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
            LinkState("A01", "A02", "duplex", 18, 0.01),
            LinkState("A01", "A03", "duplex", 21, 0.01),
            LinkState("A02", "A03", "duplex", 30, 0.02),
        ]
        return self.snapshot()

    def start(self) -> Snapshot:
        """启动或继续 MockSimulation 的运行流程。注意：重复调用应保持状态一致。"""
        self.running = True
        self.paused = False
        return self.snapshot()

    def pause(self) -> Snapshot:
        """暂停 MockSimulation 的运行流程。注意：只暂停调度，不清空当前状态。"""
        if self.running:
            self.paused = not self.paused
        return self.snapshot()

    def single_step(self) -> Snapshot:
        """执行单步推进。注意：仅在暂停或可单步状态下使用。"""
        self.running = True
        self.paused = True
        self.advance()
        return self.snapshot()

    def inject_disturbance(self, kind: str) -> Snapshot:
        """向仿真注入扰动。注意：调用方需提供合法扰动类型和参数。"""
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
        """推进仿真显示或数据状态。注意：步长应与调用方传入时间一致。"""
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
        """返回当前快照。注意：返回数据用于显示，不应被调用方回写。"""
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
            route=ReferenceRoute(40.0, 238.0, 1200.0, WORLD_WIDTH - 40.0, 238.0, 1200.0),
            route_segments=[ReferenceRoute(40.0, 238.0, 1200.0, WORLD_WIDTH - 40.0, 238.0, 1200.0)],
        )


class ControllerSimulationAdapter:
    """把 SimulationController 快照适配为现有 GUI 绘图模型。注意：需要维护尾迹缓存。"""

    def __init__(self) -> None:
        """初始化 ControllerSimulationAdapter 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        self.controller = SimulationController()
        self.speed = 1.0
        self.disturbance = "无"
        self._trail_by_node: dict[str, list[TrailPoint]] = {}
        self._last_xy_by_node: dict[str, tuple[float, float, float]] = {}
        self._processed_event_count = 0
        self.last_result_code = "OK"
        self.last_result_message = ""

    @property
    def time(self) -> float:
        """返回当前仿真时间。注意：单位为秒。"""
        return self.controller.get_snapshot().time_s

    def load_config(self, path: str) -> Snapshot:
        """读取并解析仿真配置文件。注意：文件路径由调用方保证存在且可读。"""
        result = self.controller.load_config(path)
        self.last_result_code = result.code
        self.last_result_message = result.message
        if result.code == "OK":
            self._trail_by_node.clear()
            self._last_xy_by_node.clear()
            self._processed_event_count = len(self.controller.get_recent_events(limit=1000))
            self.disturbance = "无"
        return self.snapshot()

    def start(self) -> Snapshot:
        """启动或继续 ControllerSimulationAdapter 的运行流程。注意：重复调用应保持状态一致。"""
        result = self.controller.start()
        self.last_result_code = result.code
        self.last_result_message = result.message
        return self.snapshot()

    def pause(self) -> Snapshot:
        """暂停 ControllerSimulationAdapter 的运行流程。注意：只暂停调度，不清空当前状态。"""
        snapshot = self.controller.get_snapshot()
        if snapshot.run_state == "RUNNING":
            result = self.controller.pause()
        elif snapshot.run_state == "PAUSED":
            # UI 交互便利：暂停态下同一个按钮表示继续。
            result = self.controller.start()
        else:
            result = self.controller.pause()
        self.last_result_code = result.code
        self.last_result_message = result.message
        return self.snapshot()

    def single_step(self) -> Snapshot:
        """执行单步推进。注意：仅在暂停或可单步状态下使用。"""
        result = self.controller.step()
        self.last_result_code = result.code
        self.last_result_message = result.message
        return self.snapshot()

    def reset(self) -> Snapshot:
        """复位 ControllerSimulationAdapter 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        result = self.controller.reset()
        self.last_result_code = result.code
        self.last_result_message = result.message
        if result.code == "OK":
            self._trail_by_node.clear()
            self._last_xy_by_node.clear()
            self.disturbance = "无"
        return self.snapshot()

    def poll(self) -> Snapshot:
        """轮询当前快照。注意：该操作不推进仿真。"""

        return self.snapshot()

    def advance(self) -> Snapshot:
        """推进仿真显示或数据状态。注意：步长应与调用方传入时间一致。"""
        return self.poll()

    def snapshot(self) -> Snapshot:
        """返回当前快照。注意：返回数据用于显示，不应被调用方回写。"""
        return self._convert_snapshot(self.controller.get_snapshot())

    def inject_disturbance(self, kind: str) -> Snapshot:
        """向仿真注入扰动。注意：调用方需提供合法扰动类型和参数。"""
        command = self._disturbance_command(kind)
        result = self.controller.inject_disturbance(command)
        self.last_result_code = result.code
        self.last_result_message = result.message
        if result.code == "OK":
            self.disturbance = {
                "wind": "风场",
                "fault": "节点故障",
                "loss": "链路丢包",
                "clear": "无",
            }[kind]
        return self.snapshot()

    def set_speed(self, speed: float) -> None:
        """设置播放速度。注意：只影响界面或控制器调度倍率。"""
        self.speed = speed
        self.controller.set_playback_rate(speed)

    def close(self) -> None:
        """释放 ControllerSimulationAdapter 持有的资源。注意：关闭后不应继续调用运行接口。"""
        self.controller.close()

    def _convert_snapshot(self, snapshot: ControllerSnapshot) -> Snapshot:
        """把控制器快照转换为 GUI 绘图模型。注意：需要同步维护轨迹缓存和显示字段。"""
        self._sync_disturbance_from_events()
        nodes: list[NodeState] = []
        for node in snapshot.nodes:
            previous = self._last_xy_by_node.get(node.node_id)
            if previous is None:
                vx = node.vx_mps
                vy = node.vy_mps
            else:
                previous_x, previous_y, previous_time = previous
                dt = max(1e-6, snapshot.time_s - previous_time)
                vx = (node.x_m - previous_x) / dt
                vy = (node.y_m - previous_y) / dt
            self._last_xy_by_node[node.node_id] = (node.x_m, node.y_m, snapshot.time_s)

            trail = self._trail_by_node.setdefault(node.node_id, [])
            if not trail or trail[-1].time != snapshot.time_s:
                trail.append(TrailPoint(node.x_m, node.y_m, node.altitude_m, snapshot.time_s))
            trail[:] = [point for point in trail if snapshot.time_s - point.time <= TRAIL_SECONDS]
            nodes.append(
                NodeState(
                    node_id=node.node_id,
                    role=node.role,
                    x=node.x_m,
                    y=node.y_m,
                    vx=vx,
                    vy=vy,
                    altitude=node.altitude_m,
                    health=node.health,
                    trail=list(trail),
                    cross_track_error=node.cross_track_error_m,
                    distance_to_go=node.distance_to_go_m,
                )
            )

        links: list[LinkState] = []
        for link in snapshot.links:
            source, _, target = link.link_id.partition("-")
            links.append(
                LinkState(
                    source=source,
                    target=target,
                    direction=link.direction,
                    latency_ms=round(link.latency_ms),
                    loss=link.loss_rate,
                    ok=link.status == "normal",
                )
            )
        route = None
        if snapshot.route is not None:
            route = self._convert_route(snapshot.route)
        route_segments = [
            self._convert_route(segment)
            for segment in snapshot.route_segments
        ]
        if not route_segments and route is not None:
            route_segments = [route]
        return Snapshot(
            time=snapshot.time_s,
            duration=snapshot.duration_s,
            step=snapshot.step_s,
            run_state=snapshot.run_state,
            control_report=snapshot.control_report,
            disturbance=self._visible_disturbance(snapshot),
            nodes=nodes,
            links=links,
            route=route,
            route_segments=route_segments,
        )

    @staticmethod
    def _convert_route(route) -> ReferenceRoute:  # noqa: ANN001
        """把控制器航线状态转换为 GUI 参考航线。注意：空航线返回空值。"""
        return ReferenceRoute(
            start_x=route.start_x_m,
            start_y=route.start_y_m,
            start_altitude=route.start_altitude_m,
            end_x=route.end_x_m,
            end_y=route.end_y_m,
            end_altitude=route.end_altitude_m,
        )

    def _visible_disturbance(self, snapshot: ControllerSnapshot) -> str:
        """返回当前界面应显示的扰动名称。注意：已清除或过期扰动显示为无。"""
        if any(node.health != "normal" for node in snapshot.nodes):
            return "节点故障"
        if any(link.status != "normal" for link in snapshot.links):
            return "链路丢包"
        if snapshot.run_state == "READY" and self.disturbance == "无":
            return "无"
        return self.disturbance

    def _sync_disturbance_from_events(self) -> None:
        """根据控制器事件同步扰动显示状态。注意：只处理尚未消费的新事件。"""
        events = self.controller.get_recent_events(limit=1000)
        for event in events[self._processed_event_count:]:
            if event.source != "Disturbance":
                continue
            if event.message == "清除扰动" or event.message.startswith("扰动结束"):
                self.disturbance = "无"
            elif "wind" in event.message:
                self.disturbance = "风场"
            elif "node_fault" in event.message:
                self.disturbance = "节点故障"
            elif "link_loss" in event.message or "link_fault" in event.message:
                self.disturbance = "链路丢包"
        self._processed_event_count = len(events)

    def _disturbance_command(self, kind: str) -> dict[str, object]:
        """生成 GUI 按钮对应的扰动命令。注意：命令结构需与控制器注入接口一致。"""
        if kind == "wind":
            return {"type": "wind", "duration_s": 8.0, "params": {"speed_mps": 8.0, "direction_deg": 90.0}}
        if kind == "fault":
            return {"type": "node_fault", "target": "A02", "duration_s": 10.0, "params": {"mode": "degraded"}}
        if kind == "loss":
            return {"type": "link_loss", "target": "A01-A02", "duration_s": 12.0, "params": {"loss_rate": 0.3}}
        return {"type": "clear"}

def node_altitude(index: int, time_value: float) -> float:
    """读取节点高度用于侧视图显示。注意：缺省时使用 0 作为兜底。"""

    return 1200.0 + index * 35.0 + math.sin(time_value / 6.0 + index) * 12.0


def link_direction_label(direction: str) -> str:
    """生成通信链路方向显示文本。注意：只负责界面文案，不改变链路状态。"""

    return {"duplex": "双向", "simplex": "单向"}.get(direction, direction)


class Theme:
    """单个 UI 主题的集中配色。注意：主题切换时画布和控件共用这些颜色。"""

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
        """初始化 Theme 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
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
    """基于按钮的选项选择器。注意：弹出菜单位置由控件主动控制。"""

    currentIndexChanged = Signal()

    def __init__(self, min_width: int, popup_side: str = "below", parent: QWidget | None = None) -> None:
        """初始化 SelectButton 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
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
        """向控件添加一个选项。注意：选项文本和附加数据需保持对应。"""
        self._items.append((text, data))
        if self._index == -1:
            self.setCurrentIndex(0, emit=False)

    def addItems(self, texts: list[str]) -> None:
        """批量添加控件选项。注意：按输入顺序追加。"""
        for text in texts:
            self.addItem(text, text)

    def setCurrentIndex(self, index: int, *, emit: bool = True) -> None:
        """设置当前选中项。注意：索引越界时不应破坏控件状态。"""
        if index < 0 or index >= len(self._items):
            return
        if index == self._index:
            return
        self._index = index
        self.setText(f"{self._items[index][0]}  ▾")
        if emit:
            self.currentIndexChanged.emit()

    def currentText(self) -> str:
        """返回当前选项文本。注意：无选项时返回空字符串。"""
        if self._index < 0:
            return ""
        return self._items[self._index][0]

    def currentData(self) -> object | None:
        """返回当前选项附加数据。注意：无选项时返回空值。"""
        if self._index < 0:
            return None
        return self._items[self._index][1]

    def show_menu(self) -> None:
        """显示下拉菜单。注意：菜单项选择会同步当前索引。"""
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
    """支持平移和缩放的俯视编队视图。注意：只负责显示，不修改仿真状态。"""

    viewChanged = Signal()
    manualViewChanged = Signal()
    resetViewRequested = Signal()

    def __init__(self, parent: QWidget | None = None) -> None:
        """初始化 TopView 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        super().__init__(parent)
        self.snapshot: Snapshot | None = None
        self.theme = THEMES["light"]
        self.scale_value = 1.0
        self.offset = self._default_offset()
        self.auto_center = False
        self.show_grid = True
        self._manual_view = False
        self._pan_origin: QPointF | None = None
        self._selection_origin: QPointF | None = None
        self._selection_current: QPointF | None = None
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setFrameShape(QFrame.Shape.NoFrame)
        self.setMinimumHeight(360)
        self.setMouseTracking(True)

    def set_theme(self, theme: Theme) -> None:
        """设置当前主题。注意：需要同步更新画布和控件颜色。"""
        self.theme = theme
        self.viewport().update()

    def set_snapshot(self, snapshot: Snapshot) -> None:
        """设置用于绘制的快照。注意：只更新显示缓存，不推进仿真。"""
        self.snapshot = snapshot
        if self.auto_center:
            self._apply_auto_center()
        elif not self._manual_view:
            self._fit_route_to_view()
        self.viewport().update()

    def reset_view(self) -> None:
        """重置视图缩放和平移。注意：不修改仿真数据。"""
        self._manual_view = False
        self.scale_value = 1.0
        self.offset = self._default_offset()
        self._fit_route_to_view()
        self.viewport().update()
        self.viewChanged.emit()
        self.manualViewChanged.emit()
        self.resetViewRequested.emit()

    @staticmethod
    def _default_offset() -> QPointF:
        """计算俯视图默认平移量。注意：用于把初始场景放到画布可见区域。"""
        return QPointF(TOP_VIEW_ORIGIN_MARGIN, TOP_VIEW_ORIGIN_MARGIN)

    def wheelEvent(self, event) -> None:  # noqa: ANN001
        """处理鼠标滚轮事件。注意：用于缩放视图并保持交互焦点。"""
        delta = event.pixelDelta().y() or event.angleDelta().y()
        if delta == 0:
            return
        cursor = event.position()
        before = QPointF(
            (cursor.x() - self.offset.x()) / self.scale_value,
            (cursor.y() - self.offset.y()) / self.scale_value,
        )
        factor = math.pow(1.001, delta)
        self.scale_value = min(VIEW_MAX_SCALE, max(VIEW_MIN_SCALE, self.scale_value * factor))
        self.offset = QPointF(
            cursor.x() - before.x() * self.scale_value,
            cursor.y() - before.y() * self.scale_value,
        )
        self._manual_view = True
        self.viewport().update()
        self.viewChanged.emit()
        self.manualViewChanged.emit()
        event.accept()

    def mousePressEvent(self, event) -> None:  # noqa: ANN001
        """处理鼠标按下事件。注意：记录拖拽或框选起点。"""
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
        """处理鼠标移动事件。注意：拖拽过程中只更新视图状态。"""
        if self._pan_origin is not None:
            delta = event.position() - self._pan_origin
            self.offset += QPointF(delta.x(), delta.y())
            self._pan_origin = event.position()
            self._manual_view = True
            self.viewport().update()
            self.viewChanged.emit()
            self.manualViewChanged.emit()
            event.accept()
        elif self._selection_origin is not None:
            self._selection_current = event.position()
            self.viewport().update()
            event.accept()

    def mouseReleaseEvent(self, event) -> None:  # noqa: ANN001
        """处理鼠标释放事件。注意：结束拖拽或框选操作。"""
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
        """处理鼠标双击事件。注意：通常用于快速重置或聚焦视图。"""
        if event.button() == Qt.MouseButton.LeftButton:
            self.reset_view()
            event.accept()

    def paintEvent(self, event) -> None:  # noqa: ARG002, ANN001
        """处理 Qt 绘制事件。注意：只在当前快照基础上渲染画面。"""
        painter = QPainter(self.viewport())
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), self.theme.canvas)
        painter.translate(self.offset)
        painter.scale(self.scale_value, self.scale_value)
        if self.show_grid:
            self._draw_grid(painter)
        if self.snapshot:
            if self.snapshot.nodes:
                self._draw_route(painter)
            self._draw_links(painter, self.snapshot)
            self._draw_nodes(painter, self.snapshot)
        painter.resetTransform()
        self._draw_selection(painter)

    def _viewport_to_world(self, point: QPointF) -> QPointF:
        """把视口坐标转换为世界坐标。注意：依赖当前缩放和平移状态。"""
        return QPointF(
            (point.x() - self.offset.x()) / self.scale_value,
            (point.y() - self.offset.y()) / self.scale_value,
        )

    def _zoom_to_selection(self) -> None:
        """执行 to selection 缩放。注意：保持选区或鼠标焦点附近的世界坐标稳定。"""
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
        self.scale_value = min(VIEW_MAX_SCALE, max(VIEW_MIN_SCALE, scale))

        center_x = (world_start.x() + world_end.x()) / 2.0
        center_y = (world_start.y() + world_end.y()) / 2.0
        self.offset = QPointF(
            viewport.width() / 2.0 - center_x * self.scale_value,
            viewport.height() / 2.0 - center_y * self.scale_value,
        )
        self._manual_view = True
        self.viewChanged.emit()
        self.manualViewChanged.emit()

    def _draw_selection(self, painter: QPainter) -> None:
        """绘制 selection 画面元素。注意：只做渲染，不修改仿真状态。"""
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
        """应用 auto center 设置。注意：只修改对应显示或运行参数。"""
        if not self.snapshot or not self.snapshot.nodes:
            return
        active = [node for node in self.snapshot.nodes if node.health == "normal"]
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

    def _fit_route_to_view(self) -> None:
        """把航线范围适配到当前俯视图。注意：只调整显示缩放和平移。"""
        if self.snapshot is None or not self._route_segments():
            self.offset = self._default_offset()
            return
        routes = self._route_segments()
        rect = self.viewport().rect()
        if rect.width() <= 0 or rect.height() <= 0:
            return
        xs = [value for route in routes for value in (route.start_x, route.end_x)]
        ys = [value for route in routes for value in (route.start_y, route.end_y)]
        min_x = min(xs)
        max_x = max(xs)
        min_y = min(ys)
        max_y = max(ys)
        span_x = max(1.0, max_x - min_x)
        span_y = max(1.0, max_y - min_y)
        available_width = max(1.0, rect.width() - TOP_VIEW_ORIGIN_MARGIN * 2.0)
        available_height = max(1.0, rect.height() - TOP_VIEW_ORIGIN_MARGIN * 2.0)
        scale_x = available_width / span_x
        scale_y = available_height / span_y
        self.scale_value = min(1.0, max(VIEW_MIN_SCALE, min(scale_x, scale_y)))
        self.offset = QPointF(
            TOP_VIEW_ORIGIN_MARGIN - min_x * self.scale_value,
            TOP_VIEW_ORIGIN_MARGIN - min_y * self.scale_value,
        )

    def _draw_grid(self, painter: QPainter) -> None:
        """绘制 grid 画面元素。注意：只做渲染，不修改仿真状态。"""
        rect = self.viewport().rect()
        left = (rect.left() - self.offset.x()) / self.scale_value
        right = (rect.right() - self.offset.x()) / self.scale_value
        top = (rect.top() - self.offset.y()) / self.scale_value
        bottom = (rect.bottom() - self.offset.y()) / self.scale_value
        spacing = self._grid_world_spacing()
        start_x = math.floor(left / spacing) * spacing
        end_x = math.ceil(right / spacing) * spacing
        start_y = math.floor(top / spacing) * spacing
        end_y = math.ceil(bottom / spacing) * spacing

        painter.setPen(QPen(self.theme.grid, 1.0 / self.scale_value))
        for x in range(start_x, end_x + spacing, spacing):
            painter.drawLine(x, start_y, x, end_y)
        for y in range(start_y, end_y + spacing, spacing):
            painter.drawLine(start_x, y, end_x, y)

    def _grid_world_spacing(self) -> int:
        return adaptive_world_grid_spacing(self.scale_value)

    def _draw_route(self, painter: QPainter) -> None:
        """绘制 route 画面元素。注意：只做渲染，不修改仿真状态。"""
        routes = self._route_segments()
        if not routes:
            return
        pen = QPen(self.theme.route, 2.0 / self.scale_value)
        pen.setDashPattern([8, 7])
        painter.setPen(pen)
        for route in routes:
            painter.drawLine(QPointF(route.start_x, route.start_y), QPointF(route.end_x, route.end_y))
        painter.setBrush(self.theme.ink)
        painter.setPen(Qt.PenStyle.NoPen)
        marker_radius = 5.0 / self.scale_value
        painter.drawEllipse(QPointF(routes[0].start_x, routes[0].start_y), marker_radius, marker_radius)
        for route in routes:
            painter.drawEllipse(QPointF(route.end_x, route.end_y), marker_radius, marker_radius)

    def _route_segments(self) -> list[ReferenceRoute]:
        """返回需要绘制的航段列表。注意：优先使用多航段快照，缺省时退回当前航段。"""
        if self.snapshot is None:
            return []
        if self.snapshot.route_segments:
            return self.snapshot.route_segments
        if self.snapshot.route is not None:
            return [self.snapshot.route]
        return []

    def _draw_links(self, painter: QPainter, snapshot: Snapshot) -> None:
        """绘制 links 画面元素。注意：只做渲染，不修改仿真状态。"""
        by_id = {node.node_id: node for node in snapshot.nodes}
        for link in snapshot.links:
            source = by_id[link.source]
            target = by_id[link.target]
            color = QColor(self.theme.link if link.ok else self.theme.warn)
            color.setAlphaF(0.58 if link.ok else 0.75)
            painter.setPen(QPen(color, 2 if link.ok else 3))
            painter.drawLine(QPointF(source.x, source.y), QPointF(target.x, target.y))

    def _draw_nodes(self, painter: QPainter, snapshot: Snapshot) -> None:
        """绘制 nodes 画面元素。注意：只做渲染，不修改仿真状态。"""
        for index, node in enumerate(snapshot.nodes):
            self._draw_trail(painter, node, index, snapshot.time)
            color = self.theme.warn if node.health != "normal" else self.theme.leader if index == 0 else self.theme.wingman
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
        """绘制 trail 画面元素。注意：只做渲染，不修改仿真状态。"""
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
    """高度随待飞距变化的侧视图。注意：横向视野与俯视图同步。"""

    ALTITUDE_MIN_DEFAULT = 1120.0
    ALTITUDE_MAX_DEFAULT = 1320.0
    PLOT_BOTTOM_MARGIN = 24.0
    PLOT_VERTICAL_MARGINS = 52.0
    ALTITUDE_GRID_SPACING = 40

    def __init__(self, top_view: TopView, parent: QWidget | None = None) -> None:
        """初始化 SideView 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        super().__init__(parent)
        self.top_view = top_view
        self.snapshot: Snapshot | None = None
        self.theme = THEMES["light"]
        self.show_grid = True
        self.altitude_min = self.ALTITUDE_MIN_DEFAULT
        self.altitude_max = self.ALTITUDE_MAX_DEFAULT
        self._pan_origin: QPointF | None = None
        self._selection_origin: QPointF | None = None
        self._selection_current: QPointF | None = None
        self.setMinimumHeight(150)
        self.setMouseTracking(True)

    def set_theme(self, theme: Theme) -> None:
        """设置当前主题。注意：需要同步更新画布和控件颜色。"""
        self.theme = theme
        self.update()

    def set_snapshot(self, snapshot: Snapshot) -> None:
        """设置用于绘制的快照。注意：只更新显示缓存，不推进仿真。"""
        self.snapshot = snapshot
        self.update()

    def paintEvent(self, event) -> None:  # noqa: ARG002, ANN001
        """处理 Qt 绘制事件。注意：只在当前快照基础上渲染画面。"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), self.theme.canvas)
        if self.show_grid:
            self._draw_grid(painter)
        if self.snapshot:
            if self.snapshot.nodes:
                self._draw_reference(painter)
            self._draw_trails(painter, self.snapshot)
            self._draw_nodes(painter, self.snapshot)
        painter.setPen(self.theme.muted)
        painter.drawText(QPointF(self.width() - 76, self.height() - 8), "待飞距")
        painter.drawText(QPointF(12, 20), "高度")
        self._draw_selection(painter)

    def wheelEvent(self, event) -> None:  # noqa: ANN001
        """处理鼠标滚轮事件。注意：用于缩放视图并保持交互焦点。"""
        delta = event.pixelDelta().y() or event.angleDelta().y()
        if delta == 0:
            return
        before_x = self._screen_to_world_x(event.position().x())
        old_scale = self.top_view.scale_value
        factor = math.pow(1.001, delta)
        self.top_view.scale_value = min(VIEW_MAX_SCALE, max(VIEW_MIN_SCALE, old_scale * factor))
        self.top_view.offset.setX(event.position().x() - before_x * self.top_view.scale_value)
        self._preserve_top_view_vertical_center(old_scale)
        self._emit_shared_view_changed()
        event.accept()

    def mousePressEvent(self, event) -> None:  # noqa: ANN001
        """处理鼠标按下事件。注意：记录拖拽或框选起点。"""
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
        """处理鼠标移动事件。注意：拖拽过程中只更新视图状态。"""
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
        """处理鼠标释放事件。注意：结束拖拽或框选操作。"""
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
        """处理鼠标双击事件。注意：通常用于快速重置或聚焦视图。"""
        if event.button() == Qt.MouseButton.LeftButton:
            self.top_view.reset_view()
            event.accept()

    def reset_altitude_view(self) -> None:
        """重置侧视图高度方向显示范围。注意：保持与俯视图横向视野同步。"""
        self.altitude_min = self.ALTITUDE_MIN_DEFAULT
        self.altitude_max = self.ALTITUDE_MAX_DEFAULT
        self.update()

    def _map_x(self, x: float) -> float:
        """映射 x 坐标。注意：需使用当前缩放和平移参数。"""
        return x * self.top_view.scale_value + self.top_view.offset.x()

    def _screen_to_world_x(self, x: float) -> float:
        """把屏幕坐标转换为 to world x。注意：依赖当前视图缩放和平移。"""
        return (x - self.top_view.offset.x()) / self.top_view.scale_value

    def _screen_to_altitude(self, y: float) -> float:
        """把屏幕坐标转换为 to altitude。注意：依赖当前视图缩放和平移。"""
        plot_height = max(1.0, self.height() - self.PLOT_VERTICAL_MARGINS)
        ratio = (self.height() - self.PLOT_BOTTOM_MARGIN - y) / plot_height
        return self.altitude_min + ratio * (self.altitude_max - self.altitude_min)

    def _pan_altitude(self, delta_y: float) -> None:
        """平移 altitude 视图。注意：只改变显示偏移，不改变仿真数据。"""
        plot_height = max(1.0, self.height() - self.PLOT_VERTICAL_MARGINS)
        altitude_delta = delta_y / plot_height * (self.altitude_max - self.altitude_min)
        self.altitude_min += altitude_delta
        self.altitude_max += altitude_delta

    def _preserve_top_view_vertical_center(self, old_scale: float) -> None:
        """保持俯视图垂向中心不被侧视图同步改动。注意：只同步横向范围。"""
        viewport = self.top_view.viewport().rect()
        center_y = (viewport.height() / 2.0 - self.top_view.offset.y()) / old_scale
        self.top_view.offset.setY(viewport.height() / 2.0 - center_y * self.top_view.scale_value)

    def _emit_shared_view_changed(self) -> None:
        """发送 shared view changed 信号。注意：避免循环触发视图同步。"""
        self.top_view.viewport().update()
        self.top_view.viewChanged.emit()
        self.top_view.manualViewChanged.emit()

    def _zoom_to_selection(self) -> None:
        """执行 to selection 缩放。注意：保持选区或鼠标焦点附近的世界坐标稳定。"""
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
            self.top_view.scale_value = min(VIEW_MAX_SCALE, max(VIEW_MIN_SCALE, self.width() / world_width * 0.94))
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
        """绘制 selection 画面元素。注意：只做渲染，不修改仿真状态。"""
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
        """映射 y 坐标。注意：需使用当前缩放和平移参数。"""
        return self.height() - self.PLOT_BOTTOM_MARGIN - (
            (altitude - self.altitude_min) / (self.altitude_max - self.altitude_min)
        ) * (self.height() - self.PLOT_VERTICAL_MARGINS)

    def _draw_grid(self, painter: QPainter) -> None:
        """绘制 grid 画面元素。注意：只做渲染，不修改仿真状态。"""
        painter.setPen(QPen(self.theme.grid, 1))
        spacing = self._grid_world_spacing()
        left = self._screen_to_world_x(0.0)
        right = self._screen_to_world_x(float(self.width()))
        start_x = math.floor(left / spacing) * spacing
        end_x = math.ceil(right / spacing) * spacing
        for world_x in range(start_x, end_x + spacing, spacing):
            x = self._map_x(float(world_x))
            painter.drawLine(QPointF(x, 0.0), QPointF(x, float(self.height())))

        altitude_spacing = self.ALTITUDE_GRID_SPACING
        start_altitude = math.floor(self.altitude_min / altitude_spacing) * altitude_spacing
        end_altitude = math.ceil(self.altitude_max / altitude_spacing) * altitude_spacing
        for altitude in range(start_altitude, end_altitude + altitude_spacing, altitude_spacing):
            y = self._map_y(float(altitude))
            painter.drawLine(QPointF(0.0, y), QPointF(float(self.width()), y))

    def _grid_world_spacing(self) -> int:
        return adaptive_world_grid_spacing(self.top_view.scale_value)

    def _draw_reference(self, painter: QPainter) -> None:
        """绘制 reference 画面元素。注意：只做渲染，不修改仿真状态。"""
        routes = self._route_segments()
        if not routes:
            return
        pen = QPen(self.theme.route, 2)
        pen.setDashPattern([7, 6])
        painter.setPen(pen)
        for route in routes:
            painter.drawLine(
                QPointF(self._map_x(route.start_x), self._map_y(route.start_altitude)),
                QPointF(self._map_x(route.end_x), self._map_y(route.end_altitude)),
            )

    def _route_segments(self) -> list[ReferenceRoute]:
        """返回需要绘制的航段列表。注意：优先使用多航段快照，缺省时退回当前航段。"""
        if self.snapshot is None:
            return []
        if self.snapshot.route_segments:
            return self.snapshot.route_segments
        if self.snapshot.route is not None:
            return [self.snapshot.route]
        return []

    def _draw_trails(self, painter: QPainter, snapshot: Snapshot) -> None:
        """绘制 trails 画面元素。注意：只做渲染，不修改仿真状态。"""
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
        """绘制 nodes 画面元素。注意：只做渲染，不修改仿真状态。"""
        for index, node in enumerate(snapshot.nodes):
            x = self._map_x(node.x)
            if x < -24 or x > self.width() + 24:
                continue
            color = self.theme.warn if node.health != "normal" else self.theme.leader if index == 0 else self.theme.wingman
            y = self._map_y(node.altitude)
            painter.setBrush(color)
            painter.setPen(QPen(self.theme.panel, 2))
            painter.drawEllipse(QPointF(x, y), 8, 8)
            painter.setPen(self.theme.ink)
            painter.drawText(QPointF(x + 10, y + 4), node.node_id)


class LogDialog(QDialog):
    """仿真事件弹窗。注意：只展示日志文本。"""

    def __init__(self, parent: QWidget | None = None) -> None:
        """初始化 LogDialog 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
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
        """追加一条显示内容。注意：超出容量时需要裁剪旧记录。"""
        self.text.append(f"{time_value:05.1f}s  {source:<10} {message}")


class StageFullscreenDialog(QDialog):
    """只用于全屏实时显示区的顶层外壳。注意：退出时需归还原控件。"""

    def __init__(self, owner: "MainWindow") -> None:
        """初始化 StageFullscreenDialog 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        super().__init__(owner)
        self.owner = owner
        self.setWindowTitle("二维实时显示")
        self.setWindowFlag(Qt.WindowType.FramelessWindowHint, True)

    def keyPressEvent(self, event) -> None:  # type: ignore[override]
        """处理键盘事件。注意：快捷键只影响窗口交互状态。"""
        if event.key() == Qt.Key.Key_Escape:
            self.owner._exit_stage_fullscreen()
            return
        super().keyPressEvent(event)

    def closeEvent(self, event) -> None:  # type: ignore[override]
        """处理窗口关闭事件。注意：关闭前需要释放控制器资源。"""
        if self.owner._stage_fullscreen_dialog is self:
            self.owner._exit_stage_fullscreen()
        event.accept()


class MainWindow(QMainWindow):
    """PySide6 主界面外壳。注意：负责组装控件并绑定控制器操作。"""

    def __init__(
        self,
        *,
        project_root: Path | str | None = None,
        config_state_path: Path | str | None = None,
        auto_load_config: bool = True,
    ) -> None:
        """初始化 MainWindow 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        super().__init__()
        self.project_root = Path(project_root).resolve() if project_root is not None else default_project_root()
        if config_state_path is None:
            self.config_state_path = self.project_root / APP_CONFIG_FILE_NAME
        else:
            state_path = Path(config_state_path)
            self.config_state_path = state_path if state_path.is_absolute() else self.project_root / state_path
        self.setWindowTitle("编队仿真")
        self.resize(1440, 900)
        self.setMinimumSize(1280, 780)
        self.sim = ControllerSimulationAdapter()
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
        self.disturbance_buttons: list[QPushButton] = []
        self._build_ui()
        self._install_button_cursors()
        self._apply_theme()
        self._update_snapshot(self.sim.snapshot())
        self._log("SimControl", "初始化界面，等待加载配置")
        if auto_load_config:
            self._load_last_config_from_state()

    def _build_ui(self) -> None:
        """构建主窗口全部 UI 区域。注意：控件引用需保存供后续事件更新使用。"""
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
        """构建顶部工具栏。注意：按钮和状态标签需要绑定到窗口槽函数。"""
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
        """构建左侧日志和配置面板。注意：面板宽度不能挤压主画布。"""
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
        self.config_name.setWordWrap(True)
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
            self.disturbance_buttons.append(button)
            grid.addWidget(button, index // 2, index % 2)
        layout.addWidget(disturb_group)
        layout.addStretch(1)
        return panel

    def _build_stage(self) -> QWidget:
        """构建中央仿真画布区域。注意：俯视图和侧视图需要共享横向视野。"""
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
        """构建右侧状态表区域。注意：列宽需避免出现横向滚动条。"""
        panel = QFrame()
        panel.setObjectName("panel")
        panel.setFixedWidth(400)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(10, 12, 10, 10)
        layout.setSpacing(8)
        self.node_table = QTableWidget(0, 6)
        self.node_table.setHorizontalHeaderLabels(["ID", "侧偏(m)", "待飞距(m)", "高度(m)", "速度(m/s)", "状态"])
        self.link_table = QTableWidget(0, 5)
        self.link_table.setHorizontalHeaderLabels(["链路", "方向", "延迟", "丢包", "状态"])
        self._configure_table(self.node_table, [48, 58, 74, 58, 76, 50])
        self._configure_table(self.link_table, [86, 52, 58, 50, 54])
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
        """配置状态表通用样式。注意：表格只读且不显示多余行号。"""
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
        """为按钮安装手型光标。注意：只影响交互提示，不改变按钮逻辑。"""
        for button in self.findChildren(QPushButton):
            button.setCursor(Qt.CursorShape.PointingHandCursor)

    def _apply_theme(self) -> None:
        """应用 theme 设置。注意：只修改对应显示或运行参数。"""
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
        """更新 snapshot 状态。注意：保持界面显示和内部数据一致。"""
        self.run_state_label.setText(snapshot.run_state)
        self.report_label.setText(f"回报：{snapshot.control_report}")
        self.step_label.setText(f"步长：{snapshot.step:.3f}s")
        self.timeline_label.setText(f"{snapshot.time:.1f} / {snapshot.duration:.0f}s")
        self.progress.setValue(round(snapshot.time / snapshot.duration * 1000) if snapshot.duration else 0)
        config_loaded = snapshot.run_state != "UNLOADED"
        self.start_button.setEnabled(config_loaded and snapshot.run_state != "FINISHED")
        self.pause_button.setEnabled(snapshot.run_state in {"RUNNING", "PAUSED"})
        self.step_button.setEnabled(snapshot.run_state in {"READY", "PAUSED"})
        self.reset_button.setEnabled(config_loaded)
        for button in self.disturbance_buttons:
            button.setEnabled(config_loaded and snapshot.run_state != "FINISHED")
        self.start_button.setText("继续" if snapshot.run_state == "PAUSED" else "开始")
        self.top_view.set_snapshot(snapshot)
        self.side_view.set_snapshot(snapshot)
        self._update_tables(snapshot)

    def _update_tables(self, snapshot: Snapshot) -> None:
        """更新 tables 状态。注意：保持界面显示和内部数据一致。"""
        self.node_table.setRowCount(len(snapshot.nodes))
        for row, node in enumerate(snapshot.nodes):
            speed = math.hypot(node.vx, node.vy)
            side_offset = node.cross_track_error
            if side_offset is None:
                side_offset = (node.y - WORLD_HEIGHT / 2) * 0.8
            distance_to_go = node.distance_to_go
            if distance_to_go is None:
                distance_to_go = max(0.0, (WORLD_WIDTH - node.x) * 4)
            status = {"normal": "正常", "degraded": "降级", "fault": "故障", "lost": "失联"}.get(node.health, node.health)
            values = [node.node_id, f"{side_offset:.0f}", f"{distance_to_go:.0f}", f"{node.altitude:.0f}", f"{speed:.1f}", status]
            for column, value in enumerate(values):
                self.node_table.setItem(row, column, QTableWidgetItem(value))

        self.link_table.setRowCount(len(snapshot.links))
        for row, link in enumerate(snapshot.links):
            values = [
                f"{link.source}-{link.target}",
                link_direction_label(link.direction),
                f"{link.latency_ms}ms",
                f"{link.loss * 100:.0f}%",
                "正常" if link.ok else "丢包",
            ]
            for column, value in enumerate(values):
                self.link_table.setItem(row, column, QTableWidgetItem(value))

    def _start(self) -> None:
        """响应开始按钮并启动仿真。注意：需要同步按钮状态和日志。"""
        snapshot = self.sim.start()
        self._update_snapshot(snapshot)
        if self.sim.last_result_code == "OK":
            self.timer.start()
        self._log("UI", f"start -> {self.sim.last_result_code}, state={snapshot.run_state}")

    def _pause(self) -> None:
        """响应暂停按钮并切换暂停状态。注意：暂停不清空当前快照。"""
        snapshot = self.sim.pause()
        if snapshot.run_state == "PAUSED":
            self.timer.stop()
        elif snapshot.run_state == "RUNNING":
            self.timer.start()
        self._update_snapshot(snapshot)
        self._log("UI", f"pause/start -> {self.sim.last_result_code}, state={snapshot.run_state}")

    def _step(self) -> None:
        """响应单步按钮并推进一拍。注意：单步后界面需要立即刷新。"""
        self.timer.stop()
        snapshot = self.sim.single_step()
        self._update_snapshot(snapshot)
        self._log("UI", f"step -> {self.sim.last_result_code}, state={snapshot.run_state}")

    def _reset(self) -> None:
        """响应重置按钮并恢复初始状态。注意：保留当前配置路径。"""
        self.timer.stop()
        snapshot = self.sim.reset()
        self._update_snapshot(snapshot)
        self._log("SimControl", f"reset -> {self.sim.last_result_code}, state={snapshot.run_state}")

    def _on_tick(self) -> None:
        """处理 tick 信号回调。注意：回调内避免耗时操作阻塞界面。"""
        snapshot = self.sim.poll()
        self._update_snapshot(snapshot)
        if snapshot.run_state in {"READY", "PAUSED", "FINISHED"}:
            self.timer.stop()

    def _inject_disturbance(self, kind: str) -> None:
        """响应扰动按钮并下发扰动命令。注意：失败时需要记录控制器返回信息。"""
        messages = {
            "wind": "注入风场脉冲",
            "fault": "注入 A02 控制效率下降",
            "loss": "注入链路丢包",
            "clear": "清除运行期扰动",
        }
        snapshot = self.sim.inject_disturbance(kind)
        self._update_snapshot(snapshot)
        self._log("Disturb", f"{messages[kind]} -> {self.sim.last_result_code}, state={snapshot.run_state}")

    def _choose_config(self) -> None:
        """处理 config 选择流程。注意：用户取消时不改变当前配置。"""
        path, _ = QFileDialog.getOpenFileName(
            self,
            "选择配置文件",
            str(self._config_dialog_start_dir()),
            "Config (*.yaml *.yml *.json)",
        )
        if not path:
            return
        self._apply_config_path(path)

    def _apply_config_path(self, path: str, *, remember: bool = True) -> None:
        """应用 config path 设置。注意：只修改对应显示或运行参数。"""
        self.timer.stop()
        self._update_snapshot(self.sim.load_config(path))
        if self.sim.last_result_code == "OK":
            display_path = self._display_config_path(Path(path))
            self.config_name.setText(display_path)
            self.config_name.setToolTip(display_path)
            self._log("Config", f"加载配置文件 {display_path}")
            if remember:
                self._save_last_config_path(Path(path))
        else:
            self._log("WARN", f"加载配置失败 {Path(path).name}: {self.sim.last_result_message}")

    def _config_dialog_start_dir(self) -> Path:
        """处理 dialog start dir 配置路径。注意：兼容源码运行和打包运行路径。"""
        relative_path = self._read_last_config_path()
        if relative_path is None:
            return self.project_root
        config_path = (self.project_root / relative_path).resolve()
        candidate = config_path.parent
        return candidate if candidate.exists() else self.project_root

    def _display_config_path(self, path: Path) -> str:
        """生成 config path 显示文本。注意：仅用于界面展示。"""
        relative_path = self._relative_to_project_root(path)
        return relative_path if relative_path is not None else path.name

    def _load_last_config_from_state(self) -> None:
        """加载上次使用的配置路径。注意：路径不存在时回退到默认配置。"""
        relative_path = self._read_last_config_path()
        if relative_path is None:
            return
        config_path = (self.project_root / relative_path).resolve()
        if not config_path.exists():
            self._log("WARN", f"config.ini 指向的配置不存在：{relative_path}")
            return
        self._apply_config_path(str(config_path), remember=False)

    def _read_last_config_path(self) -> str | None:
        """读取 last config path 数据。注意：缺省或失败时应使用安全兜底。"""
        if not self.config_state_path.exists():
            return None
        parser = ConfigParser()
        try:
            parser.read(self.config_state_path, encoding="utf-8")
        except OSError as exc:
            self._log("WARN", f"读取 config.ini 失败：{exc}")
            return None
        value = parser.get(APP_CONFIG_SECTION, APP_CONFIG_KEY_LAST_CONFIG, fallback="").strip()
        return value or None

    def _save_last_config_path(self, path: Path) -> None:
        """保存 last config path 数据。注意：写入失败不应影响主仿真流程。"""
        relative_path = self._relative_to_project_root(path)
        if relative_path is None:
            self._log("WARN", "配置路径无法相对到程序目录，未更新 config.ini")
            return
        parser = ConfigParser()
        parser[APP_CONFIG_SECTION] = {APP_CONFIG_KEY_LAST_CONFIG: relative_path}
        self.config_state_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            with self.config_state_path.open("w", encoding="utf-8") as handle:
                parser.write(handle)
        except OSError as exc:
            self._log("WARN", f"写入 config.ini 失败：{exc}")

    def _relative_to_project_root(self, path: Path) -> str | None:
        """计算 to project root 相对路径。注意：路径不可相对化时返回原始路径。"""
        try:
            relative_path = os.path.relpath(path.resolve(), self.project_root)
        except ValueError:
            return None
        if os.path.isabs(relative_path):
            return None
        try:
            return Path(relative_path).as_posix()
        except ValueError:
            return None

    def _on_speed_changed(self, value: int) -> None:
        """处理 speed changed 信号回调。注意：回调内避免耗时操作阻塞界面。"""
        speed = value / 10.0
        self.sim.set_speed(speed)
        self.speed_label.setText(f"{speed:.1f}x")

    def _on_theme_changed(self) -> None:
        """处理 theme changed 信号回调。注意：回调内避免耗时操作阻塞界面。"""
        self.theme_key = self.theme_select.currentData()
        self.theme = THEMES[self.theme_key]
        self._apply_theme()
        self._log("UI", f"切换主题：{self.theme_select.currentText()}")

    def _on_auto_center_changed(self) -> None:
        """处理 auto center changed 信号回调。注意：回调内避免耗时操作阻塞界面。"""
        self.top_view.auto_center = self.auto_center.isChecked()
        self.top_view.set_snapshot(self.sim.snapshot())

    def _on_grid_changed(self) -> None:
        """处理 grid changed 信号回调。注意：回调内避免耗时操作阻塞界面。"""
        show_grid = self.grid_toggle.isChecked()
        self.top_view.show_grid = show_grid
        self.side_view.show_grid = show_grid
        self.top_view.viewport().update()
        self.side_view.update()

    def _disable_auto_center(self) -> None:
        """关闭自动居中选项。注意：用户手动平移或缩放后应避免自动抢回视图。"""
        if self.auto_center.isChecked():
            self.auto_center.setChecked(False)

    def _reset_view(self) -> None:
        """响应重置视图按钮。注意：同时重置俯视图和侧视图显示范围。"""
        self.top_view.reset_view()
        self.side_view.update()

    def _toggle_fullscreen(self) -> None:
        """切换仿真画布全屏状态。注意：需要保存并恢复原布局。"""
        if self._stage_fullscreen_dialog is not None:
            self._exit_stage_fullscreen()
        else:
            self._enter_stage_fullscreen()

    def _enter_stage_fullscreen(self) -> None:
        """进入 stage fullscreen 模式。注意：需要保存退出时恢复的界面状态。"""
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
        """退出 stage fullscreen 模式。注意：需要恢复进入前的布局状态。"""
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
        """设置 fullscreen button state 状态。注意：保持控件状态和内部标志同步。"""
        if self.fullscreen_button is None:
            return
        self.fullscreen_button.setText("↙" if active else "⛶")
        self.fullscreen_button.setToolTip("退出全屏" if active else "全屏显示")
        self.fullscreen_button.setAccessibleName("退出全屏" if active else "全屏显示")

    def _log(self, source: str, message: str) -> None:
        """追加一条界面日志。注意：日志容量由日志面板负责裁剪。"""
        self.log_dialog.append(self.sim.time, source, message)

    def closeEvent(self, event) -> None:  # type: ignore[override]
        """处理窗口关闭事件。注意：关闭前需要释放控制器资源。"""
        self.timer.stop()
        self.sim.close()
        super().closeEvent(event)


def run_gui(argv: list[str] | None = None) -> int:
    """启动 PySide6 GUI。注意：一个进程只能持有一个 QApplication 主循环。"""

    app = QApplication(argv or [])
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(run_gui())
