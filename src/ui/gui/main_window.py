"""编队仿真 PySide6 主窗口。注意：正式 GUI 入口在本模块。"""

from __future__ import annotations

from collections.abc import Callable
from configparser import ConfigParser
import json
import math
import os
import sys
from dataclasses import dataclass, field
from pathlib import Path


# 该模块既支持 `python -m src.ui.gui.main_window`，也支持 IDE 直接运行文件。
# 直接运行文件时，解释器默认只把 src/ui/gui 加入 sys.path。
# 下面的自修正只处理这种入口差异，不改变正式包结构。
# 继续保留绝对导入，便于测试、打包和跨模块重构时保持一致。
# 不依赖 VS Code 的 PYTHONPATH，是为了兼容终端、Run Code 和调试器差异。
# 不在 src/main.py 中转发，是因为当前正式 GUI 入口已经在本模块底部。
# 路径注入发生在 PySide6 导入之后也可以，但放在 src 包导入之前更直观。
# 若后续改为安装型包运行，这段逻辑会因 __package__ 非空自动跳过。
# 这里不创建 QApplication，避免单元测试加载模块时意外进入 GUI 生命周期。
def _ensure_project_root_on_path() -> None:
    """确保直接执行本文件时也能导入项目包。注意：包导入场景不修改 sys.path。"""

    # 包方式启动时 Python 已经知道顶层包，避免重复改写导入搜索路径。
    if __package__:
        return

    # 本文件位于 src/ui/gui/main_window.py，向上三层正好回到项目根目录。
    project_root = Path(__file__).resolve().parents[3]
    # 只有确实看到 src 目录时才注入，避免误把无关父目录加入 sys.path。
    if (project_root / "src").is_dir() and str(project_root) not in sys.path:
        # VS Code 直接调试当前文件时只会加入脚本目录，这里补入项目根目录。
        sys.path.insert(0, str(project_root))


_ensure_project_root_on_path()

from PySide6.QtCore import QPoint, QPointF, QRectF, QSignalBlocker, Qt, QTimer, Signal
from PySide6.QtGui import QAction, QActionGroup, QColor, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import (
    QAbstractItemView,
    QAbstractScrollArea,
    QAbstractSpinBox,
    QApplication,
    QCheckBox,
    QDialog,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QFrame,
    QGraphicsView,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMenu,
    QProgressBar,
    QPushButton,
    QHeaderView,
    QSizePolicy,
    QSlider,
    QSplitter,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from src.algorithm.context.leaf_types import WayLineS, WayPointInputS, to_display_inputs
from src.algorithm.units.algo.arc_path import arc_radius as _arc_radius_fn, arc_swept_rad
from src.algorithm.entity.leader_follower_hold.leader import waypoint_inputs_to_waylines
from src.algorithm.units.process.tra_plan.avoidance.obstacle import ObstacleS, make_circle, make_rect
from src.algorithm.units.process.tra_plan.avoidance.planner import plan_avoidance_route
from src.runner.sim_control import SimulationController
from src.runner.sim_control import SimulationSnapshot as ControllerSnapshot


# 世界坐标范围（米）：用于 mock 数据居中、待飞距/侧偏兜底估算等。
WORLD_WIDTH = 1600.0
WORLD_HEIGHT = 520.0
# 尾迹保留时长（秒）：超过该时间的轨迹采样点会被丢弃，控制内存与视觉长度。
TRAIL_SECONDS = 18.0
# 俯视图初始平移留白，避免场景紧贴左上角边缘。
TOP_VIEW_ORIGIN_MARGIN = 40.0
# 视图缩放上下限，防止缩到看不见或放大到失真。
VIEW_MIN_SCALE = 0.05
VIEW_MAX_SCALE = 3.5
# 自适应铺满时只占用视口 80%，四周留出可视边距。
FIT_VIEWPORT_RATIO = 0.80
# 网格基准间距（世界坐标）与其在屏幕上允许的疏密区间，配合自适应算法使用。
WORLD_GRID_SPACING = 48
GRID_MIN_SCREEN_SPACING = 36.0
GRID_MAX_SCREEN_SPACING = 96.0
# config.ini 中记忆“上次加载配置”用的小节名、键名与文件名。
APP_CONFIG_SECTION = "config"
APP_CONFIG_KEY_LAST_CONFIG = "last_config"
APP_CONFIG_FILE_NAME = "config.ini"


def adaptive_world_grid_spacing(scale_value: float) -> int:
    """按当前缩放自适应网格间距（世界坐标），保证屏幕上网格疏密适中。

    参数 scale_value 为视图缩放倍率；返回值是网格在世界坐标中的间距（像素/米）。
    """

    spacing = WORLD_GRID_SPACING
    # 缩放为 0 会导致除零/无限循环，这里做下限保护。
    safe_scale = max(scale_value, 0.001)
    # 缩得太小时网格在屏幕上过密：成倍放大间距直到屏幕间距不低于下限。
    while spacing * safe_scale < GRID_MIN_SCREEN_SPACING:
        spacing *= 2
    # 放得太大时网格在屏幕上过疏：成倍缩小间距直到不超过上限（保留 ≥1 防退化）。
    while spacing > 1 and spacing * safe_scale > GRID_MAX_SCREEN_SPACING:
        spacing = max(1, spacing // 2)
    return spacing


def default_project_root() -> Path:
    """返回项目根目录。注意：打包后路径和源码运行路径不同。"""

    # PyInstaller 等打包后 sys.frozen 为真，根目录就是可执行文件所在目录。
    if getattr(sys, "frozen", False):
        return Path(sys.executable).resolve().parent

    # 源码运行：__file__ 向上 3 级即仓库根；再结合 cwd 一起作为候选。
    source_root = Path(__file__).resolve().parents[3]
    cwd = Path.cwd().resolve()
    # 依次探测候选目录，谁含有 configs/ 子目录就认定它是项目根。
    for candidate in (cwd, cwd.parent, source_root):
        if (candidate / "configs").exists():
            return candidate
    # 全部落空时退回当前工作目录，保证函数总有返回值。
    return cwd


@dataclass
class TrailPoint:
    """仿真时间中的一个轨迹采样点。注意：用于绘制历史尾迹。"""

    x: float  # 采样时刻的世界 east 坐标
    y: float  # 采样时刻的世界 north 坐标
    altitude: float  # 采样时刻高度
    time: float  # 采样仿真时刻，用于按 TRAIL_SECONDS 老化淡出


@dataclass
class NodeState:
    """单个飞机节点的显示状态。注意：字段用于 GUI 绘图和表格。"""

    node_id: str
    role: str
    x: float
    y: float
    vx: float  # 横向速度，用于在俯视图里旋转机头朝向
    vy: float  # 纵向速度
    altitude: float = 1200.0  # 高度，仅侧视图使用
    vertical_speed: float = 0.0  # 天向速度，供整体跟踪表显示
    health: str = "normal"  # 健康枚举：normal/degraded/fault/lost
    trail: list[TrailPoint] = field(default_factory=list)  # 历史尾迹采样
    cross_track_error: float | None = None  # 侧偏，None 时由表格兜底估算
    distance_to_go: float | None = None  # 待飞距，None 时由表格兜底估算
    track_pos_err_x: float = 0.0  # 航迹系前向位置误差
    track_pos_err_y: float = 0.0  # 航迹系垂向位置误差
    track_pos_err_z: float = 0.0  # 航迹系侧向位置误差
    cmd_pos_x: float = 0.0  # 当前目标位置 east（槽位/M_i）
    cmd_pos_y: float = 0.0  # 当前目标位置 north（槽位/M_i）
    rally_phase: str = ""   # 集结阶段，如 JOINING/FLYING、CATCHUP、HOLD


@dataclass
class LinkState:
    """单条通信链路的显示状态。注意：loss 为 0 到 1 的比例。"""

    source: str  # 源节点 id
    target: str  # 目标节点 id
    direction: str  # duplex/simplex
    latency_ms: int  # 延迟（毫秒）
    loss: float  # 丢包率 0..1
    ok: bool = True  # 是否正常（丢包过高则置 False）


@dataclass
class ReferenceRoute:
    """俯视图和侧视图共用的参考航段。注意：坐标单位为米。"""

    start_x: float  # 航段起点 east 坐标
    start_y: float  # 航段起点 north 坐标
    start_altitude: float  # 航段起点高度（侧视图用）
    end_x: float
    end_y: float
    end_altitude: float
    radius: float = 0.0  # 圆弧半径；>0 表示本段为圆弧（俯视图按弧采样）
    center_x: float = 0.0  # 圆弧圆心 east（radius>0 有意义）
    center_y: float = 0.0  # 圆弧圆心 north
    turn_sign: float = 0.0  # 转向：+1 左转/逆时针、-1 右转/顺时针


def reference_route_points(route: ReferenceRoute, step_deg: float = 6.0) -> list[tuple[float, float]]:
    """把单个参考航段展开为世界坐标折线点：直线段返回两端点，圆弧段按 step_deg 采样。

    与预览 route_to_polyline 的采样口径一致，使 committed 航线与预览对同一圆弧画法相同。
    """
    if route.radius <= 0.0 or route.turn_sign == 0.0:
        return [(route.start_x, route.start_y), (route.end_x, route.end_y)]
    a_start = math.atan2(route.start_y - route.center_y, route.start_x - route.center_x)
    a_end = math.atan2(route.end_y - route.center_y, route.end_x - route.center_x)
    delta = math.atan2(math.sin(a_end - a_start), math.cos(a_end - a_start))  # wrap 到 (-pi,pi]
    # 取与 turn_sign 同向的扫掠；wrap 后符号相反则补一圈（与 arc_path.arc_swept_rad 同口径）。
    if route.turn_sign >= 0.0 and delta < 0.0:
        delta += 2.0 * math.pi
    elif route.turn_sign < 0.0 and delta > 0.0:
        delta -= 2.0 * math.pi
    segments = max(1, int(abs(math.degrees(delta)) / step_deg))
    return [
        (
            route.center_x + route.radius * math.cos(a_start + delta * (k / segments)),
            route.center_y + route.radius * math.sin(a_start + delta * (k / segments)),
        )
        for k in range(segments + 1)
    ]


@dataclass
class ObstacleView:
    """俯视图显示用的二维障碍（无限高柱体）。注意：当前仅供 UI 显示与勾选，规划后端后续接入。"""

    obstacle_id: str  # 障碍唯一标识，列表显示/勾选用
    kind: str  # "circle" | "rect"
    enabled: bool = True  # 是否启用（参与避障）
    center_x: float = 0.0  # 圆心 east（kind=circle）
    center_y: float = 0.0  # 圆心 north
    radius: float = 0.0  # 半径，米（kind=circle）
    min_x: float = 0.0  # 矩形 east 下界（kind=rect）
    min_y: float = 0.0  # 矩形 north 下界
    max_x: float = 0.0  # 矩形 east 上界
    max_y: float = 0.0  # 矩形 north 上界

    def label(self) -> str:
        """生成左面板勾选列表的显示文本。注意：仅用于界面展示。"""
        if self.kind == "rect":
            return f"{self.obstacle_id}  矩形 ({self.min_x:.0f},{self.min_y:.0f})-({self.max_x:.0f},{self.max_y:.0f})"
        return f"{self.obstacle_id}  圆 ({self.center_x:.0f},{self.center_y:.0f}) r{self.radius:.0f}"


def _safe_float(value: object, default: float = 0.0) -> float:
    """把任意配置值安全转成 float。注意：非法值（如字符串）返回默认值，避免 UI-only 字段拖垮加载流程。"""
    try:
        return float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return default


def parse_avoidance_config(path: str) -> tuple[list[ObstacleView], float]:
    """从配置 JSON 解析 avoidance 障碍与膨胀间距，供 UI 显示。

    注意：仅读取、不校验飞行约束。本函数为“安全解析”——任何缺失/非法字段都退化为默认值或被跳过，
    绝不抛异常拖垮 _apply_config_path（该流程在控制器加载成功后调用）。
    约定与文档 §4.2 一致：顶层 enabled=false 或 obstacles 为空 → 完全跳过避障，返回空。
    """
    try:
        with open(path, "r", encoding="utf-8") as handle:
            data = json.load(handle)
    except (OSError, ValueError):
        # 非 JSON（如 YAML）或读取失败时不影响主流程，返回空障碍集。
        return [], 0.0
    avoidance = data.get("avoidance") if isinstance(data, dict) else None
    if not isinstance(avoidance, dict):
        return [], 0.0
    # 顶层总开关：显式关闭即完全跳过避障，等价于现状（不显示、不绘制、不可生成）。
    if not avoidance.get("enabled", True):
        return [], 0.0
    clearance = _safe_float(avoidance.get("clearance_m", 0.0))
    obstacles: list[ObstacleView] = []
    raw_obstacles = avoidance.get("obstacles", [])
    if not isinstance(raw_obstacles, list):
        return [], clearance
    for index, raw in enumerate(raw_obstacles):
        if not isinstance(raw, dict):
            continue
        obstacle_id = str(raw.get("id", f"OB{index + 1}"))
        enabled = bool(raw.get("enabled", True))
        if str(raw.get("type", "circle")) == "rect":
            lo = raw.get("min", {})
            hi = raw.get("max", {})
            lo = lo if isinstance(lo, dict) else {}
            hi = hi if isinstance(hi, dict) else {}
            obstacles.append(
                ObstacleView(
                    obstacle_id=obstacle_id,
                    kind="rect",
                    enabled=enabled,
                    min_x=_safe_float(lo.get("east_m", 0.0)),
                    min_y=_safe_float(lo.get("north_m", 0.0)),
                    max_x=_safe_float(hi.get("east_m", 0.0)),
                    max_y=_safe_float(hi.get("north_m", 0.0)),
                )
            )
        else:
            center = raw.get("center", {})
            center = center if isinstance(center, dict) else {}
            obstacles.append(
                ObstacleView(
                    obstacle_id=obstacle_id,
                    kind="circle",
                    enabled=enabled,
                    center_x=_safe_float(center.get("east_m", 0.0)),
                    center_y=_safe_float(center.get("north_m", 0.0)),
                    radius=_safe_float(raw.get("radius_m", 0.0)),
                )
            )
    return obstacles, clearance


@dataclass
class AvoidanceParams:
    """避障规划参数与长机原航线，从配置 JSON 解析，供 plan_avoidance_route 调用。"""

    turn_radius_m: float = 0.0
    leg_margin_m: float = 0.0
    clearance_m: float = 0.0
    simplify_clearance_m: float = 0.0
    simplify_clearance_explicit: bool = False
    turn_switch_penalty_m: float = 0.0
    turn_angle_weight_m: float = 0.0
    resolution_m: float = 10.0
    margin_m: float = 0.0
    speed_mps: float = 0.0
    allow_arc: bool = True  # 航段自身是否可为曲线：开启则折叠贴障大弧并把拐点烘焙成圆弧段；关闭只留直线骨架+交接圆弧
    waypoints: list[tuple[float, float, float]] = field(default_factory=list)  # (east, north, altitude)


class AvoidanceWindow(QDialog):
    """避障规划子窗口。注意：控件由 MainWindow 填充，本类只固化窗口元数据。"""

    param_order = [
        # 1. 先锁定飞机物理转弯能力。
        "turn_radius_m",
        # 2. 再确认圆弧之间保留的直线余度。
        "leg_length_margin_m",
        # 3. 安全边界优先于搜索质量参数。
        "clearance_m",
        # 4. 栅格精度决定 A* 离散程度。
        "grid.resolution_m",
        # 5. 搜索包围盒大小影响能否绕开障碍。
        "grid.margin_m",
        # 6. 去冗余参数只在安全约束稳定后调整。
        "simplify_clearance_m",
        # 7. 方向切换惩罚用于减少碎段。
        "turn_switch_penalty_m",
        # 8. 航迹角惩罚最后再微调硬拐。
        "turn_angle_weight_m",
    ]

    def __init__(self, parent: QWidget | None = None) -> None:
        """初始化避障规划窗口。注意：非模态窗口，便于对照主画布预览。"""
        super().__init__(parent)
        self.setWindowTitle("避障规划")
        self.setModal(False)
        self.setMinimumSize(820, 560)


def parse_avoidance_params(path: str) -> AvoidanceParams | None:
    """从配置 JSON 解析避障规划参数与长机航点。注意：安全解析；缺 avoidance/航点不足时返回 None。"""
    try:
        with open(path, "r", encoding="utf-8") as handle:
            data = json.load(handle)
    except (OSError, ValueError):
        return None
    if not isinstance(data, dict):
        return None
    avoidance = data.get("avoidance")
    if not isinstance(avoidance, dict) or not avoidance.get("enabled", True):
        return None
    grid = avoidance.get("grid") if isinstance(avoidance.get("grid"), dict) else {}
    route = data.get("route") if isinstance(data.get("route"), dict) else {}
    raw_waypoints = route.get("waypoints", []) if isinstance(route, dict) else []
    waypoints: list[tuple[float, float, float]] = []
    if isinstance(raw_waypoints, list):
        for raw in raw_waypoints:
            if isinstance(raw, dict):
                # 与控制器 _route_point_from_config 一致：兼容 x_m/east、y_m/north、altitude_m/h 两套字段名。
                waypoints.append(
                    (
                        _safe_float(raw.get("x_m", raw.get("east", 0.0))),
                        _safe_float(raw.get("y_m", raw.get("north", 0.0))),
                        _safe_float(raw.get("altitude_m", raw.get("h", 0.0))),
                    )
                )
    if len(waypoints) < 2:
        return None
    clearance_m = _safe_float(avoidance.get("clearance_m", 0.0))
    simplify_clearance_explicit = "simplify_clearance_m" in avoidance
    simplify_clearance_m = _safe_float(
        avoidance.get("simplify_clearance_m", clearance_m)
    )
    return AvoidanceParams(
        turn_radius_m=_safe_float(avoidance.get("turn_radius_m", 0.0)),
        leg_margin_m=_safe_float(avoidance.get("leg_length_margin_m", 0.0)),
        clearance_m=clearance_m,
        simplify_clearance_m=simplify_clearance_m,
        simplify_clearance_explicit=simplify_clearance_explicit,
        turn_switch_penalty_m=_safe_float(avoidance.get("turn_switch_penalty_m", 0.0)),
        turn_angle_weight_m=_safe_float(avoidance.get("turn_angle_weight_m", 0.0)),
        resolution_m=_safe_float(grid.get("resolution_m", 10.0)) if isinstance(grid, dict) else 10.0,
        margin_m=_safe_float(grid.get("margin_m", 0.0)) if isinstance(grid, dict) else 0.0,
        speed_mps=_safe_float(route.get("speed_mps", 0.0)) if isinstance(route, dict) else 0.0,
        allow_arc=bool(avoidance.get("allow_arc", True)),
        waypoints=waypoints,
    )


def _obstacle_view_to_backend(view: "ObstacleView") -> ObstacleS:
    """把 UI 障碍转成后端 ObstacleS（供规划调用）。"""
    if view.kind == "rect":
        return make_rect(view.obstacle_id, view.min_x, view.min_y, view.max_x, view.max_y)
    return make_circle(view.obstacle_id, view.center_x, view.center_y, view.radius)


def _sample_wayline_arc(line: WayLineS, step_deg: float = 6.0) -> list[tuple[float, float]]:
    """采样圆弧航段为折线点（含两端）。注意：复用 arc_swept_rad 求扫掠角。"""
    center = line.start.center
    radius = _arc_radius_fn(line)
    a_start = math.atan2(line.start.pos.north - center.north, line.start.pos.east - center.east)
    swept = arc_swept_rad(line)
    segments = max(1, int(abs(math.degrees(swept)) / step_deg))
    return [
        (
            center.east + radius * math.cos(a_start + swept * (k / segments)),
            center.north + radius * math.sin(a_start + swept * (k / segments)),
        )
        for k in range(segments + 1)
    ]


def route_to_polyline(route: list[WayPointInputS]) -> list[tuple[float, float]]:
    """把 WayPointInputS 列表展开为折线点，供俯视图绘制预览航线。

    显示只画航段几何：去掉转弯信息（交接半径 r），直线航段画直线、曲率航段（turnSign）画弧。
    与配置航线/避障采用后的显示规则一致（见 leaf_types.to_display_inputs）。
    """
    if len(route) < 2:
        return []
    lines = waypoint_inputs_to_waylines(to_display_inputs(route))
    raw: list[tuple[float, float]] = []
    for line in lines:
        if line.start.turnSign != 0.0:
            raw.extend(_sample_wayline_arc(line))
        else:
            raw.append((line.start.pos.east, line.start.pos.north))
            raw.append((line.end.pos.east, line.end.pos.north))
    polyline: list[tuple[float, float]] = []
    for point in raw:
        if not polyline or math.hypot(point[0] - polyline[-1][0], point[1] - polyline[-1][1]) > 1e-6:
            polyline.append(point)
    return polyline


def preview_route_marker_points(route: list[WayPointInputS]) -> list[tuple[float, float]]:
    """把预览航线转换为航点标记坐标。注意：圆弧只标记端点，不标记中间采样点。"""
    if len(route) < 2:
        return []
    # 与预览折线一致先转 display inputs，避免把过渡半径 r 当成真实航点。
    lines = waypoint_inputs_to_waylines(to_display_inputs(route))
    if not lines:
        return []
    raw = [(lines[0].start.pos.east, lines[0].start.pos.north)]
    raw.extend((line.end.pos.east, line.end.pos.north) for line in lines)
    markers: list[tuple[float, float]] = []
    for point in raw:
        # 相邻航段共用端点只画一个黑点，避免连接处显得更粗。
        if not markers or math.hypot(point[0] - markers[-1][0], point[1] - markers[-1][1]) > 1e-6:
            markers.append(point)
    return markers


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
    cpu_utilization: float = 0.0


def is_leader_node(node: NodeState) -> bool:
    """判断节点是否为长机。注意：GUI 显示必须遵循控制器 role，而不是节点顺序。"""

    return node.role.strip().lower() in {"leader", "rally_leader"}


def leader_node_from(nodes: list[NodeState]) -> NodeState | None:
    """从节点列表中取长机。注意：缺少显式长机时回退首节点，保持旧配置可显示。"""

    return next((node for node in nodes if is_leader_node(node)), nodes[0] if nodes else None)


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
        # 时间归零并清空所有扰动相关计时器，回到“待命”初始态。
        self.time = 0.0
        self.running = False
        self.paused = False
        self.disturbance = "无"
        self.disturbance_until = 0.0
        self.fault_node = None
        self.loss_until = 0.0
        # 预置三机楔形编队：1 长机 + 2 僚机，坐标为演示用初值。
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
        # 进入“运行且暂停”态后只推进一拍，模拟逐帧调试。
        self.running = True
        self.paused = True
        self.advance()
        return self.snapshot()

    def inject_disturbance(self, kind: str) -> Snapshot:
        """向仿真注入扰动。注意：调用方需提供合法扰动类型和参数。"""
        # 各扰动设置“持续到 disturbance_until 时刻”的窗口，到期后自动恢复。
        if kind == "wind":
            self.disturbance = "风场"
            self.disturbance_until = self.time + 8.0
        elif kind == "fault":
            # 节点故障锁定 A02：advance 中对该节点用更弱的控制增益模拟失效。
            self.disturbance = "节点故障"
            self.fault_node = "A02"
            self.disturbance_until = self.time + 10.0
        elif kind == "loss":
            # 链路丢包额外维护 loss_until，供链路退化判断使用。
            self.disturbance = "链路丢包"
            self.loss_until = self.time + 12.0
            self.disturbance_until = self.time + 12.0
        elif kind == "clear":
            # 清除：复位所有扰动计时器与故障节点标记。
            self.disturbance = "无"
            self.disturbance_until = 0.0
            self.loss_until = 0.0
            self.fault_node = None
        return self.snapshot()

    def advance(self) -> Snapshot:
        """推进仿真显示或数据状态。注意：步长应与调用方传入时间一致。"""
        # 到达总时长则停机，不再推进时间。
        if self.time >= self.duration:
            self.running = False
            self.paused = False
            return self.snapshot()

        # 按播放倍率推进仿真时间，并夹在 duration 内防止越界。
        self.time = min(self.duration, self.time + self.step * self.speed)
        # 风场扰动时给出非零侧向风强度，否则为 0。
        wind = 1.8 if self.disturbance == "风场" else 0.0
        # 楔形队形相对长机的横/纵偏置：长机在前，两僚机在后两侧。
        formation = [(0.0, 0.0), (-54.0, 58.0), (-54.0, -58.0)]
        leader = self.nodes[0]

        for index, node in enumerate(self.nodes):
            _, dy = formation[index]
            # 长机沿正弦轨迹机动，僚机则跟随长机纵向位置加各自队形偏移。
            target_y = 238.0 + math.sin(self.time / 8.0) * 34.0 if index == 0 else leader.y + dy
            # 故障节点用更小的跟踪增益，表现为“跟不上、收敛慢”。
            gain = 0.012 if self.fault_node == node.node_id else 0.04
            node.vx = 4.8 + index * 0.12
            # 纵向速度向目标 y 收敛，并叠加随相位变化的风扰。
            node.vy += (target_y - node.y) * gain + wind * math.sin(self.time + index)
            # x 方向额外乘 3.2 让画面横向推进更明显（纯演示系数）。
            node.x += node.vx * self.step * self.speed * 3.2
            node.y += node.vy * self.step * self.speed
            # 飞出右边界后从左侧重新进入，并清空尾迹避免横贯屏幕的连线。
            if node.x > WORLD_WIDTH + 60.0:
                node.x = -30.0
                node.trail.clear()
            # 追加当前采样点并裁掉超过保留时长的旧点。
            node.trail.append(TrailPoint(node.x, node.y, node_altitude(index, self.time), self.time))
            node.trail = [point for point in node.trail if self.time - point.time <= TRAIL_SECONDS]

        # 扰动窗口到期后自动清除，恢复正常显示。
        if self.disturbance != "无" and self.time > self.disturbance_until:
            self.disturbance = "无"
            self.fault_node = None

        for index, link in enumerate(self.links):
            # 丢包扰动期内除第三条外的链路进入退化态（高丢包高延迟）。
            degraded = self.time < self.loss_until and index != 2
            link.loss = 0.26 + index * 0.05 if degraded else 0.01 + index * 0.006
            link.latency_ms = 76 + index * 8 if degraded else 18 + index * 5 + round(math.sin(self.time + index) * 3)
            # 丢包率超过 20% 视为链路异常。
            link.ok = link.loss < 0.2

        return self.snapshot()

    def snapshot(self) -> Snapshot:
        """返回当前快照。注意：返回数据用于显示，不应被调用方回写。"""
        # 运行状态与“控制回报”文案根据当前是否运行/暂停/扰动类型联合决定。
        if not self.running:
            run_state = "READY"
            report = "待命"
        elif self.paused:
            run_state = "PAUSED"
            report = "保持"
        elif self.disturbance == "风场":
            # 运行中且处于各类扰动时，给出对应的控制策略回报文案。
            run_state = "RUNNING"
            report = "抗风"
        elif self.disturbance == "节点故障":
            run_state = "RUNNING"
            report = "重构"
        elif self.disturbance == "链路丢包":
            run_state = "RUNNING"
            report = "保链"
        else:
            # 无扰动的正常运行：集结。
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
            cpu_utilization=0.0,
        )


class ControllerSimulationAdapter:
    """把 SimulationController 快照适配为现有 GUI 绘图模型。注意：需要维护尾迹缓存。"""

    def __init__(self) -> None:
        """初始化 ControllerSimulationAdapter 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        self.controller = SimulationController()
        self.speed = 1.0
        self.disturbance = "无"
        # 控制器只给瞬时位置，尾迹需由本适配器按 node_id 自行累积缓存。
        self._trail_by_node: dict[str, list[TrailPoint]] = {}
        # 记录上一帧位置与时间，用于差分估算速度（控制器速度字段不一定可靠）。
        self._last_xy_by_node: dict[str, tuple[float, float, float]] = {}
        # 已消费的事件数游标，避免重复处理历史扰动事件。
        self._processed_event_count = 0
        # 缓存最近一次控制器调用的返回码/消息，供 UI 记录日志与判断成败。
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
        # 仅在加载成功时重置缓存：清空旧尾迹/速度缓存，扰动复位为“无”。
        if result.code == "OK":
            self._trail_by_node.clear()
            self._last_xy_by_node.clear()
            self.speed = self.controller.playback_rate
            # 把事件游标推到当前末尾，避免把加载前的旧事件当成新扰动消费。
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
            # 控制器 reset 会按配置重建模块，需要把 UI 当前倍率重新下发给墙钟调度。
            self.controller.set_playback_rate(self.speed)
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
        # 注入成功后立即把本地显示标签同步为中文名，无需等事件回流。
        if result.code == "OK":
            self.disturbance = {
                "wind": "风场",
                "fault": "节点故障",
                "loss": "链路丢包",
                "clear": "无",
            }[kind]
        return self.snapshot()

    def apply_avoidance_route(self, route: list[WayPointInputS]) -> Snapshot:
        """采用一条避障规划航线，替换长机航线。注意：成功后清空尾迹缓存（航线已变）。"""
        result = self.controller.apply_avoidance_route(route)
        self.last_result_code = result.code
        self.last_result_message = result.message
        if result.code == "OK":
            self._trail_by_node.clear()
            self._last_xy_by_node.clear()
        return self.snapshot()

    def clear_avoidance_route(self) -> Snapshot:
        """清除避障航线覆盖，恢复配置原始长机航线。"""
        result = self.controller.clear_avoidance_route()
        self.last_result_code = result.code
        self.last_result_message = result.message
        if result.code == "OK":
            self._trail_by_node.clear()
            self._last_xy_by_node.clear()
        return self.snapshot()

    def set_speed(self, speed: float) -> None:
        """设置播放速度。注意：只影响界面或控制器调度倍率。"""
        # 记录倍率并下发给控制器调度（影响推进节奏，不影响本适配器换算）。
        self.speed = speed
        self.controller.set_playback_rate(speed)

    def set_duration(self, duration_s: float) -> Snapshot:
        """设置仿真总时长。注意：只改变停止边界，不改变步长。"""
        result = self.controller.set_duration(duration_s)
        self.last_result_code = result.code
        self.last_result_message = result.message
        return self.snapshot()

    def close(self) -> None:
        """释放 ControllerSimulationAdapter 持有的资源。注意：关闭后不应继续调用运行接口。"""
        self.controller.close()

    def _convert_snapshot(self, snapshot: ControllerSnapshot) -> Snapshot:
        """把控制器快照转换为 GUI 绘图模型。注意：需要同步维护轨迹缓存和显示字段。"""
        # 先把事件流里的扰动状态同步过来，使显示与控制器内部状态一致。
        self._sync_disturbance_from_events()
        nodes: list[NodeState] = []
        for node in snapshot.nodes:
            previous = self._last_xy_by_node.get(node.node_id)
            if previous is None:
                # 首帧无历史可差分，直接采用控制器给出的速度分量。
                vx = node.vx_mps
                vy = node.vy_mps
            else:
                # 只有仿真时间推进时才用位移差分；暂停同帧刷新时保留控制器速度，避免机头归零朝东。
                previous_x, previous_y, previous_time = previous
                dt = snapshot.time_s - previous_time
                if dt > 1e-9:
                    vx = (node.x_m - previous_x) / dt
                    vy = (node.y_m - previous_y) / dt
                else:
                    vx = node.vx_mps
                    vy = node.vy_mps
            self._last_xy_by_node[node.node_id] = (node.x_m, node.y_m, snapshot.time_s)

            # 取出该节点尾迹缓存；仅当时间戳推进时追加新点，避免同一帧重复入栈。
            trail = self._trail_by_node.setdefault(node.node_id, [])
            if not trail or trail[-1].time != snapshot.time_s:
                trail.append(TrailPoint(node.x_m, node.y_m, node.altitude_m, snapshot.time_s))
            # 原地裁剪超期尾迹点（用切片赋值以保持同一列表对象）。
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
                    vertical_speed=node.vz_mps,
                    health=node.health,
                    trail=list(trail),
                    cross_track_error=node.cross_track_error_m,
                    distance_to_go=node.distance_to_go_m,
                    track_pos_err_x=node.track_pos_err_x_m,
                    track_pos_err_y=node.track_pos_err_y_m,
                    track_pos_err_z=node.track_pos_err_z_m,
                    cmd_pos_x=node.cmd_pos_east_m,
                    cmd_pos_y=node.cmd_pos_north_m,
                    rally_phase=node.rally_phase,
                )
            )

        links: list[LinkState] = []
        for link in snapshot.links:
            # 链路 id 形如 "A01-A02"，按短横线拆出源/目标节点。
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
        # 兼容“单航线”与“多航段”两种来源：优先多航段，缺省时用单航线兜底。
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
            cpu_utilization=snapshot.cpu_utilization,
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
            radius=route.radius_m,
            center_x=route.center_x_m,
            center_y=route.center_y_m,
            turn_sign=route.turn_sign,
        )

    def _visible_disturbance(self, snapshot: ControllerSnapshot) -> str:
        """返回当前界面应显示的扰动名称。注意：已清除或过期扰动显示为无。"""
        # 优先按实际效果判定：有节点异常 → 节点故障，有链路异常 → 链路丢包。
        if any(node.health != "normal" for node in snapshot.nodes):
            return "节点故障"
        if any(link.status != "normal" for link in snapshot.links):
            return "链路丢包"
        # 就绪态且本地也认为无扰动时显式返回“无”，避免残留旧标签。
        if snapshot.run_state == "READY" and self.disturbance == "无":
            return "无"
        return self.disturbance

    def _sync_disturbance_from_events(self) -> None:
        """根据控制器事件同步扰动显示状态。注意：只处理尚未消费的新事件。"""
        events = self.controller.get_recent_events(limit=1000)
        # 只遍历游标之后的新事件，按消息关键字解析出当前应显示的扰动名称。
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
        # 推进游标到末尾，下次只处理更新的事件。
        self._processed_event_count = len(events)

    def _disturbance_command(self, kind: str) -> dict[str, object]:
        """生成 GUI 按钮对应的扰动命令。注意：命令结构需与控制器注入接口一致。"""
        # 把 UI 按钮种类翻译为控制器扰动命令字典；目标节点/链路与参数为预设演示值。
        if kind == "wind":
            return {"type": "wind", "duration_s": 8.0, "params": {"speed_mps": 8.0, "direction_deg": 90.0}}
        if kind == "fault":
            # 节点故障：目标 A02，降级模式持续 10s。
            return {"type": "node_fault", "target": "A02", "duration_s": 10.0, "params": {"mode": "degraded"}}
        if kind == "loss":
            return {"type": "link_loss", "target": "A01-A02", "duration_s": 12.0, "params": {"loss_rate": 0.3}}
        # 其余（clear）统一下发清除命令。
        return {"type": "clear"}

def node_altitude(index: int, time_value: float) -> float:
    """读取节点高度用于侧视图显示。注意：缺省时使用 0 作为兜底。"""

    # 基准高度 1200m，按机序错开 35m 层差，再叠加随时间起伏的正弦扰动。
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
        # 把传入的颜色字符串统一转成 QColor，供样式表与自绘画布复用。
        self.bg = QColor(bg)  # 窗口背景
        self.panel = QColor(panel)  # 面板/分组背景
        self.ink = QColor(ink)  # 主文字色
        self.muted = QColor(muted)  # 次要文字/轴标注
        self.line = QColor(line)  # 边框/分隔线
        self.canvas = QColor(canvas)  # 画布底色
        self.grid = QColor(grid)  # 网格线
        self.route = QColor(route)  # 参考航线
        self.leader = QColor(leader)  # 长机
        self.wingman = QColor(wingman)  # 僚机
        self.link = QColor(link)  # 正常链路
        self.warn = QColor(warn)  # 异常/告警
        self.accent = QColor(accent)  # 强调色（滑块/选框等）
        self.field = QColor(field)  # 输入控件背景


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
        # (显示文本, 附加数据) 列表与当前选中索引，-1 表示尚无选中项。
        self._items: list[tuple[str, object | None]] = []
        self._index = -1
        self._menu = QMenu(self)
        # 弹出方向："below" 在按钮下方，"right" 在按钮右侧（用于侧栏窄面板）。
        self._popup_side = popup_side
        self.setObjectName("selectButton")
        self.setMinimumWidth(min_width)
        # 点击按钮即弹出菜单；菜单收起时复位按钮按下态。
        self.clicked.connect(self.show_menu)
        self._menu.aboutToHide.connect(lambda: self.setDown(False))

    def addItem(self, text: str, data: object | None = None) -> None:
        """向控件添加一个选项。注意：选项文本和附加数据需保持对应。"""
        self._items.append((text, data))
        # 添加第一项时自动选中它，但不触发信号（避免初始化期误触回调）。
        if self._index == -1:
            self.setCurrentIndex(0, emit=False)

    def addItems(self, texts: list[str]) -> None:
        """批量添加控件选项。注意：按输入顺序追加。"""
        for text in texts:
            self.addItem(text, text)

    def setCurrentIndex(self, index: int, *, emit: bool = True) -> None:
        """设置当前选中项。注意：索引越界时不应破坏控件状态。"""
        # 越界索引直接忽略，保持原状态不被破坏。
        if index < 0 or index >= len(self._items):
            return
        # 选中项未变化则不重复刷新文本/不重复发信号。
        if index == self._index:
            return
        self._index = index
        # 按钮文本带下三角符号提示这是可下拉的选择器。
        self.setText(f"{self._items[index][0]}  ▾")
        if emit:
            self.currentIndexChanged.emit()

    def setCurrentText(self, text: str, *, emit: bool = True) -> None:
        """按文本设置当前选项。注意：文本不存在时会追加为新选项。"""
        normalized = str(text)
        for index, (item_text, _) in enumerate(self._items):
            if item_text == normalized:
                self.setCurrentIndex(index, emit=emit)
                return
        self._items.append((normalized, normalized))
        self.setCurrentIndex(len(self._items) - 1, emit=emit)

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
        # 每次弹出都重建菜单项，保证与最新选项列表/选中态一致。
        self._menu.clear()
        self._menu.setMinimumWidth(self.width())
        for index, (text, _) in enumerate(self._items):
            action = QAction(text, self._menu)
            action.setCheckable(True)
            # 当前项打勾；点击某项时把 row 绑定进 lambda 以更新选中索引。
            action.setChecked(index == self._index)
            action.triggered.connect(lambda checked=False, row=index: self.setCurrentIndex(row))
            self._menu.addAction(action)
        # 计算弹出锚点（按钮局部坐标），右侧弹出留出 34px 横向间隙。
        if self._popup_side == "right":
            point = QPoint(self.width() + 34, 0)
        else:
            point = QPoint(0, self.height() + 2)
        # 转换为全局坐标后弹出菜单。
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
        # 避障障碍（来自配置，独立于仿真快照）：加载配置后由主窗口注入，仅用于显示。
        self.obstacles: list[ObstacleView] = []
        self.obstacle_clearance = 0.0
        # 避障规划预览航线（折线点），由“生成航线”注入；None 表示无预览。
        self.preview_route_polyline: list[tuple[float, float]] | None = None
        # 预览航线的航点黑点，独立于折线采样点，避免圆弧采样点被误画成航点。
        self.preview_route_markers: list[tuple[float, float]] | None = None
        # 视图变换由 scale_value(缩放) 与 offset(平移) 两个量描述：屏幕 = 世界*scale + offset。
        self.scale_value = 1.0
        self.offset = self._default_offset()
        self.auto_center = False
        self.show_grid = True
        # _manual_view 为真表示用户手动调过视角，此后禁止自动铺满抢镜。
        self._manual_view = False
        # 中键拖拽起点；左键框选起止点（None 表示当前无对应操作进行中）。
        self._pan_origin: QPointF | None = None
        self._selection_origin: QPointF | None = None
        self._selection_current: QPointF | None = None
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setFrameShape(QFrame.Shape.NoFrame)
        self.setMinimumHeight(360)
        # 开启鼠标跟踪，未按键也能收到移动事件（框选/悬停需要）。
        self.setMouseTracking(True)

    def set_theme(self, theme: Theme) -> None:
        """设置当前主题。注意：需要同步更新画布和控件颜色。"""
        self.theme = theme
        self.viewport().update()

    def set_obstacles(self, obstacles: list[ObstacleView], clearance: float) -> None:
        """设置用于显示的避障障碍集与膨胀间距。注意：只更新显示，不推进仿真。"""
        self.obstacles = obstacles
        self.obstacle_clearance = clearance
        self.viewport().update()

    def set_preview_route(
        self,
        polyline: list[tuple[float, float]] | None,
        markers: list[tuple[float, float]] | None = None,
    ) -> None:
        """设置避障预览航线折线和航点标记（None 清除）。注意：只更新显示，不推进仿真。"""
        self.preview_route_polyline = polyline
        self.preview_route_markers = markers
        self.viewport().update()

    def set_snapshot(self, snapshot: Snapshot, *, fit_view: bool = False) -> None:
        """设置用于绘制的快照。注意：只更新显示缓存，不推进仿真。"""
        self.snapshot = snapshot
        # 自动居中优先；否则仅在请求 fit_view 且用户未手动调过视角时铺满。
        if self.auto_center:
            self._apply_auto_center()
        elif fit_view and not self._manual_view:
            self._fit_route_to_view()
        self.viewport().update()

    def resizeEvent(self, event) -> None:  # noqa: ANN001
        """处理控件尺寸变化事件，使航线/编队在新视口尺寸下重新适配显示。"""
        super().resizeEvent(event)
        # 没有快照或用户已手动操作过视图时，不强行重排，避免抢走用户的视角。
        if self.snapshot is None or self._manual_view:
            return
        if self.auto_center:
            # 自动居中模式：保持缩放，仅把编队几何中心移到视口中心。
            self._apply_auto_center()
        else:
            # 默认模式：把整条航线和飞机包围盒重新缩放铺满视口。
            self._fit_route_to_view()
        self.viewport().update()
        # 通知侧视图等监听者同步横向视野。
        self.viewChanged.emit()

    def reset_view(self) -> None:
        """重置视图缩放和平移。注意：不修改仿真数据。"""
        # 清除手动标记并复位缩放/平移，再按航线自适应铺满一次。
        self._manual_view = False
        self.scale_value = 1.0
        self.offset = self._default_offset()
        self._fit_route_to_view()
        self.viewport().update()
        # 依次通知：视图已变、来自“重置”动作、请求侧视图也自适应显示范围。
        self.viewChanged.emit()
        self.manualViewChanged.emit()
        self.resetViewRequested.emit()

    @staticmethod
    def _default_offset() -> QPointF:
        """计算俯视图默认平移量。注意：用于把初始场景放到画布可见区域。"""
        return QPointF(TOP_VIEW_ORIGIN_MARGIN, TOP_VIEW_ORIGIN_MARGIN)

    def wheelEvent(self, event) -> None:  # noqa: ANN001
        """处理鼠标滚轮事件。注意：用于缩放视图并保持交互焦点。"""
        # 优先用像素级滚动量（触控板），退回角度量（普通滚轮）。
        delta = event.pixelDelta().y() or event.angleDelta().y()
        if delta == 0:
            return
        cursor = event.position()
        # 记录缩放前光标对应的世界坐标，作为缩放锚点。
        before = QPointF(
            (cursor.x() - self.offset.x()) / self.scale_value,
            (cursor.y() - self.offset.y()) / self.scale_value,
        )
        # 指数因子使每格滚动产生固定比例缩放，手感线性；再夹到上下限。
        factor = math.pow(1.001, delta)
        self.scale_value = min(VIEW_MAX_SCALE, max(VIEW_MIN_SCALE, self.scale_value * factor))
        # 反解 offset，使锚点世界坐标缩放后仍落在光标处（“以光标为中心缩放”）。
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
        # 中键启动拖拽平移并切换为抓手光标；左键启动框选缩放。
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
            # 平移量按屏幕位移直接累加到 offset，并刷新拖拽起点为当前位置。
            delta = event.position() - self._pan_origin
            self.offset += QPointF(delta.x(), delta.y())
            self._pan_origin = event.position()
            self._manual_view = True
            self.viewport().update()
            self.viewChanged.emit()
            self.manualViewChanged.emit()
            event.accept()
        elif self._selection_origin is not None:
            # 框选进行中：仅更新选框终点并重绘虚线框。
            self._selection_current = event.position()
            self.viewport().update()
            event.accept()

    def mouseReleaseEvent(self, event) -> None:  # noqa: ANN001
        """处理鼠标释放事件。注意：结束拖拽或框选操作。"""
        if event.button() == Qt.MouseButton.MiddleButton:
            # 结束平移，恢复普通光标。
            self._pan_origin = None
            self.setCursor(Qt.CursorShape.ArrowCursor)
            event.accept()
        elif event.button() == Qt.MouseButton.LeftButton:
            # 松开左键即把选框区域放大铺满，再清空选框状态。
            self._zoom_to_selection()
            self._selection_origin = None
            self._selection_current = None
            self.viewport().update()
            event.accept()

    def mouseDoubleClickEvent(self, event) -> None:  # noqa: ANN001
        """处理鼠标双击事件。注意：通常用于快速重置或聚焦视图。"""
        # 双击左键快速重置视图。
        if event.button() == Qt.MouseButton.LeftButton:
            self.reset_view()
            event.accept()

    def paintEvent(self, event) -> None:  # noqa: ARG002, ANN001
        """处理 Qt 绘制事件。注意：只在当前快照基础上渲染画面。"""
        painter = QPainter(self.viewport())
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        # 先铺画布底色。
        painter.fillRect(self.rect(), self.theme.canvas)
        # 把 offset/scale 装进画家变换：之后均按世界坐标绘制，线宽需除以 scale 保持视觉粗细。
        painter.translate(self.offset)
        painter.scale(self.scale_value, self.scale_value)
        if self.show_grid:
            self._draw_grid(painter)
        # 障碍画在网格之上、航线/节点之下，避免遮挡飞机与航线。
        self._draw_obstacles(painter)
        # 避障预览航线画在障碍之上、节点之下。
        self._draw_preview_route(painter)
        if self.snapshot:
            # 绘制顺序：航线在底，链路其次，节点最上，保证遮挡关系正确。
            # 有预览时只显示绿色预览线和预览黑点，避免与 committed 航线形成两根线。
            if self.snapshot.nodes and self.preview_route_polyline is None:
                self._draw_route(painter)
            self._draw_links(painter, self.snapshot)
            self._draw_slot_targets(painter, self.snapshot)
            self._draw_nodes(painter, self.snapshot)
        # 选框是屏幕坐标元素，需先复位变换再绘制，避免被缩放。
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
        # 规整选框：取左右上下边界，兼容任意拖拽方向。
        left = min(self._selection_origin.x(), self._selection_current.x())
        right = max(self._selection_origin.x(), self._selection_current.x())
        top = min(self._selection_origin.y(), self._selection_current.y())
        bottom = max(self._selection_origin.y(), self._selection_current.y())
        # 选框太小（<8px）视为误点，忽略以免误缩放。
        if right - left < 8 or bottom - top < 8:
            return

        # 把选框两角换算到世界坐标，得到目标世界区域的宽高（下限 1 防除零）。
        world_start = self._viewport_to_world(QPointF(left, top))
        world_end = self._viewport_to_world(QPointF(right, bottom))
        world_width = max(1.0, abs(world_end.x() - world_start.x()))
        world_height = max(1.0, abs(world_end.y() - world_start.y()))
        viewport = self.viewport().rect()
        # 取宽高两方向较小的缩放比，使整个选区都能容下；0.94 留一点边距。
        margin = 0.94
        scale = min(viewport.width() / world_width, viewport.height() / world_height) * margin
        self.scale_value = min(VIEW_MAX_SCALE, max(VIEW_MIN_SCALE, scale))

        # 把选区世界中心平移到视口中心。
        center_x = (world_start.x() + world_end.x()) / 2.0
        center_y = (world_start.y() + world_end.y()) / 2.0
        self.offset = QPointF(
            viewport.width() / 2.0 - center_x * self.scale_value,
            viewport.height() / 2.0 - center_y * self.scale_value,
        )
        self._manual_view = True
        if self.auto_center:
            # 自动居中开启时，框选只表达“调整缩放比例”，中心仍交给自动居中维护。
            self._apply_auto_center()
            self.viewChanged.emit()
            return
        self.viewChanged.emit()
        self.manualViewChanged.emit()

    def _draw_selection(self, painter: QPainter) -> None:
        """绘制 selection 画面元素。注意：只做渲染，不修改仿真状态。"""
        if self._selection_origin is None or self._selection_current is None:
            return
        # 同样规整为左上/右下边界。
        left = min(self._selection_origin.x(), self._selection_current.x())
        right = max(self._selection_origin.x(), self._selection_current.x())
        top = min(self._selection_origin.y(), self._selection_current.y())
        bottom = max(self._selection_origin.y(), self._selection_current.y())
        # 选框过小不画，避免一个像素点的杂线。
        if right - left < 2 or bottom - top < 2:
            return
        selection = QRectF(left, top, right - left, bottom - top)
        # 用强调色虚线框表示框选区域，仅描边不填充。
        pen = QPen(self.theme.accent, 1.4)
        pen.setDashPattern([5, 4])
        painter.setPen(pen)
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawRect(selection)

    def _apply_auto_center(self) -> None:
        """应用 auto center 设置。注意：只修改对应显示或运行参数。"""
        if not self.snapshot or not self.snapshot.nodes:
            return
        # 优先以正常节点的质心为中心；全部异常时退回所有节点。
        active = [node for node in self.snapshot.nodes if node.health == "normal"]
        if not active:
            active = self.snapshot.nodes
        center_x = sum(node.x for node in active) / len(active)
        center_y = sum(node.y for node in active) / len(active)
        rect = self.viewport().rect()
        # 只平移不缩放：把质心移到视口正中。
        self.offset = QPointF(
            rect.width() / 2.0 - center_x * self.scale_value,
            rect.height() / 2.0 - center_y * self.scale_value,
        )
        self.viewChanged.emit()

    def _fit_route_to_view(self) -> None:
        """把航线和飞机范围适配到当前俯视图。注意：只调整显示缩放和平移。"""
        # 无快照/无包围盒时退回默认平移，保持画面可用。
        if self.snapshot is None:
            self.offset = self._default_offset()
            return
        bounds = self._route_and_node_bounds()
        if bounds is None:
            self.offset = self._default_offset()
            return
        min_x, max_x, min_y, max_y = bounds
        rect = self.viewport().rect()
        # 视口尚未布局（宽高为 0）时直接返回，等下次有效尺寸再算。
        if rect.width() <= 0 or rect.height() <= 0:
            return
        # 包围盒跨度（下限 1 防退化）；可用区只取视口的 FIT_VIEWPORT_RATIO 留边距。
        span_x = max(1.0, max_x - min_x)
        span_y = max(1.0, max_y - min_y)
        available_width = max(1.0, rect.width() * FIT_VIEWPORT_RATIO)
        available_height = max(1.0, rect.height() * FIT_VIEWPORT_RATIO)
        scale_x = available_width / span_x
        scale_y = available_height / span_y
        # 取两方向较小缩放保证 x/y 都装得下，并夹到上下限。
        self.scale_value = min(VIEW_MAX_SCALE, max(VIEW_MIN_SCALE, min(scale_x, scale_y)))
        # 把包围盒中心平移到视口中心。
        center_x = (min_x + max_x) / 2.0
        center_y = (min_y + max_y) / 2.0
        self.offset = QPointF(
            rect.width() / 2.0 - center_x * self.scale_value,
            rect.height() / 2.0 - center_y * self.scale_value,
        )

    def _route_and_node_bounds(self) -> tuple[float, float, float, float] | None:
        """计算航线与全部飞机的世界坐标包围盒，返回 (min_x,max_x,min_y,max_y)。

        无快照或无任何点时返回空值，供自适应缩放判断是否回退到默认视图。
        """
        if self.snapshot is None:
            return None
        # 包围盒同时纳入飞机当前位置与所有航段端点，确保两者都能落在可见区。
        xs = [node.x for node in self.snapshot.nodes]
        ys = [node.y for node in self.snapshot.nodes]
        for route in self._route_segments():
            xs.extend([route.start_x, route.end_x])
            ys.extend([route.start_y, route.end_y])
        # 让启用的障碍也纳入包围盒，使自适应铺满时障碍不被挤出视野。
        for obstacle in self.obstacles:
            if not obstacle.enabled:
                continue
            if obstacle.kind == "rect":
                xs.extend([obstacle.min_x, obstacle.max_x])
                ys.extend([obstacle.min_y, obstacle.max_y])
            else:
                xs.extend([obstacle.center_x - obstacle.radius, obstacle.center_x + obstacle.radius])
                ys.extend([obstacle.center_y - obstacle.radius, obstacle.center_y + obstacle.radius])
        if not xs or not ys:
            return None
        return min(xs), max(xs), min(ys), max(ys)

    def _draw_grid(self, painter: QPainter) -> None:
        """绘制 grid 画面元素。注意：只做渲染，不修改仿真状态。"""
        # 反解视口四角对应的世界坐标范围（画家已应用 offset/scale）。
        rect = self.viewport().rect()
        left = (rect.left() - self.offset.x()) / self.scale_value
        right = (rect.right() - self.offset.x()) / self.scale_value
        top = (rect.top() - self.offset.y()) / self.scale_value
        bottom = (rect.bottom() - self.offset.y()) / self.scale_value
        spacing = self._grid_world_spacing()
        # 把可见范围对齐到网格间距的整数倍，确定起止网格线坐标。
        start_x = math.floor(left / spacing) * spacing
        end_x = math.ceil(right / spacing) * spacing
        start_y = math.floor(top / spacing) * spacing
        end_y = math.ceil(bottom / spacing) * spacing

        # 线宽除以 scale，使网格线在任意缩放下都呈现 1px 视觉粗细。
        painter.setPen(QPen(self.theme.grid, 1.0 / self.scale_value))
        # 仅绘制可见区内的竖线与横线，避免遍历整个世界。
        for x in range(start_x, end_x + spacing, spacing):
            painter.drawLine(x, start_y, x, end_y)
        for y in range(start_y, end_y + spacing, spacing):
            painter.drawLine(start_x, y, end_x, y)

    def _grid_world_spacing(self) -> int:
        """返回俯视图当前应使用的网格世界间距（依据自身缩放自适应）。"""
        return adaptive_world_grid_spacing(self.scale_value)

    def _obstacle_center(self, obstacle: ObstacleView) -> tuple[float, float]:
        """返回障碍中心世界坐标。注意：矩形取几何中心，圆取圆心。"""
        if obstacle.kind == "rect":
            return (obstacle.min_x + obstacle.max_x) / 2.0, (obstacle.min_y + obstacle.max_y) / 2.0
        return obstacle.center_x, obstacle.center_y

    def _stroke_obstacle_shape(self, painter: QPainter, obstacle: ObstacleView, inflate: float) -> None:
        """按当前画笔/画刷描绘障碍轮廓。注意：inflate>0 时整体外扩（矩形按方角近似）。"""
        if obstacle.kind == "rect":
            painter.drawRect(
                QRectF(
                    obstacle.min_x - inflate,
                    obstacle.min_y - inflate,
                    (obstacle.max_x - obstacle.min_x) + 2.0 * inflate,
                    (obstacle.max_y - obstacle.min_y) + 2.0 * inflate,
                )
            )
        else:
            radius = obstacle.radius + inflate
            painter.drawEllipse(QPointF(obstacle.center_x, obstacle.center_y), radius, radius)

    def _draw_obstacles(self, painter: QPainter) -> None:
        """绘制避障障碍与膨胀圈。注意：只做渲染，不修改仿真状态。"""
        if not self.obstacles:
            return
        for obstacle in self.obstacles:
            if obstacle.enabled:
                # 膨胀圈：障碍外扩 clearance，橙色虚线、不填充。
                ring = QColor(self.theme.warn)
                ring.setAlphaF(0.85)
                ring_pen = QPen(ring, 1.6 / self.scale_value)
                ring_pen.setDashPattern([6, 5])
                painter.setPen(ring_pen)
                painter.setBrush(Qt.BrushStyle.NoBrush)
                if self.obstacle_clearance > 0.0:
                    self._stroke_obstacle_shape(painter, obstacle, self.obstacle_clearance)
                # 障碍本体：半透明填充 + 实线描边。
                fill = QColor(self.theme.warn)
                fill.setAlphaF(0.28)
                painter.setBrush(fill)
                painter.setPen(QPen(self.theme.warn, 2.0 / self.scale_value))
                self._stroke_obstacle_shape(painter, obstacle, 0.0)
            else:
                # 未勾选：灰色虚线、不填充、不画膨胀，弱化表示“本次不避”。
                faint = QColor(self.theme.muted)
                faint.setAlphaF(0.7)
                faint_pen = QPen(faint, 1.4 / self.scale_value)
                faint_pen.setDashPattern([4, 5])
                painter.setPen(faint_pen)
                painter.setBrush(Qt.BrushStyle.NoBrush)
                self._stroke_obstacle_shape(painter, obstacle, 0.0)
            # 在障碍中心标注 id（标签不随缩放，保持屏幕尺寸）。
            center_x, center_y = self._obstacle_center(obstacle)
            painter.save()
            painter.translate(center_x, center_y)
            painter.scale(1.0 / self.scale_value, 1.0 / self.scale_value)
            painter.setPen(QPen(self.theme.ink if obstacle.enabled else self.theme.muted, 1))
            text = obstacle.obstacle_id if obstacle.enabled else f"{obstacle.obstacle_id}（未勾选）"
            painter.drawText(QPointF(-10.0, 4.0), text)
            painter.restore()

    def _draw_preview_route(self, painter: QPainter) -> None:
        """绘制避障预览航线（绿色虚线折线）和航点黑点。注意：只做渲染，不修改仿真状态。"""
        polyline = self.preview_route_polyline
        if not polyline or len(polyline) < 2:
            return
        pen = QPen(QColor("#2E7D32"), 2.6 / self.scale_value)
        pen.setDashPattern([7, 5])
        painter.setPen(pen)
        painter.setBrush(Qt.BrushStyle.NoBrush)
        for start, end in zip(polyline, polyline[1:]):
            painter.drawLine(QPointF(start[0], start[1]), QPointF(end[0], end[1]))
        self._draw_route_markers(painter, self.preview_route_markers or [])

    def _draw_route(self, painter: QPainter) -> None:
        """绘制 route 画面元素。注意：只做渲染，不修改仿真状态。"""
        routes = self._route_segments()
        if not routes:
            return
        # 航线用虚线绘制（线宽随缩放归一）；圆弧段按弧采样，与预览画法一致。
        pen = QPen(self.theme.route, 2.0 / self.scale_value)
        pen.setDashPattern([8, 7])
        painter.setPen(pen)
        for route in routes:
            points = reference_route_points(route)
            for start, end in zip(points, points[1:]):
                painter.drawLine(QPointF(start[0], start[1]), QPointF(end[0], end[1]))
        self._draw_route_markers(
            painter,
            [(routes[0].start_x, routes[0].start_y)] + [(route.end_x, route.end_y) for route in routes],
        )

    def _draw_route_markers(self, painter: QPainter, markers: list[tuple[float, float]]) -> None:
        """绘制航点黑点。注意：仅画端点标记，不包含圆弧折线采样点。"""
        if not markers:
            return
        painter.setBrush(self.theme.ink)
        painter.setPen(Qt.PenStyle.NoPen)
        marker_radius = 5.0 / self.scale_value
        for east, north in markers:
            painter.drawEllipse(QPointF(east, north), marker_radius, marker_radius)

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
        # 先建 id->节点索引，便于按链路端点取坐标。
        by_id = {node.node_id: node for node in snapshot.nodes}
        for link in snapshot.links:
            source = by_id[link.source]
            target = by_id[link.target]
            # 正常链路用链路色半透明细线，异常链路用警示色更不透明的粗线突出。
            color = QColor(self.theme.link if link.ok else self.theme.warn)
            color.setAlphaF(0.58 if link.ok else 0.75)
            painter.setPen(QPen(color, (2 if link.ok else 3) / self.scale_value))
            painter.drawLine(QPointF(source.x, source.y), QPointF(target.x, target.y))

    def _draw_nodes(self, painter: QPainter, snapshot: Snapshot) -> None:
        """绘制 nodes 画面元素。注意：只做渲染，不修改仿真状态。"""
        for node in snapshot.nodes:
            is_leader = is_leader_node(node)
            # 先画历史尾迹，再画机体，使机体压在尾迹之上。
            self._draw_trail(painter, node, is_leader, snapshot.time)
            # 颜色优先级：异常>长机>僚机。
            color = self.theme.warn if node.health != "normal" else self.theme.leader if is_leader else self.theme.wingman
            painter.save()
            # 平移到机体位置，按速度方向旋转机头朝向。
            painter.translate(node.x, node.y)
            painter.rotate(math.degrees(math.atan2(node.vy, node.vx)))
            # 反缩放使机体图标在任意视图缩放下保持固定屏幕大小。
            painter.scale(1.0 / self.scale_value, 1.0 / self.scale_value)
            painter.setBrush(color)
            painter.setPen(QPen(self.theme.panel, 2))
            # 用箭头状多边形表示飞机：尖端朝 +x（机头），尾部带内凹缺口。
            path = QPainterPath(QPointF(12, 0))
            path.lineTo(-9, -7)
            path.lineTo(-4.5, 0)
            path.lineTo(-9, 7)
            path.closeSubpath()
            painter.drawPath(path)
            painter.restore()
            # 在机体左上方标注节点 ID（标签不随机体旋转）。
            painter.setPen(QPen(self.theme.ink, 1))
            painter.drawText(QPointF(node.x - 13, node.y - 18), node.node_id)

    def _draw_slot_targets(self, painter: QPainter, snapshot: Snapshot) -> None:
        """绘制僚机目标槽位标记（菱形 + 连线）。注意：只做渲染，不修改仿真状态。"""
        for node in snapshot.nodes:
            if is_leader_node(node):
                continue
            # 目标点为原点时跳过（初始化默认值，尚未收到有效指令）
            if node.cmd_pos_x == 0.0 and node.cmd_pos_y == 0.0:
                continue
            color = QColor(self.theme.warn if node.health != "normal" else self.theme.wingman)
            color.setAlphaF(0.70)
            # 节点到目标点的虚线
            pen = QPen(color, 1.0 / self.scale_value, Qt.PenStyle.DashLine)
            painter.setPen(pen)
            painter.drawLine(QPointF(node.x, node.y), QPointF(node.cmd_pos_x, node.cmd_pos_y))
            # 目标位置的空心菱形
            r = 7.0 / self.scale_value
            diamond = QPainterPath()
            diamond.moveTo(node.cmd_pos_x, node.cmd_pos_y - r)
            diamond.lineTo(node.cmd_pos_x + r, node.cmd_pos_y)
            diamond.lineTo(node.cmd_pos_x, node.cmd_pos_y + r)
            diamond.lineTo(node.cmd_pos_x - r, node.cmd_pos_y)
            diamond.closeSubpath()
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.setPen(QPen(color, 1.5 / self.scale_value))
            painter.drawPath(diamond)

    def _draw_trail(self, painter: QPainter, node: NodeState, is_leader: bool, current_time: float) -> None:
        """绘制 trail 画面元素。注意：只做渲染，不修改仿真状态。"""
        # 两个采样点即可表达刚启动后的位移，避免运行初期看起来像静止。
        if len(node.trail) <= 1:
            return
        base = self.theme.leader if is_leader else self.theme.wingman
        # 逐相邻点对连线：越旧的段透明度越低，形成淡出拖尾。
        for previous, current in zip(node.trail, node.trail[1:]):
            age = max(0.0, current_time - current.time)
            # 透明度随存活时间线性衰减，并设 0.08 下限防止完全消失突变。
            alpha = max(0.08, 1.0 - age / TRAIL_SECONDS)
            color = QColor(base)
            # 长机尾迹整体比僚机略浓。
            color.setAlphaF((0.52 if is_leader else 0.44) * alpha)
            # 世界坐标已整体缩放，线宽反向缩放才能在长航线低缩放下保持可见。
            painter.setPen(QPen(color, 2.4 / self.scale_value))
            painter.drawLine(QPointF(previous.x, previous.y), QPointF(current.x, current.y))


class SideView(QWidget):
    """高度侧视图。注意：横轴可按当前航段里程或用户视角投影显示。"""

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
        self.segment_locked = True
        self.auto_center = False
        self.view_angle_deg = 0.0
        self.horizontal_scale = 1.0
        self.horizontal_offset = 0.0
        # 用户手动缩放/拖动侧视图后，运行期刷新不再强行重排横轴。
        self._manual_horizontal_view = False
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
        if self.auto_center:
            self._apply_auto_center()
        elif not self._manual_horizontal_view:
            self._fit_horizontal_view()
        self.update()

    def resizeEvent(self, event) -> None:  # noqa: ANN001
        """处理控件尺寸变化事件，使自动居中在新尺寸下保持居中。"""
        super().resizeEvent(event)
        if self.snapshot is None:
            return
        if self.auto_center:
            self._apply_auto_center()
        elif not self._manual_horizontal_view:
            self._fit_horizontal_view()
        self.update()

    def set_segment_locked(self, locked: bool) -> None:
        """设置是否锁定当前航段。注意：无当前航段时内部会退回手动视角投影。"""
        self.segment_locked = locked
        self._manual_horizontal_view = False
        self._fit_horizontal_view()
        self.update()

    def set_view_angle_deg(self, angle_deg: float) -> None:
        """设置手动视角。注意：0 表示面朝正北，90 表示面朝正东。"""
        self.view_angle_deg = angle_deg % 360.0
        if not self._locked_route():
            self._manual_horizontal_view = False
            self._fit_horizontal_view()
        self.update()

    def lock_available(self) -> bool:
        """返回当前快照是否有可锁定航段。注意：零长度航段不算可锁定。"""
        return self._route_unit(self.snapshot.route if self.snapshot else None) is not None

    def current_view_angle_deg(self) -> float:
        """返回侧视图当前视角角度。注意：航段锁定时由当前航段自动计算。"""
        route = self._locked_route()
        if route is None:
            return self.view_angle_deg
        return self._route_view_angle_deg(route)

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
        painter.drawText(QPointF(self.width() - 86, self.height() - 8), self._axis_label())
        painter.drawText(QPointF(12, 20), "高度")
        self._draw_selection(painter)

    def wheelEvent(self, event) -> None:  # noqa: ANN001
        """处理鼠标滚轮事件。注意：用于缩放视图并保持交互焦点。"""
        delta = event.pixelDelta().y() or event.angleDelta().y()
        if delta == 0:
            return
        before_x = self._screen_to_world_x(event.position().x())
        factor = math.pow(1.001, delta)
        self.horizontal_scale = min(VIEW_MAX_SCALE, max(VIEW_MIN_SCALE, self.horizontal_scale * factor))
        self.horizontal_offset = event.position().x() - before_x * self.horizontal_scale
        self._manual_horizontal_view = True
        self.update()
        self.top_view.manualViewChanged.emit()
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
            self.horizontal_offset += delta.x()
            self._pan_altitude(delta.y())
            self._manual_horizontal_view = True
            self._pan_origin = event.position()
            self.update()
            self.top_view.manualViewChanged.emit()
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
        """处理鼠标双击事件。注意：快速重置侧视图横轴和高度轴。"""
        if event.button() == Qt.MouseButton.LeftButton:
            self.reset_view()
            event.accept()

    def reset_view(self) -> None:
        """重置侧视图显示范围。注意：同时自适应横轴和高度轴。"""
        self._manual_horizontal_view = False
        self._fit_horizontal_view()
        self._fit_altitude_view()
        self.update()

    def _apply_auto_center(self) -> None:
        """应用自动居中。注意：只平移横轴和高度轴，不改变缩放或高度跨度。"""
        if self.snapshot is None or not self.snapshot.nodes:
            return
        # 与俯视图一致：优先以正常节点质心为中心，全部异常时退回全部节点。
        active = [node for node in self.snapshot.nodes if node.health == "normal"]
        if not active:
            active = self.snapshot.nodes
        center_x = sum(self._horizontal_for_point(node.x, node.y) for node in active) / len(active)
        self.horizontal_offset = self.width() / 2.0 - center_x * self.horizontal_scale
        altitude_span = max(1.0, self.altitude_max - self.altitude_min)
        center_altitude = sum(node.altitude for node in active) / len(active)
        self.altitude_min = center_altitude - altitude_span / 2.0
        self.altitude_max = center_altitude + altitude_span / 2.0

    def _map_x(self, x: float) -> float:
        """映射侧视图横轴坐标。注意：横轴含义由当前模式决定。"""
        return x * self.horizontal_scale + self.horizontal_offset

    def _screen_to_world_x(self, x: float) -> float:
        """把屏幕坐标转换为侧视图横轴坐标。注意：保留旧名称兼容测试。"""
        return (x - self.horizontal_offset) / self.horizontal_scale

    def _screen_to_altitude(self, y: float) -> float:
        """把屏幕坐标转换为 altitude。注意：依赖当前高度视野。"""
        plot_height = max(1.0, self.height() - self.PLOT_VERTICAL_MARGINS)
        ratio = (self.height() - self.PLOT_BOTTOM_MARGIN - y) / plot_height
        return self.altitude_min + ratio * (self.altitude_max - self.altitude_min)

    def _pan_altitude(self, delta_y: float) -> None:
        """平移 altitude 视图。注意：只改变显示偏移，不改变仿真数据。"""
        plot_height = max(1.0, self.height() - self.PLOT_VERTICAL_MARGINS)
        altitude_delta = delta_y / plot_height * (self.altitude_max - self.altitude_min)
        self.altitude_min += altitude_delta
        self.altitude_max += altitude_delta

    def _zoom_to_selection(self) -> None:
        """执行 to selection 缩放。注意：选区可分别影响横轴和高度轴。"""
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
            self.horizontal_scale = min(VIEW_MAX_SCALE, max(VIEW_MIN_SCALE, self.width() / world_width * 0.94))
            center_x = (start_x + end_x) / 2.0
            self.horizontal_offset = self.width() / 2.0 - center_x * self.horizontal_scale
            self._manual_horizontal_view = True

        if has_height:
            altitude_top = self._screen_to_altitude(top)
            altitude_bottom = self._screen_to_altitude(bottom)
            center = (altitude_top + altitude_bottom) / 2.0
            span = max(8.0, abs(altitude_top - altitude_bottom) / 0.94)
            self.altitude_min = center - span / 2.0
            self.altitude_max = center + span / 2.0

        if self.auto_center:
            # 自动居中开启时，框选只调整缩放/高度跨度，中心继续由自动居中维护。
            self._apply_auto_center()
            self.update()
            return
        self.update()
        self.top_view.manualViewChanged.emit()

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
        """映射 y 坐标。注意：高度越高屏幕 y 越小。"""
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
        """返回侧视图当前应使用的横轴网格间距。"""
        return adaptive_world_grid_spacing(self.horizontal_scale)

    def _draw_reference(self, painter: QPainter) -> None:
        """绘制 reference 画面元素。注意：只做渲染，不修改仿真状态。"""
        routes = self._route_segments()
        if not routes:
            return
        pen = QPen(self.theme.route, 2)
        pen.setDashPattern([7, 6])
        painter.setPen(pen)
        for route in routes:
            start_x = self._horizontal_for_point(route.start_x, route.start_y)
            end_x = self._horizontal_for_point(route.end_x, route.end_y)
            painter.drawLine(
                QPointF(self._map_x(start_x), self._map_y(route.start_altitude)),
                QPointF(self._map_x(end_x), self._map_y(route.end_altitude)),
            )

    def _route_segments(self) -> list[ReferenceRoute]:
        """返回侧视图需要绘制的航段列表。注意：锁定时只画当前航段。"""
        if self.snapshot is None:
            return []
        if self._locked_route() is not None:
            return [self.snapshot.route] if self.snapshot.route is not None else []
        if self.snapshot.route_segments:
            return self.snapshot.route_segments
        if self.snapshot.route is not None:
            return [self.snapshot.route]
        return []

    def _draw_trails(self, painter: QPainter, snapshot: Snapshot) -> None:
        """绘制 trails 画面元素。注意：只做渲染，不修改仿真状态。"""
        for node in snapshot.nodes:
            if len(node.trail) <= 2:
                continue
            is_leader = is_leader_node(node)
            base = self.theme.leader if is_leader else self.theme.wingman
            for previous, current in zip(node.trail, node.trail[1:]):
                x1 = self._map_x(self._horizontal_for_point(previous.x, previous.y))
                x2 = self._map_x(self._horizontal_for_point(current.x, current.y))
                if (x1 < -24 and x2 < -24) or (x1 > self.width() + 24 and x2 > self.width() + 24):
                    continue
                age = max(0.0, snapshot.time - current.time)
                alpha = max(0.08, 1.0 - age / TRAIL_SECONDS)
                color = QColor(base)
                color.setAlphaF((0.48 if is_leader else 0.40) * alpha)
                painter.setPen(QPen(color, 2))
                painter.drawLine(QPointF(x1, self._map_y(previous.altitude)), QPointF(x2, self._map_y(current.altitude)))

    def _draw_nodes(self, painter: QPainter, snapshot: Snapshot) -> None:
        """绘制 nodes 画面元素。注意：只做渲染，不修改仿真状态。"""
        for node in snapshot.nodes:
            x = self._map_x(self._horizontal_for_point(node.x, node.y))
            if x < -24 or x > self.width() + 24:
                continue
            is_leader = is_leader_node(node)
            color = self.theme.warn if node.health != "normal" else self.theme.leader if is_leader else self.theme.wingman
            y = self._map_y(node.altitude)
            painter.setBrush(color)
            painter.setPen(QPen(self.theme.panel, 2))
            painter.drawEllipse(QPointF(x, y), 8, 8)
            painter.setPen(self.theme.ink)
            painter.drawText(QPointF(x + 10, y + 4), node.node_id)

    def _axis_label(self) -> str:
        """返回横轴标签。注意：标签需反映当前横轴语义。"""
        return "航段里程" if self._locked_route() is not None else "投影距离"

    def _locked_route(self) -> ReferenceRoute | None:
        """返回当前锁定航段。注意：仅使用快照中的当前航段，不做人工选择。"""
        if not self.segment_locked or self.snapshot is None:
            return None
        route = self.snapshot.route
        return route if self._route_unit(route) is not None else None

    def _route_unit(self, route: ReferenceRoute | None) -> tuple[float, float] | None:
        """返回航段单位方向。注意：零长度航段无法定义锁定横轴。"""
        if route is None:
            return None
        dx = route.end_x - route.start_x
        dy = route.end_y - route.start_y
        length = math.hypot(dx, dy)
        if length <= 1e-6:
            return None
        return dx / length, dy / length

    def _route_view_angle_deg(self, route: ReferenceRoute) -> float:
        """把航段方向换算成视角。注意：0 为面朝正北，90 为面朝正东。"""
        unit = self._route_unit(route)
        if unit is None:
            return self.view_angle_deg
        ux, uy = unit
        return math.degrees(math.atan2(-uy, ux)) % 360.0

    def _horizontal_for_point(self, x: float, y: float) -> float:
        """计算侧视图横轴坐标。注意：锁定时为当前航段里程，非锁定时为视角投影。"""
        route = self._locked_route()
        if route is not None:
            unit = self._route_unit(route)
            if unit is not None:
                ux, uy = unit
                return (x - route.start_x) * ux + (y - route.start_y) * uy
        angle = math.radians(self.view_angle_deg)
        return x * math.cos(angle) - y * math.sin(angle)

    def _horizontal_bounds(self) -> tuple[float, float] | None:
        """计算侧视图横向包围盒。注意：同时纳入航段、节点与尾迹。"""
        if self.snapshot is None:
            return None
        values: list[float] = []
        for route in self._route_segments():
            values.append(self._horizontal_for_point(route.start_x, route.start_y))
            values.append(self._horizontal_for_point(route.end_x, route.end_y))
        for node in self.snapshot.nodes:
            values.append(self._horizontal_for_point(node.x, node.y))
            for point in node.trail:
                values.append(self._horizontal_for_point(point.x, point.y))
        if not values:
            return None
        return min(values), max(values)

    def _altitude_bounds(self) -> tuple[float, float] | None:
        """计算侧视图高度包围盒。注意：同时纳入航段、节点与尾迹。"""
        if self.snapshot is None:
            return None
        values: list[float] = []
        for route in self._route_segments():
            values.append(route.start_altitude)
            values.append(route.end_altitude)
        for node in self.snapshot.nodes:
            values.append(node.altitude)
            values.extend(point.altitude for point in node.trail)
        if not values:
            return None
        return min(values), max(values)

    def _fit_horizontal_view(self) -> None:
        """自适应侧视图横轴范围。注意：不改变高度轴。"""
        bounds = self._horizontal_bounds()
        if bounds is None:
            self.horizontal_scale = 1.0
            self.horizontal_offset = self.width() / 2.0
            return
        left, right = bounds
        if math.isclose(left, right, abs_tol=1e-6):
            left -= 50.0
            right += 50.0
        span = max(1.0, right - left)
        width = max(1.0, float(self.width()))
        self.horizontal_scale = min(VIEW_MAX_SCALE, max(VIEW_MIN_SCALE, width / span * 0.86))
        center = (left + right) / 2.0
        self.horizontal_offset = width / 2.0 - center * self.horizontal_scale

    def _fit_altitude_view(self) -> None:
        """自适应侧视图高度范围。注意：不改变横轴缩放和平移。"""
        bounds = self._altitude_bounds()
        if bounds is None:
            self.altitude_min = self.ALTITUDE_MIN_DEFAULT
            self.altitude_max = self.ALTITUDE_MAX_DEFAULT
            return
        bottom, top = bounds
        if math.isclose(bottom, top, abs_tol=1e-6):
            bottom -= 50.0
            top += 50.0
        span = max(80.0, (top - bottom) / 0.86)
        center = (bottom + top) / 2.0
        self.altitude_min = center - span / 2.0
        self.altitude_max = center + span / 2.0


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
        # 解析项目根目录：显式传入则用之，否则自动探测。
        self.project_root = Path(project_root).resolve() if project_root is not None else default_project_root()
        # config.ini 路径：缺省放项目根下；相对路径则相对项目根解析。
        if config_state_path is None:
            self.config_state_path = self.project_root / APP_CONFIG_FILE_NAME
        else:
            state_path = Path(config_state_path)
            self.config_state_path = state_path if state_path.is_absolute() else self.project_root / state_path
        self.current_config_path: Path | None = None
        self.setWindowTitle("编队仿真")
        self.resize(1440, 900)
        self.setMinimumSize(1280, 780)
        # 接入真实控制器适配器作为数据源。
        self.sim = ControllerSimulationAdapter()
        self.theme_key = "light"
        self.theme = THEMES[self.theme_key]
        # 100ms 定时器驱动运行期界面刷新（约 10 FPS）。
        self.timer = QTimer(self)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self._on_tick)
        self.log_dialog = LogDialog(self)
        # 这些引用在 _build_ui 中赋值，先声明便于类型提示与后续全屏切换访问。
        self.main_layout: QHBoxLayout | None = None
        self.stage: QWidget | None = None
        self.fullscreen_button: QPushButton | None = None
        # 全屏时用占位控件顶替原位置，退出时据此还原布局位置与拉伸系数。
        self._stage_placeholder: QWidget | None = None
        self._stage_fullscreen_dialog: StageFullscreenDialog | None = None
        self._stage_layout_index = 1
        self._stage_layout_stretch = 1
        self.disturbance_buttons: list[QPushButton] = []
        # 避障障碍（来自配置）与其勾选框，加载配置后填充。
        self.obstacles: list[ObstacleView] = []
        self.obstacle_checkboxes: list[QCheckBox] = []
        # 避障规划参数（来自配置）与“生成航线”得到的预览航线（采用前）。
        self._avoidance_params: AvoidanceParams | None = None
        self._preview_route: list[WayPointInputS] | None = None
        self._segment_lock_preferred = True
        self._live_monitor: "LiveMonitorWindow | None" = None
        self._offline_plot: "OfflinePlotWindow | None" = None
        self._data_analysis_window: "DataAnalysisWindow | None" = None
        self.avoidance_window: AvoidanceWindow | None = None
        # 组装界面 -> 设置手型光标 -> 应用主题 -> 用初始快照刷新一次显示。
        self._build_ui()
        self._install_button_cursors()
        self._apply_theme()
        self._update_snapshot(self.sim.snapshot())
        self._log("SimControl", "初始化界面，等待加载配置")
        # 自动加载上次使用过的配置，提升复用体验。
        if auto_load_config:
            self._load_last_config_from_state()

    def _build_ui(self) -> None:
        """构建主窗口全部 UI 区域。注意：控件引用需保存供后续事件更新使用。"""
        self._build_menus()
        root = QWidget()
        self.setCentralWidget(root)
        # 整体竖向：中央区域只保留主区，顶部入口交给菜单栏。
        outer = QVBoxLayout(root)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        # 主区横向三栏：左面板(固定) + 中央画布(可伸展，stretch=1) + 右面板(固定)。
        main = QHBoxLayout()
        self.main_layout = main
        main.setContentsMargins(8, 8, 8, 8)
        main.setSpacing(8)
        outer.addLayout(main, 1)
        main.addWidget(self._build_left_panel(), 0)
        # 保存 stage 引用：全屏切换时需要把它在布局间搬移。
        self.stage = self._build_stage()
        main.addWidget(self.stage, 1)
        main.addWidget(self._build_right_panel(), 0)
        self._build_avoidance_window()

    def _build_menus(self) -> None:
        """构建菜单栏入口。注意：常驻控制集中到菜单，避免占用主界面高度。"""
        # 保存菜单引用，避免 PySide 临时包装对象回收后测试或后续逻辑无法稳定访问。
        self.monitor_menu = self.menuBar().addMenu("控制监控(&V)")
        self.monitor_menu.addAction("数据监控(&M)").triggered.connect(self._open_live_monitor)
        # 离线分析是 upstream 新增入口，rebase 后继续归在控制监控菜单下。
        self.monitor_menu.addAction("离线分析(&A)").triggered.connect(self._open_offline_plot)

        # 数据分析是独立离线工具入口，不复用控制监控下的旧离线回放窗口。
        self.data_analysis_menu = self.menuBar().addMenu("数据分析(&D)")
        self.data_analysis_menu.addAction("控制效果分析(&A)").triggered.connect(self._open_data_analysis_window)

        # 避障规划放到菜单栏顶层入口，避免窄左栏承载复杂参数面板。
        self.avoidance_action = QAction("避障规划(&O)", self)
        self.avoidance_action.triggered.connect(self._open_avoidance_window)
        self.menuBar().addAction(self.avoidance_action)

        # 帮助菜单承载低频入口，避免主题/日志控件常驻占用主画布顶部空间。
        self.help_menu = self.menuBar().addMenu("帮助(&H)")
        # QActionGroup 同样由窗口持有，保证浅色/深色两个动作始终互斥。
        self.theme_action_group = QActionGroup(self)
        self.theme_action_group.setExclusive(True)
        self.light_theme_action = QAction("浅色模式", self)
        self.light_theme_action.setCheckable(True)
        self.light_theme_action.setChecked(True)
        self.light_theme_action.triggered.connect(lambda checked=False: self._set_theme("light"))
        self.dark_theme_action = QAction("深色模式", self)
        self.dark_theme_action.setCheckable(True)
        self.dark_theme_action.triggered.connect(lambda checked=False: self._set_theme("dark"))
        self.theme_action_group.addAction(self.light_theme_action)
        self.theme_action_group.addAction(self.dark_theme_action)
        self.help_menu.addAction(self.light_theme_action)
        self.help_menu.addAction(self.dark_theme_action)
        self.help_menu.addSeparator()
        self.log_action = self.help_menu.addAction("日志")
        self.log_action.triggered.connect(self.log_dialog.show)

    def _build_status_group(self) -> QWidget:
        """构建左侧运行状态分组。注意：只放高频状态，不挤占主画布顶部。"""
        self.status_group = QGroupBox("状态")
        layout = QVBoxLayout(self.status_group)
        layout.setContentsMargins(10, 18, 10, 10)
        layout.setSpacing(8)
        # 高频变化的运行状态保留为独立胶囊，便于一眼确认当前生命周期。
        self.run_state_label = QLabel("READY")
        self.run_state_label.setObjectName("statusPill")
        # 控制回报跟随状态放在左侧栏，删除顶部 header 后仍保持可见。
        self.report_label = QLabel("回报：待命")
        self.report_label.setObjectName("reportPill")
        layout.addWidget(self.run_state_label)
        layout.addWidget(self.report_label)
        return self.status_group

    def _build_left_panel(self) -> QWidget:
        """构建左侧日志和配置面板。注意：面板宽度不能挤压主画布。"""
        panel = QFrame()
        panel.setObjectName("panel")
        panel.setFixedWidth(216)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(10, 12, 10, 10)
        layout.setSpacing(10)
        layout.addWidget(self._build_status_group())
        # “配置”分组：用表单布局把标签与控件按行对齐。
        config_group = QGroupBox("配置")
        form = QFormLayout(config_group)
        form.setContentsMargins(10, 18, 10, 10)
        form.setHorizontalSpacing(8)
        form.setVerticalSpacing(8)
        # 当前配置名标签，开启自动换行避免长路径撑宽面板。
        self.config_name = QLabel("未选择")
        self.config_name.setWordWrap(True)
        choose_config = QPushButton("选择文件")
        choose_config.clicked.connect(self._choose_config)
        # 场景/算法下拉：窄面板内向右弹出菜单避免被裁切。
        self.scenario_select = SelectButton(132, popup_side="right")
        self.scenario_select.addItems(["三机楔形", "五机纵队", "受限重构"])
        self.algorithm_select = SelectButton(128, popup_side="right")
        self.algorithm_select.addItems(["Follow", "Consensus", "RuleBased"])
        self.duration_input = QLineEdit()
        self.duration_input.setObjectName("durationInput")
        self.duration_input.setMinimumWidth(96)
        self.duration_input.setPlaceholderText("秒")
        self.duration_input.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.duration_input.editingFinished.connect(self._on_duration_changed)
        form.addRow("配置", choose_config)
        form.addRow("", self.config_name)
        form.addRow("场景", self.scenario_select)
        form.addRow("算法", self.algorithm_select)
        form.addRow("时长(s)", self.duration_input)
        layout.addWidget(config_group)

        # “播放”分组：速度滑块范围 1..200，对应 0.1x..20.0x（见 _on_speed_changed 除以 10）。
        playback_group = QGroupBox("播放")
        playback_layout = QVBoxLayout(playback_group)
        playback_layout.setContentsMargins(10, 18, 10, 10)
        status_row = QHBoxLayout()
        self.cpu_label = QLabel("CPU 0%")
        self.cpu_label.setToolTip("仿真线程忙碌时间 / 墙钟统计周期")
        self.speed_label = QLabel("1.0x")
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(1, 200)
        self.speed_slider.setValue(10)  # 默认 1.0x
        self.speed_slider.valueChanged.connect(self._on_speed_changed)
        playback_layout.addWidget(self.speed_slider)
        status_row.addWidget(self.cpu_label)
        status_row.addStretch(1)
        status_row.addWidget(self.speed_label)
        playback_layout.addLayout(status_row)
        layout.addWidget(playback_group)

        # “运行期扰动”分组：四个按钮排成 2x2 网格。
        disturb_group = QGroupBox("运行期扰动")
        grid = QGridLayout(disturb_group)
        grid.setContentsMargins(10, 18, 10, 10)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(8)
        # (按钮文案, 扰动种类) ——种类传给 _inject_disturbance/适配器。
        actions: list[tuple[str, str]] = [
            ("风场脉冲", "wind"),
            ("节点故障", "fault"),
            ("链路丢包", "loss"),
            ("清除扰动", "clear"),
        ]
        for index, (text, kind) in enumerate(actions):
            button = QPushButton(text)
            # 默认参数绑定 kind，避免闭包共享同一变量的经典陷阱。
            button.clicked.connect(lambda checked=False, value=kind: self._inject_disturbance(value))
            # 收集按钮以便按运行态统一启用/禁用。
            self.disturbance_buttons.append(button)
            # index//2 为行、index%2 为列，铺成两行两列。
            grid.addWidget(button, index // 2, index % 2)
        layout.addWidget(disturb_group)

        # "演示场景"分组：快捷加载预置配置文件。
        demo_group = QGroupBox("演示场景")
        demo_layout = QVBoxLayout(demo_group)
        demo_layout.setContentsMargins(10, 18, 10, 10)
        demo_layout.setSpacing(8)
        btn_hold = QPushButton("编队保持")
        btn_hold.setToolTip("加载 configs/base.json — 三机楔形保持队形演示")
        btn_hold.clicked.connect(lambda: self._load_demo_config("base.json"))
        btn_rally = QPushButton("集结演示")
        btn_rally.setToolTip("加载 configs/rally_demo.json — 三机分散后集结演示")
        btn_rally.clicked.connect(lambda: self._load_demo_config("rally_demo.json"))
        demo_layout.addWidget(btn_hold)
        demo_layout.addWidget(btn_rally)
        layout.addWidget(demo_group)

        # 底部弹性占位把上面各分组顶到面板顶部。
        layout.addStretch(1)
        return panel

    def _build_avoidance_window(self) -> None:
        """构建避障规划子窗口。注意：窗口默认隐藏，通过菜单栏入口打开。"""
        dialog = AvoidanceWindow(self)
        self.avoidance_window = dialog
        # 子窗口自己留边距，避免在不同平台窗口边框下贴边。
        root = QVBoxLayout(dialog)
        root.setContentsMargins(14, 14, 14, 14)
        root.setSpacing(10)
        # 顶部只放标题和当前配置名，具体操作放右侧分组。
        header = QHBoxLayout()
        title = QLabel("避障规划")
        title.setObjectName("stageTitle")
        self.avoidance_config_label = QLabel("未加载配置")
        self.avoidance_config_label.setObjectName("reportPill")
        header.addWidget(title)
        header.addStretch(1)
        header.addWidget(self.avoidance_config_label)
        root.addLayout(header)

        # 三列布局与草图一致：障碍、参数、操作反馈。
        columns = QHBoxLayout()
        columns.setSpacing(12)
        root.addLayout(columns, 1)
        columns.addWidget(self._build_avoidance_obstacle_group(dialog), 0)
        columns.addWidget(self._build_avoidance_param_group(dialog), 1)
        columns.addWidget(self._build_avoidance_action_group(dialog), 0)

        # 初始化时还没有配置，先让各控件进入“不可生成”的一致状态。
        self._rebuild_obstacle_list()
        self._sync_avoidance_param_widgets()
        self._update_avoidance_status()

    def _build_avoidance_obstacle_group(self, parent: QWidget) -> QWidget:
        """构建避障窗口左侧障碍选择区。注意：列表内容随配置动态重建。"""
        group = QGroupBox("障碍选择", parent)
        group.setMinimumWidth(230)
        layout = QVBoxLayout(group)
        layout.setContentsMargins(10, 18, 10, 10)
        layout.setSpacing(8)
        # 障碍列表由 JSON 动态决定，每次加载配置后整体重建。
        self.obstacle_list_container = QWidget(group)
        self.obstacle_list_layout = QVBoxLayout(self.obstacle_list_container)
        self.obstacle_list_layout.setContentsMargins(0, 0, 0, 0)
        self.obstacle_list_layout.setSpacing(6)
        layout.addWidget(self.obstacle_list_container)
        # 摘要区替代草图里的说明卡，保留必要状态但不占太多空间。
        self.obstacle_summary = QLabel("")
        self.obstacle_summary.setObjectName("avoidHint")
        self.obstacle_summary.setWordWrap(True)
        layout.addWidget(self.obstacle_summary, 1)
        return group

    def _build_avoidance_param_group(self, parent: QWidget) -> QWidget:
        """构建避障窗口参数区。注意：顺序与设计文档第 8 节保持一致。"""
        group = QGroupBox("参数", parent)
        group.setMinimumWidth(380)
        layout = QVBoxLayout(group)
        layout.setContentsMargins(10, 18, 10, 10)
        layout.setSpacing(8)
        # tooltip 文案直接来自设计文档语义，避免界面标签过长。
        tips = {
            "turn_radius_m": (
                "作用：约束拐点圆弧的最小转弯半径。\n"
                "影响：越大转弯越平缓，但更容易腿太短或圆弧触障。\n"
                "建议：按飞机能力给定；无约束时先取 200~300 m。"
            ),
            "leg_length_margin_m": (
                "作用：要求相邻圆弧之间保留额外直线余度。\n"
                "影响：越大越保守，但更容易触发腿长不足。\n"
                "建议：R 确定后再调，先试 0.2R~0.5R。"
            ),
            "clearance_m": (
                "作用：A* 搜索时对障碍做外扩，形成安全边界。\n"
                "影响：越大越安全但绕行更远，窄通道更可能无路。\n"
                "建议：优先按业务安全距离，常用 80~150 m。"
            ),
            "grid.resolution_m": (
                "作用：决定 A* 栅格离散精度。\n"
                "影响：越小路径越细但更慢；越大更快但更粗。\n"
                "建议：先取小于等于 R/10，例如 R=300 m 时 20~30 m。"
            ),
            "grid.margin_m": (
                "作用：扩展起终点和障碍外侧的搜索包围盒。\n"
                "影响：越大绕行空间越足但网格规模增大。\n"
                "建议：先取安全间距 + 转弯半径，或直接取 300 m。"
            ),
            "simplify_clearance_m": (
                "作用：A* 后视线去冗余使用的障碍外扩距离。\n"
                "影响：越小越容易拉直、航段更少，但更贴近障碍。\n"
                "建议：初始等于安全间距；减少碎段时试 0.5 倍安全间距。"
            ),
            "turn_switch_penalty_m": (
                "作用：惩罚 A* 中每次 8 邻域方向切换。\n"
                "影响：越大越少频繁换向，但可能绕远或贴边。\n"
                "建议：减少碎段时从 1 倍栅格间距试起。"
            ),
            "turn_angle_weight_m": (
                "作用：按每 45° 航迹角变化增加线性代价。\n"
                "影响：可减少硬拐；过大时会和最短路目标拉扯。\n"
                "建议：最后再调，先试转向切换惩罚的 0.25~0.5 倍。"
            ),
        }
        # 物理约束区：前三项决定可飞性和安全边界。
        self.turn_radius_spin = self._make_param_spin(maximum=100000.0, step=10.0, tooltip=tips["turn_radius_m"])
        self.leg_margin_spin = self._make_param_spin(maximum=100000.0, step=10.0, tooltip=tips["leg_length_margin_m"])
        self.clearance_spin = self._make_param_spin(maximum=100000.0, step=10.0, tooltip=tips["clearance_m"])
        # 搜索范围区：分辨率和边界余量直接影响 A* 速度与可达性。
        self.resolution_spin = self._make_param_spin(maximum=100000.0, step=5.0, tooltip=tips["grid.resolution_m"])
        self.margin_spin = self._make_param_spin(maximum=100000.0, step=50.0, tooltip=tips["grid.margin_m"])
        # 去冗余安全距可独立调；旧配置未显式配置时会跟随安全间距。
        self.simplify_clearance_spin = self._make_param_spin(
            maximum=100000.0,
            step=10.0,
            tooltip=tips["simplify_clearance_m"],
            on_change=self._on_simplify_clearance_changed,
        )
        # 两个惩罚参数的单位不是普通米值，需要在输入框后缀里区分。
        self.turn_switch_penalty_spin = self._make_param_spin(
            maximum=100000.0,
            step=1.0,
            suffix=" m/次",
            tooltip=tips["turn_switch_penalty_m"],
        )
        self.turn_angle_weight_spin = self._make_param_spin(
            maximum=100000.0,
            step=1.0,
            suffix=" m/45°",
            tooltip=tips["turn_angle_weight_m"],
        )
        # 参数表格采用两列：左侧固定标签、右侧输入框吃掉剩余宽度。
        param_grid = QGridLayout()
        param_grid.setContentsMargins(0, 0, 0, 0)
        param_grid.setHorizontalSpacing(10)
        param_grid.setVerticalSpacing(8)
        param_grid.setColumnStretch(1, 1)
        # rows 顺序必须与 AvoidanceWindow.param_order 保持一致，测试会锁定。
        rows = [
            ("转弯半径 R", "turn_radius_m", self.turn_radius_spin),
            ("航段余度 L", "leg_length_margin_m", self.leg_margin_spin),
            ("安全间距", "clearance_m", self.clearance_spin),
            ("栅格间距", "grid.resolution_m", self.resolution_spin),
            ("搜索边界余量", "grid.margin_m", self.margin_spin),
            ("拉直安全间距", "simplify_clearance_m", self.simplify_clearance_spin),
            ("转向切换惩罚", "turn_switch_penalty_m", self.turn_switch_penalty_spin),
            ("航迹角惩罚", "turn_angle_weight_m", self.turn_angle_weight_spin),
        ]
        for row, (caption, key, spin) in enumerate(rows):
            label = QLabel(caption)
            label.setObjectName("paramLabel")
            # 标签和输入框都挂 tooltip，鼠标停在任一处都能看到解释。
            label.setMinimumWidth(104)
            label.setToolTip(tips[key])
            spin.setToolTip(tips[key])
            param_grid.addWidget(label, row, 0)
            param_grid.addWidget(spin, row, 1)
        layout.addLayout(param_grid)
        # allow_arc 与交接半径正交，保留为单独开关避免误归类到长度参数。
        self.allow_arc_check = QCheckBox("航段带圆弧")
        self.allow_arc_check.setToolTip(
            "开启：把连续贴同一障碍的拐点折叠成沿膨胀圆的大弧，并将直线-直线拐点烘焙成相切圆弧段，航段显示为曲线；"
            "关闭：仅保留直线骨架，拐点不折叠大弧，飞行时长机按转弯半径平滑过弯（显示为尖角）。"
        )
        self.allow_arc_check.toggled.connect(self._on_avoidance_param_changed)
        layout.addWidget(self.allow_arc_check)
        return group

    def _build_avoidance_action_group(self, parent: QWidget) -> QWidget:
        """构建避障窗口右侧操作与反馈区。注意：重置表示恢复配置默认航线。"""
        group = QGroupBox("操作与反馈", parent)
        group.setMinimumWidth(240)
        layout = QVBoxLayout(group)
        layout.setContentsMargins(10, 18, 10, 10)
        layout.setSpacing(8)
        # 生成只产生预览；采用才会替换控制器里的长机航线。
        button_row = QHBoxLayout()
        button_row.setSpacing(8)
        self.generate_route_button = QPushButton("生成航线")
        self.generate_route_button.clicked.connect(self._generate_route)
        self.adopt_route_button = QPushButton("采用航线")
        self.adopt_route_button.clicked.connect(self._adopt_route)
        self.adopt_route_button.setEnabled(False)
        button_row.addWidget(self.generate_route_button, 1)
        button_row.addWidget(self.adopt_route_button, 1)
        layout.addLayout(button_row)
        # 重置的语义是清除覆盖航线，而不是把参数恢复成配置值。
        self.reset_route_button = QPushButton("重置")
        self.reset_route_button.setToolTip("清除已采用的避障航线，恢复配置中的默认长机航线。")
        self.reset_route_button.clicked.connect(self._reset_avoidance_route)
        layout.addWidget(self.reset_route_button)
        # 状态区承载规划成功、失败原因和重置结果，避免弹窗打断调参。
        self.avoidance_status = QLabel("")
        self.avoidance_status.setObjectName("avoidHint")
        self.avoidance_status.setWordWrap(True)
        self.avoidance_status.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop)
        layout.addWidget(self.avoidance_status, 1)
        return group

    def _rebuild_obstacle_list(self) -> None:
        """按当前障碍集重建左面板勾选列表。注意：只改显示控件，不触发规划。"""
        # 清空旧复选框/占位标签。
        while self.obstacle_list_layout.count():
            item = self.obstacle_list_layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()
        self.obstacle_checkboxes = []
        if not self.obstacles:
            placeholder = QLabel("（当前配置无障碍）")
            placeholder.setObjectName("reportPill")
            self.obstacle_list_layout.addWidget(placeholder)
            self.generate_route_button.setEnabled(False)
            self.obstacle_summary.setText("当前配置未提供 avoidance.obstacles，避障规划不可用。")
            return
        self.generate_route_button.setEnabled(True)
        for obstacle in self.obstacles:
            checkbox = QCheckBox(obstacle.label())
            checkbox.setChecked(obstacle.enabled)
            # 默认参数绑定 obstacle，避免闭包共享变量。
            checkbox.toggled.connect(lambda checked, ob=obstacle: self._on_obstacle_toggled(ob, checked))
            self.obstacle_checkboxes.append(checkbox)
            self.obstacle_list_layout.addWidget(checkbox)
        # 摘要跟随勾选状态刷新，让用户不用手动数复选框。
        enabled = sum(1 for obstacle in self.obstacles if obstacle.enabled)
        self.obstacle_summary.setText(f"已启用 {enabled}/{len(self.obstacles)} 个障碍。\n安全膨胀随“安全间距”实时刷新。")

    def _make_param_spin(
        self,
        *,
        maximum: float,
        step: float,
        tooltip: str = "",
        suffix: str = " m",
        on_change: Callable[[float], None] | None = None,
    ) -> QDoubleSpinBox:
        """构造规划参数数值框（米，非负，无上下按钮，直接键入）。注意：值变更即让已有预览失效。"""
        spin = QDoubleSpinBox()
        spin.setRange(0.0, maximum)
        spin.setSingleStep(step)
        spin.setDecimals(1)
        spin.setSuffix(suffix)
        # 去掉上下微调按钮：直接键入数值。
        spin.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.NoButtons)
        if tooltip:
            spin.setToolTip(tooltip)
        spin.valueChanged.connect(on_change or self._on_avoidance_param_changed)
        return spin

    def _on_avoidance_param_changed(self, _value: object = None) -> None:
        """规划参数被用户调整：使已有预览失效（需按新参数重新生成）。注意：安全间距变化同步刷新膨胀圈显示。"""
        if (
            self.sender() is self.clearance_spin
            and self._avoidance_params is not None
            and not self._avoidance_params.simplify_clearance_explicit
        ):
            # 旧配置未显式给 simplify_clearance_m 时，它继续跟随安全间距。
            self.simplify_clearance_spin.blockSignals(True)
            self.simplify_clearance_spin.setValue(self.clearance_spin.value())
            self.simplify_clearance_spin.blockSignals(False)
        self._invalidate_preview()
        if self.obstacles:
            self.top_view.set_obstacles(self.obstacles, self.clearance_spin.value())

    def _on_simplify_clearance_changed(self, _value: object = None) -> None:
        """用户单独调整拉直安全间距。注意：一旦手改，即不再跟随安全间距联动。"""
        if self._avoidance_params is not None:
            # 用户手动改过拉直安全距后，后续安全间距变化不再覆盖它。
            self._avoidance_params.simplify_clearance_explicit = True
        self._on_avoidance_param_changed(_value)

    def _sync_avoidance_param_widgets(self) -> None:
        """把解析到的规划参数灌进界面控件。注意：无 avoidance 配置时禁用；编程赋值屏蔽信号避免误失效。"""
        params = self._avoidance_params
        has_params = params is not None
        widgets = (
            self.turn_radius_spin,
            self.leg_margin_spin,
            self.clearance_spin,
            self.resolution_spin,
            self.margin_spin,
            self.simplify_clearance_spin,
            self.turn_switch_penalty_spin,
            self.turn_angle_weight_spin,
            self.allow_arc_check,
            self.generate_route_button,
            self.reset_route_button,
        )
        for widget in widgets:
            widget.setEnabled(has_params)
        if not has_params:
            # 没有 avoidance 或有效航点时，采用按钮也必须保持禁用。
            self.adopt_route_button.setEnabled(False)
            return
        # 配置值灌入控件时屏蔽信号，避免加载配置被误判为用户调参。
        for spin, value in (
            (self.turn_radius_spin, params.turn_radius_m),
            (self.leg_margin_spin, params.leg_margin_m),
            (self.clearance_spin, params.clearance_m),
            (self.resolution_spin, params.resolution_m),
            (self.margin_spin, params.margin_m),
            (self.simplify_clearance_spin, params.simplify_clearance_m),
            (self.turn_switch_penalty_spin, params.turn_switch_penalty_m),
            (self.turn_angle_weight_spin, params.turn_angle_weight_m),
        ):
            spin.blockSignals(True)
            spin.setValue(value)
            spin.blockSignals(False)
        self.allow_arc_check.blockSignals(True)
        self.allow_arc_check.setChecked(params.allow_arc)
        self.allow_arc_check.blockSignals(False)

    def _on_obstacle_toggled(self, obstacle: ObstacleView, checked: bool) -> None:
        """勾选/取消某障碍。注意：勾选集变化使已生成的预览失效，需重新生成。"""
        obstacle.enabled = checked
        self._invalidate_preview()
        self.top_view.viewport().update()
        self._rebuild_obstacle_list()
        self._update_avoidance_status()

    def _invalidate_preview(self) -> None:
        """清除当前预览航线并禁用“采用”。注意：障碍勾选/配置变化后调用。"""
        self._preview_route = None
        self.top_view.set_preview_route(None)
        if hasattr(self, "adopt_route_button"):
            self.adopt_route_button.setEnabled(False)

    def _update_avoidance_status(self) -> None:
        """空闲时在反馈区显示操作提示（生成成功/失败时由 _generate_route 覆盖）。"""
        if not self.obstacles:
            self.avoidance_status.setText("未加载障碍：当前配置无 avoidance.obstacles。")
            return
        enabled = sum(1 for obstacle in self.obstacles if obstacle.enabled)
        self.avoidance_status.setText(
            f"已勾选 {enabled}/{len(self.obstacles)} 个障碍。\n设置参数后点「生成航线」预览，满意再「采用航线」。"
        )

    def _set_obstacles_from_config(self, path: str) -> None:
        """从配置文件解析障碍与规划参数并刷新显示。注意：解析失败时清空，保持界面一致。"""
        obstacles, clearance = parse_avoidance_config(path)
        self.obstacles = obstacles
        self._avoidance_params = parse_avoidance_params(path)
        if hasattr(self, "avoidance_config_label"):
            self.avoidance_config_label.setText(Path(path).name)
        self.top_view.set_obstacles(obstacles, clearance)
        self._invalidate_preview()
        self._rebuild_obstacle_list()
        self._sync_avoidance_param_widgets()
        self._update_avoidance_status()

    def _generate_route(self) -> None:
        """响应“生成航线”：跑 plan_avoidance_route，成功则预览，失败则显示 ERR_AVOID_* 原因。"""
        if self._avoidance_params is None or len(self._avoidance_params.waypoints) < 2:
            self._invalidate_preview()
            self.avoidance_status.setText("当前配置无可规划航线（缺 route.waypoints 或 avoidance）")
            return
        params = self._avoidance_params
        # 只把当前勾选项交给后端；未勾选障碍仍留在库里但不参与规划。
        enabled = [_obstacle_view_to_backend(ob) for ob in self.obstacles if ob.enabled]
        if not enabled:
            # 未选择任何障碍：等价于维持原航线，不生成 R 圆弧航线，也不允许采用。
            self._invalidate_preview()
            self.avoidance_status.setText("未选择障碍 · 维持原航线")
            self._log("Avoid", "未选择障碍，跳过生成（维持原航线）")
            return
        # 规划参数以界面控件为准（用户可现场调），覆盖配置解析值。
        # 旧配置未显式配置 simplify_clearance_m 时，让去冗余安全距跟随当前安全间距控件，保持旧行为。
        clearance_m = self.clearance_spin.value()
        simplify_clearance_m = self.simplify_clearance_spin.value()
        try:
            # 所有可调参数均以子窗口当前值为准，覆盖加载时的配置快照。
            result = plan_avoidance_route(
                params.waypoints,
                enabled,
                turn_radius_m=self.turn_radius_spin.value(),
                leg_margin_m=self.leg_margin_spin.value(),
                clearance_m=clearance_m,
                simplify_clearance_m=simplify_clearance_m,
                turn_switch_penalty_m=self.turn_switch_penalty_spin.value(),
                turn_angle_weight_m=self.turn_angle_weight_spin.value(),
                speed_mps=params.speed_mps,
                resolution_m=self.resolution_spin.value(),
                margin_m=self.margin_spin.value(),
                allow_arc=self.allow_arc_check.isChecked(),
            )
        except ValueError as exc:
            self._invalidate_preview()
            self.avoidance_status.setText(f"参数错误：{exc}")
            self._log("WARN", f"生成航线参数错误：{exc}")
            return
        if result.ok and result.route is not None:
            self._preview_route = result.route
            # 预览线只进画布，不进入控制器，直到用户点击“采用航线”。
            self.top_view.set_preview_route(route_to_polyline(result.route), preview_route_marker_points(result.route))
            _preview_lines = waypoint_inputs_to_waylines(result.route)
            arcs = sum(1 for line in _preview_lines if line.start.turnSign != 0.0)
            self.adopt_route_button.setEnabled(True)
            self.avoidance_status.setText(f"预览就绪：{len(_preview_lines)} 段（{arcs} 圆弧）· 可采用")
            self._log("Avoid", f"生成航线成功：{len(_preview_lines)} 段，{arcs} 圆弧")
        else:
            self._invalidate_preview()
            self.avoidance_status.setText(f"{result.code}：{result.detail}")
            self._log("Avoid", f"生成航线失败 {result.code}: {result.detail}")

    def _adopt_route(self) -> None:
        """响应“采用航线”：把预览航线下发控制器替换长机航线（采用后点播放仿真）。"""
        if self._preview_route is None:
            return
        preview_route = self._preview_route
        snapshot = self.sim.apply_avoidance_route(preview_route)
        if self.sim.last_result_code == "OK":
            # 采用成功后 committed 航线已更新，绿色预览线必须清掉，避免同线重复绘制。
            self._invalidate_preview()
        self._update_snapshot(snapshot, fit_top_view=False)
        if self.sim.last_result_code == "OK":
            self.avoidance_status.setText("已采用避障航线 · 点播放仿真")
            self._log("Avoid", "已采用避障航线，长机航线已替换")
        else:
            self.avoidance_status.setText(f"采用失败 {self.sim.last_result_code}")
            self._log("WARN", f"采用航线失败 {self.sim.last_result_code}: {self.sim.last_result_message}")

    def _reset_avoidance_route(self) -> None:
        """响应“重置”：清除已采用避障航线，恢复配置默认长机航线。"""
        self._invalidate_preview()
        snapshot = self.sim.clear_avoidance_route()
        self._update_snapshot(snapshot, fit_top_view=False)
        if self.sim.last_result_code == "OK":
            # 控制器已回到配置航线，画布快照也同步刷新。
            self.avoidance_status.setText("已恢复默认航线")
            self._log("Avoid", "已清除避障航线，恢复默认航线")
        else:
            self.avoidance_status.setText(f"重置失败 {self.sim.last_result_code}")
            self._log("WARN", f"重置航线失败 {self.sim.last_result_code}: {self.sim.last_result_message}")

    def _open_avoidance_window(self) -> None:
        """打开避障规划子窗口。注意：重复触发只激活已有窗口。"""
        if self.avoidance_window is None:
            return
        self.avoidance_window.show()
        self.avoidance_window.raise_()
        self.avoidance_window.activateWindow()

    def _build_stage(self) -> QWidget:
        """构建中央仿真画布区域。注意：俯视图和侧视图需要共享横向视野。"""
        stage = QFrame()
        stage.setObjectName("panel")
        layout = QVBoxLayout(stage)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # 画布顶部工具条：标题 + 全屏按钮 + 图例 + 网格/居中/重置开关。
        toolbar = QHBoxLayout()
        toolbar.setContentsMargins(12, 10, 12, 10)
        toolbar.setSpacing(8)
        title = QLabel("二维实时显示")
        title.setObjectName("stageTitle")
        # 全屏切换按钮（⛶），保存引用以便切换其图标/提示。
        fullscreen = QPushButton("⛶")
        fullscreen.setFixedSize(30, 30)
        fullscreen.clicked.connect(self._toggle_fullscreen)
        self.fullscreen_button = fullscreen
        toolbar.addWidget(title)
        toolbar.addWidget(fullscreen)
        toolbar.addStretch(1)
        # 图例标签（颜色由样式表按 objectName 着色）。
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
        # 网格开关默认开；居中/重置视图绑定到对应槽函数。
        self.grid_toggle = QCheckBox("网格")
        self.grid_toggle.setChecked(True)
        self.grid_toggle.stateChanged.connect(self._on_grid_changed)
        self.auto_center = QCheckBox("自动居中")
        self.auto_center.stateChanged.connect(self._on_auto_center_changed)
        self.segment_lock = QCheckBox("航段锁定")
        self.segment_lock.setChecked(True)
        self.segment_lock.stateChanged.connect(self._on_segment_lock_changed)
        self.view_angle_input = QSpinBox()
        self.view_angle_input.setRange(0, 360)
        self.view_angle_input.setPrefix("视角 ")
        self.view_angle_input.setSuffix("°")
        self.view_angle_input.setValue(0)
        self.view_angle_input.setFixedWidth(118)
        self.view_angle_input.valueChanged.connect(self._on_view_angle_changed)
        self.view_angle_slider = QSlider(Qt.Orientation.Horizontal)
        self.view_angle_slider.setRange(0, 360)
        self.view_angle_slider.setValue(0)
        self.view_angle_slider.setFixedWidth(116)
        self.view_angle_slider.valueChanged.connect(self._on_view_angle_changed)
        reset_view = QPushButton("重置视图")
        reset_view.clicked.connect(self._reset_view)
        toolbar.addWidget(self.grid_toggle)
        toolbar.addWidget(self.auto_center)
        toolbar.addWidget(self.segment_lock)
        toolbar.addWidget(self.view_angle_input)
        toolbar.addWidget(self.view_angle_slider)
        toolbar.addWidget(reset_view)
        layout.addLayout(toolbar)

        # 创建俯视图与侧视图；侧视图独立维护高度轴和横向投影轴。
        self.top_view = TopView()
        self.side_view = SideView(self.top_view)
        # 信号联动：俯视图手动操作 -> 关闭自动居中；重置 -> 侧视图也恢复默认显示范围。
        self.top_view.viewChanged.connect(self.side_view.update)
        self.top_view.manualViewChanged.connect(self._disable_auto_center)
        self.top_view.resetViewRequested.connect(self.side_view.reset_view)
        # 俯视图/侧视图之间用细分隔线承载拖动调整，不额外占用明显空间。
        self.view_splitter = QSplitter(Qt.Orientation.Vertical)
        self.view_splitter.setObjectName("viewSplitter")
        self.view_splitter.setChildrenCollapsible(False)
        self.view_splitter.setHandleWidth(7)
        self.view_splitter.addWidget(self.top_view)
        self.view_splitter.addWidget(self.side_view)
        self.view_splitter.setStretchFactor(0, 4)
        self.view_splitter.setStretchFactor(1, 1)
        self.view_splitter.setSizes([620, 180])
        layout.addWidget(self.view_splitter, 1)

        # 底部时间轴：时间文本 + 控制按钮 + 进度条。
        timeline = QHBoxLayout()
        timeline.setContentsMargins(12, 6, 12, 6)
        self.timeline_label = QLabel("0.0 / 120s")
        self.play_button = QPushButton("开始")
        self.step_button = QPushButton("单步")
        self.reset_button = QPushButton("重置")
        # 进度条用 0..1000 的千分刻度承载 time/duration 比例，便于平滑显示。
        self.progress = QProgressBar()
        self.progress.setObjectName("progress")
        self.progress.setRange(0, 1000)
        self.progress.setTextVisible(False)
        # 播放/暂停合并为一个按钮，文案随运行态切换为当前可执行动作。
        self.play_button.clicked.connect(self._toggle_play_pause)
        self.step_button.clicked.connect(self._step)
        self.reset_button.clicked.connect(self._reset)
        for widget in [self.timeline_label, self.play_button, self.step_button, self.reset_button, self.progress]:
            timeline.addWidget(widget)
        # 让进度条吃掉剩余横向空间。
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
        # 节点误差表、整体跟踪表和链路表分开显示，避免把全局航线指标误认为单机误差。
        self.node_table = QTableWidget(0, 5)
        self.node_table.setHorizontalHeaderLabels(["ID", "前向(m)", "垂向(m)", "侧向(m)", "状态"])
        self.overall_table = QTableWidget(0, 5)
        self.overall_table.setHorizontalHeaderLabels(["侧偏(m)", "待飞距(m)", "高度(m)", "地速(m/s)", "天向速度(m/s)"])
        self.link_table = QTableWidget(0, 5)
        self.link_table.setHorizontalHeaderLabels(["链路", "方向", "延迟", "丢包", "状态"])
        self._configure_table(self.node_table, [48, 88, 88, 88, 50], expandable=True)
        self._configure_table(self.overall_table, [60, 72, 62, 70, 92], height=64)
        self._configure_table(self.link_table, [86, 52, 58, 50, 54], expandable=True)
        node_title = QLabel("节点跟踪误差")
        node_title.setObjectName("sectionTitle")
        overall_title = QLabel("整体跟踪情况")
        overall_title.setObjectName("sectionTitle")
        link_title = QLabel("链路状态")
        link_title.setObjectName("sectionTitle")
        layout.addWidget(overall_title)
        layout.addWidget(self.overall_table)
        layout.addSpacing(8)
        layout.addWidget(node_title)
        layout.addWidget(self.node_table, 1)
        layout.addSpacing(8)
        layout.addWidget(link_title)
        layout.addWidget(self.link_table, 1)
        return panel

    def _configure_table(
        self, table: QTableWidget, widths: list[int], *, height: int = 138, expandable: bool = False
    ) -> None:
        """配置状态表通用样式。注意：表格只读且不显示多余行号。"""
        # 隐藏行号列；横向滚动条默认关闭（靠列宽与末列拉伸控制），纵向按需出现。
        table.verticalHeader().setVisible(False)
        table.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        table.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        # 忽略内容自适应尺寸，避免表格随数据撑大破坏面板布局。
        table.setSizeAdjustPolicy(QAbstractScrollArea.SizeAdjustPolicy.AdjustIgnored)
        table.setAlternatingRowColors(False)
        # 表格只读、不可选中。
        table.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        header = table.horizontalHeader()
        # 最后一列拉伸吃掉余宽；其余列固定为指定最小宽度，避免表格内部留空。
        header.setStretchLastSection(False)
        for index, width in enumerate(widths[:-1]):
            header.setSectionResizeMode(index, QHeaderView.ResizeMode.Fixed)
            table.setColumnWidth(index, width)
        last_index = len(widths) - 1
        table.setColumnWidth(last_index, widths[last_index])
        header.setSectionResizeMode(last_index, QHeaderView.ResizeMode.Stretch)
        # 固定行高；节点/链路表优先吃掉面板剩余高度，空间不足时再出现纵向滚动条。
        table.verticalHeader().setDefaultSectionSize(30)
        table.verticalHeader().setMinimumSectionSize(30)
        if expandable:
            table.setMinimumHeight(height)
            table.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        else:
            table.setFixedHeight(height)

    @staticmethod
    def _centered_table_item(value: str) -> QTableWidgetItem:
        """创建居中显示的表格单元格。注意：三张状态表都保持一致对齐。"""
        item = QTableWidgetItem(value)
        item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        return item

    def _install_button_cursors(self) -> None:
        """为按钮安装手型光标。注意：只影响交互提示，不改变按钮逻辑。"""
        for button in self.findChildren(QPushButton):
            button.setCursor(Qt.CursorShape.PointingHandCursor)

    def _apply_theme(self) -> None:
        """应用 theme 设置。注意：只修改对应显示或运行参数。"""
        # 由当前主题派生若干交互态颜色（悬停/按下/选中），统一注入 Qt 样式表。
        theme = self.theme
        button_hover = theme.line.lighter(108)
        button_pressed = theme.line.darker(108)
        button_border_hover = theme.accent
        menu_selected = theme.line.lighter(112)
        # 用 f-string 把主题色名填入 QSS；注意 QSS 的花括号在 f-string 中需写成 {{ }}。
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
            QLineEdit {{
                background: {theme.field.name()};
                color: {theme.ink.name()};
                border: 1px solid {theme.line.name()};
                border-radius: 6px;
                min-height: 30px;
                padding: 0 10px;
            }}
            QLineEdit:focus {{
                border-color: {theme.accent.name()};
            }}
            QLineEdit:disabled {{
                color: {theme.muted.name()};
                background: {theme.line.name()};
                border-color: {theme.line.name()};
            }}
            QDoubleSpinBox {{
                background: {theme.field.name()};
                color: {theme.ink.name()};
                border: 1px solid {theme.line.name()};
                border-radius: 6px;
                min-height: 26px;
                padding: 0 6px;
            }}
            QDoubleSpinBox:focus {{
                border-color: {theme.accent.name()};
            }}
            QDoubleSpinBox:disabled {{
                color: {theme.muted.name()};
                background: {theme.line.name()};
                border-color: {theme.line.name()};
            }}
            QLabel#paramLabel {{
                color: {theme.ink.name()};
            }}
            QLabel#avoidHint {{
                color: {theme.muted.name()};
                background: {theme.field.name()};
                border: 1px solid {theme.line.name()};
                border-radius: 6px;
                padding: 8px 10px;
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
            QSplitter#viewSplitter {{
                background: transparent;
            }}
            QSplitter#viewSplitter::handle:vertical {{
                background: transparent;
                border-top: 1px solid {theme.line.name()};
            }}
            QSplitter#viewSplitter::handle:vertical:hover {{
                border-top-color: {theme.accent.name()};
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
        # 样式表只管控件；画布颜色需单独下发给两个自绘视图。
        self.top_view.set_theme(theme)
        self.side_view.set_theme(theme)

    def _update_snapshot(self, snapshot: Snapshot, *, fit_top_view: bool = False, fit_side_view: bool = False) -> None:
        """更新 snapshot 状态。注意：保持界面显示和内部数据一致。"""
        # 左侧状态文本与时间轴。
        self.run_state_label.setText(snapshot.run_state)
        self.report_label.setText(f"回报：{snapshot.control_report}")
        self.timeline_label.setText(f"{snapshot.time:.1f} / {snapshot.duration:.0f}s")
        self.cpu_label.setText(f"CPU {snapshot.cpu_utilization * 100:.0f}%")
        self._sync_duration_input(snapshot)
        # 进度 = time/duration 换算到千分刻度；duration 为 0 时置 0 防除零。
        self.progress.setValue(round(snapshot.time / snapshot.duration * 1000) if snapshot.duration else 0)
        # 依据运行态启停各按钮：未加载配置(UNLOADED)时全部相关操作不可用。
        config_loaded = snapshot.run_state != "UNLOADED"
        self.play_button.setEnabled(config_loaded and snapshot.run_state != "FINISHED")
        self.step_button.setEnabled(snapshot.run_state in {"READY", "PAUSED"})
        self.reset_button.setEnabled(config_loaded)
        # 仿真结束后禁止再注入扰动。
        for button in self.disturbance_buttons:
            button.setEnabled(config_loaded and snapshot.run_state != "FINISHED")
        # 单个播放控制按钮始终显示“点下去会发生什么”。
        self.play_button.setText({"RUNNING": "暂停", "PAUSED": "继续"}.get(snapshot.run_state, "开始"))
        # 把快照下发给两视图与状态表；仅在需要时让视图自适应铺满。
        self.top_view.set_snapshot(snapshot, fit_view=fit_top_view)
        self.side_view.set_snapshot(snapshot)
        if fit_side_view:
            self.side_view.reset_view()
        self._sync_side_view_controls()
        self._update_tables(snapshot)

    def _update_tables(self, snapshot: Snapshot) -> None:
        """更新 tables 状态。注意：保持界面显示和内部数据一致。"""
        # 节点表：逐机显示航迹系三轴位置误差，整体航线指标拆到单独表格。
        self.node_table.setRowCount(len(snapshot.nodes))
        for row, node in enumerate(snapshot.nodes):
            # 健康枚举翻译成中文；未知值原样显示。
            status = {"normal": "正常", "degraded": "降级", "fault": "故障", "lost": "失联"}.get(node.health, node.health)
            # 节点表固定展示五列；rally_phase 仅保留在快照中，不写入隐藏的越界列。
            values = [
                node.node_id,
                f"{node.track_pos_err_x:.1f}",
                f"{node.track_pos_err_y:.1f}",
                f"{node.track_pos_err_z:.1f}",
                status,
            ]
            for column, value in enumerate(values):
                self.node_table.setItem(row, column, self._centered_table_item(value))

        # 整体跟踪表：用长机代表当前全局航线跟踪情况，缺少显式长机时才回退首节点。
        self.overall_table.setRowCount(1 if snapshot.nodes else 0)
        if snapshot.nodes:
            leader = leader_node_from(snapshot.nodes)
            assert leader is not None
            side_offset = leader.cross_track_error
            if side_offset is None:
                side_offset = (leader.y - WORLD_HEIGHT / 2) * 0.8
            distance_to_go = leader.distance_to_go
            if distance_to_go is None:
                distance_to_go = max(0.0, (WORLD_WIDTH - leader.x) * 4)
            # 地速按水平面速度模长显示，不把垂向爬升率计入整体跟踪表。
            ground_speed = math.hypot(leader.vx, leader.vy)
            values = [
                f"{side_offset:.0f}",
                f"{distance_to_go:.0f}",
                f"{leader.altitude:.0f}",
                f"{ground_speed:.0f}",
                f"{leader.vertical_speed:.0f}",
            ]
            for column, value in enumerate(values):
                self.overall_table.setItem(0, column, self._centered_table_item(value))

        # 链路表：丢包率换算成百分比，ok 标志映射为正常/丢包文案。
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
                self.link_table.setItem(row, column, self._centered_table_item(value))

    def _toggle_play_pause(self) -> None:
        """响应播放/暂停按钮。注意：按钮文案显示下一步动作。"""
        # RUNNING 下执行暂停，其余可用状态执行开始/继续；禁用态不会触发此槽。
        if self.sim.snapshot().run_state == "RUNNING":
            self._pause()
        else:
            self._start()

    def _start(self) -> None:
        """响应开始按钮并启动仿真。注意：需要同步按钮状态和日志。"""
        snapshot = self.sim.start()
        self._update_snapshot(snapshot)
        # 只有控制器确认 OK 才开启刷新定时器，避免空转。
        if self.sim.last_result_code == "OK":
            self.timer.start()
            if self._live_monitor is not None:
                self._live_monitor.follow(self.sim.controller)
        self._log("UI", f"start -> {self.sim.last_result_code}, state={snapshot.run_state}")

    def _pause(self) -> None:
        """响应暂停按钮并切换暂停状态。注意：暂停不清空当前快照。"""
        snapshot = self.sim.pause()
        # 暂停/继续切换：停或起刷新定时器与运行态保持一致。
        if snapshot.run_state == "PAUSED":
            self.timer.stop()
        elif snapshot.run_state == "RUNNING":
            self.timer.start()
        self._update_snapshot(snapshot)
        self._log("UI", f"pause/start -> {self.sim.last_result_code}, state={snapshot.run_state}")

    def _step(self) -> None:
        """响应单步按钮并推进一拍。注意：单步后界面需要立即刷新。"""
        # 单步前先停掉自动刷新，确保只前进一拍。
        self.timer.stop()
        snapshot = self.sim.single_step()
        self._update_snapshot(snapshot)
        self._log("UI", f"step -> {self.sim.last_result_code}, state={snapshot.run_state}")
        if self.sim.last_result_code == "OK" and self._live_monitor is not None:
            self._live_monitor.follow(self.sim.controller)

    def _reset(self) -> None:
        """响应重置按钮并恢复初始状态。注意：保留当前配置路径。"""
        self.timer.stop()
        snapshot = self.sim.reset()
        # 重置后队形回到初值，请求俯视图与侧视图重新自适应铺满。
        self._update_snapshot(snapshot, fit_top_view=True, fit_side_view=True)
        if self._live_monitor is not None:
            self._live_monitor.unfollow()
        self._log("SimControl", f"reset -> {self.sim.last_result_code}, state={snapshot.run_state}")

    def _on_tick(self) -> None:
        """处理 tick 信号回调。注意：回调内避免耗时操作阻塞界面。"""
        # 定时轮询最新快照刷新界面（不推进仿真，推进在控制器线程内进行）。
        snapshot = self.sim.poll()
        self._update_snapshot(snapshot)
        # 进入非运行态(就绪/暂停/结束)就停掉定时器，省去无谓刷新。
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

    def _load_demo_config(self, filename: str) -> None:
        """加载 configs/ 目录下的预置演示配置。注意：文件不存在时记录告警。"""
        path = self.project_root / "configs" / filename
        if not path.exists():
            self._log("WARN", f"演示配置不存在：{path}")
            return
        self._apply_config_path(str(path))

    def _choose_config(self) -> None:
        """处理 config 选择流程。注意：用户取消时不改变当前配置。"""
        # 起始目录优先用上次配置所在目录，过滤常见配置扩展名。
        path, _ = QFileDialog.getOpenFileName(
            self,
            "选择配置文件",
            str(self._config_dialog_start_dir()),
            "Config (*.yaml *.yml *.json)",
        )
        # 用户取消则 path 为空，保持现状不变。
        if not path:
            return
        self._apply_config_path(path)

    def _apply_config_path(self, path: str, *, remember: bool = True) -> None:
        """应用 config path 设置。注意：只修改对应显示或运行参数。"""
        # 切换配置前停掉定时器，加载后请求自适应铺满新场景。
        self.timer.stop()
        snapshot = self.sim.load_config(path)
        if self.sim.last_result_code == "OK":
            self._sync_speed_controls(self.sim.speed)
            # 先注入障碍再铺满，使自适应视野包含障碍区。
            self._set_obstacles_from_config(path)
        self._update_snapshot(snapshot, fit_top_view=True, fit_side_view=True)
        if self.sim.last_result_code == "OK":
            # 成功：更新配置名标签/提示，并按需把该路径记入 config.ini。
            config_path = Path(path).resolve()
            self.current_config_path = config_path
            display_path = self._display_config_path(config_path)
            self.config_name.setText(display_path)
            self.config_name.setToolTip(display_path)
            self._log("Config", f"加载配置文件 {display_path}")
            # remember=False 用于“自动加载上次配置”场景，避免重复写回。
            if remember:
                self._save_last_config_path(config_path)
            if self._live_monitor is not None:
                self._live_monitor.unfollow()
        else:
            # 失败只记录告警，不改动当前已加载配置。
            self._log("WARN", f"加载配置失败 {Path(path).name}: {self.sim.last_result_message}")

    def _sync_speed_controls(self, speed: float) -> None:
        """同步 speed controls 显示。注意：程序设置滑条时不重复下发倍率。"""
        # 配置加载后控制器已持有倍率，这里只让滑条和文本追上当前真实倍率。
        slider_value = max(self.speed_slider.minimum(), min(self.speed_slider.maximum(), round(speed * 10)))
        with QSignalBlocker(self.speed_slider):
            self.speed_slider.setValue(slider_value)
        self.speed_label.setText(f"{speed:.1f}x")

    def _config_dialog_start_dir(self) -> Path:
        """处理 dialog start dir 配置路径。注意：兼容源码运行和打包运行路径。"""
        # config.ini 里存的是相对项目根的路径；据此推出对话框起始目录。
        relative_path = self._read_last_config_path()
        if relative_path is None:
            return self.project_root
        config_path = (self.project_root / relative_path).resolve()
        candidate = config_path.parent
        # 目录已不存在则退回项目根，避免对话框打开到无效位置。
        return candidate if candidate.exists() else self.project_root

    def _display_config_path(self, path: Path) -> str:
        """生成 config path 显示文本。注意：仅用于界面展示。"""
        # 优先展示相对项目根的路径，无法相对化时退而显示文件名。
        relative_path = self._relative_to_project_root(path)
        return relative_path if relative_path is not None else path.name

    def _load_last_config_from_state(self) -> None:
        """加载上次使用的配置路径。注意：路径不存在时回退到默认配置。"""
        relative_path = self._read_last_config_path()
        if relative_path is None:
            return
        config_path = (self.project_root / relative_path).resolve()
        # 记录的配置文件可能已被删除/移动，缺失时仅告警不报错。
        if not config_path.exists():
            self._log("WARN", f"config.ini 指向的配置不存在：{relative_path}")
            return
        # remember=False：这是自动恢复，不需要再次写回。
        self._apply_config_path(str(config_path), remember=False)

    def _read_last_config_path(self) -> str | None:
        """读取 last config path 数据。注意：缺省或失败时应使用安全兜底。"""
        # 文件不存在直接返回空（首次运行属正常情况）。
        if not self.config_state_path.exists():
            return None
        parser = ConfigParser()
        try:
            parser.read(self.config_state_path, encoding="utf-8")
        except OSError as exc:
            # 读失败不致命，记录告警并按“无记录”处理。
            self._log("WARN", f"读取 config.ini 失败：{exc}")
            return None
        # 取 [config] last_config；空白串归一化为 None。
        value = parser.get(APP_CONFIG_SECTION, APP_CONFIG_KEY_LAST_CONFIG, fallback="").strip()
        return value or None

    def _save_last_config_path(self, path: Path) -> None:
        """保存 last config path 数据。注意：写入失败不应影响主仿真流程。"""
        # 只记录相对路径，便于项目整体移动后仍能定位；不可相对化则放弃记忆。
        relative_path = self._relative_to_project_root(path)
        if relative_path is None:
            self._log("WARN", "配置路径无法相对到程序目录，未更新 config.ini")
            return
        parser = ConfigParser()
        parser[APP_CONFIG_SECTION] = {APP_CONFIG_KEY_LAST_CONFIG: relative_path}
        # 确保父目录存在再写。
        self.config_state_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            with self.config_state_path.open("w", encoding="utf-8") as handle:
                parser.write(handle)
        except OSError as exc:
            # 写盘失败不应中断主流程，记录告警即可。
            self._log("WARN", f"写入 config.ini 失败：{exc}")

    def _relative_to_project_root(self, path: Path) -> str | None:
        """计算 to project root 相对路径。注意：路径不可相对化时返回原始路径。"""
        try:
            # Windows 上跨盘符无法相对化会抛 ValueError。
            relative_path = os.path.relpath(path.resolve(), self.project_root)
        except ValueError:
            return None
        # relpath 仍返回绝对路径（如不同盘）则视为不可相对化。
        if os.path.isabs(relative_path):
            return None
        try:
            # 统一用正斜杠存储，跨平台一致。
            return Path(relative_path).as_posix()
        except ValueError:
            return None

    def _on_speed_changed(self, value: int) -> None:
        """处理 speed changed 信号回调。注意：回调内避免耗时操作阻塞界面。"""
        # 滑块整数值 / 10 得到倍率（1->0.1x, 10->1.0x, 200->20.0x）。
        speed = value / 10.0
        self.sim.set_speed(speed)
        self.speed_label.setText(f"{speed:.1f}x")

    def _on_segment_lock_changed(self) -> None:
        """处理 segment lock changed 信号回调。注意：只改变侧视图显示方式。"""
        checked = self.segment_lock.isChecked()
        previous_angle = self.side_view.current_view_angle_deg()
        if self.segment_lock.isEnabled():
            self._segment_lock_preferred = checked
        if not checked:
            self.side_view.view_angle_deg = previous_angle
        self.side_view.set_segment_locked(checked)
        self._sync_side_view_controls()

    def _on_view_angle_changed(self, value: int) -> None:
        """处理 view angle changed 信号回调。注意：航段锁定时滑条只显示自动值。"""
        with QSignalBlocker(self.view_angle_input):
            self.view_angle_input.setValue(value)
        with QSignalBlocker(self.view_angle_slider):
            self.view_angle_slider.setValue(value)
        if not self.segment_lock.isChecked():
            self.side_view.set_view_angle_deg(float(value))

    def _sync_side_view_controls(self) -> None:
        """同步侧视图控制状态。注意：程序刷新控件时不触发用户回调。"""
        lock_available = self.side_view.lock_available()
        locked = lock_available and self._segment_lock_preferred

        with QSignalBlocker(self.segment_lock):
            self.segment_lock.setEnabled(lock_available)
            self.segment_lock.setChecked(locked)
        if self.side_view.segment_locked != locked:
            self.side_view.set_segment_locked(locked)

        angle = round(self.side_view.current_view_angle_deg()) % 360
        with QSignalBlocker(self.view_angle_input):
            self.view_angle_input.setEnabled(not locked)
            self.view_angle_input.setValue(angle)
        with QSignalBlocker(self.view_angle_slider):
            self.view_angle_slider.setEnabled(not locked)
            self.view_angle_slider.setValue(angle)

    def _on_duration_changed(self) -> None:
        """处理 duration changed 信号回调。注意：只在非运行态下更新控制器时长。"""
        try:
            duration_s = float(self.duration_input.text())
        except ValueError:
            self._log("WARN", f"非法仿真时长：{self.duration_input.text()}")
            self._sync_duration_input(self.sim.snapshot())
            return
        snapshot = self.sim.set_duration(duration_s)
        self._update_snapshot(snapshot)
        if self.sim.last_result_code == "OK":
            try:
                self._persist_config_duration(duration_s)
            except Exception as exc:  # noqa: BLE001
                self._log("WARN", f"写入配置时长失败：{exc}")
            self._log("Config", f"设置仿真时长 {duration_s:g}s")
        else:
            self._log("WARN", f"设置仿真时长失败：{self.sim.last_result_message}")
            self._sync_duration_input(self.sim.snapshot())

    def _persist_config_duration(self, duration_s: float) -> None:
        """把当前仿真时长写回配置文件。注意：只更新 duration_s 字段。"""
        if self.current_config_path is None:
            raise ValueError("未加载配置文件")
        path = self.current_config_path
        suffix = path.suffix.lower()
        text = path.read_text(encoding="utf-8")
        if suffix == ".json":
            config = json.loads(text)
            if not isinstance(config, dict):
                raise ValueError("config root must be an object")
            config["duration_s"] = duration_s
            path.write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            return
        if suffix in {".yaml", ".yml"}:
            try:
                import yaml
            except ImportError as exc:  # pragma: no cover - 依赖运行环境
                raise ValueError("YAML config requires PyYAML") from exc
            config = yaml.safe_load(text)
            if not isinstance(config, dict):
                raise ValueError("config root must be an object")
            config["duration_s"] = duration_s
            path.write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False), encoding="utf-8")
            return
        raise ValueError("config must be .json, .yaml, or .yml")

    def _sync_duration_input(self, snapshot: Snapshot) -> None:
        """同步 duration input 显示。注意：加载配置后以控制器快照为准。"""
        if snapshot.run_state == "UNLOADED":
            self.duration_input.setEnabled(False)
            return
        self.duration_input.setText(self._format_duration_text(snapshot.duration))
        self.duration_input.setEnabled(snapshot.run_state in {"READY", "PAUSED"})

    @staticmethod
    def _format_duration_text(duration_s: float) -> str:
        """格式化仿真时长文本。注意：整数秒不显示小数。"""
        if math.isfinite(duration_s) and duration_s.is_integer():
            return str(int(duration_s))
        return f"{duration_s:.3f}".rstrip("0").rstrip(".")

    def _set_theme(self, theme_key: str) -> None:
        """切换界面主题。注意：只改变显示，不改变仿真状态。"""
        self.theme_key = theme_key
        self.theme = THEMES[self.theme_key]
        self.light_theme_action.setChecked(theme_key == "light")
        self.dark_theme_action.setChecked(theme_key == "dark")
        self._apply_theme()
        theme_text = "浅色模式" if theme_key == "light" else "深色模式"
        self._log("UI", f"切换主题：{theme_text}")

    def _on_auto_center_changed(self) -> None:
        """处理 auto center changed 信号回调。注意：回调内避免耗时操作阻塞界面。"""
        # 同步开关状态到两个视图，并立即用当前快照触发一次居中重排。
        checked = self.auto_center.isChecked()
        snapshot = self.sim.snapshot()
        self.top_view.auto_center = checked
        self.side_view.auto_center = checked
        self.top_view.set_snapshot(snapshot)
        self.side_view.set_snapshot(snapshot)
        self._sync_side_view_controls()

    def _on_grid_changed(self) -> None:
        """处理 grid changed 信号回调。注意：回调内避免耗时操作阻塞界面。"""
        # 网格开关对俯视图与侧视图统一生效，并各自重绘。
        show_grid = self.grid_toggle.isChecked()
        self.top_view.show_grid = show_grid
        self.side_view.show_grid = show_grid
        self.top_view.viewport().update()
        self.side_view.update()

    def _disable_auto_center(self) -> None:
        """关闭自动居中选项。注意：用户手动平移或缩放后应避免自动抢回视图。"""
        # 取消勾选会再次触发 _on_auto_center_changed，从而同步关闭俯视图自动居中。
        if self.auto_center.isChecked():
            self.auto_center.setChecked(False)

    def _reset_view(self) -> None:
        """响应重置视图按钮。注意：同时重置俯视图和侧视图显示范围。"""
        # 俯视图重置会经信号链触发侧视图自适应，这里再补一次重绘保证及时刷新。
        self.top_view.reset_view()
        self.side_view.update()

    def _toggle_fullscreen(self) -> None:
        """切换仿真画布全屏状态。注意：需要保存并恢复原布局。"""
        # 以是否已存在全屏窗口为标志在进入/退出之间切换。
        if self._stage_fullscreen_dialog is not None:
            self._exit_stage_fullscreen()
        else:
            self._enter_stage_fullscreen()

    def _enter_stage_fullscreen(self) -> None:
        """进入 stage fullscreen 模式。注意：需要保存退出时恢复的界面状态。"""
        # 前置条件：stage/布局存在且当前未全屏。
        if self.stage is None or self.main_layout is None or self._stage_fullscreen_dialog is not None:
            return

        index = self.main_layout.indexOf(self.stage)
        if index < 0:
            return

        # 记录 stage 在主布局中的位置与拉伸系数，供退出时原样还原。
        self._stage_layout_index = index
        self._stage_layout_stretch = self.main_layout.stretch(index)
        self.main_layout.removeWidget(self.stage)

        # 用占位控件顶住原位，避免左右面板布局塌陷。
        self._stage_placeholder = QWidget()
        self.main_layout.insertWidget(self._stage_layout_index, self._stage_placeholder, self._stage_layout_stretch)

        # 把 stage 移入无边框全屏对话框（reparent 到对话框布局）。
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
        # 前置条件：处于全屏态。
        if self.stage is None or self.main_layout is None or self._stage_fullscreen_dialog is None:
            return

        # 先把 stage 从对话框取出，再销毁对话框。
        dialog = self._stage_fullscreen_dialog
        dialog.layout().removeWidget(self.stage)
        dialog.hide()
        dialog.deleteLater()
        self._stage_fullscreen_dialog = None

        # 移除并销毁占位控件。
        if self._stage_placeholder is not None:
            placeholder_index = self.main_layout.indexOf(self._stage_placeholder)
            if placeholder_index >= 0:
                self.main_layout.removeWidget(self._stage_placeholder)
            self._stage_placeholder.deleteLater()
            self._stage_placeholder = None

        # 把 stage 插回原位置（用 min 兜底防止索引越界）并还原拉伸系数。
        insert_index = min(self._stage_layout_index, self.main_layout.count())
        self.main_layout.insertWidget(insert_index, self.stage, self._stage_layout_stretch)
        self._set_fullscreen_button_state(False)
        self.stage.show()
        # reparent 后强制重绘两视图，避免残留旧画面。
        self.top_view.update()
        self.side_view.update()

    def _set_fullscreen_button_state(self, active: bool) -> None:
        """设置 fullscreen button state 状态。注意：保持控件状态和内部标志同步。"""
        if self.fullscreen_button is None:
            return
        # 全屏时显示“退出”图标/提示，否则显示“进入全屏”。
        self.fullscreen_button.setText("↙" if active else "⛶")
        self.fullscreen_button.setToolTip("退出全屏" if active else "全屏显示")
        self.fullscreen_button.setAccessibleName("退出全屏" if active else "全屏显示")

    def _open_offline_plot(self) -> None:
        """打开离线控制误差回放窗口。"""
        from src.ui.gui.offline_plot import OfflinePlotWindow
        if self._offline_plot is None:
            self._offline_plot = OfflinePlotWindow(self)
        self._offline_plot.show()
        self._offline_plot.raise_()

    def _open_data_analysis_window(self) -> None:
        """打开离线控制效果数据分析窗口。"""
        from src.ui.gui.data_analysis_window import DataAnalysisWindow
        # 新数据分析窗口独立持有，避免影响旧 OfflinePlotWindow 生命周期。
        if self._data_analysis_window is None:
            self._data_analysis_window = DataAnalysisWindow(self)
        self._data_analysis_window.show()
        self._data_analysis_window.raise_()

    def _open_live_monitor(self) -> None:
        """打开实时控制监控窗口。"""
        from src.ui.gui.live_monitor import LiveMonitorWindow
        if self._live_monitor is None:
            self._live_monitor = LiveMonitorWindow(self)
        if self.sim.snapshot().run_state != "UNLOADED":
            self._live_monitor.follow(self.sim.controller)
        self._live_monitor.show()
        self._live_monitor.raise_()

    def _log(self, source: str, message: str) -> None:
        """追加一条界面日志。注意：日志容量由日志面板负责裁剪。"""
        self.log_dialog.append(self.sim.time, source, message)

    def closeEvent(self, event) -> None:  # type: ignore[override]
        """处理窗口关闭事件。注意：关闭前需要释放控制器资源。"""
        # 关窗前停定时器并释放控制器资源，避免后台线程泄漏。
        self.timer.stop()
        if self._live_monitor is not None:
            self._live_monitor.close()
        if self._offline_plot is not None:
            self._offline_plot.close()
        if self._data_analysis_window is not None:
            self._data_analysis_window.close()
        self.sim.close()
        super().closeEvent(event)


def run_gui(argv: list[str] | None = None) -> int:
    """启动 PySide6 GUI。注意：一个进程只能持有一个 QApplication 主循环。"""

    # 创建 Qt 应用、最大化显示主窗口并进入事件循环；返回值作为进程退出码。
    app = QApplication(argv or [])
    window = MainWindow()
    window.showMaximized()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(run_gui())
