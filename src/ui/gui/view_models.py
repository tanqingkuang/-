"""GUI 显示数据模型与视图通用常量。注意：不依赖具体窗口控件。"""

from __future__ import annotations

import math
import sys
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path

from src.data.geo import GeoOrigin

# 世界坐标范围（米）：用于 mock 数据居中、待飞距/侧偏兜底估算等。
WORLD_WIDTH = 1600.0
WORLD_HEIGHT = 520.0
# 尾迹默认取飞行总时长的一半，超过该窗口的轨迹采样点会被丢弃。
TRAIL_DURATION_RATIO = 0.5
# 俯视图初始平移留白，避免场景紧贴左上角边缘。
TOP_VIEW_ORIGIN_MARGIN = 40.0
# 视图缩放上下限，防止缩到看不见或放大到失真；下限需覆盖 100km 级地图自适应。
VIEW_MIN_SCALE = 0.002
VIEW_MAX_SCALE = 3.5
# 自适应铺满时只占用视口 80%，四周留出可视边距。
FIT_VIEWPORT_RATIO = 0.80
# 网格基准间距（世界坐标）与其在屏幕上允许的疏密区间，配合自适应算法使用。
WORLD_GRID_SPACING = 48
GRID_MIN_SCREEN_SPACING = 36.0
GRID_MAX_SCREEN_SPACING = 96.0
# 每五条次网格强化一条主线，便于不增加刻度文字时快速估算距离。
GRID_MAJOR_INTERVAL = 5
# config.ini 中记忆“上次加载配置”用的小节名、键名与文件名。
APP_CONFIG_SECTION = "config"
APP_CONFIG_KEY_LAST_CONFIG = "last_config"
APP_CONFIG_FILE_NAME = "config.ini"
# GUI 播放倍率滑条采用离散档位，兼顾低倍率细调和高倍率快速跳转。
PLAYBACK_RATE_VALUES: tuple[float, ...] = (
    tuple(round(index / 10.0, 1) for index in range(1, 21))
    + tuple(float(index) for index in range(3, 11))
    + tuple(float(index) for index in range(12, 21, 2))
    + tuple(float(index) for index in range(23, 51, 3))
)
PLAYBACK_RATE_SLIDER_MIN = 1
PLAYBACK_RATE_SLIDER_MAX = len(PLAYBACK_RATE_VALUES)


def slider_value_to_playback_rate(value: int) -> float:
    """把倍率滑条位置转换成播放倍率。注意：输入越界时按最近档位夹紧。"""

    index = max(0, min(len(PLAYBACK_RATE_VALUES) - 1, int(value) - PLAYBACK_RATE_SLIDER_MIN))
    return PLAYBACK_RATE_VALUES[index]


def playback_rate_to_slider_value(rate: float) -> int:
    """把播放倍率转换成最近滑条位置。注意：配置倍率不必刚好落在 GUI 档位上。"""

    safe_rate = rate if math.isfinite(rate) else 1.0
    nearest_index = min(
        range(len(PLAYBACK_RATE_VALUES)),
        key=lambda index: abs(PLAYBACK_RATE_VALUES[index] - safe_rate),
    )
    return PLAYBACK_RATE_SLIDER_MIN + nearest_index


def trail_seconds_for_duration(duration_s: float) -> float:
    """按飞行总时长计算默认尾迹保留时长。注意：非法或负时长按 0 处理。"""

    if not math.isfinite(duration_s):
        return 0.0
    return max(0.0, duration_s * TRAIL_DURATION_RATIO)


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


def is_major_grid_line(coordinate: int, spacing: int) -> bool:
    """判断世界坐标是否落在主网格线上。注意：spacing 必须是正整数。"""

    return coordinate % (spacing * GRID_MAJOR_INTERVAL) == 0


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


@dataclass(frozen=True)
class TrailPoint:
    """仿真时间中的一个不可变轨迹采样点。注意：位置采用 ENU 三轴坐标。"""

    x: float  # 采样时刻的世界 east 坐标
    y: float  # 采样时刻的世界 north 坐标
    altitude: float  # 采样时刻高度
    time: float  # 采样仿真时刻，用于按当前尾迹窗口老化淡出
    path_distance: float = 0.0  # 从本轮尾迹起点累计的路程，裁剪后仍保留原基准
    point_id: int = -1  # 同一尾迹 generation 内严格递增的稳定逻辑序号；手工构造时允许 -1


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
    trail: Sequence[TrailPoint] = field(default_factory=list)  # 2D/3D 共用的稳定历史尾迹序列
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
class RallyGeometryView:
    """单个集结节点的本地圆、集结圆与关键点。注意：按配置预计算，运行期可刷新同一字段。"""

    node_id: str
    slot_x: float  # 松散目标点 M_i（盘旋圆上的切出点），east
    slot_y: float  # north
    center_x: float  # 盘旋圆圆心，east
    center_y: float  # north
    radius: float  # 盘旋圆半径
    entry_x: float  # 切入点 T，east
    entry_y: float  # north
    local_center_x: float = 0.0  # 本地待命盘旋圆圆心，east
    local_center_y: float = 0.0  # 本地待命盘旋圆圆心，north
    local_radius: float = 0.0  # 本地待命盘旋圆半径
    local_tangent_x: float = 0.0  # 本地圆切出点，east
    local_tangent_y: float = 0.0  # 本地圆切出点，north
    fallback_used: bool = False  # True 表示切线几何退化为直飞兜底


@dataclass
class ObstacleView:
    """俯视图显示用的二维障碍（无限高柱体）。注意：当前仅供 UI 显示与勾选，规划后端后续接入。"""

    obstacle_id: str  # 障碍唯一标识，列表显示/勾选用
    kind: str  # "circle" | "rect" | "polygon"
    enabled: bool = True  # 是否启用（参与避障）
    center_x: float = 0.0  # 圆心 east（kind=circle）
    center_y: float = 0.0  # 圆心 north
    radius: float = 0.0  # 半径，米（kind=circle）
    min_x: float = 0.0  # 矩形 east 下界（kind=rect）
    min_y: float = 0.0  # 矩形 north 下界
    max_x: float = 0.0  # 矩形 east 上界
    max_y: float = 0.0  # 矩形 north 上界
    vertices: list[tuple[float, float]] = field(default_factory=list)  # 旋转矩形/多边形顶点

    def label(self) -> str:
        """生成左面板勾选列表的显示文本。注意：仅用于界面展示。"""
        if self.kind == "polygon":
            return f"{self.obstacle_id}  矩形 {len(self.vertices)}点"
        if self.kind == "rect":
            return f"{self.obstacle_id}  矩形 ({self.min_x:.0f},{self.min_y:.0f})-({self.max_x:.0f},{self.max_y:.0f})"
        return f"{self.obstacle_id}  圆 ({self.center_x:.0f},{self.center_y:.0f}) r{self.radius:.0f}"


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
    rally_geometry: list[RallyGeometryView] = field(default_factory=list)
    terrain_display_file: str | None = None  # 3D 态势显示用地形布局文件；不参与仿真语义
    blocked_route_segments: list[ReferenceRoute] = field(default_factory=list)  # 被封锁的原始参考航线；仅避障覆盖生效时非空


def is_leader_node(node: NodeState) -> bool:
    """判断节点是否为长机。注意：GUI 显示必须遵循控制器 role，而不是节点顺序。"""

    return node.role.strip().lower() in {"leader", "rally_leader"}


def leader_node_from(nodes: list[NodeState]) -> NodeState | None:
    """从节点列表中取长机。注意：缺少显式长机时回退首节点，保持旧配置可显示。"""

    return next((node for node in nodes if is_leader_node(node)), nodes[0] if nodes else None)


def link_direction_label(direction: str) -> str:
    """生成通信链路方向显示文本。注意：只负责界面文案，不改变链路状态。"""

    return {"duplex": "双向", "simplex": "单向"}.get(direction, direction)
