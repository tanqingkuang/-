"""编队仿真 PySide6 主窗口入口。注意：正式 GUI 入口仍在本模块。"""

from __future__ import annotations

import sys
from pathlib import Path


# 该模块既支持 `python -m src.ui.gui.main_window`，也支持 IDE 直接运行文件。
# 直接运行文件时，解释器默认只把 src/ui/gui 加入 sys.path。
# 下面的自修正只处理这种入口差异，不改变正式包结构。
def _ensure_project_root_on_path() -> None:
    """确保直接执行本文件时也能导入项目包。注意：包导入场景不修改 sys.path。"""

    if __package__:
        return
    project_root = Path(__file__).resolve().parents[3]
    if (project_root / "src").is_dir() and str(project_root) not in sys.path:
        sys.path.insert(0, str(project_root))


_ensure_project_root_on_path()

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication, QCheckBox, QFileDialog, QHBoxLayout, QMainWindow, QPushButton, QWidget

from src.algorithm.units.process.tra_plan.avoidance.planner import plan_avoidance_route
from src.ui.gui.avoidance_tools import (
    AvoidanceParams,
    AvoidanceWindow,
    ObstacleView,
    _inflated_polygon_vertices,
    parse_avoidance_config,
    parse_avoidance_params,
    preview_route_marker_points,
    route_inputs_to_config,
    route_to_polyline,
)
from src.ui.gui.dialogs import LogDialog, StageFullscreenDialog
from src.ui.gui.features.registry import build_gui_feature_registry
from src.ui.gui.main_window_actions import MainWindowActionMixin
from src.ui.gui.main_window_avoidance import MainWindowAvoidanceMixin
from src.ui.gui.main_window_layout import MainWindowLayoutMixin
from src.ui.gui.main_window_style import MainWindowStyleMixin
from src.ui.gui.side_view import SideView
from src.ui.gui.simulation_adapter import ControllerSimulationAdapter, MockSimulation
from src.ui.gui.theme_widgets import THEMES, SelectButton, Theme
from src.ui.gui.top_view import TopView
from src.ui.gui.view_models import (
    APP_CONFIG_FILE_NAME,
    APP_CONFIG_KEY_LAST_CONFIG,
    APP_CONFIG_SECTION,
    FIT_VIEWPORT_RATIO,
    GRID_MAX_SCREEN_SPACING,
    GRID_MIN_SCREEN_SPACING,
    TOP_VIEW_ORIGIN_MARGIN,
    TRAIL_SECONDS,
    VIEW_MAX_SCALE,
    VIEW_MIN_SCALE,
    WORLD_GRID_SPACING,
    WORLD_HEIGHT,
    WORLD_WIDTH,
    LinkState,
    NodeState,
    ReferenceRoute,
    Snapshot,
    TrailPoint,
    adaptive_world_grid_spacing,
    default_project_root,
    is_leader_node,
    leader_node_from,
    reference_route_points,
)
from src.data.geo import GeoOrigin
from src.algorithm.context.leaf_types import WayPointInputS


class MainWindow(
    MainWindowLayoutMixin,
    MainWindowAvoidanceMixin,
    MainWindowStyleMixin,
    MainWindowActionMixin,
    QMainWindow,
):
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
        # GUI 可选功能按运行档位选择具体实现，裁剪逻辑不散落在主窗口流程里。
        self.features = build_gui_feature_registry()
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
        self._top_view_geo_origin: GeoOrigin | None = None
        self._segment_lock_preferred = True
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



def run_gui(argv: list[str] | None = None) -> int:
    """启动 PySide6 GUI。注意：一个进程只能持有一个 QApplication 主循环。"""

    # 创建 Qt 应用、最大化显示主窗口并进入事件循环；返回值作为进程退出码。
    app = QApplication(argv or [])
    window = MainWindow()
    window.showMaximized()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(run_gui())
