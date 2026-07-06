"""MainWindow 布局构建方法。注意：只负责控件组装和表格配置。"""

from __future__ import annotations

from pathlib import Path

from PySide6.QtCore import QSignalBlocker, Qt
from PySide6.QtGui import QAction, QActionGroup
from PySide6.QtWidgets import (
    QAbstractItemView,
    QAbstractScrollArea,
    QAbstractSpinBox,
    QCheckBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QProgressBar,
    QPushButton,
    QHeaderView,
    QSizePolicy,
    QSlider,
    QSplitter,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from src.ui.gui.dialogs import StageFullscreenDialog
from src.ui.gui.side_view import SideView
from src.ui.gui.theme_widgets import THEMES, SelectButton
from src.ui.gui.top_view import TopView
from src.ui.gui.view_models import (
    PLAYBACK_RATE_SLIDER_MAX,
    PLAYBACK_RATE_SLIDER_MIN,
    playback_rate_to_slider_value,
    Snapshot,
)


class MainWindowLayoutMixin:
    """拆分主窗口布局构建逻辑。注意：由 MainWindow 继承使用。"""

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

        # 3D 态势放到顶层入口；窗口主体独立在 situation3d 包中，降低主界面冲突面。
        self.situation3d_action = QAction("3D态势(&3)", self)
        self.situation3d_action.triggered.connect(self._open_situation3d_window)
        self.menuBar().addAction(self.situation3d_action)

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
        # “场景”即编队队形选择：选项按配置的队形列表动态填充，运行时热切换。
        self.scenario_select = SelectButton(132, popup_side="right")
        self.scenario_select.currentIndexChanged.connect(self._on_scenario_selected)
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

        # “播放”分组：滑块按离散倍率档位跳转，避免 50x 上限压缩低倍率可调空间。
        playback_group = QGroupBox("播放")
        playback_layout = QVBoxLayout(playback_group)
        playback_layout.setContentsMargins(10, 18, 10, 10)
        status_row = QHBoxLayout()
        self.cpu_label = QLabel("CPU 0%")
        self.cpu_label.setToolTip("仿真线程忙碌时间 / 墙钟统计周期")
        self.speed_label = QLabel("1.0x")
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(PLAYBACK_RATE_SLIDER_MIN, PLAYBACK_RATE_SLIDER_MAX)
        self.speed_slider.setValue(playback_rate_to_slider_value(1.0))  # 默认 1.0x
        self.speed_slider.setToolTip("播放倍率：0.1-2 每 0.1，2-10 每 1，10-20 每 2，20-50 每 3")
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
        self.top_view_coordinate = QLineEdit()
        self.top_view_coordinate.setReadOnly(True)
        self.top_view_coordinate.setFixedWidth(238)
        self.top_view_coordinate.setPlaceholderText("单击画布显示经纬度")
        self.top_view_coordinate.setToolTip("单击二维实时显示区域后，这里显示 longitude_deg, latitude_deg，可直接复制。")
        self.top_view_coordinate_hint = QLabel("(lon, lat)")
        self.top_view_coordinate_hint.setObjectName("coordinateHint")
        # 全屏切换按钮（⛶），保存引用以便切换其图标/提示。
        fullscreen = QPushButton("⛶")
        fullscreen.setFixedSize(30, 30)
        fullscreen.clicked.connect(self._toggle_fullscreen)
        self.fullscreen_button = fullscreen
        toolbar.addWidget(title)
        toolbar.addWidget(fullscreen)
        toolbar.addWidget(self.top_view_coordinate)
        toolbar.addWidget(self.top_view_coordinate_hint)
        toolbar.addStretch(1)
        # 图例与链路显示开关（颜色由样式表按 objectName 着色）。
        self.legend_leader = QLabel("● 长机")
        self.legend_leader.setObjectName("legendLeader")
        self.legend_wingman = QLabel("● 僚机")
        self.legend_wingman.setObjectName("legendWingman")
        self.legend_link = QCheckBox("通信链路")
        self.legend_link.setObjectName("legendLink")
        self.legend_link.setChecked(True)
        self.legend_link.setToolTip("显示或隐藏俯视图中的通信链路线")
        self.legend_link.stateChanged.connect(self._on_link_visibility_changed)
        self.legend_warn = QLabel("● 异常状态")
        self.legend_warn.setObjectName("legendWarn")
        for widget in [self.legend_leader, self.legend_wingman, self.legend_link, self.legend_warn]:
            widget.setContentsMargins(0, 0, 2, 0)
            toolbar.addWidget(widget)
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
        self.top_view.pointClicked.connect(self._on_top_view_point_clicked)
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
