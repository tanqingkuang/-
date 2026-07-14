"""MainWindow 避障规划面板与航线采用逻辑。注意：不处理主播放状态机。"""

from __future__ import annotations

from pathlib import Path
from typing import Callable

from PySide6.QtCore import QSignalBlocker, Qt
from PySide6.QtWidgets import (
    QAbstractSpinBox,
    QCheckBox,
    QDoubleSpinBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from src.ui.gui.avoidance_panel_view_model import (
    adopt_enabled,
    avoidance_status_text,
    export_enabled,
    param_widgets_enabled,
    simplify_should_follow,
)
from src.ui.gui.avoidance_tools import (
    AVOIDANCE_PARAM_SPECS,
    AvoidanceParams,
    AvoidanceWindow,
    obstacle_spec_to_view,
    obstacle_view_to_spec,
)

_JSON_ROUTE_FILTER = "JSON 文件 (*.json)"
_DIAMOND_XML_ROUTE_FILTER = "钻石 XML (*.XML *.xml)"
_ROUTE_EXPORT_FILTERS = f"{_JSON_ROUTE_FILTER};;{_DIAMOND_XML_ROUTE_FILTER}"


class MainWindowAvoidanceMixin:
    """拆分主窗口避障规划交互。注意：由 MainWindow 继承使用。"""

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
        # 参数表格采用两列：左侧固定标签、右侧输入框吃掉剩余宽度。
        param_grid = QGridLayout()
        param_grid.setContentsMargins(0, 0, 0, 0)
        param_grid.setHorizontalSpacing(10)
        param_grid.setVerticalSpacing(8)
        param_grid.setColumnStretch(1, 1)
        # 单一规格同时创建控件、排列顺序与 tooltip，避免平行列表漂移。
        for row, spec in enumerate(AVOIDANCE_PARAM_SPECS):
            on_change = getattr(self, spec.on_change_method) if spec.on_change_method else None
            spin = self._make_param_spin(
                maximum=spec.maximum,
                step=spec.step,
                tooltip=spec.tooltip,
                suffix=spec.suffix,
                on_change=on_change,
            )
            setattr(self, spec.widget_attr, spin)
            label = QLabel(spec.caption)
            label.setObjectName("paramLabel")
            # 标签和输入框都挂 tooltip，鼠标停在任一处都能看到解释。
            label.setMinimumWidth(104)
            label.setToolTip(spec.tooltip)
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
        # 第二行放重置和航线输出，按钮宽度与上排保持一致。
        secondary_row = QHBoxLayout()
        secondary_row.setSpacing(8)
        # 重置的语义是清除覆盖航线，而不是把参数恢复成配置值。
        self.reset_route_button = QPushButton("重置")
        self.reset_route_button.setToolTip("清除已采用的避障航线，恢复配置中的默认长机航线。")
        self.reset_route_button.clicked.connect(self._reset_avoidance_route)
        self.export_route_button = QPushButton("航线输出")
        self.export_route_button.setToolTip("把当前预览避障航线输出为 route_file 可读取的航线文件。")
        self.export_route_button.clicked.connect(self._export_route)
        self.export_route_button.setEnabled(False)
        secondary_row.addWidget(self.reset_route_button, 1)
        secondary_row.addWidget(self.export_route_button, 1)
        layout.addLayout(secondary_row)
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
        params = self._avoidance_params
        if simplify_should_follow(
            self.sender() is self.clearance_spin,
            params is not None,
            params.simplify_clearance_explicit if params is not None else False,
        ):
            # 旧配置未显式给 simplify_clearance_m 时，它继续跟随安全间距。
            self.simplify_clearance_spin.blockSignals(True)
            self.simplify_clearance_spin.setValue(self.clearance_spin.value())
            self.simplify_clearance_spin.blockSignals(False)
        self._invalidate_preview()
        if self.obstacles:
            self.top_view.set_obstacles(self.obstacles, self.clearance_spin.value())
        self._update_situation3d_snapshot(self.sim.snapshot())

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
        has_preview = self._preview_route is not None
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
            widget.setEnabled(param_widgets_enabled(has_params))
        self.adopt_route_button.setEnabled(adopt_enabled(has_preview))
        self.export_route_button.setEnabled(export_enabled(has_params, has_preview))
        if not has_params:
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
        self._update_situation3d_snapshot(self.sim.snapshot())

    def _invalidate_preview(self) -> None:
        """清除当前预览航线并禁用“采用”。注意：障碍勾选/配置变化后调用。"""
        self._preview_route = None
        self.top_view.set_preview_route(None)
        if hasattr(self, "adopt_route_button"):
            self.adopt_route_button.setEnabled(adopt_enabled(False))
        if hasattr(self, "export_route_button"):
            self.export_route_button.setEnabled(export_enabled(self._avoidance_params is not None, False))

    def _update_avoidance_status(self) -> None:
        """空闲时在反馈区显示操作提示（生成成功/失败时由 _generate_route 覆盖）。"""
        enabled = sum(1 for obstacle in self.obstacles if obstacle.enabled)
        self.avoidance_status.setText(avoidance_status_text(enabled, len(self.obstacles)))

    def _set_obstacles_from_config(self, path: str) -> None:
        """应用 runner 已解析的障碍与规划参数。注意：本方法不直接读配置文件。"""
        data = self.sim.gui_config
        obstacles = [obstacle_spec_to_view(obstacle) for obstacle in data.obstacles]
        clearance = data.obstacle_clearance_m
        self.obstacles = obstacles
        self._avoidance_params = data.avoidance_params
        if hasattr(self, "avoidance_config_label"):
            self.avoidance_config_label.setText(Path(path).name)
        self.top_view.set_obstacles(obstacles, clearance)
        self._invalidate_preview()
        self._rebuild_obstacle_list()
        self._sync_avoidance_param_widgets()
        self._update_avoidance_status()
        self._update_situation3d_snapshot(self.sim.snapshot())

    def _generate_route(self) -> None:
        """响应“生成航线”：跑 plan_avoidance_route，成功则预览，失败则显示 ERR_AVOID_* 原因。"""
        if self._avoidance_params is None or len(self._avoidance_params.waypoints) < 2:
            self._invalidate_preview()
            self.avoidance_status.setText("当前配置无可规划航线（缺 route.waypoints 或 avoidance）")
            return
        params = self._avoidance_params
        # 只把当前勾选项交给后端；未勾选障碍仍留在库里但不参与规划。
        enabled = [obstacle_view_to_spec(obstacle) for obstacle in self.obstacles if obstacle.enabled]
        if not enabled:
            # 未选择任何障碍：等价于维持原航线，不生成 R 圆弧航线，也不允许采用。
            self._invalidate_preview()
            self._report_avoidance_result("未选择障碍 · 维持原航线", "未选择障碍，跳过生成（维持原航线）")
            return
        # 规划参数以界面控件为准（用户可现场调），覆盖配置解析值。
        # 旧配置未显式配置 simplify_clearance_m 时，让去冗余安全距跟随当前安全间距控件，保持旧行为。
        clearance_m = self.clearance_spin.value()
        simplify_clearance_m = self.simplify_clearance_spin.value()
        try:
            # 所有可调参数均以子窗口当前值为准，覆盖加载时的配置快照。
            result = self.sim.plan_avoidance_route(
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
            self._report_avoidance_result(f"参数错误：{exc}", f"生成航线参数错误：{exc}", level="WARN")
            return
        if result.ok and result.route is not None:
            self._preview_route = result.route
            # 预览线只进画布，不进入控制器，直到用户点击“采用航线”。
            self.top_view.set_preview_route(list(result.route.polyline), list(result.route.markers))
            segment_count = result.route.segment_count
            arcs = result.route.arc_count
            self.adopt_route_button.setEnabled(True)
            self.export_route_button.setEnabled(True)
            self._report_avoidance_result(
                f"预览就绪：{segment_count} 段（{arcs} 圆弧）· 可采用",
                f"生成航线成功：{segment_count} 段，{arcs} 圆弧",
            )
        else:
            self._invalidate_preview()
            self._report_avoidance_result(
                f"{result.code}：{result.detail}",
                f"生成航线失败 {result.code}: {result.detail}",
            )

    def _report_avoidance_result(self, status: str, log_message: str, *, level: str = "Avoid") -> None:
        """同步避障状态文本与对应日志。"""

        # 状态先落到当前窗口，再把同一结果写入可追溯日志。
        self.avoidance_status.setText(status)
        self._log(level, log_message)

    def _export_route(self) -> None:
        """响应“航线输出”：把当前预览航线写成 route_file 文件。注意：只输出已生成但未失效的预览。"""
        if self._preview_route is None:
            self.avoidance_status.setText("请先生成航线，再输出。")
            return
        config_path = self.current_config_path or (self.project_root / "configs" / "base.json")
        default_path, default_filter = self.sim.route_export_defaults(config_path)
        selected, selected_filter = QFileDialog.getSaveFileName(
            self.avoidance_window or self,
            "输出避障航线",
            str(default_path),
            _ROUTE_EXPORT_FILTERS,
            default_filter,
        )
        if not selected:
            return
        route_path = Path(selected)
        if not route_path.suffix:
            # QFileDialog 在部分平台不会自动追加过滤器后缀，这里按用户选中的格式补齐。
            suffix = ".XML" if "XML" in selected_filter or "xml" in selected_filter else ".json"
            route_path = route_path.with_suffix(suffix)
        speed_mps = self._avoidance_params.speed_mps if self._avoidance_params is not None else 0.0
        geo_reference = self._avoidance_params.geo_reference if self._avoidance_params is not None else None
        try:
            # 通过 runner 应用层保存，确保格式策略与控制器加载链路一致。
            written = self.sim.export_route(config_path, route_path, self._preview_route, speed_mps, geo_reference)
        except (OSError, ValueError) as exc:
            self._report_avoidance_result(f"航线输出失败：{exc}", f"航线输出失败：{exc}", level="WARN")
            return
        self._report_avoidance_result(f"已输出航线：{written}", f"已输出避障航线：{written}")

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
            self._report_avoidance_result("已采用避障航线 · 点播放仿真", "已采用避障航线，长机航线已替换")
        else:
            self._report_avoidance_result(
                f"采用失败 {self.sim.last_result_code}",
                f"采用航线失败 {self.sim.last_result_code}: {self.sim.last_result_message}",
                level="WARN",
            )

    def _reset_avoidance_route(self) -> None:
        """响应“重置”：清除已采用避障航线，恢复配置默认长机航线。"""
        self._invalidate_preview()
        snapshot = self.sim.clear_avoidance_route()
        self._update_snapshot(snapshot, fit_top_view=False)
        if self.sim.last_result_code == "OK":
            # 控制器已回到配置航线，画布快照也同步刷新。
            self._report_avoidance_result("已恢复默认航线", "已清除避障航线，恢复默认航线")
        else:
            self._report_avoidance_result(
                f"重置失败 {self.sim.last_result_code}",
                f"重置航线失败 {self.sim.last_result_code}: {self.sim.last_result_message}",
                level="WARN",
            )

    def _open_avoidance_window(self) -> None:
        """打开避障规划子窗口。注意：重复触发只激活已有窗口。"""
        if self.avoidance_window is None:
            return
        self.avoidance_window.show()
        self.avoidance_window.raise_()
        self.avoidance_window.activateWindow()
