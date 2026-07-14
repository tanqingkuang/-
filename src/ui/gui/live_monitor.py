"""实时控制数据监控窗口。"""

from __future__ import annotations

import logging
from collections import deque

from PySide6.QtCharts import QChart, QChartView, QLineSeries, QValueAxis
from PySide6.QtCore import QMargins, QPointF, Qt, QTimer
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDialog,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from src.runner.sim_control import NodeState, SimulationController, SimulationSnapshot
from src.ui.gui.chart_common import (
    CHART_PALETTE,
    CONTROL_ERROR_CHANNELS,
    ChannelSpec,
    apply_y_range,
    build_chart_sidebar,
    refresh_chart_node_panel,
)

# ── 常量 ─────────────────────────────────────────────────────────────────────

_POLL_MS = 100
_WIN_OPTIONS_S = [30.0, 60.0, 120.0]
_WIN_DEFAULT = 60.0
_MAX_PTS = int(120 * 10 * 1.2)
_DEFAULT_CTRL_COLOR = "#888888"

_CTRL_COLORS = {
    "待命": _DEFAULT_CTRL_COLOR,
    "集结": "#2ecc71",
    "保持": "#4c8ef5",
    "重构": "#c0392b",
}
# 控制器可以扩展回报文本，因此映射之外必须保留可诊断的默认颜色路径。
LOGGER = logging.getLogger(__name__)

# ── 主窗口 ────────────────────────────────────────────────────────────────────


class LiveMonitorWindow(QDialog):
    """实时控制误差监控窗口。以 100 ms 为周期轮询快照，滚动显示三轴位置/速度误差曲线。"""

    def __init__(self, parent: QWidget | None = None) -> None:
        """初始化窗口控件与内部状态，构建侧边栏和空图表区。"""
        super().__init__(parent)
        self.setWindowTitle("控制监控")
        self.resize(1200, 760)
        self.setWindowFlags(
            self.windowFlags()
            | Qt.WindowType.WindowMinimizeButtonHint
            | Qt.WindowType.WindowMaximizeButtonHint
        )

        self._ctrl: SimulationController | None = None
        self._win_s = _WIN_DEFAULT
        self._last_t = -1.0

        # 节点表保存颜色和显隐状态，图表重建时不丢失用户勾选。
        # node_id -> {color, visible, cb}
        self._nodes: dict[str, dict] = {}
        # 每条曲线独立缓存点列，某个通道缺点不会影响其他通道时间轴。
        # ch.key -> node_id -> deque[(t, v)]
        self._bufs: dict[str, dict[str, deque[tuple[float, float]]]] = {}
        # series 是当前图表控件的临时引用，重建图表时会整体刷新。
        # (node_id, ch.key) -> QLineSeries
        self._series: dict[tuple[str, str], QLineSeries] = {}
        # 行缓存集中保存坐标轴和当前值标签，便于轮询时批量更新。
        # ch.key -> (QChart, x_ax, y_ax, val_labels, zero_series)
        self._rows: dict[str, tuple] = {}

        self._strategy_strip: QLabel | None = None
        self._last_report: str = "待命"
        # 未知回报按值去重，避免 10Hz 轮询对同一扩展状态反复告警。
        self._unknown_control_reports: set[str] = set()
        self._rebuild_needed = False
        self._ch_cbs: dict[str, QCheckBox] = {}
        self._node_lay: QVBoxLayout
        self._right_lay: QVBoxLayout

        self._timer = QTimer(self)
        self._timer.setInterval(_POLL_MS)
        self._timer.timeout.connect(self._poll)

        self._build_ui()

    # ── Public API ────────────────────────────────────────────────────────────

    def follow(self, ctrl: SimulationController) -> None:
        """绑定控制器并启动轮询。若 ctrl 与当前相同则只确保定时器运行，否则清空旧数据重新绑定。"""
        # 同一控制器重复打开窗口时保留历史曲线，只恢复轮询定时器。
        if self._ctrl is ctrl:
            self._timer.start()
            return
        # 新控制器视为新数据源，必须清空旧节点、旧曲线和时间游标。
        self._timer.stop()
        self._ctrl = ctrl
        self._last_t = -1.0
        self._nodes.clear()
        self._bufs.clear()
        self._refresh_node_panel()
        self._rebuild_charts()
        self._timer.start()

    def unfollow(self) -> None:
        """停止轮询并清空所有缓冲区和节点列表。注意：用于切换或关闭数据源。"""
        self._timer.stop()
        self._ctrl = None
        self._last_t = -1.0
        self._last_report = "待命"
        self._nodes.clear()
        self._bufs.clear()
        self._refresh_node_panel()
        self._rebuild_charts()

    def _control_report_color(self, report: str) -> str:
        """返回控制回报颜色，未知值只记录一次告警。"""

        color = _CTRL_COLORS.get(report)
        if color is not None:
            return color
        if report not in self._unknown_control_reports:
            self._unknown_control_reports.add(report)
            LOGGER.warning("未知控制回报，使用默认颜色：%s", report)
        return _DEFAULT_CTRL_COLOR

    def reset_stream(self, ctrl: SimulationController) -> None:
        """重置监控曲线并保持控制器绑定。注意：仿真 reset 后节点面板仍应显示当前配置节点。"""
        self._timer.stop()
        self._ctrl = ctrl
        self._last_t = -1.0
        self._bufs.clear()

        snap = ctrl.get_snapshot()
        self._last_report = snap.control_report
        old_nodes = self._nodes
        self._nodes = {}
        for node in snap.nodes:
            old = old_nodes.get(node.node_id, {})
            color = old.get("color", CHART_PALETTE[len(self._nodes) % len(CHART_PALETTE)])
            visible = old.get("visible", True)
            self._nodes[node.node_id] = {"color": color, "visible": visible, "cb": None}
        self._refresh_node_panel()
        self._rebuild_charts()
        self._poll()
        self._timer.start()

    # ── UI 构建 ───────────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        """组装顶层布局：左侧边栏 + 右侧图表区。"""
        # 外层布局只创建一次；右侧图表内容随通道和节点选择动态重建。
        root_lay = QVBoxLayout(self)
        root_lay.setContentsMargins(0, 0, 0, 0)
        body = QWidget()
        body_lay = QHBoxLayout(body)
        body_lay.setContentsMargins(6, 6, 6, 6)
        body_lay.setSpacing(6)
        body_lay.addWidget(self._build_sidebar())
        right = QWidget()
        self._right_lay = QVBoxLayout(right)
        self._right_lay.setContentsMargins(0, 0, 0, 0)
        self._right_lay.setSpacing(2)
        body_lay.addWidget(right, stretch=1)
        root_lay.addWidget(body)
        self._rebuild_charts()

    def _build_sidebar(self) -> QWidget:
        """构建左侧边栏：节点列表（动态）、通道 checkbox、时间窗口选择。"""
        win_box = QGroupBox("时间窗口")
        win_lay = QHBoxLayout(win_box)
        combo = QComboBox()
        for s in _WIN_OPTIONS_S:
            combo.addItem(f"{int(s)}s", s)
        combo.setCurrentIndex(_WIN_OPTIONS_S.index(_WIN_DEFAULT))
        combo.currentIndexChanged.connect(
            lambda i, c=combo: self._set_win(c.itemData(i))
        )
        win_lay.addWidget(combo)
        sidebar = build_chart_sidebar(
            empty_text="（等待数据）",
            rebuild_charts=self._rebuild_charts,
            extra_widgets=(win_box,),
        )
        self._node_lay = sidebar.node_layout
        self._ch_cbs = sidebar.channel_checkboxes
        return sidebar.widget

    def _refresh_node_panel(self) -> None:
        """清空节点面板并按当前 _nodes 重新填充 checkbox。"""
        # 节点列表整体重建，避免旧节点控件在配置或数据源变化后残留。
        refresh_chart_node_panel(
            self._node_lay,
            self._nodes,
            empty_text="（等待数据）",
            rebuild_charts=self._rebuild_charts,
        )

    # ── 图表 ──────────────────────────────────────────────────────────────────

    def _rebuild_charts(self) -> None:
        """按当前勾选的通道和节点重建所有子图，完成后将缓冲区历史数据填入。"""
        # QChartView 与 series 依赖当前通道/节点集合，重建前先移除旧控件。
        while self._right_lay.count():
            item = self._right_lay.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        self._series.clear()
        self._rows.clear()

        self._strategy_strip = QLabel(self._last_report)
        self._strategy_strip.setFixedHeight(26)
        self._strategy_strip.setAlignment(Qt.AlignmentFlag.AlignCenter)
        clr = self._control_report_color(self._last_report)
        self._strategy_strip.setStyleSheet(
            f"background:{clr}; color:white; font-weight:bold;"
        )
        self._right_lay.addWidget(self._strategy_strip)

        active = [ch for ch in CONTROL_ERROR_CHANNELS if self._ch_cbs[ch.key].isChecked()]
        # 隐藏节点的数据仍保留在缓冲区，只是不为它创建可见曲线。
        visible_nids = [nid for nid, nd in self._nodes.items() if nd["visible"]]

        if not active:
            # 所有通道关闭时显示提示，避免右侧空白区域误判为程序卡住。
            lbl = QLabel("请勾选至少一个通道")
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self._right_lay.addWidget(lbl, stretch=1)
            return

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        inner = QWidget()
        inner_lay = QVBoxLayout(inner)
        inner_lay.setContentsMargins(0, 0, 0, 0)
        inner_lay.setSpacing(2)
        scroll.setWidget(inner)
        self._right_lay.addWidget(scroll, stretch=1)

        for i, ch in enumerate(active):
            # 只有最后一个可见子图显示 X 轴刻度，减少重复标签占用高度。
            show_x = i == len(active) - 1
            chart, x_ax, y_ax, val_labels, zero_s, val_panel = self._make_chart(
                ch, visible_nids, show_x
            )
            self._rows[ch.key] = (chart, x_ax, y_ax, val_labels, zero_s)

            row = QWidget()
            row_lay = QHBoxLayout(row)
            row_lay.setContentsMargins(0, 0, 0, 0)
            row_lay.setSpacing(4)
            view = QChartView(chart)
            view.setRenderHint(QPainter.RenderHint.Antialiasing)
            if len(active) <= 4:
                # 通道较少时均分高度，方便同时观察多轴误差。
                view.setMinimumHeight(100)
                row_lay.addWidget(view, stretch=1)
                row_lay.addWidget(val_panel)
                inner_lay.addWidget(row, stretch=1)
            else:
                # 通道较多时使用固定行高，由滚动区承载溢出内容。
                view.setFixedHeight(155)
                row_lay.addWidget(view, stretch=1)
                row_lay.addWidget(val_panel)
                inner_lay.addWidget(row)

        self._repopulate()

    def _make_chart(
        self, ch: ChannelSpec, node_ids: list[str], show_x: bool
    ) -> tuple[QChart, QValueAxis, QValueAxis, dict[str, QLabel], QLineSeries, QWidget]:
        """创建单个通道的 QChart，包含 y=0 灰色虚线基准、各节点误差曲线和当前值面板。"""
        # 每个通道独立坐标轴，避免不同单位的误差互相压缩显示范围。
        chart = QChart()
        chart.setMargins(QMargins(2, 2, 6, 2))
        chart.legend().setVisible(False)
        chart.setBackgroundBrush(QColor("#fafafa"))

        hdr_font = QFont()
        hdr_font.setPointSize(9)
        chart.setTitleFont(hdr_font)
        title_str = f"{ch.label}  ({ch.unit})" if ch.unit else ch.label
        chart.setTitle(title_str)

        x_ax = QValueAxis()
        x_ax.setRange(0.0, self._win_s)
        x_ax.setLabelsVisible(show_x)
        if show_x:
            x_ax.setTitleText("t (s)")
        x_ax.setGridLineColor(QColor("#e0e0e0"))
        chart.addAxis(x_ax, Qt.AlignmentFlag.AlignBottom)

        y_ax = QValueAxis()
        small = QFont()
        small.setPointSize(8)
        y_ax.setLabelsFont(small)
        y_ax.setRange(-1.0, 1.0)
        y_ax.setTickCount(3)
        y_ax.setGridLineColor(QColor("#e0e0e0"))
        chart.addAxis(y_ax, Qt.AlignmentFlag.AlignLeft)

        # y=0 基准线（灰色虚线，端点随 X 轴实时更新，保证始终在可见范围内）
        zero = QLineSeries()
        zero.setPen(QPen(QColor("#999999"), 1.2, Qt.PenStyle.DashLine))
        zero.append(0.0, 0.0)
        zero.append(1.0, 0.0)
        chart.addSeries(zero)
        zero.attachAxis(x_ax)
        zero.attachAxis(y_ax)

        for nid in node_ids:
            color = QColor(self._nodes[nid]["color"])
            s = QLineSeries()
            s.setPen(QPen(color, 2.0))
            # 每架机在当前通道上只画一条误差曲线。
            chart.addSeries(s)
            s.attachAxis(x_ax)
            s.attachAxis(y_ax)
            self._series[(nid, ch.key)] = s

        # 当前值面板
        panel = QWidget()
        panel.setFixedWidth(68)
        p_lay = QVBoxLayout(panel)
        p_lay.setContentsMargins(2, 4, 2, 4)
        p_lay.setSpacing(2)
        val_labels: dict[str, QLabel] = {}
        for nid in node_ids:
            lbl = QLabel(nid)
            # 当前值标签与节点曲线同色，减少额外图例控件。
            lbl.setStyleSheet(
                f"color:{self._nodes[nid]['color']}; font-weight:bold; font-size:11px;"
            )
            lbl.setAlignment(Qt.AlignmentFlag.AlignRight)
            p_lay.addWidget(lbl)
            val_labels[nid] = lbl
        p_lay.addStretch()

        return chart, x_ax, y_ax, val_labels, zero, panel

    def _repopulate(self) -> None:
        """图表重建后将缓冲区历史数据填入新 series，并立即刷新坐标轴范围。"""
        # 重建只销毁 Qt 图表对象，历史点仍由 _bufs 保存。
        # 只填入当前时间窗口内的点，与 _poll() 行为保持一致。
        t_min = self._last_t - self._win_s
        for (nid, ch_key), series in self._series.items():
            pts = self._bufs.get(ch_key, {}).get(nid)
            if pts:
                series.replace([QPointF(t, v) for t, v in pts if t >= t_min])
        self._refresh_axes()

    # ── 轮询 ──────────────────────────────────────────────────────────────────

    def _poll(self) -> None:
        """100 ms 定时器回调：读取快照，追加缓冲区，刷新 series 和坐标轴。"""
        if self._ctrl is None:
            return
        snap = self._ctrl.get_snapshot()
        t = snap.time_s
        # 一、策略条不依赖时间推进，暂停态也要先更新。
        self._update_strategy_strip(snap.control_report)
        # 二、新节点必须先注册，采集时对应曲线和标签才已存在。
        self._maybe_rebuild_for_new_nodes(snap.nodes)
        # 三、注册完成后再按仿真时间去重，暂停期间仍能看到新节点。
        if t <= self._last_t:
            # READY/PAUSED/FINISHED 状态时间可能不变，避免重复追加同一时刻点。
            return
        self._last_t = t
        # 四、只对新时刻采集一次权威快照数据。
        self._ingest_snapshot(snap)
        # 五、最后把滚动窗口数据批量下发到 QtCharts。
        self._refresh_series_and_axes(t)

    def _update_strategy_strip(self, report: str) -> None:
        """刷新策略色条。注意：暂停时仍需反映最新控制回报。"""

        if self._strategy_strip is None:
            return
        self._last_report = report
        color = self._control_report_color(report)
        self._strategy_strip.setText(report)
        self._strategy_strip.setStyleSheet(
            f"background:{color}; color:white; font-weight:bold;"
        )

    def _maybe_rebuild_for_new_nodes(self, nodes: list[NodeState]) -> None:
        """登记新节点，并在需要时重建节点面板与曲线对象。"""

        for node in nodes:
            if node.node_id in self._nodes:
                continue
            # 新节点按发现顺序取色，保证窗口生命周期内颜色稳定。
            color = CHART_PALETTE[len(self._nodes) % len(CHART_PALETTE)]
            self._nodes[node.node_id] = {
                "color": color, "visible": True, "cb": None,
            }
            self._rebuild_needed = True
        if not self._rebuild_needed:
            return
        self._rebuild_needed = False
        self._refresh_node_panel()
        self._rebuild_charts()

    def _ingest_snapshot(self, snapshot: SimulationSnapshot) -> None:
        """把快照有效通道值追加到所有节点的有界缓冲区。"""

        t = snapshot.time_s
        for node in snapshot.nodes:
            for channel in CONTROL_ERROR_CHANNELS:
                value = channel.act(node)
                if value is None:
                    continue
                # 不论 visible，所有节点均持续采集，隐藏期间不会丢失数据。
                (self._bufs
                 .setdefault(channel.key, {})
                 .setdefault(node.node_id, deque(maxlen=_MAX_PTS))
                 .append((t, value)))

    def _refresh_series_and_axes(self, t: float) -> None:
        """把当前滚动窗口数据下发到曲线、数值标签和坐标轴。"""

        t_min = t - self._win_s
        for ch_key, (_, x_ax, y_ax, val_labels, zero_s) in self._rows.items():
            all_y: list[float] = []
            for nid, node_view in self._nodes.items():
                if not node_view["visible"]:
                    continue
                series = self._series.get((nid, ch_key))
                if series is None:
                    continue
                buf = self._bufs.get(ch_key, {}).get(nid, deque())
                # 仅将当前滚动窗口内的数据送入 QtCharts，控制刷新成本。
                points = [QPointF(tt, value) for tt, value in buf if tt >= t_min]
                series.replace(points)
                all_y.extend(point.y() for point in points)
                label = val_labels.get(nid)
                if label and buf:
                    # 当前值显示该节点最近一个有效点，便于快速读数。
                    label.setText(f"{nid}\n{buf[-1][1]:.2f}")
            x_ax.setRange(t_min, t + 2.0)
            # 0 基准线随 X 轴窗口移动，始终覆盖整个可见区间。
            zero_s.replace([QPointF(t_min, 0.0), QPointF(t + 2.0, 0.0)])
            apply_y_range(y_ax, all_y)


    # ── 工具 ──────────────────────────────────────────────────────────────────

    def _refresh_axes(self) -> None:
        """根据缓冲区当前数据刷新所有子图的坐标轴范围，用于暂停后切换节点/通道。"""
        t_min = self._last_t - self._win_s
        t_max = self._last_t + 2.0
        for ch_key, (_, x_ax, y_ax, _lbl, zero_s) in self._rows.items():
            all_y: list[float] = []
            for nid, nd in self._nodes.items():
                if not nd["visible"]:
                    continue
                buf = self._bufs.get(ch_key, {}).get(nid, deque())
                # 重新定轴时只考虑当前可见时间窗口的数据。
                all_y.extend(v for tt, v in buf if tt >= t_min)
            if self._last_t >= 0:
                x_ax.setRange(t_min, t_max)
                zero_s.replace([QPointF(t_min, 0.0), QPointF(t_max, 0.0)])
            apply_y_range(y_ax, all_y)

    def _set_win(self, s: float) -> None:
        """更新滚动时间窗口宽度（秒）。"""
        # 切换窗口宽度只改变显示范围，不清理历史缓存。
        # 调 _repopulate 而非 _refresh_axes，确保新窗口内的历史点也填入 series。
        self._win_s = s
        self._repopulate()

    def closeEvent(self, event) -> None:  # type: ignore[override]
        """关闭前停止轮询定时器。"""
        self._timer.stop()
        super().closeEvent(event)
