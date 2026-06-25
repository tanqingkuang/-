"""离线控制误差回放绘图窗口。"""

from __future__ import annotations

import json
from dataclasses import fields as dc_fields
from pathlib import Path

from PySide6.QtCharts import QChart, QChartView, QLineSeries, QValueAxis
from PySide6.QtCore import QMargins, QPointF, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import (
    QCheckBox,
    QDialog,
    QFileDialog,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from src.runner.sim_control import NodeState
from src.ui.gui.live_monitor import Ch, _hdg_dev, _PALETTE, _apply_y_range

_NODE_STATE_FIELDS = {f.name for f in dc_fields(NodeState)}
_X_MARGIN_S = 0.5  # X 轴右侧留白，避免末尾点被裁剪

# 离线分析通道定义：与实时监控独立维护，后续可按需扩展
OFFLINE_CHANNELS: list[Ch] = [
    # 前向轴 x
    Ch("perr_x", "前向位置误差", "m",   "前向轴 x", True,
       lambda n: n.track_pos_err_x_m),
    Ch("verr_x", "前向速度误差", "m/s", "前向轴 x", False,
       lambda n: n.track_vel_err_x_mps),
    # 垂向轴 y
    Ch("perr_y", "垂向位置误差", "m",   "垂向轴 y", True,
       lambda n: n.track_pos_err_y_m),
    Ch("verr_y", "垂向速度误差", "m/s", "垂向轴 y", False,
       lambda n: n.track_vel_err_y_mps),
    # 侧向轴 z
    Ch("perr_z", "侧向位置误差", "m",   "侧向轴 z", True,
       lambda n: n.track_pos_err_z_m),
    Ch("verr_z", "侧向速度误差", "m/s", "侧向轴 z", False,
       lambda n: n.track_vel_err_z_mps),
    Ch("hdg_dev", "航迹角偏差",  "°",   "侧向轴 z", False,
       _hdg_dev),
]


class OfflinePlotWindow(QDialog):
    """离线控制误差回放绘图窗口。从 snapshots.jsonl 加载仿真记录并绘制完整时序曲线。"""

    def __init__(self, parent: QWidget | None = None) -> None:
        """初始化窗口控件与内部状态，构建顶栏、侧边栏和空图表区。"""
        super().__init__(parent)
        self.setWindowTitle("离线回放")
        self.resize(1200, 760)
        self.setWindowFlags(
            self.windowFlags()
            | Qt.WindowType.WindowMinimizeButtonHint
            | Qt.WindowType.WindowMaximizeButtonHint
        )

        # node_id -> {color: str, visible: bool, cb: QCheckBox | None}
        self._nodes: dict[str, dict] = {}
        # ch.key -> node_id -> list[(time_s, value)]
        self._bufs: dict[str, dict[str, list[tuple[float, float]]]] = {}
        self._t_min = 0.0
        self._t_max = 1.0

        # (node_id, ch.key) -> QLineSeries
        self._series: dict[tuple[str, str], QLineSeries] = {}
        # ch.key -> (QChart, x_ax, y_ax, zero_series)
        self._rows: dict[str, tuple] = {}
        self._ch_cbs: dict[str, QCheckBox] = {}
        self._path_label: QLabel
        self._node_lay: QVBoxLayout
        self._right_lay: QVBoxLayout

        self._build_ui()

    # ── UI 构建 ───────────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        """组装顶层布局：文件选择顶栏 + 左侧边栏 + 右侧图表区。"""
        root_lay = QVBoxLayout(self)
        root_lay.setContentsMargins(0, 0, 0, 0)
        root_lay.setSpacing(0)

        top = QWidget()
        top_lay = QHBoxLayout(top)
        top_lay.setContentsMargins(8, 6, 8, 4)
        top_lay.setSpacing(8)
        open_btn = QPushButton("打开日志…")
        open_btn.setFixedWidth(90)
        open_btn.clicked.connect(self._choose_file)
        self._path_label = QLabel("（未加载）")
        self._path_label.setStyleSheet("color: #666;")
        top_lay.addWidget(open_btn)
        top_lay.addWidget(self._path_label, stretch=1)
        root_lay.addWidget(top)

        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setFrameShadow(QFrame.Shadow.Sunken)
        root_lay.addWidget(sep)

        body = QWidget()
        body_lay = QHBoxLayout(body)
        body_lay.setContentsMargins(6, 4, 6, 6)
        body_lay.setSpacing(6)
        body_lay.addWidget(self._build_sidebar())
        right = QWidget()
        self._right_lay = QVBoxLayout(right)
        self._right_lay.setContentsMargins(0, 0, 0, 0)
        self._right_lay.setSpacing(2)
        body_lay.addWidget(right, stretch=1)
        root_lay.addWidget(body, stretch=1)

        self._rebuild_charts()

    def _build_sidebar(self) -> QWidget:
        """构建左侧边栏：节点列表（动态）和通道 checkbox（按轴分组）。"""
        sb = QWidget()
        sb.setFixedWidth(170)
        lay = QVBoxLayout(sb)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(4)

        node_box = QGroupBox("节点")
        self._node_lay = QVBoxLayout(node_box)
        self._node_lay.setSpacing(2)
        self._node_lay.addWidget(QLabel("（未加载）"))
        lay.addWidget(node_box)

        ch_box = QGroupBox("通道")
        ch_lay = QVBoxLayout(ch_box)
        ch_lay.setSpacing(1)
        cur_grp = ""
        for ch in OFFLINE_CHANNELS:
            if ch.group != cur_grp:
                cur_grp = ch.group
                sep = QLabel(f"  {ch.group}")
                sep.setStyleSheet("color:#888; font-size:11px;")
                ch_lay.addWidget(sep)
            cb = QCheckBox(ch.label)
            cb.setChecked(ch.on)
            cb.toggled.connect(self._rebuild_charts)
            self._ch_cbs[ch.key] = cb
            ch_lay.addWidget(cb)
        lay.addWidget(ch_box)

        lay.addStretch()
        return sb

    def _refresh_node_panel(self) -> None:
        """清空节点面板并按当前 _nodes 重新填充 checkbox。"""
        while self._node_lay.count():
            w = self._node_lay.takeAt(0).widget()
            if w:
                w.deleteLater()
        if not self._nodes:
            self._node_lay.addWidget(QLabel("（未加载）"))
            return
        for nid, nd in self._nodes.items():
            cb = QCheckBox(nid)
            cb.setChecked(nd["visible"])
            cb.setStyleSheet(f"color:{nd['color']}; font-weight:bold;")
            cb.toggled.connect(lambda v, n=nid: self._toggle_node(n, v))
            nd["cb"] = cb
            self._node_lay.addWidget(cb)

    def _toggle_node(self, nid: str, visible: bool) -> None:
        """切换节点可见性并重建图表。"""
        if nid in self._nodes:
            self._nodes[nid]["visible"] = visible
        self._rebuild_charts()

    # ── 数据加载 ──────────────────────────────────────────────────────────────

    def _choose_file(self) -> None:
        """弹出文件选择对话框，加载选中的 snapshots.jsonl 文件。"""
        path, _ = QFileDialog.getOpenFileName(
            self, "选择快照文件", "logs", "快照文件 (snapshots.jsonl *.jsonl)"
        )
        if path:
            self._load_file(path)

    def _load_file(self, path: str) -> None:
        """解析 JSONL 快照文件，填充缓冲区并重建图表。出错时在顶栏显示简短提示。"""
        self._nodes.clear()
        self._bufs.clear()
        self._t_min = 0.0
        self._t_max = 1.0

        try:
            records: list[dict] = []
            with open(path, encoding="utf-8") as fh:
                for raw in fh:
                    stripped = raw.strip()
                    if stripped:
                        records.append(json.loads(stripped))
        except (OSError, json.JSONDecodeError) as exc:
            self._path_label.setText(f"加载失败: {exc}")
            self._refresh_node_panel()
            self._rebuild_charts()
            return

        if not records:
            self._path_label.setText("文件为空")
            self._refresh_node_panel()
            self._rebuild_charts()
            return

        self._path_label.setText(Path(path).name)
        self._path_label.setToolTip(path)

        times = [float(r.get("time_s", 0.0)) for r in records]
        self._t_min = min(times)
        self._t_max = max(times)

        for record in records:
            t = float(record.get("time_s", 0.0))
            for node_dict in (record.get("nodes") or []):
                if not isinstance(node_dict, dict):
                    continue
                nid = str(node_dict.get("node_id", ""))
                if not nid:
                    continue
                if nid not in self._nodes:
                    color = _PALETTE[len(self._nodes) % len(_PALETTE)]
                    self._nodes[nid] = {"color": color, "visible": True, "cb": None}
                node_kw = {k: v for k, v in node_dict.items() if k in _NODE_STATE_FIELDS}
                try:
                    node = NodeState(**node_kw)
                except TypeError:
                    continue
                for ch in OFFLINE_CHANNELS:
                    v = ch.act(node)
                    if v is not None:
                        (self._bufs
                         .setdefault(ch.key, {})
                         .setdefault(nid, [])
                         .append((t, v)))

        self._refresh_node_panel()
        self._rebuild_charts()

    # ── 图表 ──────────────────────────────────────────────────────────────────

    def _rebuild_charts(self) -> None:
        """按当前勾选的通道和节点重建所有子图，完成后将缓冲区数据填入。"""
        while self._right_lay.count():
            item = self._right_lay.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        self._series.clear()
        self._rows.clear()

        active = [ch for ch in OFFLINE_CHANNELS if self._ch_cbs[ch.key].isChecked()]
        visible_nids = [nid for nid, nd in self._nodes.items() if nd["visible"]]

        if not active or not self._bufs:
            msg = "请打开日志文件，并勾选至少一个通道" if not self._bufs else "请勾选至少一个通道"
            lbl = QLabel(msg)
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
            show_x = i == len(active) - 1
            chart, x_ax, y_ax, zero_s = self._make_chart(ch, visible_nids, show_x)
            self._rows[ch.key] = (chart, x_ax, y_ax, zero_s)

            row = QWidget()
            row_lay = QHBoxLayout(row)
            row_lay.setContentsMargins(0, 0, 0, 0)
            view = QChartView(chart)
            view.setRenderHint(QPainter.RenderHint.Antialiasing)
            view.setRubberBand(QChartView.RubberBand.RectangleRubberBand)
            if len(active) <= 4:
                view.setMinimumHeight(100)
                row_lay.addWidget(view, stretch=1)
                inner_lay.addWidget(row, stretch=1)
            else:
                view.setFixedHeight(155)
                row_lay.addWidget(view, stretch=1)
                inner_lay.addWidget(row)

        self._populate_series()

    def _make_chart(
        self, ch: Ch, node_ids: list[str], show_x: bool
    ) -> tuple[QChart, QValueAxis, QValueAxis, QLineSeries]:
        """创建单通道 QChart，含 y=0 灰色虚线基准和各节点误差曲线。"""
        chart = QChart()
        chart.setMargins(QMargins(2, 2, 6, 2))
        chart.legend().setVisible(False)
        chart.setBackgroundBrush(QColor("#fafafa"))

        hdr_font = QFont()
        hdr_font.setPointSize(9)
        chart.setTitleFont(hdr_font)
        title_str = f"{ch.label}  ({ch.unit})" if ch.unit else ch.label
        chart.setTitle(title_str)

        t_end = self._t_max + _X_MARGIN_S
        x_ax = QValueAxis()
        x_ax.setRange(self._t_min, t_end)
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

        zero = QLineSeries()
        zero.setPen(QPen(QColor("#999999"), 1.2, Qt.PenStyle.DashLine))
        zero.append(self._t_min, 0.0)
        zero.append(t_end, 0.0)
        chart.addSeries(zero)
        zero.attachAxis(x_ax)
        zero.attachAxis(y_ax)

        for nid in node_ids:
            color = QColor(self._nodes[nid]["color"])
            s = QLineSeries()
            s.setPen(QPen(color, 2.0))
            chart.addSeries(s)
            s.attachAxis(x_ax)
            s.attachAxis(y_ax)
            self._series[(nid, ch.key)] = s

        return chart, x_ax, y_ax, zero

    def _populate_series(self) -> None:
        """将缓冲区全量数据填入 series 并刷新 X/Y 轴范围和零基准线端点。"""
        t_end = self._t_max + _X_MARGIN_S
        for ch_key, (_, x_ax, y_ax, zero_s) in self._rows.items():
            all_y: list[float] = []
            for nid, nd in self._nodes.items():
                if not nd["visible"]:
                    continue
                series = self._series.get((nid, ch_key))
                if series is None:
                    continue
                pts = self._bufs.get(ch_key, {}).get(nid, [])
                series.replace([QPointF(t, v) for t, v in pts])
                all_y.extend(v for _, v in pts)
            x_ax.setRange(self._t_min, t_end)
            zero_s.replace([QPointF(self._t_min, 0.0), QPointF(t_end, 0.0)])
            _apply_y_range(y_ax, all_y)
