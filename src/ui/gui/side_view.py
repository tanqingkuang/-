"""侧视二维画布。注意：横向视野跟随俯视图。"""

from __future__ import annotations

import math

from PySide6.QtCore import QPoint, QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import QWidget

from src.ui.gui.theme_widgets import THEMES, Theme
from src.ui.gui.top_view import TopView
from src.ui.gui.view_models import (
    FIT_VIEWPORT_RATIO,
    VIEW_MAX_SCALE,
    VIEW_MIN_SCALE,
    ReferenceRoute,
    Snapshot,
    adaptive_world_grid_spacing,
    is_leader_node,
    leader_node_from,
    reference_route_points,
)

class SideView(QWidget):
    """高度侧视图。注意：横轴可按当前航段里程或用户视角投影显示。"""

    # 侧视图横向范围独立于 TopView 的像素变换，但语义上跟随同一世界坐标系。
    # 高度轴由本类单独维护，避免俯视图缩放时压缩或拉伸高度读数。
    # segment_locked 为真时横轴沿当前航段里程展开，否则按用户视角投影。
    # 框选和拖拽状态保留为屏幕坐标，绘制选框时不受高度映射影响。
    # 本类只消费快照和主题，不持有控制器，便于 offscreen 构造与交互测试。
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
        self.trail_seconds = 0.0
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
        # 直接在 QWidget 上绘制，横轴和高度轴都由本类显式控制。
        # 固定绘制顺序能保证参考线、轨迹、节点和选框的遮挡关系稳定。
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
        # 当前滚轮只缩放横轴，高度轴由框选或拖拽维护，避免两轴同时跳变。
        # 缩放前记录鼠标下方的世界横坐标，缩放后反推 offset 以保持锚点不动。
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
        # 屏幕向下拖动对应高度范围整体上移，和常见图表平移手感保持一致。
        # 用当前高度跨度换算像素位移，缩放后拖动灵敏度自然随跨度变化。
        plot_height = max(1.0, self.height() - self.PLOT_VERTICAL_MARGINS)
        altitude_delta = delta_y / plot_height * (self.altitude_max - self.altitude_min)
        self.altitude_min += altitude_delta
        self.altitude_max += altitude_delta

    def _zoom_to_selection(self) -> None:
        """执行 to selection 缩放。注意：选区可分别影响横轴和高度轴。"""
        # 横向选区需要足够宽，避免普通点击或窄框误触横轴缩放。
        # 高度选区阈值更低，便于用户快速放大局部高度层。
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
        # 横向网格间距按当前缩放自适应，和俯视图保持相近读数密度。
        # 高度网格固定米制间隔，避免缩放时高度标签频繁跳档。
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
        # 参考航段横坐标统一走 _horizontal_for_point，锁航段和自由视角共用。
        # 这里只画高度剖面，不绘制航点符号，避免侧视图与俯视图信息重复。
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
        if self.trail_seconds <= 0.0:
            return
        # 尾迹按当前横轴投影后再裁剪屏幕范围，减少不可见线段绘制。
        # 透明度随时间衰减，和俯视图一致表达“越新越醒目”。
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
                if age > self.trail_seconds:
                    continue
                alpha = max(0.08, 1.0 - age / self.trail_seconds)
                color = QColor(base)
                color.setAlphaF((0.48 if is_leader else 0.40) * alpha)
                painter.setPen(QPen(color, 2))
                painter.drawLine(QPointF(x1, self._map_y(previous.altitude)), QPointF(x2, self._map_y(current.altitude)))

    def _draw_nodes(self, painter: QPainter, snapshot: Snapshot) -> None:
        """绘制 nodes 画面元素。注意：只做渲染，不修改仿真状态。"""
        # 节点横坐标与轨迹共用同一投影函数，避免机体和尾迹错位。
        # 屏幕外节点直接跳过，避免长航线场景下无意义地绘制标签。
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
        # 无快照、关闭锁定或航段退化时都返回 None，让调用方自然退回手动视角。
        # 不在这里选择其它航段，避免侧视图和控制器声明的当前航段不一致。
        if not self.segment_locked or self.snapshot is None:
            return None
        route = self.snapshot.route
        return route if self._route_unit(route) is not None else None

    def _route_unit(self, route: ReferenceRoute | None) -> tuple[float, float] | None:
        """返回航段单位方向。注意：零长度航段无法定义锁定横轴。"""
        # 零长度航段没有稳定方向，继续锁定会让投影和角度计算产生抖动。
        # 返回单位向量而不是角度，后续点投影可直接做点积。
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
        # 锁定航段时横轴原点取航段起点，读数就是沿航段的里程。
        # 非锁定时按用户视角投影，仍返回米制距离，网格和框选逻辑无需分支。
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
        # 横轴自适应只处理横向窗口，不碰高度范围，避免 resize 时高度读数跳变。
        # 退化包围盒人为扩展 50 米，使单点场景仍有可读横向尺度。
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
        # 高度自适应用 0.86 留白，节点、参考线和标签都不会贴边。
        # 单一高度扩展成 100 米窗口，保证静态初始状态也有纵向尺度。
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
