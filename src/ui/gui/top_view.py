"""俯视二维画布。注意：只处理视图交互与绘制。"""

from __future__ import annotations

import math

from PySide6.QtCore import QPoint, QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QPainter, QPainterPath, QPen, QPolygonF
from PySide6.QtWidgets import QFrame, QGraphicsView, QWidget

from src.ui.gui.avoidance_tools import _rounded_inflated_polygon_points, preview_route_marker_points, route_to_polyline
from src.ui.gui.theme_widgets import THEMES, Theme
from src.ui.gui.trail_path_cache import TrailPathCache
from src.ui.gui.view_models import (
    FIT_VIEWPORT_RATIO,
    TOP_VIEW_ORIGIN_MARGIN,
    VIEW_MAX_SCALE,
    VIEW_MIN_SCALE,
    ObstacleView,
    RallyGeometryView,
    ReferenceRoute,
    Snapshot,
    adaptive_world_grid_spacing,
    is_major_grid_line,
    is_leader_node,
    leader_node_from,
    reference_route_points,
)

class TopView(QGraphicsView):
    """支持平移和缩放的俯视编队视图。注意：只负责显示，不修改仿真状态。"""

    viewChanged = Signal()
    manualViewChanged = Signal()
    resetViewRequested = Signal()
    pointClicked = Signal(float, float)

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
        # 视图变换由 scale_value(缩放) 与 offset(世界原点屏幕位置) 描述：x 向右、north/y 向上。
        self.scale_value = 1.0
        self.offset = self._default_offset()
        self.auto_center = False
        self.show_grid = True
        # 通信链路默认显示，可由主窗口工具条复选框关闭。
        self.show_links = True
        self.trail_seconds = 0.0
        # 每架飞机独立维护世界坐标路径块；视图平移和缩放只交给 QPainter 变换处理。
        self._trail_path_caches: dict[str, TrailPathCache] = {}
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
        # 节点退出后及时释放对应路径；仍在场的缓存由绘制时按队列 revision 增量同步。
        active_node_ids = {node.node_id for node in snapshot.nodes}
        for node_id in self._trail_path_caches.keys() - active_node_ids:
            del self._trail_path_caches[node_id]
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
        before = self._viewport_to_world(cursor)
        # 指数因子使每格滚动产生固定比例缩放，手感线性；再夹到上下限。
        factor = math.pow(1.001, delta)
        self.scale_value = min(VIEW_MAX_SCALE, max(VIEW_MIN_SCALE, self.scale_value * factor))
        # 反解 offset，使锚点世界坐标缩放后仍落在光标处（“以光标为中心缩放”）。
        self.offset = QPointF(
            cursor.x() - before.x() * self.scale_value,
            cursor.y() + before.y() * self.scale_value,
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
            if self._selection_is_click():
                # 单击不改变视角，只把屏幕点反解成 ENU 坐标给上层展示经纬度。
                point = self._viewport_to_world(event.position())
                self.pointClicked.emit(point.x(), point.y())
            else:
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
        # 把 offset/scale 装进画家变换：之后均按 ENU 世界坐标绘制，north/y 正方向显示在屏幕上方。
        painter.translate(self.offset)
        painter.scale(self.scale_value, -self.scale_value)
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
            self._draw_rally_geometry(painter, self.snapshot)
            if self.show_links:
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
            (self.offset.y() - point.y()) / self.scale_value,
        )

    def _selection_is_click(self) -> bool:
        """判断当前左键操作是否为单击。注意：小于缩放框阈值时不触发框选缩放。"""
        if self._selection_origin is None or self._selection_current is None:
            return False
        # 与 _zoom_to_selection 的 8px 阈值保持一致，小拖动仍按点击处理。
        return (
            abs(self._selection_origin.x() - self._selection_current.x()) < 8
            and abs(self._selection_origin.y() - self._selection_current.y()) < 8
        )

    def _world_to_viewport(self, point: QPointF) -> QPointF:
        """把世界坐标转换为视口坐标。注意：north/y 正方向映射到屏幕上方。"""
        return QPointF(
            point.x() * self.scale_value + self.offset.x(),
            self.offset.y() - point.y() * self.scale_value,
        )

    def _draw_screen_text(self, painter: QPainter, x: float, y: float, dx: float, dy: float, text: str) -> None:
        """按屏幕坐标绘制文本。注意：避免世界 y 轴翻转导致文字倒置。"""
        screen = self._world_to_viewport(QPointF(x, y))
        painter.save()
        painter.resetTransform()
        painter.drawText(QPointF(screen.x() + dx, screen.y() + dy), text)
        painter.restore()

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
            viewport.height() / 2.0 + center_y * self.scale_value,
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
            rect.height() / 2.0 + center_y * self.scale_value,
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
            rect.height() / 2.0 + center_y * self.scale_value,
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
            if obstacle.kind == "polygon":
                xs.extend(point[0] for point in obstacle.vertices)
                ys.extend(point[1] for point in obstacle.vertices)
            elif obstacle.kind == "rect":
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
        # 反解视口四角对应的世界坐标范围（画家已应用 offset/scale，north/y 向上）。
        rect = self.viewport().rect()
        top_left = self._viewport_to_world(QPointF(rect.left(), rect.top()))
        bottom_right = self._viewport_to_world(QPointF(rect.right(), rect.bottom()))
        left = min(top_left.x(), bottom_right.x())
        right = max(top_left.x(), bottom_right.x())
        bottom = min(top_left.y(), bottom_right.y())
        top = max(top_left.y(), bottom_right.y())
        spacing = self._grid_world_spacing()
        # 把可见范围对齐到网格间距的整数倍，确定起止网格线坐标。
        start_x = math.floor(left / spacing) * spacing
        end_x = math.ceil(right / spacing) * spacing
        start_y = math.floor(bottom / spacing) * spacing
        end_y = math.ceil(top / spacing) * spacing

        # 先画更细、更淡的次网格，再覆盖每五格一条的主网格；线宽反除 scale 以保持屏幕粗细稳定。
        painter.setPen(QPen(self.theme.minor_grid, 0.55 / self.scale_value))
        for x in range(start_x, end_x + spacing, spacing):
            if not is_major_grid_line(x, spacing):
                painter.drawLine(x, start_y, x, end_y)
        for y in range(start_y, end_y + spacing, spacing):
            if not is_major_grid_line(y, spacing):
                painter.drawLine(start_x, y, end_x, y)

        # 6:4 虚线既能和连续次网格区分，也保持缩放后节奏稳定。
        major_pen = QPen(self.theme.grid, 1.0 / self.scale_value)
        major_pen.setDashPattern([6.0, 4.0])
        painter.setPen(major_pen)
        for x in range(start_x, end_x + spacing, spacing):
            if is_major_grid_line(x, spacing):
                painter.drawLine(x, start_y, x, end_y)
        for y in range(start_y, end_y + spacing, spacing):
            if is_major_grid_line(y, spacing):
                painter.drawLine(start_x, y, end_x, y)

    def _grid_world_spacing(self) -> int:
        """返回俯视图当前应使用的网格世界间距（依据自身缩放自适应）。"""
        return adaptive_world_grid_spacing(self.scale_value)

    def _obstacle_center(self, obstacle: ObstacleView) -> tuple[float, float]:
        """返回障碍中心世界坐标。注意：矩形取几何中心，圆取圆心。"""
        if obstacle.kind == "polygon" and obstacle.vertices:
            return (
                sum(point[0] for point in obstacle.vertices) / len(obstacle.vertices),
                sum(point[1] for point in obstacle.vertices) / len(obstacle.vertices),
            )
        if obstacle.kind == "rect":
            return (obstacle.min_x + obstacle.max_x) / 2.0, (obstacle.min_y + obstacle.max_y) / 2.0
        return obstacle.center_x, obstacle.center_y

    def _stroke_obstacle_shape(self, painter: QPainter, obstacle: ObstacleView, inflate: float) -> None:
        """按当前画笔/画刷描绘障碍轮廓。注意：inflate>0 时整体外扩（polygon 为显示近似）。"""
        if obstacle.kind == "polygon" and obstacle.vertices:
            # polygon 安全间距按圆角外扩显示，与后端 inside() 的“点到边距离≤clearance”边界一致，
            # 避免 miter 尖角在角部凸出、令折线看上去擦过所画膨胀框。
            vertices = _rounded_inflated_polygon_points(obstacle.vertices, inflate)
            polygon = QPolygonF([QPointF(east, north) for east, north in vertices])
            painter.drawPolygon(polygon)
        elif obstacle.kind == "rect":
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
            painter.setPen(QPen(self.theme.ink if obstacle.enabled else self.theme.muted, 1))
            text = obstacle.obstacle_id if obstacle.enabled else f"{obstacle.obstacle_id}（未勾选）"
            self._draw_screen_text(painter, center_x, center_y, -10.0, 4.0, text)

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
        # 未飞参考航线使用浅灰蓝细虚线；线宽随缩放归一，圆弧段按弧采样。
        route_color = QColor(self.theme.formation_reference)
        route_color.setAlphaF(0.82)
        pen = QPen(route_color, 1.0 / self.scale_value)
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

    def _draw_rally_geometry(self, painter: QPainter, snapshot: Snapshot) -> None:
        """绘制集结盘旋圆、切入点 T（三角）与松散目标点 M_i（实心圆）。

        注意：只在待命盘旋和集结执行阶段显示——集结完成后这些参考点已经不代表飞机当前目标，
        常驻显示只会长期遮挡后续航段/僚机目标标记，用规范化 rally_phase 过滤及时隐藏。
        长机/僚机分色，避免多机场景下分不清哪个圆属于哪架机；圆在屏幕上太小（缩得很远）时只画
        圆和标记点、不画文字标签，避免多机标签互相重叠看不清。
        """
        if not snapshot.rally_geometry:
            return
        node_by_id = {node.node_id: node for node in snapshot.nodes}
        marker_r = 6.0 / self.scale_value
        # HOLD 后不再显示集结辅助几何，避免遮挡正常任务航线和队形目标。
        visible_phases = {"LOCAL_LOITER", "RALLY_TRANSIT", "RALLY_LOITER", "RALLY_EXITED", "CATCHUP", "LOOSE", "COMPRESS"}

        for geometry in snapshot.rally_geometry:
            node = node_by_id.get(geometry.node_id)
            if node is None or node.rally_phase not in visible_phases:
                continue
            color = QColor(self.theme.leader if is_leader_node(node) else self.theme.wingman)

            # 集结圆使用节点主色，本地待命圆降低透明度，视觉上区分“当前等待区”和“目标集结区”。
            circle_pen = QPen(color, 1.4 / self.scale_value)
            circle_pen.setDashPattern([5, 4])
            painter.setPen(circle_pen)
            painter.setBrush(Qt.BrushStyle.NoBrush)
            if geometry.local_radius > 0.0:
                local_color = QColor(color)
                local_color.setAlphaF(0.45)
                local_pen = QPen(local_color, 1.1 / self.scale_value)
                local_pen.setDashPattern([3, 5])
                painter.setPen(local_pen)
                painter.drawEllipse(QPointF(geometry.local_center_x, geometry.local_center_y), geometry.local_radius, geometry.local_radius)
                # 这条线是规划切线的可视化提示；真实控制仍由算法层输出指令。
                painter.drawLine(QPointF(geometry.local_tangent_x, geometry.local_tangent_y), QPointF(geometry.entry_x, geometry.entry_y))
                painter.setPen(circle_pen)
            painter.drawEllipse(QPointF(geometry.center_x, geometry.center_y), geometry.radius, geometry.radius)

            show_labels = geometry.radius * self.scale_value > 40.0
            self._draw_rally_marker(painter, geometry, color, marker_r, show_labels)

    def _draw_rally_marker(
        self, painter: QPainter, geometry: RallyGeometryView, color: QColor, marker_r: float, show_labels: bool
    ) -> None:
        """绘制单个节点的 M_i（实心圆点）和 T（空心三角）标记，视野足够大时附带文字标签。"""
        painter.setBrush(color)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(QPointF(geometry.slot_x, geometry.slot_y), marker_r, marker_r)
        if show_labels:
            painter.setPen(QPen(color, 1))
            self._draw_screen_text(painter, geometry.slot_x, geometry.slot_y, 8.0, -6.0, f"{geometry.node_id} M")

        triangle = QPainterPath()
        triangle.moveTo(geometry.entry_x, geometry.entry_y + marker_r)
        triangle.lineTo(geometry.entry_x + marker_r, geometry.entry_y - marker_r)
        triangle.lineTo(geometry.entry_x - marker_r, geometry.entry_y - marker_r)
        triangle.closeSubpath()
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.setPen(QPen(color, 1.5 / self.scale_value))
        painter.drawPath(triangle)
        if show_labels:
            painter.setPen(QPen(color, 1))
            self._draw_screen_text(painter, geometry.entry_x, geometry.entry_y, 8.0, -6.0, f"{geometry.node_id} T")

        if geometry.local_radius > 0.0:
            # 本地切出点用方形，集结切入点用三角，避免两个 T 点在多机场景中混淆。
            painter.setBrush(color)
            painter.setPen(QPen(color, 1.2 / self.scale_value))
            painter.drawRect(
                QRectF(
                    geometry.local_tangent_x - marker_r,
                    geometry.local_tangent_y - marker_r,
                    marker_r * 2.0,
                    marker_r * 2.0,
                )
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
            # 正常通信链路用青色短虚线弱化显示，异常链路用警示色粗实线突出。
            color = QColor(self.theme.link if link.ok else self.theme.warn)
            color.setAlphaF(0.72 if link.ok else 0.75)
            pen = QPen(color, (1 if link.ok else 3) / self.scale_value)
            if link.ok:
                pen.setDashPattern([3.0, 4.0])
            painter.setPen(pen)
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
            # 用民航机俯视剪影表示飞机：钝鼻朝 +x，主翼与尾翼保留可辨识的屏幕尺寸。
            path = QPainterPath(QPointF(18, -1.35))
            path.lineTo(13.2, -2.85)
            path.lineTo(3, -3.4)
            path.lineTo(-6, -18)
            path.lineTo(-8, -17)
            path.lineTo(-3, -3)
            path.lineTo(-15.5, -2.4)
            path.lineTo(-16.5, -7.2)
            path.lineTo(-18.6, -6.8)
            path.lineTo(-17.4, 0)
            path.lineTo(-18.6, 6.8)
            path.lineTo(-16.5, 7.2)
            path.lineTo(-15.5, 2.4)
            path.lineTo(-3, 3)
            path.lineTo(-8, 17)
            path.lineTo(-6, 18)
            path.lineTo(3, 3.4)
            path.lineTo(13.2, 2.85)
            path.lineTo(18, 1.35)
            path.quadTo(QPointF(21, 0), QPointF(18, -1.35))
            path.closeSubpath()
            painter.drawPath(path)
            painter.restore()
            # 在机体左上方标注节点 ID（标签不随机体旋转）。
            painter.setPen(QPen(self.theme.ink, 1))
            self._draw_screen_text(painter, node.x, node.y, -13.0, -18.0, node.node_id)

    def _draw_slot_targets(self, painter: QPainter, snapshot: Snapshot) -> None:
        """绘制各机目标槽位标记（菱形 + 连线）。注意：只做渲染，不修改仿真状态；长机/僚机都画，
        长机在 JOINING 阶段目标是盘旋圆切入点/圆上投影点，跟僚机的槽位目标是同一套 cmd_pos 机制。
        """
        visible_phases = {"LOCAL_LOITER", "RALLY_TRANSIT", "RALLY_LOITER", "RALLY_EXITED", "CATCHUP", "LOOSE", "COMPRESS"}
        for node in snapshot.nodes:
            # 仅集结场景绘制目标槽位（HOLD 等其他阶段不需要此标记）
            if node.rally_phase not in visible_phases:
                continue
            # 目标点为原点时跳过（初始化默认值，尚未收到有效指令）
            if node.cmd_pos_x == 0.0 and node.cmd_pos_y == 0.0:
                continue
            base = self.theme.leader if is_leader_node(node) else self.theme.wingman
            color = QColor(self.theme.warn if node.health != "normal" else base)
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
        # 0 秒时直接跳过绘制，避免后续透明度计算出现除零分支。
        # 两个采样点即可表达刚启动后的位移，避免运行初期看起来像静止。
        if self.trail_seconds <= 0.0 or len(node.trail) <= 1:
            return
        base = self.theme.leader if is_leader else self.theme.wingman
        cache = self._trail_path_caches.setdefault(node.node_id, TrailPathCache())
        cache.synchronize(
            node.trail,
            projector=lambda point: (point.x, point.y),
            semantic_key="俯视_EN",
        )
        painter.save()
        try:
            # drawPath 同时使用画笔和画刷；禁用前序航点/机体遗留画刷，避免开放折线首尾闭合填充。
            painter.setBrush(Qt.BrushStyle.NoBrush)
            # 中间稳定块按八档透明度合并，首尾活动块单独画，因此调用次数恒定不随点数增长。
            for batch in cache.render_batches(current_time=current_time, trail_seconds=self.trail_seconds):
                color = QColor(base)
                # 长机尾迹整体比僚机略浓。
                color.setAlphaF((0.52 if is_leader else 0.44) * batch.opacity_factor)
                # cosmetic 画笔在世界变换后仍保持固定像素宽度；圆角可消除急转折线的尖刺感。
                pen = QPen(color, 2.4)
                pen.setCosmetic(True)
                pen.setCapStyle(Qt.PenCapStyle.RoundCap)
                pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
                if not is_leader:
                    # 按稳定累计里程换算设备像素相位，删头和加尾都不会让旧虚线跳动。
                    pen.setDashPattern([6.0, 4.0])
                    pen.setDashOffset(batch.start_path_distance * self.scale_value / 2.4)
                painter.setPen(pen)
                painter.drawPath(batch.path)
        finally:
            # 尾迹绘制不得改变后续飞机、标签和障碍的画刷/画笔状态。
            painter.restore()
