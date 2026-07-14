"""二维尾迹分块路径缓存回归测试。"""

from __future__ import annotations

import math
import os
import unittest
from dataclasses import dataclass
from unittest.mock import Mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QPointF, Qt
from PySide6.QtGui import QColor, QImage, QPainter, QPainterPath
from PySide6.QtWidgets import QApplication

from src.ui.gui.side_view import SideView
from src.ui.gui.top_view import TopView
from src.ui.gui.trail_path_cache import (
    DEFAULT_TRAIL_MAX_SEGMENTS_PER_CHUNK,
    TrailPathCache,
    TrailPointLike,
    _farthest_point_from_segment,
    opacity_bucket,
    simplify_polyline_indices,
)
from src.ui.gui.view_models import NodeState, Snapshot, TrailPoint


@dataclass(frozen=True)
class _测试点:
    """提供缓存测试所需的稳定点字段。"""

    x: float
    y: float
    altitude: float
    time: float
    path_distance: float


def _生成测试点(count: int, *, start: int = 0) -> list[_测试点]:
    """生成坐标与时间均稳定递增的测试尾迹。"""

    return [
        _测试点(
            x=float(index),
            y=float(index % 37),
            altitude=1200.0 + float(index % 11),
            time=float(index) / 10.0,
            path_distance=float(index),
        )
        for index in range(start, start + count)
    ]


class _计数序列(list[_测试点]):
    """记录全量迭代次数，用于锁定普通序列热路径复杂度。"""

    def __init__(self, points: list[_测试点]) -> None:
        """保存测试点并清零迭代计数。"""

        super().__init__(points)
        self.iteration_count = 0

    def __iter__(self):  # noqa: ANN204
        """记录一次全量迭代。注意：端点索引读取不会进入这里。"""

        self.iteration_count += 1
        return super().__iter__()


class 二维尾迹路径缓存测试(unittest.TestCase):
    """验证增量同步、固定绘制上界与投影失效语义。"""

    app: QApplication

    @classmethod
    def setUpClass(cls) -> None:
        """建立离屏 Qt 应用，供路径和视图构造使用。"""

        cls.app = QApplication.instance() or QApplication([])

    def test_尾迹点以显式协议约束必需字段(self) -> None:
        """缓存接受结构化点对象，并公开可供静态检查复用的字段契约。"""

        point = _测试点(1.0, 2.0, 3.0, 4.0, 5.0)
        self.assertIsInstance(point, TrailPointLike)

    def test_六千点删头加尾时中间块对象保持不变(self) -> None:
        """滑动窗口只允许首尾块重建，不能搬动既有中间路径。"""

        points = _生成测试点(6000)
        cache = TrailPathCache(chunk_size=64, opacity_buckets=8)
        cache.synchronize(points, projector=lambda point: (point.x, point.y), semantic_key="俯视")
        before = cache.chunk_paths
        build_count = cache.stats.chunk_path_builds

        cache.synchronize(
            [*points[1:], _生成测试点(1, start=6000)[0]],
            projector=lambda point: (point.x, point.y),
            semantic_key="俯视",
        )
        after = cache.chunk_paths

        self.assertEqual(len(before), len(after))
        self.assertGreater(len(after), 4)
        for old_path, new_path in zip(before[1:-1], after[1:-1]):
            self.assertIs(old_path, new_path)
        self.assertGreaterEqual(cache.stats.last_reused_chunk_paths, len(after) - 2)
        self.assertEqual(cache.stats.chunk_path_builds, build_count + 2)
        self.assertEqual(cache.stats.last_appended_points, 1)
        self.assertEqual(cache.stats.last_removed_points, 1)

    def test_透明度批次绘制次数与点数无关(self) -> None:
        """大量尾迹必须合并为固定数量路径，而不是逐段调用画家。"""

        points = _生成测试点(6000)
        cache = TrailPathCache(chunk_size=64, opacity_buckets=8)
        cache.synchronize(points, projector=lambda point: (point.x, point.y), semantic_key="俯视")

        batches = cache.render_batches(current_time=points[-1].time, trail_seconds=1000.0)

        self.assertLessEqual(len(batches), 10)
        self.assertEqual(cache.stats.max_draw_path_calls, 10)
        self.assertEqual(sum(batch.segment_count for batch in batches), cache.stats.display_segment_count)
        self.assertLessEqual(
            cache.stats.display_segment_count,
            cache.stats.chunk_count * DEFAULT_TRAIL_MAX_SEGMENTS_PER_CHUNK,
        )
        self.assertTrue(all(isinstance(batch.path, QPainterPath) for batch in batches))

    def test_局部简化保留尖锐拐点(self) -> None:
        """块内预算有限时必须保留最大几何误差点，避免把折角拉成直线。"""

        points = [
            QPointF(0.0, 0.0),
            QPointF(1.0, 0.0),
            QPointF(2.0, 0.0),
            QPointF(3.0, 10.0),
            QPointF(4.0, 0.0),
            QPointF(5.0, 0.0),
            QPointF(6.0, 0.0),
        ]

        kept = simplify_polyline_indices(points, tolerance=0.1, max_segments=2)

        self.assertEqual(kept, (0, 3, 6))

    def test_固定十赫兹盘旋圆弧按默认预算保持亚米级误差(self) -> None:
        """一块 10 Hz 盘旋样本不能因段预算不足退化成肉眼可见的多边形。"""

        radius = 200.0
        # 20 m/s 绕 200 m 半径飞行时，每个 0.1 s 固定样本前进 0.01 rad。
        points = [
            QPointF(radius * math.cos(index * 0.01), radius * math.sin(index * 0.01))
            for index in range(129)
        ]

        kept = simplify_polyline_indices(points)
        max_error_squared = max(
            _farthest_point_from_segment(points, start, end)[1]
            if end - start > 1
            else 0.0
            for start, end in zip(kept, kept[1:])
        )

        self.assertLessEqual(max_error_squared, 0.75**2 + 1e-9)

    def test_最大容量尾迹显示线段受硬上限约束(self) -> None:
        """32768 个原始点不能继续把数万条线段交给 QPainter 栅格化。"""

        points = _生成测试点(32768)
        cache = TrailPathCache()

        cache.synchronize(points, projector=lambda point: (point.x, point.y), semantic_key="俯视")
        batches = cache.render_batches(current_time=points[-1].time, trail_seconds=4000.0)

        self.assertLessEqual(cache.stats.chunk_count, 257)
        self.assertLessEqual(cache.stats.display_segment_count, 2056)
        self.assertEqual(sum(batch.segment_count for batch in batches), cache.stats.display_segment_count)

    def test_未变化普通序列热路径不再全量扫描(self) -> None:
        """无 revision 的兼容序列也应按稳定首尾签名跳过重复全表比对。"""

        points = _计数序列(_生成测试点(6000))
        cache = TrailPathCache()
        cache.synchronize(points, projector=lambda point: (point.x, point.y), semantic_key="俯视")
        points.iteration_count = 0

        cache.synchronize(points, projector=lambda point: (point.x, point.y), semantic_key="俯视")

        self.assertEqual(points.iteration_count, 0)

    def test_一次跨多块删头只重建最终部分首块(self) -> None:
        """骤然缩短尾迹窗口时应整块丢弃，不能为每个过期点反复执行局部简化。"""

        points = _生成测试点(6000)
        cache = TrailPathCache(chunk_size=64)
        cache.synchronize(points, projector=lambda point: (point.x, point.y), semantic_key="俯视")
        build_count = cache.stats.chunk_path_builds

        cache.synchronize(points[5500:], projector=lambda point: (point.x, point.y), semantic_key="俯视")

        self.assertEqual(cache.stats.last_removed_points, 5500)
        self.assertLessEqual(cache.stats.chunk_path_builds - build_count, 1)

    def test_平移缩放不重建路径而投影语义改变会重建(self) -> None:
        """显示变换应留给画家，只有侧视投影规则变化才使缓存失效。"""

        points = _生成测试点(130)
        cache = TrailPathCache(chunk_size=64, opacity_buckets=8)
        cache.synchronize(points, projector=lambda point: (point.x, point.y), semantic_key=("俯视",))
        before = cache.chunk_paths
        build_count = cache.stats.chunk_path_builds

        # 同一语义下即使传入另一投影闭包，也视为仅外部显示变换，不应触碰世界路径。
        cache.synchronize(points, projector=lambda point: (point.x * 9.0, point.y * 7.0), semantic_key=("俯视",))
        self.assertEqual(cache.stats.chunk_path_builds, build_count)
        for old_path, new_path in zip(before, cache.chunk_paths):
            self.assertIs(old_path, new_path)

        cache.synchronize(points, projector=lambda point: (point.x, point.altitude), semantic_key=("侧视", 90.0))
        self.assertEqual(cache.stats.chunk_path_builds, build_count + len(before))
        for old_path, new_path in zip(before, cache.chunk_paths):
            self.assertIsNot(old_path, new_path)

    def test_透明度分档边界稳定(self) -> None:
        """透明度分档需夹紧越界年龄，并把最新段归入最高档。"""

        self.assertEqual(opacity_bucket(age=-1.0, trail_seconds=10.0, bucket_count=8), 7)
        self.assertEqual(opacity_bucket(age=0.0, trail_seconds=10.0, bucket_count=8), 7)
        self.assertEqual(opacity_bucket(age=10.0, trail_seconds=10.0, bucket_count=8), 0)
        self.assertEqual(opacity_bucket(age=99.0, trail_seconds=10.0, bucket_count=8), 0)

    def test_俯视图大量尾迹只批量绘制路径(self) -> None:
        """俯视尾迹不得再逐段 setPen 与 drawLine。"""

        view = TopView()
        view.trail_seconds = 1000.0
        points = [TrailPoint(point.x, point.y, point.altitude, point.time, point.path_distance) for point in _生成测试点(6000)]
        painter = Mock()

        view._draw_trail(
            painter,
            NodeState("A01", "leader", points[-1].x, points[-1].y, 1.0, 0.0, trail=points),
            True,
            points[-1].time,
        )

        painter.drawLine.assert_not_called()
        self.assertLessEqual(painter.drawPath.call_count, 10)
        self.assertEqual(painter.setPen.call_count, painter.drawPath.call_count)
        self.assertLessEqual(sum(call.args[0].elementCount() - 1 for call in painter.drawPath.call_args_list), 380)

    def test_俯视尾迹开放路径不会继承画刷形成黑色填充面(self) -> None:
        """批量折线必须强制禁用遗留画刷，不能把开放路径首尾闭合为黑色三角形。"""

        view = TopView()
        view.trail_seconds = 10.0
        background = QColor("#ffffff")
        image = QImage(220, 180, QImage.Format.Format_ARGB32_Premultiplied)
        image.fill(background)
        points = [
            TrailPoint(10.0, 10.0, 1200.0, 0.0, 0.0),
            TrailPoint(110.0, 160.0, 1200.0, 1.0, 180.0),
            TrailPoint(210.0, 10.0, 1200.0, 2.0, 360.0),
        ]
        inherited_brush_color = QColor("#101820")
        painter = QPainter(image)
        try:
            # 模拟航点或上一架飞机留下的深色填充画刷；旧 drawPath 会把折线内部整片填黑。
            painter.setBrush(inherited_brush_color)
            view._draw_trail(
                painter,
                NodeState("A01", "leader", 210.0, 10.0, 1.0, 0.0, trail=points),
                True,
                2.0,
            )
            restored_brush = painter.brush()
        finally:
            painter.end()

        self.assertEqual(image.pixelColor(110, 60), background)
        self.assertNotEqual(image.pixelColor(110, 160), background)
        self.assertEqual(restored_brush.style(), Qt.BrushStyle.SolidPattern)
        self.assertEqual(restored_brush.color(), inherited_brush_color)

    def test_俯视当前机位只追加队列外实时端点段(self) -> None:
        """当前机位变化只能新增队尾连线，不能写入队列或重建稳定历史路径。"""

        view = TopView()
        view.trail_seconds = 10.0
        trail = [
            TrailPoint(0.0, 0.0, 1200.0, 0.0, 0.0),
            TrailPoint(10.0, 4.0, 1200.0, 1.0, 20.0),
        ]
        initial_painter = Mock()
        view._draw_trail(
            initial_painter,
            NodeState("A02", "wingman", 10.0, 4.0, 1.0, 0.0, trail=trail),
            False,
            1.0,
        )
        cache = view._trail_path_caches["A02"]
        history_paths = cache.chunk_paths
        history_builds = cache.stats.chunk_path_builds
        stable_trail = tuple(trail)

        painter = Mock()
        view._draw_trail(
            painter,
            NodeState("A02", "wingman", 16.0, 9.0, 1.0, 0.0, trail=trail),
            False,
            1.0,
        )

        painter.drawLine.assert_called_once_with(QPointF(10.0, 4.0), QPointF(16.0, 9.0))
        realtime_pen = painter.setPen.call_args_list[-1].args[0]
        self.assertEqual(realtime_pen.style(), Qt.PenStyle.CustomDashLine)
        self.assertAlmostEqual(realtime_pen.dashOffset(), 20.0 * view.scale_value / 2.4)
        self.assertAlmostEqual(realtime_pen.color().alphaF(), 0.44, delta=0.01)
        self.assertEqual(tuple(trail), stable_trail)
        self.assertEqual(cache.stats.chunk_path_builds, history_builds)
        for before, after in zip(history_paths, cache.chunk_paths):
            self.assertIs(before, after)

    def test_俯视单点尾迹可连实时端点且空队列或重合时不画(self) -> None:
        """一个稳定采样点足以连接当前机位，但无队尾或端点重合时不得制造退化线段。"""

        view = TopView()
        view.trail_seconds = 10.0
        tail = TrailPoint(10.0, 4.0, 1200.0, 1.0, 20.0)

        moving_painter = Mock()
        view._draw_trail(
            moving_painter,
            NodeState("A01", "leader", 16.0, 9.0, 1.0, 0.0, trail=[tail]),
            True,
            1.0,
        )
        moving_painter.drawPath.assert_not_called()
        moving_painter.drawLine.assert_called_once_with(QPointF(10.0, 4.0), QPointF(16.0, 9.0))

        same_painter = Mock()
        view._draw_trail(
            same_painter,
            NodeState("A01", "leader", 10.0, 4.0, 1.0, 0.0, trail=[tail]),
            True,
            1.0,
        )
        same_painter.drawLine.assert_not_called()

        empty_painter = Mock()
        view._draw_trail(
            empty_painter,
            NodeState("A01", "leader", 16.0, 9.0, 1.0, 0.0, trail=[]),
            True,
            1.0,
        )
        empty_painter.drawLine.assert_not_called()

    def test_侧视图大量尾迹只批量绘制路径(self) -> None:
        """侧视尾迹不得再映射并逐段 drawLine。"""

        view = SideView()
        view.resize(900, 260)
        view.trail_seconds = 1000.0
        points = [TrailPoint(point.x, point.y, point.altitude, point.time, point.path_distance) for point in _生成测试点(6000)]
        snapshot = Snapshot(
            time=points[-1].time,
            duration=1000.0,
            step=0.1,
            run_state="运行",
            control_report="保持",
            disturbance="无",
            nodes=[
                NodeState(
                    "A01",
                    "leader",
                    points[-1].x,
                    points[-1].y,
                    1.0,
                    0.0,
                    altitude=points[-1].altitude,
                    trail=points,
                )
            ],
            links=[],
        )
        view.snapshot = snapshot
        painter = Mock()

        view._draw_trails(painter, snapshot)

        painter.drawLine.assert_not_called()
        painter.setBrush.assert_called_with(Qt.BrushStyle.NoBrush)
        self.assertLessEqual(painter.drawPath.call_count, 10)
        self.assertEqual(painter.setPen.call_count, painter.drawPath.call_count)
        self.assertLessEqual(sum(call.args[0].elementCount() - 1 for call in painter.drawPath.call_args_list), 380)

    def test_侧视当前机位只追加队列外实时端点段(self) -> None:
        """侧视实时段使用投影距离和当前高度，同时保持历史缓存与队列不变。"""

        view = SideView()
        view.resize(900, 260)
        view.segment_locked = False
        view.view_angle_deg = 0.0
        view.trail_seconds = 10.0
        trail = [
            TrailPoint(0.0, 5.0, 1180.0, 0.0, 0.0),
            TrailPoint(10.0, 5.0, 1200.0, 1.0, 20.0),
        ]
        initial = Snapshot(
            time=1.0,
            duration=10.0,
            step=0.1,
            run_state="运行",
            control_report="保持",
            disturbance="无",
            nodes=[NodeState("A01", "leader", 10.0, 5.0, 1.0, 0.0, altitude=1200.0, trail=trail)],
            links=[],
        )
        view._draw_trails(Mock(), initial)
        cache = view._trail_path_caches["A01"]
        history_paths = cache.chunk_paths
        history_builds = cache.stats.chunk_path_builds
        stable_trail = tuple(trail)
        current = Snapshot(
            time=1.0,
            duration=10.0,
            step=0.1,
            run_state="运行",
            control_report="保持",
            disturbance="无",
            nodes=[NodeState("A01", "leader", 25.0, 5.0, 1.0, 0.0, altitude=1230.0, trail=trail)],
            links=[],
        )
        painter = Mock()

        view._draw_trails(painter, current)

        painter.drawLine.assert_called_once_with(QPointF(10.0, 1200.0), QPointF(25.0, 1230.0))
        realtime_pen = painter.setPen.call_args_list[-1].args[0]
        self.assertEqual(realtime_pen.style(), Qt.PenStyle.SolidLine)
        self.assertAlmostEqual(realtime_pen.color().alphaF(), 0.48, delta=0.01)
        self.assertEqual(tuple(trail), stable_trail)
        self.assertEqual(cache.stats.chunk_path_builds, history_builds)
        for before, after in zip(history_paths, cache.chunk_paths):
            self.assertIs(before, after)

    def test_侧视单点尾迹可连实时端点且空队列或投影重合时不画(self) -> None:
        """侧视图允许单点队尾连当前高度，但投影端点重合或无尾迹时跳过。"""

        view = SideView()
        view.resize(900, 260)
        view.segment_locked = False
        view.view_angle_deg = 0.0
        view.trail_seconds = 10.0
        tail = TrailPoint(10.0, 5.0, 1200.0, 1.0, 20.0)

        moving = Snapshot(
            time=1.0,
            duration=10.0,
            step=0.1,
            run_state="运行",
            control_report="保持",
            disturbance="无",
            nodes=[NodeState("A01", "leader", 25.0, 5.0, 1.0, 0.0, altitude=1230.0, trail=[tail])],
            links=[],
        )
        moving_painter = Mock()
        view._draw_trails(moving_painter, moving)
        moving_painter.drawPath.assert_not_called()
        moving_painter.drawLine.assert_called_once_with(QPointF(10.0, 1200.0), QPointF(25.0, 1230.0))

        same = Snapshot(
            time=1.0,
            duration=10.0,
            step=0.1,
            run_state="运行",
            control_report="保持",
            disturbance="无",
            nodes=[NodeState("A01", "leader", 10.0, 99.0, 1.0, 0.0, altitude=1200.0, trail=[tail])],
            links=[],
        )
        same_painter = Mock()
        view._draw_trails(same_painter, same)
        same_painter.drawLine.assert_not_called()

        empty = Snapshot(
            time=1.0,
            duration=10.0,
            step=0.1,
            run_state="运行",
            control_report="保持",
            disturbance="无",
            nodes=[NodeState("A01", "leader", 25.0, 5.0, 1.0, 0.0, altitude=1230.0, trail=[])],
            links=[],
        )
        empty_painter = Mock()
        view._draw_trails(empty_painter, empty)
        empty_painter.drawLine.assert_not_called()


if __name__ == "__main__":
    unittest.main()
