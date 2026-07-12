"""尾迹 ViewModel 的函数级回归测试。注意：本文件不构造 Qt 对象。"""

from __future__ import annotations

import ast
import unittest
from dataclasses import dataclass
from pathlib import Path

from src.ui.gui.trail_view_model import (
    MAX_TRAIL_DRAW_POINTS,
    TRAIL_HEAD_RAW_POINTS,
    TrailViewModel,
    prune_trail,
    sample_trail_for_display,
)
from src.ui.gui.view_models import trail_seconds_for_duration


@dataclass
class SampleTrailPoint:
    """测试用尾迹点。注意：只暴露 prune_trail 需要的 time 字段。"""

    name: str
    time: float


class TrailViewModelTests(unittest.TestCase):
    """覆盖尾迹控制稳定业务规则，避免依赖完整 GUI 长链条。"""

    def test_view_model_and_tests_do_not_import_pyside6(self) -> None:
        """TrailViewModel 及本测试文件不得在 import 中引入 PySide6。"""

        paths = [Path("src/ui/gui/trail_view_model.py"), Path(__file__)]
        for path in paths:
            with self.subTest(path=str(path)):
                tree = ast.parse(path.read_text(encoding="utf-8"))
                imported_roots = {
                    alias.name.split(".", maxsplit=1)[0]
                    for node in ast.walk(tree)
                    if isinstance(node, ast.Import)
                    for alias in node.names
                }
                imported_roots.update(
                    node.module.split(".", maxsplit=1)[0]
                    for node in ast.walk(tree)
                    if isinstance(node, ast.ImportFrom) and node.module is not None
                )
                self.assertNotIn("PySide6", imported_roots)

    def test_duration_sync_uses_default_seconds_and_expands_range(self) -> None:
        """按飞行时长同步默认尾迹秒数，长时长会放宽控件上限。"""

        view_model = TrailViewModel()
        update = view_model.on_duration_synced(2400.0)

        self.assertEqual(update.seconds, trail_seconds_for_duration(2400.0))
        self.assertEqual(update.range_max, trail_seconds_for_duration(2400.0))
        self.assertFalse(update.refresh_features)

    def test_same_duration_skips_manual_override_until_duration_or_reset_changes(self) -> None:
        """同一时长不重复覆盖手动值，时长变化或重置后才重新同步。"""

        view_model = TrailViewModel()
        first = view_model.on_duration_synced(120.0)
        repeated = view_model.on_duration_synced(120.0)
        changed = view_model.on_duration_synced(240.0)
        view_model.on_reset()
        after_reset = view_model.on_duration_synced(240.0)

        self.assertEqual(first.seconds, trail_seconds_for_duration(120.0))
        self.assertIsNone(repeated.seconds)
        self.assertEqual(repeated.range_max, 600.0)
        self.assertFalse(repeated.refresh_features)
        self.assertEqual(changed.seconds, trail_seconds_for_duration(240.0))
        self.assertEqual(after_reset.seconds, trail_seconds_for_duration(240.0))

    def test_manual_seconds_clamps_negative_and_refreshes_features(self) -> None:
        """手动输入负值按 0 处理，并要求刷新 3D 快照。"""

        view_model = TrailViewModel()
        negative = view_model.on_manual_seconds(-3.0)
        positive = view_model.on_manual_seconds(6.5)

        self.assertEqual(negative.seconds, 0.0)
        self.assertEqual(negative.range_max, 600.0)
        self.assertTrue(negative.refresh_features)
        self.assertEqual(positive.seconds, 6.5)
        self.assertEqual(positive.range_max, 600.0)
        self.assertTrue(positive.refresh_features)

    def test_manual_seconds_never_shrinks_widened_range(self) -> None:
        """长航时同步放宽范围后，手动改小尾迹不得把上限缩回 600。"""

        view_model = TrailViewModel()
        widened = view_model.on_duration_synced(2400.0)
        manual = view_model.on_manual_seconds(6.5)
        skipped = view_model.on_duration_synced(2400.0)

        self.assertGreater(widened.range_max, 600.0)
        self.assertEqual(manual.range_max, widened.range_max)
        self.assertEqual(skipped.range_max, widened.range_max)

    def test_prune_trail_closes_on_zero_or_negative_seconds(self) -> None:
        """尾迹窗口为 0 或负数时直接返回空列表。"""

        trail = [SampleTrailPoint("old", 1.0), SampleTrailPoint("new", 2.0)]

        self.assertEqual(prune_trail(trail, current_time=2.0, trail_seconds=0.0), [])
        self.assertEqual(prune_trail(trail, current_time=2.0, trail_seconds=-1.0), [])

    def test_sample_trail_passthrough_when_within_limit(self) -> None:
        """点数未超上限时原列表直接返回，不产生额外拷贝或抽样。"""

        trail = [SampleTrailPoint(str(index), float(index)) for index in range(MAX_TRAIL_DRAW_POINTS)]

        self.assertIs(sample_trail_for_display(trail), trail)

    def test_sample_trail_bounds_length_and_keeps_head_raw(self) -> None:
        """长尾迹抽样后长度有界，且最近的原始点全部保留(轨迹头必须贴住机体)。"""

        trail = [SampleTrailPoint(str(index), float(index)) for index in range(12000)]

        sampled = sample_trail_for_display(trail)

        self.assertEqual(len(sampled), MAX_TRAIL_DRAW_POINTS)
        # 头部原始点逐一保留：抽样导致的轨迹头滞后正是用户可见的"脱节"。
        self.assertEqual(sampled[-TRAIL_HEAD_RAW_POINTS:], trail[-TRAIL_HEAD_RAW_POINTS:])
        # 首点保留，淡出尾端不会整体缩短。
        self.assertIs(sampled[0], trail[0])
        # 抽样结果保持时间单调，绘制端按相邻点对连线不会出现回折线段。
        times = [point.time for point in sampled]
        self.assertEqual(times, sorted(times))

    def test_sample_trail_clamps_tiny_limit(self) -> None:
        """异常小的上限被夹到安全值，不抛异常也不返回超限结果。"""

        trail = [SampleTrailPoint(str(index), float(index)) for index in range(100)]

        sampled = sample_trail_for_display(trail, max_points=1)

        self.assertLessEqual(len(sampled), 4)
        self.assertIs(sampled[-1], trail[-1])

    def test_prune_trail_keeps_boundary_and_does_not_require_sorted_input(self) -> None:
        """尾迹裁剪保留恰好等于窗口的点，且不假设输入已按时间排序。"""

        trail = [
            SampleTrailPoint("new", 9.5),
            SampleTrailPoint("old", 4.9),
            SampleTrailPoint("edge", 5.0),
            SampleTrailPoint("future", 11.0),
        ]

        kept = prune_trail(trail, current_time=10.0, trail_seconds=5.0)

        self.assertEqual([point.name for point in kept], ["new", "edge", "future"])


if __name__ == "__main__":
    unittest.main()
