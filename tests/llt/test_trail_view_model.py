"""尾迹 ViewModel 的函数级回归测试。注意：本文件不构造 Qt 对象。"""

from __future__ import annotations

import ast
import unittest
from dataclasses import FrozenInstanceError
from pathlib import Path

from src.ui.gui.trail_view_model import (
    MAX_TRAIL_POINTS,
    TrailBuffer,
    TrailViewModel,
)
from src.ui.gui.view_models import trail_seconds_for_duration


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

    def test_trail_buffer_exposes_public_hard_capacity(self) -> None:
        """共享尾迹队列必须有公开且足以覆盖内置最长默认窗口的硬容量。"""

        self.assertEqual(MAX_TRAIL_POINTS, 32_768)
        self.assertEqual(TrailBuffer().capacity, MAX_TRAIL_POINTS)

    def test_trail_buffer_capacity_only_drops_head_and_preserves_history(self) -> None:
        """队列满后只弹出最老点，尚未过期的历史点对象及坐标不得移动。"""

        trail = TrailBuffer(capacity=3)
        first = trail.append_position(1.0, 11.0, 101.0, 1.0)
        second = trail.append_position(2.0, 12.0, 102.0, 2.0)
        third = trail.append_position(3.0, 13.0, 103.0, 3.0)
        second_coordinates = (second.x, second.y, second.altitude)

        fourth = trail.append_position(4.0, 14.0, 104.0, 4.0)

        self.assertNotIn(first, trail)
        self.assertEqual(list(trail), [second, third, fourth])
        self.assertIs(trail[0], second)
        self.assertEqual((second.x, second.y, second.altitude), second_coordinates)
        self.assertEqual([point.point_id for point in trail], [1, 2, 3])
        self.assertEqual(trail.first_sequence, 1)
        self.assertEqual(trail.end_sequence, 4)

    def test_trail_buffer_prunes_from_head_and_keeps_time_boundary(self) -> None:
        """时间裁剪通过弹头完成，保留窗口边界点并报告连续逻辑序号。"""

        trail = TrailBuffer(capacity=8)
        old = trail.append_position(1.0, 0.0, 10.0, 1.0)
        boundary = trail.append_position(2.0, 0.0, 20.0, 5.0)
        recent = trail.append_position(3.0, 0.0, 30.0, 9.0)

        removed = trail.expire(current_time=10.0, trail_seconds=5.0)

        self.assertEqual(removed, 1)
        self.assertNotIn(old, trail)
        self.assertEqual(list(trail), [boundary, recent])
        self.assertIs(trail[0], boundary)
        self.assertEqual(trail[0:2], [boundary, recent])
        self.assertEqual(trail.first_sequence, 1)
        self.assertEqual(trail.end_sequence, 3)

    def test_trail_points_are_immutable_and_duplicate_frame_is_not_appended(self) -> None:
        """已提交点不可修改，同一仿真时刻的重复快照不会产生重复节点。"""

        trail = TrailBuffer(capacity=8)
        point = trail.append_position(1.0, 2.0, 3.0, 4.0)
        repeated = trail.append_position(9.0, 9.0, 9.0, 4.0)

        self.assertIs(repeated, point)
        self.assertEqual(len(trail), 1)
        with self.assertRaises(FrozenInstanceError):
            point.x = 99.0

    def test_trail_snapshot_is_stable_and_clear_changes_generation(self) -> None:
        """旧快照不随队列追加而变化，清空后 generation 明确通知绘制缓存重建。"""

        trail = TrailBuffer(capacity=8)
        first = trail.append_position(1.0, 2.0, 3.0, 1.0)
        before = trail.snapshot()
        old_generation = before.generation

        trail.append_position(4.0, 5.0, 6.0, 2.0)
        after = trail.snapshot()

        self.assertEqual(list(before), [first])
        self.assertEqual(len(after), 2)
        self.assertIs(after[0], before[0])
        self.assertEqual(after.first_sequence, 0)
        self.assertEqual(after.end_sequence, 2)

        trail.clear()
        cleared = trail.snapshot()
        self.assertNotEqual(cleared.generation, old_generation)
        self.assertEqual(list(cleared), [])
        self.assertEqual(cleared.first_sequence, 0)
        self.assertEqual(cleared.end_sequence, 0)


if __name__ == "__main__":
    unittest.main()
