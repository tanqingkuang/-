"""XZ 平面线段几何共享契约测试。"""

from __future__ import annotations

import math
import unittest

from src.ui.gui.situation3d._xz_segment_geometry import (
    edge_positions,
    segment_direction,
    segment_normal,
)


class XzSegmentGeometryTests(unittest.TestCase):
    """锁定历史尾迹与活动尾迹必须共用的转向约定。"""

    def test_direction_normal_and_edges_share_one_orientation(self) -> None:
        """单位方向、左法向和两侧边缘使用同一套 XZ 符号。"""

        start = (10.0, 20.0, 30.0)
        end = (13.0, 99.0, 34.0)

        direction = segment_direction(start, end)
        normal = segment_normal(start, end)
        left, right = edge_positions(start, normal, 5.0)

        self.assertEqual(direction, (0.6, 0.8))
        self.assertEqual(normal, (-0.8, 0.6))
        self.assertEqual(left, (14.0, 20.0, 27.0))
        self.assertEqual(right, (6.0, 20.0, 33.0))

    def test_degenerate_segment_uses_explicit_finite_fallback(self) -> None:
        """纯竖直段可指定侧向兜底，且不得产生 NaN。"""

        start = (1.0, 2.0, 3.0)
        end = (1.0, 8.0, 3.0)

        normal = segment_normal(start, end, fallback=(1.0, 0.0))

        self.assertEqual(normal, (1.0, 0.0))
        self.assertTrue(all(math.isfinite(component) for component in normal))


if __name__ == "__main__":
    unittest.main()
