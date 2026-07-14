"""二维视图共享居中语义测试。"""

from __future__ import annotations

import unittest

from src.ui.gui.side_view import SIDE_VIEW_FIT_RATIO
from src.ui.gui.view_models import FIT_VIEWPORT_RATIO, NodeState, centroid_of_active_nodes


class ViewCenteringTests(unittest.TestCase):
    """锁定健康节点优先的共享质心和侧视图显式留白比例。"""

    def test_centroid_prefers_healthy_nodes_and_falls_back_to_all(self) -> None:
        """有健康节点时忽略异常节点，全部异常时仍返回全体质心。"""

        healthy = NodeState("A01", "leader", 10.0, 20.0, 0.0, 0.0, altitude=100.0)
        fault = NodeState(
            "A02",
            "wingman",
            1000.0,
            2000.0,
            0.0,
            0.0,
            altitude=900.0,
            health="fault",
        )

        self.assertEqual(centroid_of_active_nodes([healthy, fault]), (10.0, 20.0, 100.0))
        healthy.health = "fault"
        self.assertEqual(centroid_of_active_nodes([healthy, fault]), (505.0, 1010.0, 500.0))
        self.assertIsNone(centroid_of_active_nodes([]))

    def test_side_view_fit_ratio_is_explicitly_tighter_than_top_view(self) -> None:
        """侧视图因高度较小使用独立命名比例，不能留下未使用的俯视常量导入。"""

        self.assertEqual(SIDE_VIEW_FIT_RATIO, 0.86)
        self.assertGreater(SIDE_VIEW_FIT_RATIO, FIT_VIEWPORT_RATIO)


if __name__ == "__main__":
    unittest.main()
