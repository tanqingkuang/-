"""GUI 障碍类型契约测试。"""

from __future__ import annotations

import unittest

from src.runner.sim_control import ObstacleKind, ObstacleSpec
from src.ui.gui.avoidance_tools import obstacle_spec_to_view, obstacle_view_to_spec
from src.ui.gui.view_models import ObstacleView


class ObstacleKindTests(unittest.TestCase):
    """锁定应用层与 GUI 共用的障碍枚举。"""

    def test_view_normalizes_external_string_to_shared_enum(self) -> None:
        """外部字符串进入显示模型后立即转换为共享枚举。"""

        view = ObstacleView("C1", "circle", radius=20.0)

        self.assertIs(view.kind, ObstacleKind.CIRCLE)
        self.assertEqual(view.kind, "circle")

    def test_runner_and_view_conversion_preserve_enum_identity(self) -> None:
        """应用层与 GUI 往返转换不退回无约束字符串。"""

        spec = ObstacleSpec("R1", ObstacleKind.RECT, min_x=1.0, max_x=2.0)
        view = obstacle_spec_to_view(spec)
        restored = obstacle_view_to_spec(view)

        self.assertIs(view.kind, ObstacleKind.RECT)
        self.assertIs(restored.kind, ObstacleKind.RECT)

    def test_unknown_view_kind_is_rejected(self) -> None:
        """内部显示模型拒绝未知类型，避免静默落入错误绘制分支。"""

        with self.assertRaises(ValueError):
            ObstacleView("X1", "triangle")


if __name__ == "__main__":
    unittest.main()
