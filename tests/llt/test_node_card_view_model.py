"""飞机信息卡片 ViewModel 测试。"""

from __future__ import annotations

import unittest

from src.ui.gui.node_card_view_model import (
    CardBoardState,
    CardLayoutConfig,
    ScreenPoint,
    card_rect_for,
)


class NodeCardViewModelTests(unittest.TestCase):
    """锁定共享布局配置和分阶段可见性规则。"""

    def test_layout_config_is_shared_by_rect_and_visibility(self) -> None:
        """卡片矩形与可见性判定消费同一布局对象，贴边节点不得被卡片覆盖。"""

        layout = CardLayoutConfig(
            card_width=80.0,
            card_height=40.0,
            gap_x=16.0,
            gap_y=14.0,
            icon_size=48.0,
            viewport_width=200.0,
            viewport_height=120.0,
        )
        point = ScreenPoint("A01", 190.0, 10.0, is_leader=True)
        rect = card_rect_for(point, layout)
        cards = CardBoardState()

        changed = cards.update_visibility([point], layout)

        self.assertFalse(changed)
        self.assertTrue(cards.is_card_shown("A01"))
        self.assertGreaterEqual(rect.x, 0.0)
        self.assertGreaterEqual(rect.y, 0.0)
        self.assertFalse(rect.x <= point.x <= rect.x + rect.w and rect.y <= point.y <= rect.y + rect.h)

    def test_offscreen_pinned_node_never_occupies_auto_layout(self) -> None:
        """离屏固定节点不生成占位矩形，屏内自动节点仍可显示。"""

        layout = CardLayoutConfig(80.0, 40.0, viewport_width=200.0, viewport_height=120.0)
        cards = CardBoardState(overrides={"A01": True})
        points = [
            ScreenPoint("A01", -50.0, 60.0, is_leader=True),
            ScreenPoint("A02", 100.0, 60.0),
        ]

        cards.update_visibility(points, layout)

        self.assertFalse(cards.is_card_shown("A01"))
        self.assertTrue(cards.is_card_shown("A02"))


if __name__ == "__main__":
    unittest.main()
