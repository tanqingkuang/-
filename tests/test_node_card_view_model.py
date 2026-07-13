"""飞机信息卡片 ViewModel 的函数级回归测试。注意：本文件不构造 Qt 对象。"""

from __future__ import annotations

import ast
import unittest
from pathlib import Path

from src.ui.gui.node_card_view_model import CardBoardState, CardRect, ScreenPoint, card_rect_for, pick_node


class NodeCardViewModelTests(unittest.TestCase):
    """覆盖飞机拾取、卡片覆盖、遮挡优先级与滞回规则。注意：测试不依赖 GUI 控件。"""

    def test_view_model_and_tests_do_not_import_pyside6(self) -> None:
        """卡片 ViewModel 及本测试文件不得在 import 中引入 PySide6。"""

        paths = [Path("src/ui/gui/node_card_view_model.py"), Path(__file__)]
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

    def test_pick_node_returns_nearest_node_inside_radius(self) -> None:
        """点击半径内存在多架飞机时返回距离最近的一架。"""

        points = [ScreenPoint("A", 10.0, 0.0), ScreenPoint("B", 18.0, 0.0)]

        self.assertEqual(pick_node(15.0, 0.0, points), "B")
        self.assertEqual(pick_node(10.0, 0.0, points), "A")

    def test_pick_node_returns_none_when_all_nodes_are_outside_radius(self) -> None:
        """所有飞机都超出命中半径时不产生节点结果。"""

        points = [ScreenPoint("A", 20.1, 0.0), ScreenPoint("B", 0.0, 25.0)]

        self.assertIsNone(pick_node(0.0, 0.0, points))

    def test_card_rect_uses_right_upper_anchor_and_margin(self) -> None:
        """卡片固定挂在飞机右上方，恢复边距按候选卡片外扩计算。"""

        rect = card_rect_for(ScreenPoint("A", 100.0, 200.0), 92.0, 58.0, 16.0, -14.0)

        self.assertEqual(rect, CardRect(116.0, 128.0, 92.0, 58.0))
        self.assertFalse(rect.overlaps(CardRect(217.0, 128.0, 20.0, 20.0)))
        self.assertTrue(rect.overlaps(CardRect(217.0, 128.0, 20.0, 20.0), margin=10.0))

    def test_handle_click_cycles_auto_and_manual_overrides(self) -> None:
        """单击按当前实际可见性设置反向覆盖，再次单击清除覆盖。"""

        state = CardBoardState(visible={"A": True, "B": False})
        points = [ScreenPoint("A", 0.0, 0.0), ScreenPoint("B", 60.0, 0.0)]

        self.assertEqual(state.handle_click(0.0, 0.0, points), "A")
        self.assertIs(state.overrides["A"], False)
        self.assertFalse(state.is_card_shown("A"))
        self.assertEqual(state.handle_click(0.0, 0.0, points), "A")
        self.assertNotIn("A", state.overrides)
        self.assertTrue(state.is_card_shown("A"))

        self.assertEqual(state.handle_click(60.0, 0.0, points), "B")
        self.assertIs(state.overrides["B"], True)
        self.assertTrue(state.is_card_shown("B"))
        self.assertEqual(state.handle_click(60.0, 0.0, points), "B")
        self.assertNotIn("B", state.overrides)
        self.assertFalse(state.is_card_shown("B"))

    def test_blank_click_keeps_existing_overrides(self) -> None:
        """点击空白区域不清除或改变任何飞机的人工覆盖。"""

        state = CardBoardState(overrides={"A": True})

        self.assertIsNone(state.handle_click(100.0, 100.0, [ScreenPoint("A", 0.0, 0.0)]))
        self.assertEqual(state.overrides, {"A": True})

    def test_leader_claims_overlapping_card_before_wingman(self) -> None:
        """卡片重叠时长机优先占位，僚机自动退化为纯 ID 标签。"""

        state = CardBoardState()
        points = [ScreenPoint("W01", 0.0, 0.0), ScreenPoint("L01", 0.0, 0.0, is_leader=True)]

        self.assertTrue(state.update_visibility(points, 92.0, 58.0))
        self.assertTrue(state.is_card_shown("L01"))
        self.assertFalse(state.is_card_shown("W01"))

    def test_pinned_show_cards_ignore_all_overlaps(self) -> None:
        """强制显示卡片互不排斥，即使完全重叠也全部显示。"""

        state = CardBoardState(overrides={"A": True, "B": True})
        points = [ScreenPoint("A", 0.0, 0.0), ScreenPoint("B", 0.0, 0.0)]

        self.assertFalse(state.update_visibility(points, 92.0, 58.0))
        self.assertTrue(state.is_card_shown("A"))
        self.assertTrue(state.is_card_shown("B"))

    def test_visibility_hides_immediately_and_recovers_after_ten_pixel_clearance(self) -> None:
        """自动卡片重叠即隐藏，隐藏后达到十像素净空才恢复。"""

        state = CardBoardState()
        leader = ScreenPoint("L", 0.0, 0.0, is_leader=True)

        self.assertTrue(state.update_visibility([leader, ScreenPoint("W", 80.0, 0.0)], 92.0, 58.0))
        self.assertFalse(state.is_card_shown("W"))
        self.assertFalse(state.update_visibility([leader, ScreenPoint("W", 97.0, 0.0)], 92.0, 58.0))
        self.assertFalse(state.is_card_shown("W"))
        self.assertTrue(state.update_visibility([leader, ScreenPoint("W", 102.0, 0.0)], 92.0, 58.0))
        self.assertTrue(state.is_card_shown("W"))

    def test_other_aircraft_icon_blocks_an_auto_card(self) -> None:
        """候选卡片碰到其他飞机的 28 像素图标方框时自动退化。"""

        state = CardBoardState(overrides={"A": False})
        points = [ScreenPoint("A", 0.0, 0.0), ScreenPoint("B", -20.0, 43.0)]

        self.assertTrue(state.update_visibility(points, 92.0, 58.0))
        self.assertFalse(state.is_card_shown("B"))

    def test_sync_nodes_removes_departed_visibility_and_overrides(self) -> None:
        """节点退场时同时清理其人工覆盖和遮挡滞回记录。"""

        state = CardBoardState(overrides={"A": True, "B": False}, visible={"A": False, "B": True})

        self.assertTrue(state.sync_nodes({"B"}))
        self.assertEqual(state.overrides, {"B": False})
        self.assertEqual(state.visible, {"B": True})
        self.assertFalse(state.sync_nodes({"B"}))


if __name__ == "__main__":
    unittest.main()
