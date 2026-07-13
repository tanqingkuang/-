"""飞机信息卡片 ViewModel 的函数级回归测试。注意：本文件不构造 Qt 对象。"""

from __future__ import annotations

import ast
import unittest
from pathlib import Path

from src.ui.gui.node_card_view_model import (
    CardBoardState,
    CardRect,
    ScreenPoint,
    card_rect_for,
    is_point_on_screen,
    pick_node,
)


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
        """卡片默认挂在飞机右上方，恢复边距按候选卡片外扩计算。"""

        rect = card_rect_for(ScreenPoint("A", 100.0, 200.0), 92.0, 58.0, 16.0, 14.0)

        self.assertEqual(rect, CardRect(116.0, 128.0, 92.0, 58.0))
        self.assertFalse(rect.overlaps(CardRect(217.0, 128.0, 20.0, 20.0)))
        self.assertTrue(rect.overlaps(CardRect(217.0, 128.0, 20.0, 20.0), margin=10.0))

    def test_card_rect_without_viewport_stays_at_default_right_upper_anchor(self) -> None:
        """未提供视口尺寸时保持默认右上锚点，不做任何裁剪或方向切换。"""

        rect = card_rect_for(ScreenPoint("A", 620.0, 30.0), 92.0, 58.0, 16.0, 14.0)

        self.assertEqual(rect, CardRect(636.0, -42.0, 92.0, 58.0))

    def test_card_rect_switches_anchor_direction_instead_of_covering_owner(self) -> None:
        """节点贴近视口边缘时改选另一方向的锚点，卡片始终不覆盖属主机体。

        复现评审场景一：640x420 视口内节点位于 (620, 30)。默认右上锚点会让卡片
        顶部越出视口上边界；旧版“整体夹紧”会把卡片强行拉回视口内、直接盖住
        节点本身。修复后应改选左上/右下/左下中第一个完整落入视口的方向，
        卡片矩形绝不可包含节点中心。
        """

        point = ScreenPoint("A", 620.0, 30.0)
        rect = card_rect_for(point, 92.0, 58.0, 16.0, 14.0, viewport_w=640.0, viewport_h=420.0)

        self.assertEqual(rect, CardRect(512.0, 44.0, 92.0, 58.0))
        contains_owner = rect.x <= point.x <= rect.x + rect.w and rect.y <= point.y <= rect.y + rect.h
        self.assertFalse(contains_owner, "卡片矩形不得包含节点中心，否则会覆盖属主飞机")
        self.assertGreaterEqual(rect.x, 0.0)
        self.assertGreaterEqual(rect.y, 0.0)
        self.assertLessEqual(rect.x + rect.w, 640.0)
        self.assertLessEqual(rect.y + rect.h, 420.0)

    def test_card_rect_falls_back_to_best_overlap_when_no_anchor_fully_fits(self) -> None:
        """四个锚点都放不下视口时选相交面积最大的一个，卡片仍不覆盖属主机体。"""

        point = ScreenPoint("A", 5.0, 5.0)
        rect = card_rect_for(point, 92.0, 58.0, 16.0, 14.0, viewport_w=60.0, viewport_h=40.0)

        contains_owner = rect.x <= point.x <= rect.x + rect.w and rect.y <= point.y <= rect.y + rect.h
        self.assertFalse(contains_owner)

    def test_is_point_on_screen_matches_viewport_bounds(self) -> None:
        """离屏判定只看节点中心是否落在视口矩形内。"""

        self.assertTrue(is_point_on_screen(0.0, 0.0, 640.0, 420.0))
        self.assertTrue(is_point_on_screen(640.0, 420.0, 640.0, 420.0))
        self.assertFalse(is_point_on_screen(-120.0, 210.0, 640.0, 420.0))
        self.assertFalse(is_point_on_screen(320.0, -1.0, 640.0, 420.0))

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
        """卡片重叠时长机优先占位，僚机自动退化为纯 ID 标签。

        W01 相对 L01 的偏移量刻意选在“两机卡片互相重叠、但都不落入对方
        48px 图标包络”的区间内，确保本用例只验证优先级占位规则，不与
        `test_icon_envelope_covers_arbitrary_heading_not_just_narrow_box`
        验证的机体避让规则相互干扰。
        """

        state = CardBoardState()
        points = [ScreenPoint("W01", -90.0, -30.0), ScreenPoint("L01", 0.0, 0.0, is_leader=True)]

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
        """自动卡片重叠即隐藏，隐藏后达到十像素净空才恢复。

        W 的纵向偏移固定在 40px，只让水平距离穿越卡片重叠边界，避免同时
        触发机体图标避让规则，单独验证卡片间的滞回阈值。
        """

        state = CardBoardState()
        leader = ScreenPoint("L", 0.0, 0.0, is_leader=True)

        self.assertTrue(state.update_visibility([leader, ScreenPoint("W", 80.0, 40.0)], 92.0, 58.0))
        self.assertFalse(state.is_card_shown("W"))
        self.assertFalse(state.update_visibility([leader, ScreenPoint("W", 97.0, 40.0)], 92.0, 58.0))
        self.assertFalse(state.is_card_shown("W"))
        self.assertTrue(state.update_visibility([leader, ScreenPoint("W", 102.0, 40.0)], 92.0, 58.0))
        self.assertTrue(state.is_card_shown("W"))

    def test_other_aircraft_icon_blocks_an_auto_card(self) -> None:
        """候选卡片碰到其他飞机的默认图标方框时自动退化。"""

        state = CardBoardState(overrides={"A": False})
        points = [ScreenPoint("A", 0.0, 0.0), ScreenPoint("B", -20.0, 43.0)]

        self.assertTrue(state.update_visibility(points, 92.0, 58.0))
        self.assertFalse(state.is_card_shown("B"))

    def test_icon_envelope_covers_arbitrary_heading_not_just_narrow_box(self) -> None:
        """图标遮挡框需覆盖机体任意朝向的外接圆，而非过窄的旧 28px 方框。

        复现评审场景：L01=(0,0) 为长机，W01=(60,0)。旧的 28px 图标框（半宽 14px）
        与 L01 卡片矩形的下边界恰好相切、按"仅边界接触不算重叠"规则漏判为不重叠，
        导致 L01 卡片视觉上侵入 W01 机体。默认 icon_size 改为 48px 后必须正确判定重叠，
        使 L01 卡片退化，不再压住 W01 的真实机体范围。
        """

        state = CardBoardState()
        points = [ScreenPoint("L01", 0.0, 0.0, is_leader=True), ScreenPoint("W01", 60.0, 0.0)]

        self.assertTrue(state.update_visibility(points, 92.0, 58.0))
        self.assertFalse(state.is_card_shown("L01"))

    def test_sync_nodes_removes_departed_visibility_and_overrides(self) -> None:
        """节点退场时同时清理其人工覆盖、遮挡滞回记录与离屏记忆。"""

        state = CardBoardState(
            overrides={"A": True, "B": False},
            visible={"A": False, "B": True},
            on_screen={"A": False, "B": True},
        )

        self.assertTrue(state.sync_nodes({"B"}))
        self.assertEqual(state.overrides, {"B": False})
        self.assertEqual(state.visible, {"B": True})
        self.assertEqual(state.on_screen, {"B": True})
        self.assertFalse(state.sync_nodes({"B"}))

    def test_off_screen_node_never_shows_card_regardless_of_override(self) -> None:
        """节点完全离屏时不生成卡片，pinned_show 也不能凭空显示悬浮卡片。

        复现评审场景二：节点位于 (-120, 210)，640x420 视口下已完全离屏。
        """

        state = CardBoardState()
        points = [ScreenPoint("A", -120.0, 210.0)]

        # 首次检测即离屏：默认全显的初始假设被推翻，视为一次可见性变化。
        self.assertTrue(
            state.update_visibility(points, 92.0, 58.0, viewport_w=640.0, viewport_h=420.0)
        )
        self.assertFalse(state.is_card_shown("A"))

        # 用户此前把它设为强制显示，飞机离屏后同样不应出现悬浮卡片。
        state.overrides["A"] = True
        state.update_visibility(points, 92.0, 58.0, viewport_w=640.0, viewport_h=420.0)
        self.assertFalse(state.is_card_shown("A"))

    def test_off_screen_pinned_card_does_not_block_on_screen_auto_card(self) -> None:
        """离屏节点即使被强制显示，也不能参与贪心占位挡住屏内自动卡片。

        复现评审场景二的后半段：多机时孤立的离屏卡片矩形会落在视口内，
        若仍参与占位会错误地遮挡屏内其它节点的自动卡片。
        """

        state = CardBoardState(overrides={"OFFSCREEN": True})
        # OFFSCREEN 强制显示时若被夹紧到画布内，会在 (0, 138) 附近产生孤立卡片，
        # 与其重叠的屏内节点用同一片区域验证是否被错误阻挡。
        points = [
            ScreenPoint("OFFSCREEN", -120.0, 210.0),
            ScreenPoint("ONSCREEN", 40.0, 210.0, is_leader=True),
        ]

        state.update_visibility(points, 92.0, 58.0, viewport_w=640.0, viewport_h=420.0)

        self.assertFalse(state.is_card_shown("OFFSCREEN"))
        self.assertTrue(state.is_card_shown("ONSCREEN"))


if __name__ == "__main__":
    unittest.main()
