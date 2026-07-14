"""状态表 ViewModel 的函数级回归测试。注意：本文件不构造 Qt 对象。"""

from __future__ import annotations

import ast
import unittest
from pathlib import Path

from src.ui.gui.status_table_view_model import link_table_rows, node_table_rows, overall_table_row
from src.ui.gui.view_models import LinkState, NodeState


class StatusTableViewModelTests(unittest.TestCase):
    """覆盖节点、整体和链路表稳定显示规则，避免依赖完整 GUI 长链条。"""

    def test_view_model_and_tests_do_not_import_pyside6(self) -> None:
        """StatusTableViewModel 及本测试文件不得在 import 中引入 PySide6。"""

        paths = [Path("src/ui/gui/status_table_view_model.py"), Path(__file__)]
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

    def test_node_table_rows_translate_health_and_keep_five_columns(self) -> None:
        """节点表翻译已知健康枚举，未知健康值原样显示并固定五列。"""

        nodes = [
            NodeState(
                "A01",
                "leader",
                0.0,
                0.0,
                0.0,
                0.0,
                health="normal",
                track_pos_err_x=1.2,
                track_pos_err_y=-3.4,
                track_pos_err_z=5.6,
            ),
            NodeState(
                "A02",
                "wingman",
                0.0,
                0.0,
                0.0,
                0.0,
                health="mystery",
                track_pos_err_x=-7.8,
                track_pos_err_y=9.1,
                track_pos_err_z=-2.3,
                rally_phase="RALLY_TRANSIT",
            ),
        ]

        rows = node_table_rows(nodes)

        self.assertEqual(rows[0], ["A01", "1.2", "3.4", "-5.6", "正常"])
        self.assertEqual(rows[1], ["A02", "-7.8", "-9.1", "2.3", "mystery"])
        self.assertTrue(all(len(row) == 5 for row in rows))

    def test_node_table_rows_translate_all_known_health_values(self) -> None:
        """节点健康枚举 normal/degraded/fault/lost 全部翻译为中文。"""

        rows = node_table_rows([
            NodeState("N1", "wing", 0.0, 0.0, 0.0, 0.0, health="normal"),
            NodeState("N2", "wing", 0.0, 0.0, 0.0, 0.0, health="degraded"),
            NodeState("N3", "wing", 0.0, 0.0, 0.0, 0.0, health="fault"),
            NodeState("N4", "wing", 0.0, 0.0, 0.0, 0.0, health="lost"),
        ])

        self.assertEqual([row[4] for row in rows], ["正常", "降级", "故障", "失联"])

    def test_overall_table_uses_role_leader_and_exact_route_metrics(self) -> None:
        """整体表优先使用 role 长机，并采用控制器给出的侧偏与待飞距。"""

        nodes = [
            NodeState("A01", "wingman", 100.0, 120.0, 20.0, 0.0, 1300.0, cross_track_error=99.4, distance_to_go=888.6),
            NodeState(
                "A05",
                "leader",
                80.0,
                90.0,
                20.0,
                0.0,
                1200.0,
                vertical_speed=-4.4,
                cross_track_error=12.4,
                distance_to_go=345.6,
            ),
        ]

        row = overall_table_row(nodes)

        self.assertEqual(row, ["12", "346", "1200", "20", "-4"])

    def test_overall_table_marks_missing_route_metrics_without_estimation(self) -> None:
        """侧偏和待飞距缺失时显示破折号，不得用世界坐标伪造业务值。"""

        leader = NodeState(
            "A01",
            "leader",
            1590.0,
            265.0,
            3.0,
            4.0,
            1234.4,
            vertical_speed=9.6,
            cross_track_error=None,
            distance_to_go=None,
        )

        row = overall_table_row([leader])

        self.assertEqual(row, ["—", "—", "1234", "5", "10"])

    def test_overall_table_returns_none_without_nodes(self) -> None:
        """无节点时整体表返回 None，供 GUI 清空表格。"""

        self.assertIsNone(overall_table_row([]))

    def test_link_table_rows_format_direction_latency_loss_and_status(self) -> None:
        """链路表格式化方向、延迟、丢包百分比和正常/丢包状态。"""

        rows = link_table_rows([
            LinkState("A01", "A02", "duplex", 18, 0.01, ok=True),
            LinkState("A02", "A03", "simplex", 30, 0.256, ok=False),
            LinkState("A03", "A04", "mesh", 5, 0.0, ok=True),
        ])

        self.assertEqual(rows[0], ["A01-A02", "双向", "18ms", "1%", "正常"])
        self.assertEqual(rows[1], ["A02-A03", "单向", "30ms", "26%", "丢包"])
        self.assertEqual(rows[2], ["A03-A04", "mesh", "5ms", "0%", "正常"])


if __name__ == "__main__":
    unittest.main()
