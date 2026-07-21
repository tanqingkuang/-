"""GUI 仿真适配器协作边界测试。"""

from __future__ import annotations

from unittest.mock import Mock
import unittest

from src.runner.sim_control import CommandResult
from src.ui.gui.simulation_adapter import ControllerSimulationAdapter
from src.ui.gui.trail_view_model import TrailBuffer


class SimulationAdapterStructureTests(unittest.TestCase):
    """锁定命令执行、缓存生命周期和节点转换的独立协作缝。"""

    def setUp(self) -> None:
        """为每个用例建立独立适配器，并在结束时关闭控制器。"""

        self.adapter = ControllerSimulationAdapter()

    def tearDown(self) -> None:
        """释放控制器资源，避免后台线程泄漏到其他测试。"""

        self.adapter.close()

    def test_run_controller_command_records_result_once(self) -> None:
        """所有简单命令共用一次结果记录与快照返回路径。"""

        expected_snapshot = object()
        self.adapter.snapshot = Mock(return_value=expected_snapshot)

        actual = self.adapter._run_controller_command(
            lambda: CommandResult("ERR_INVALID_STATE", "测试拒绝")
        )

        self.assertIs(actual, expected_snapshot)
        self.assertEqual(self.adapter.last_result_code, "ERR_INVALID_STATE")
        self.assertEqual(self.adapter.last_result_message, "测试拒绝")
        self.adapter.snapshot.assert_called_once_with()

    def test_load_config_forwards_default_and_explicit_seed(self) -> None:
        """GUI 配置入口应始终把运行 seed 显式传给控制器，默认值为 0。"""

        self.adapter.controller.load_config = Mock(
            return_value=CommandResult("ERR_CONFIG_INVALID", "测试中止后续加载")
        )
        self.adapter.snapshot = Mock(return_value=object())

        self.adapter.load_config("default.json")
        self.adapter.controller.load_config.assert_called_once_with("default.json", seed=0)

        self.adapter.controller.load_config.reset_mock()
        self.adapter.load_config("seeded.json", seed=2)
        self.adapter.controller.load_config.assert_called_once_with("seeded.json", seed=2)

    def test_reset_trail_state_can_preserve_or_drop_cursor_and_velocity(self) -> None:
        """同一清理入口按调用场景选择是否清速度基准与固定时钟游标。"""

        self.adapter._trail_by_node["A01"] = TrailBuffer()
        self.adapter._last_xy_by_node["A01"] = (1.0, 2.0, 3.0)
        cursor = object()
        self.adapter._trail_cursor = cursor

        self.adapter._reset_trail_state()

        self.assertEqual(self.adapter._trail_by_node, {})
        self.assertIn("A01", self.adapter._last_xy_by_node)
        self.assertIs(self.adapter._trail_cursor, cursor)

        self.adapter._reset_trail_state(reset_velocity=True, reset_cursor=True)
        self.assertEqual(self.adapter._last_xy_by_node, {})
        self.assertIsNone(self.adapter._trail_cursor)

    def test_snapshot_conversion_exposes_focused_helpers(self) -> None:
        """节点、链路和集结几何不再全部内嵌在总快照转换方法中。"""

        self.assertTrue(callable(self.adapter._convert_node))
        self.assertTrue(callable(self.adapter._convert_link))
        self.assertTrue(callable(self.adapter._convert_rally_geometry))


if __name__ == "__main__":
    unittest.main()
