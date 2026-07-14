"""GUI 扰动规格表测试。"""

from __future__ import annotations

import unittest

from src.runner.sim_control import DisturbanceType
from src.ui.gui.disturbance_view_model import DISTURBANCE_ACTIONS, disturbance_action


class DisturbanceViewModelTests(unittest.TestCase):
    """锁定按钮、日志与控制器命令必须由同一数据源生成。"""

    def test_actions_cover_unique_gui_disturbance_types(self) -> None:
        """四个 GUI 动作各自对应唯一的控制器扰动类型。"""

        self.assertEqual(
            tuple(action.kind for action in DISTURBANCE_ACTIONS),
            (
                DisturbanceType.WIND,
                DisturbanceType.NODE_FAULT,
                DisturbanceType.LINK_LOSS,
                DisturbanceType.CLEAR,
            ),
        )
        self.assertEqual(len({action.button_text for action in DISTURBANCE_ACTIONS}), 4)

    def test_action_builds_structured_command_without_alias_translation(self) -> None:
        """节点故障不再经过 fault 之类的第二套字符串别名。"""

        action = disturbance_action(DisturbanceType.NODE_FAULT)

        self.assertEqual(action.command.type, DisturbanceType.NODE_FAULT)
        self.assertEqual(action.command.target, "A02")
        self.assertEqual(action.command.duration_s, 10.0)
        self.assertEqual(action.command.params, {"mode": "degraded"})
        self.assertIn("A02", action.log_text)

    def test_non_gui_disturbance_type_is_rejected_as_value_error(self) -> None:
        """控制器内部类型若没有 GUI 动作规格，应在边界给出稳定参数错误。"""

        with self.assertRaises(ValueError):
            disturbance_action(DisturbanceType.LINK_FAULT)


if __name__ == "__main__":
    unittest.main()
