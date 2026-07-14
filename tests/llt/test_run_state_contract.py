"""运行态枚举与 GUI 命名状态集合测试。"""

from __future__ import annotations

import unittest

from src.runner.sim_control import RunState
from src.ui.gui.view_models import (
    EDITABLE_RUN_STATES,
    INTERACTIVE_RUN_STATES,
    PAUSE_REQUEST_RUN_STATES,
    RALLY_BLOCKED_RUN_STATES,
    TIMER_IDLE_RUN_STATES,
    TOGGLE_START_RUN_STATES,
)


class RunStateContractTests(unittest.TestCase):
    """锁定控制器枚举值和 GUI 使用的状态组合。"""

    def test_run_state_preserves_wire_values(self) -> None:
        """运行态升级为枚举后仍保持既有字符串序列化值。"""

        self.assertEqual(
            tuple(state.value for state in RunState),
            ("UNLOADED", "READY", "RUNNING", "PAUSED", "FINISHED"),
        )
        self.assertEqual(RunState.RUNNING, "RUNNING")

    def test_gui_state_sets_have_explicit_interaction_semantics(self) -> None:
        """按钮、定时器和集结入口不再各自复制状态字符串集合。"""

        self.assertEqual(INTERACTIVE_RUN_STATES, {RunState.READY, RunState.RUNNING, RunState.PAUSED})
        self.assertEqual(EDITABLE_RUN_STATES, {RunState.READY, RunState.PAUSED})
        self.assertEqual(TIMER_IDLE_RUN_STATES, {RunState.READY, RunState.PAUSED, RunState.FINISHED})
        self.assertEqual(
            RALLY_BLOCKED_RUN_STATES,
            {RunState.UNLOADED, RunState.READY, RunState.FINISHED},
        )
        self.assertEqual(
            TOGGLE_START_RUN_STATES,
            {RunState.UNLOADED, RunState.READY, RunState.PAUSED},
        )
        self.assertEqual(PAUSE_REQUEST_RUN_STATES, {RunState.RUNNING, RunState.PAUSED})


if __name__ == "__main__":
    unittest.main()
