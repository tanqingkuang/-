"""实时与离线控制图表公共契约测试。"""

from __future__ import annotations

import ast
from pathlib import Path
from types import SimpleNamespace
import unittest

from src.ui.gui.chart_common import CONTROL_ERROR_CHANNELS, heading_deviation


class ChartCommonTests(unittest.TestCase):
    """锁定两种图表窗口共用的通道与计算口径。"""

    def test_control_error_channels_have_one_stable_definition(self) -> None:
        """七个通道的键、分组与默认勾选状态只由公共表定义。"""

        self.assertEqual(
            tuple(channel.key for channel in CONTROL_ERROR_CHANNELS),
            (
                "perr_x",
                "verr_x",
                "perr_y",
                "verr_y",
                "perr_z",
                "verr_z",
                "hdg_dev",
            ),
        )
        self.assertEqual(
            tuple(channel.key for channel in CONTROL_ERROR_CHANNELS if channel.on),
            ("perr_x", "perr_y", "perr_z"),
        )

    def test_heading_deviation_keeps_enu_wrap_contract(self) -> None:
        """航向偏差按 ENU 口径包络到负 180 至正 180 度。"""

        node = SimpleNamespace(
            cmd_vel_east_mps=0.0,
            cmd_vel_north_mps=10.0,
            psi_v_deg=-170.0,
        )

        self.assertEqual(heading_deviation(node), 100.0)
        node.cmd_vel_north_mps = 0.0
        self.assertIsNone(heading_deviation(node))

    def test_offline_window_does_not_import_live_window_private_symbols(self) -> None:
        """离线窗口只能依赖公共图表模块，不能反向耦合实时窗口私有实现。"""

        path = Path("src/ui/gui/offline_plot.py")
        tree = ast.parse(path.read_text(encoding="utf-8"))
        imported_modules = {
            node.module
            for node in ast.walk(tree)
            if isinstance(node, ast.ImportFrom) and node.module is not None
        }

        self.assertNotIn("src.ui.gui.live_monitor", imported_modules)


if __name__ == "__main__":
    unittest.main()
