"""实时控制数据监控窗口测试。"""

from __future__ import annotations

import os
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from src.ui.gui import live_monitor
from src.ui.gui.live_monitor import LiveMonitorWindow


class LiveMonitorTests(unittest.TestCase):
    """锁定控制回报颜色的未知值诊断。"""

    @classmethod
    def setUpClass(cls) -> None:
        """复用 Qt 应用实例。"""

        cls.app = QApplication.instance() or QApplication([])

    def test_unknown_control_report_warns_once_and_uses_fallback(self) -> None:
        """同一未知回报只告警一次，轮询时不会持续刷日志。"""

        window = LiveMonitorWindow()
        with patch.object(live_monitor.LOGGER, "warning") as warning:
            first = window._control_report_color("新状态")
            second = window._control_report_color("新状态")

        self.assertEqual(first, "#888888")
        self.assertEqual(second, first)
        warning.assert_called_once_with("未知控制回报，使用默认颜色：%s", "新状态")
        window.close()


if __name__ == "__main__":
    unittest.main()
