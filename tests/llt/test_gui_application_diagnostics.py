"""GUI 应用层安全降级诊断测试。"""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from src.runner import gui_application


class GuiApplicationDiagnosticsTests(unittest.TestCase):
    """锁定辅助配置失败时的 debug 可诊断性。"""

    def test_invalid_json_keeps_safe_fallback_and_logs_debug(self) -> None:
        """坏 JSON 不阻断 GUI 加载，但应留下异常诊断。"""

        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "broken.json"
            path.write_text("{broken", encoding="utf-8")

            with patch.object(gui_application.LOGGER, "debug") as debug:
                data = gui_application.load_gui_config(str(path))

        self.assertEqual(data.obstacles, ())
        self.assertIsNone(data.avoidance_params)
        self.assertGreaterEqual(debug.call_count, 1)


if __name__ == "__main__":
    unittest.main()
