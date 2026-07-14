"""离线控制误差回放窗口测试。"""

from __future__ import annotations

import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from src.ui.gui import offline_plot
from src.ui.gui.offline_plot import OfflinePlotWindow


class OfflinePlotTests(unittest.TestCase):
    """锁定离线文件加载失败的界面与日志诊断。"""

    @classmethod
    def setUpClass(cls) -> None:
        """复用 Qt 应用实例。"""

        cls.app = QApplication.instance() or QApplication([])

    def test_invalid_json_updates_label_and_logs_warning(self) -> None:
        """坏 JSON 同时显示错误并写 warning，避免关闭窗口后丢失线索。"""

        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "broken.snapshots.jsonl"
            path.write_text("{broken", encoding="utf-8")
            window = OfflinePlotWindow()

            with patch.object(offline_plot.LOGGER, "warning") as warning:
                window._load_file(str(path))

        self.assertTrue(window._path_label.text().startswith("加载失败:"))
        warning.assert_called_once()
        window.close()


if __name__ == "__main__":
    unittest.main()
