"""主窗口辅助弹窗测试。"""

from __future__ import annotations

import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from src.ui.gui.dialogs import LOG_MAX_BLOCKS, LogDialog


class LogDialogTests(unittest.TestCase):
    """锁定日志弹窗的有界保留契约。"""

    @classmethod
    def setUpClass(cls) -> None:
        """复用 Qt 应用实例。"""

        cls.app = QApplication.instance() or QApplication([])

    def test_append_discards_oldest_lines_after_capacity(self) -> None:
        """超过容量后只保留最新日志，避免长时间运行无限占用内存。"""

        dialog = LogDialog()
        for index in range(LOG_MAX_BLOCKS + 3):
            dialog.append(float(index), "TEST", f"记录 {index}")

        lines = dialog.text.toPlainText().splitlines()
        self.assertEqual(len(lines), LOG_MAX_BLOCKS)
        self.assertIn("记录 3", lines[0])
        self.assertIn(f"记录 {LOG_MAX_BLOCKS + 2}", lines[-1])
        dialog.close()


if __name__ == "__main__":
    unittest.main()
