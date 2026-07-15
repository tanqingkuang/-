"""PySide6 主窗口离屏冒烟检查。

在 offscreen 平台下构造主窗口并断言基本布局约束，代替 AGENTS 文档里的
内联 heredoc 片段，把中间噪音留在脚本内、只向 agent 暴露结构化结果。

用法（项目根目录）::

    python -X utf8 scripts/check_gui_offscreen.py

输出契约：
- 全部通过：末行打印 ``gui_offscreen=PASS``，退出码 0。
- 任一失败：逐条打印 ``[FAIL] ...``，末行打印 ``gui_offscreen=FAIL``，退出码 1。
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

# 必须在导入 PySide6 之前设置离屏平台，否则无显示环境会直接崩溃。
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

# 保证从任意工作目录执行时都能 import 到项目包。
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def main() -> int:
    """构造主窗口并逐项断言，返回进程退出码。"""
    from PySide6.QtWidgets import QApplication, QProgressBar

    from src.ui.gui.main_window import MainWindow

    app = QApplication([])
    window = MainWindow()
    window.show()
    app.processEvents()

    failures: list[str] = []

    # 节点/链路表出现横向滚动条说明列宽被挤压，是历史高发布局问题。
    node_hscroll = window.node_table.horizontalScrollBar().maximum()
    link_hscroll = window.link_table.horizontalScrollBar().maximum()
    if node_hscroll != 0:
        failures.append(f"node_table 出现横向滚动条 node_hscroll_max={node_hscroll}")
    if link_hscroll != 0:
        failures.append(f"link_table 出现横向滚动条 link_hscroll_max={link_hscroll}")
    if not isinstance(window.progress, QProgressBar):
        failures.append(f"底部进度条类型错误 progress_type={type(window.progress).__name__}")
    if not window.windowTitle():
        failures.append("窗口标题为空")

    window.close()
    app.quit()

    print(
        f"window_title={window.windowTitle()!r} node_hscroll_max={node_hscroll} "
        f"link_hscroll_max={link_hscroll} progress_type={type(window.progress).__name__}"
    )
    for failure in failures:
        print(f"[FAIL] {failure}", file=sys.stderr)
    print(f"gui_offscreen={'FAIL' if failures else 'PASS'}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
