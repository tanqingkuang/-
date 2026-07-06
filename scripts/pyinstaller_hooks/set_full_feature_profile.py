"""PyInstaller 全量版运行时钩子。注意：必须早于 GUI 主窗口构造执行。"""

from __future__ import annotations

import os

# 全量版 exe 固定使用 full 档位，避免外部环境变量改变编译入口语义。
os.environ["SIMU_GUI_FEATURE_PROFILE"] = "full"
