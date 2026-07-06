"""PyInstaller 裁剪版运行时钩子。注意：必须早于 GUI 主窗口构造执行。"""

from __future__ import annotations

import os

# 裁剪版 exe 固定使用 lite 档位，避免外部环境变量改变编译入口语义。
os.environ["SIMU_GUI_FEATURE_PROFILE"] = "lite"
