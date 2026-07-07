# Windows 运行与编译入口说明

本项目 Windows x64 exe 统一在 Windows x64 环境构建。PySide6/Qt 依赖目标平台的 Python wheel、Qt DLL 和 platform plugin，不建议在 macOS 或其他平台伪造 Windows 产物。

入口分两类：**`run_*_dev` 用于本地调试**（直接跑源码、秒级迭代、不打包）；**`build_*_release` 用于对外发布 exe**（PyInstaller 打包，耗时以 Qt 收集为主，与改动量无关）。调参/改代码请走 `run_*_dev`，不要为了看效果去跑打包。

## 功能档位

| 档位 | 功能范围 | 运行时选择方式 |
| --- | --- | --- |
| 全量版 | 保留主仿真、避障规划、控制监控、数据分析、3D 态势、帮助/主题/日志 | PyInstaller runtime hook 固化 `SIMU_GUI_FEATURE_PROFILE=full` |
| 裁剪版 | 保留主仿真、避障规划、帮助/主题/日志；裁剪控制监控、数据分析、3D 态势 | PyInstaller runtime hook 固化 `SIMU_GUI_FEATURE_PROFILE=lite` |

裁剪版不注册以下顶部入口：

- `控制监控(&V)`，包括 `数据监控(&M)` 和 `离线分析(&A)`。
- `数据分析(&D)`，包括 `控制效果分析(&A)`。
- `3D态势(&3)`。

## 四个入口

| 脚本 | 档位 | 形态 | 产物/行为 | 主要用途 |
| --- | --- | --- | --- | --- |
| `scripts/run_windows_full_dev.ps1` | 全量版 | 跑源码(不打包) | 设 `SIMU_GUI_FEATURE_PROFILE=full` 后 `python src/ui/gui/main_window.py`，秒级启动 | 本地调试，改代码重跑即可 |
| `scripts/run_windows_lite_dev.ps1` | 裁剪版 | 跑源码(不打包) | 同上，档位 `lite`（菜单/功能门控按裁剪版） | 本地调试裁剪版 |
| `scripts/build_windows_full_release.ps1` | 全量版 | `--onefile` 打包 | `dist\编队仿真.exe` | 对外发布完整功能 exe |
| `scripts/build_windows_lite_release.ps1` | 裁剪版 | `--onefile` 打包 | `dist\编队仿真-裁剪版.exe` | 对外发布裁剪功能 exe |

> 说明：`run_*_dev` 支持 `-InstallDependencies` 首次安装 `requirements-gui.txt`，其余参数透传给主程序。裁剪版打包才会 `--exclude-module` 真正剔除重功能；源码调试下裁剪只体现在 `lite` 档的菜单/功能门控，模块仍在源码树里。

## 裁剪边界

裁剪版通过 `src/ui/gui/features/disabled/` 下的空实现接管被裁剪功能，不让主窗口直接判断功能档位。PyInstaller 脚本同时排除以下模块，避免只是隐藏菜单但仍把重功能打进 exe：

- `src.ui.gui.live_monitor`
- `src.ui.gui.offline_plot`
- `src.ui.gui.data_analysis_window`
- `src.data.control_effect_analysis`
- `src.ui.gui.situation3d`
- `PySide6.QtCharts`
- `PySide6.QtQml`
- `PySide6.QtQuick`
- `PySide6.QtQuick3D`

如需新增裁剪项，先补充对应 feature 契约和 `disabled` 实现，再在裁剪版脚本中追加 `--exclude-module`。
