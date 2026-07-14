# 编队仿真

本工程是面向固定翼 / 无人机编队的仿真与评测平台。当前代码已经形成第一阶段端到端闭环：从配置加载、模型迭代、航线与编队算法、通信链路和扰动注入，到 PySide6 可视化、运行日志与自动化 ST，均可在同一套工程中实际运行和复现。

项目当前定位是“第一阶段可运行评测平台”，不再只是工程骨架或 GUI 原型；但它仍以算法联调、场景演示和回归验证为主，不等同于高保真飞行动力学平台或实装飞控系统。

正式 GUI 技术栈为 PySide6。`docs/demo.html` 只用于布局、交互和视觉风格验证，不是正式运行时入口，也不引入 Web 前端依赖。

## 当前实现状态

| 领域 | 当前已实现能力 |
| --- | --- |
| 配置与数据 | JSON 总配置；航线、障碍物、队形外部文件；经纬高输入到 ENU 的转换；配置相对路径解析。 |
| 仿真控制 | 配置加载、开始 / 暂停 / 单步 / 重置、播放倍率、运行时队形切换、快照订阅和事件查询。 |
| 飞机模型 | ENU 坐标系下的非线性三自由度质点模型；加速度输入滤波；速度、过载、滚转角和爬升 / 下降率限幅。 |
| 航线与编队 | 基础航线跟踪、长机 / 僚机编队保持、五机分散集结、队形压缩与运行时热切换。 |
| 通信与扰动 | 有向 / 双向链路、延迟与丢包；风场脉冲、节点故障、链路丢包和扰动清除。 |
| 避障 | 基于栅格 A* 与工程几何约束的离线航线规划；圆形和多边形障碍；安全间距、转弯半径、圆弧、预览、采用和航线导出。 |
| GUI | 俯视图、侧视图、轨迹与状态表；主题切换、视图缩放 / 平移 / 自动居中 / 全屏；日志与运行控制。全量版另含实时监控、离线分析、控制效果分析和 3D 态势。 |
| 记录与回归 | 每次运行落盘配置、`snapshots.jsonl` 和 `events.jsonl`；4 个自动化 ST 场景覆盖直线、编队、避障和十机规模，并维护指标与紧凑轨迹基线。 |

## 当前边界

- 避障属于运行前或暂停态下的离线规划。采用规划航线会重置到 `READY`，当前没有飞行过程中的在线重规划、动态障碍规避或自主恢复。
- 节点故障和链路丢包可用于观察退化过程，但当前没有完整的故障检测、隔离、队形重构和任务恢复闭环。
- 当前模型是三自由度质点模型，不包含六自由度气动、传感器误差、执行机构细节、真实飞控软件或硬件在环接口。
- 编队能力聚焦基础领航跟随、保持、集结和队形切换，尚未覆盖多长机协同、复杂任务编排和在线任务重规划。
- 通用 CLI / headless 产品入口尚未完成；`src/main.py` 仍是占位入口。日常演示使用 PySide6 GUI，自动化 ST 通过 `scripts/run_st.py` 无界面运行。
- 自动化 ST 主要验证仿真数值、任务结果和轨迹回归，不覆盖 GUI / 3D 渲染效果、性能基准或跨平台逐位一致性。

## 目录结构

```text
configs/                  可运行场景及航线、障碍物、队形元素
docs/                     系统 / 模块设计文档、UI demo 和说明资源
scripts/                  本地运行、自动化 ST 与跨平台构建脚本
src/algorithm/            航线、编队和避障算法
src/data/                 配置、坐标转换、日志与分析
src/environment/          三自由度模型、通信和扰动环境
src/runner/               仿真控制器、状态机和运行循环
src/ui/gui/               正式 PySide6 GUI
tests/llt/                模块与交互级自动化测试
tests/st/                 端到端场景、检查器和回归基线
```

`编队仿真.app/`、`build/`、`dist/` 和 `logs/` 都是本地生成物，不纳入版本控制。

## 快速开始

CI 当前以 Python 3.12 为基线。项目元数据、运行依赖和工具依赖统一维护在 `pyproject.toml`。先安装 GUI 运行依赖：

```bash
python -m pip install .
```

测试、静态检查和发布构建依赖分别按需安装：

```bash
python -m pip install ".[test]"
python -m pip install ".[lint]"
python -m pip install ".[build]"
```

`requirements-gui.txt` 仅为旧安装命令保留兼容入口，等价于安装项目及 `build` 依赖；新增或升级依赖时只修改 `pyproject.toml`。

在 Windows 上推荐使用开发启动脚本，直接运行源码，不执行耗时的 PyInstaller 打包：

```powershell
.\scripts\run_windows_full_dev.ps1
```

首次启动或依赖变化后也可以让脚本安装依赖：

```powershell
.\scripts\run_windows_full_dev.ps1 -InstallDependencies
```

macOS 对应入口为：

```bash
./scripts/run_macos_full_dev.sh --install-dependencies
```

也可以直接启动默认的全量版 GUI：

```bash
python src/ui/gui/main_window.py
```

不要使用 `python src/main.py` 启动；该通用入口尚未实现。

## 演示操作

1. 启动 GUI。首次启动且没有可恢复配置时，窗口处于 `UNLOADED` 状态，不会自动创建飞机。
2. 点击左侧“选择文件”，加载 `configs/` 下的配置；后续启动时，GUI 会尝试恢复上次成功加载的配置。
3. 如需演示避障，在开始仿真前打开“避障规划”，勾选障碍、生成并检查预览航线，然后点击“采用”。
4. 使用“开始 / 暂停 / 单步 / 重置”和倍率滑条控制运行；有多套队形的配置可通过“场景 / 队形”下拉框运行时热切换。
5. 使用“风场脉冲 / 节点故障 / 链路丢包 / 清除扰动”观察控制回报、节点表和链路表变化。
6. 俯视图和侧视图支持缩放、平移、框选放大、自动居中和重置视图；全量版可从菜单打开实时监控、离线分析、控制效果分析和 3D 态势。
7. 每次实际运行的数据保存在 `logs/run-*`：`config.json` 是展开后的配置快照，`snapshots.jsonl` 是 10 Hz 关键状态，`events.jsonl` 是事件记录。

推荐演示配置：

| 配置 | 用途 |
| --- | --- |
| `configs/base.json` | 三机基础航线跟踪与编队保持。 |
| `configs/rally_demo_5_aircraft.json` | 五机分散集结并进入任务航线。 |
| `configs/change.json` | 五机队形运行时热切换。 |
| `configs/single_avoidance_80km.json` | 单机 80 km 级离线避障规划与航线导出。 |
| `configs/quadrilateral_10_aircraft_a05_leader.json` | 十机规模和非首机长机验证。 |
| `configs/mountain_demo.json` | 山地场景、五机集结、障碍风险区和 3D 态势综合演示。 |

配置字段和外部文件关系见 [配置说明](configs/README.md)。移动配置文件时，需要保持 `route_file`、`avoidance.obstacles_file`、`terrain_display_file` 和 `formation.formation_files` 等相对路径有效。

## 全量版与裁剪版

| 档位 | 保留能力 |
| --- | --- |
| 全量版 `full` | 主仿真、避障规划、控制监控、数据分析、3D 态势、帮助、主题和日志。 |
| 裁剪版 `lite` | 主仿真、避障规划、帮助、主题和日志；不提供控制监控、数据分析和 3D 态势。 |

Windows 本地开发入口：

```powershell
.\scripts\run_windows_full_dev.ps1
.\scripts\run_windows_lite_dev.ps1
```

macOS 本地开发入口：

```bash
./scripts/run_macos_full_dev.sh
./scripts/run_macos_lite_dev.sh
```

功能档位和构建边界详见 [Windows 运行与编译入口说明](docs/10-Windows编译入口说明.md)。

## Windows x64 打包

Windows exe 必须在 Windows x64 环境构建。PySide6 / Qt 依赖目标平台的 Python wheel、Qt DLL 和 platform plugin，不应在 macOS 上伪造 Windows 产物。

发布期构建：

```powershell
.\scripts\build_windows_full_release.ps1
.\scripts\build_windows_lite_release.ps1
```

产物路径：

```text
dist\编队仿真.exe
dist\编队仿真-裁剪版.exe
```

仓库也提供 `.github/workflows/build-windows-exe.yml`。推送 `main` 的相关文件或手动触发 workflow 后，会生成 `formation-sim-windows-x64-full` 和 `formation-sim-windows-x64-lite` artifact。

macOS 发布构建入口为：

```bash
./scripts/build_macos_full_release.sh
./scripts/build_macos_lite_release.sh
```

生成的 `.app` 仅用于本地调试或发布，不提交到仓库。

## 自动化检查

运行 LLT：

```bash
python -m pip install ".[test]"
pytest tests/llt -q
```

运行端到端 ST：

```bash
python scripts/run_st.py
```

全部通过时输出 `ST OK`。ST 场景、三层检查、基线刷新和阈值规则详见 [自动化 ST 设计与用例清单](docs/11-自动化ST设计与用例清单.md)。

其他基础门禁：

```bash
python -m compileall -q src
python -X utf8 scripts/comment_coverage.py \
  --fail-under-module 100 \
  --fail-under-class 100 \
  --fail-under-func 100 \
  --fail-under-inline 15 \
  --worst 12
git diff --check
```

更完整的修改后自测试和 GUI 截图检查要求见 `AGENTS.md`。
