# 编队仿真

本工程用于搭建固定翼/无人机编队仿真平台原型，目标是把仿真控制、模型迭代、协同算法、节点算法、通信链路、扰动注入、GUI/CLI 等模块逐步落地为可运行系统。

当前阶段重点是工程骨架、HLD 文档和 PySide6 桌面 GUI 原型。`docs/demo.html` 仅用于界面布局和交互验证，正式 GUI 技术栈为 PySide6。

## 目录结构

```text
configs/                 配置文件目录
docs/                    系统和模块 HLD、UI demo、截图资源
scripts/                 构建脚本
src/                     正式源码
src/ui/gui/main_window.py PySide6 GUI 主窗体
tests/                   LLT/ST 测试目录
编队仿真.app              macOS 双击运行 app 包
```

## 运行 GUI

先安装 GUI 依赖：

```bash
python -m pip install -r requirements-gui.txt
```

直接运行 PySide6 GUI：

```bash
python src/ui/gui/main_window.py
```

macOS 也可以直接双击：

```text
编队仿真.app
```

启动后默认处于 `UNLOADED` 状态，不会自动创建飞机。可以在左侧“选择文件”中加载示例配置：

```text
configs/base.json
```

该配置用于当前三机楔形场景，包含 A01/A02/A03 三个节点、三条通信链路、`120s` 仿真时长和 `0.005s` 仿真步长。位置和加速度采用东北天坐标系：`x_m` 向东、`y_m` 向北、`altitude_m` 向上。飞机模型采用三自由度质点模型：算法输出东北天三轴加速度层指令，模型接收后先经过欠阻尼二阶内回路，再转换为 `N_x / N_z / phi` 输入。

## Windows x64 打包

Windows exe 需要在 Windows x64 环境构建。PySide6/Qt 依赖目标平台的 Python wheel、Qt DLL 和 platform plugin，不建议在 macOS 上直接交叉打包。

开发期快速打包：

```powershell
.\scripts\build_windows_exe_dev.ps1
```

首次打包或依赖变化后可加 `-InstallDependencies`。

产物路径：

```text
dist\编队仿真.exe
```

开发期打包仍会在 `dist\编队仿真\` 放置运行依赖文件，请和 exe 保持在同一目录层级。

发布期完整打包：

```powershell
.\scripts\build_windows_exe_release.ps1
```

产物路径：

```text
dist\编队仿真.exe
```

仓库也提供 GitHub Actions workflow：

```text
.github/workflows/build-windows-exe.yml
```

推送 `main` 或手动触发 workflow 后，会生成 `formation-sim-windows-x64` artifact。

## 基本检查

Python 语法检查：

```bash
python -m compileall -q src
```

提交前检查空白问题：

```bash
git diff --check
```

修改正式 GUI 后，需要同步 macOS app 内代码快照：

```bash
rm -rf '编队仿真.app/Contents/Resources/appsrc/src'
cp -R src '编队仿真.app/Contents/Resources/appsrc/src'
find '编队仿真.app/Contents/Resources/appsrc/src' -type d -name '__pycache__' -prune -exec rm -rf {} +
```

更多自测试要求见 `AGENTS.md`。
