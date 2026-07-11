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

也可使用与 Windows 对应的全量版、裁剪版开发启动脚本；首次或依赖变化后追加
`--install-dependencies`。发布构建会在当前 macOS 架构以标准 app 目录结构生成 `.app`：

```bash
./scripts/run_macos_full_dev.sh
./scripts/run_macos_lite_dev.sh
./scripts/build_macos_full_release.sh
./scripts/build_macos_lite_release.sh
```

开发启动脚本支持 `--python <解释器路径>`，并将其余参数透传给主程序；发布构建脚本
还支持 `--app-name <名称>` 覆盖应用名称。

启动后默认处于 `UNLOADED` 状态，不会自动创建飞机。可以在左侧“选择文件”中加载示例配置：

```text
configs/base.json
```

该配置仅作为默认演示场景入口，具体节点、航线、仿真时长和步长等参数会随验证需求调整，以文件内容为准。
示例配置会通过 `route_file`、`avoidance.obstacles_file` 和 `formation.formation_files` 引用 `configs/element/` 下的外部文件，移动配置时需要保持这些相对路径有效。

## Windows x64 打包

Windows exe 需要在 Windows x64 环境构建。PySide6/Qt 依赖目标平台的 Python wheel、Qt DLL 和 platform plugin，不建议在 macOS 上直接交叉打包。

仓库提供全量版和裁剪版两组入口，差异说明见 [Windows 编译入口说明](docs/10-Windows编译入口说明.md)。

开发调试（不打包，直接跑源码，秒级迭代）：

```powershell
.\scripts\run_windows_full_dev.ps1
.\scripts\run_windows_lite_dev.ps1
```

`run_full_dev` 走 `full` 档、`run_lite_dev` 走 `lite` 档，改代码后重跑即可，无需打包。首次或依赖变化后可加 `-InstallDependencies` 安装 `requirements-gui.txt`；其余参数透传给主程序。

发布期打包 exe：

```powershell
.\scripts\build_windows_full_release.ps1
.\scripts\build_windows_lite_release.ps1
```

发布期产物路径：

```text
dist\编队仿真.exe
dist\编队仿真-裁剪版.exe
```

仓库也提供 GitHub Actions workflow：

```text
.github/workflows/build-windows-exe.yml
```

推送 `main` 或手动触发 workflow 后，会生成 `formation-sim-windows-x64-full` 和 `formation-sim-windows-x64-lite` artifact。

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
