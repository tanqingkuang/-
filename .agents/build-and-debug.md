# 运行调试与打包

## Windows 本地开发调试（不打包，直接跑源码，秒级迭代）

```powershell
.\scripts\run_windows_full_dev.ps1
.\scripts\run_windows_lite_dev.ps1
```

`run_full_dev` / `run_lite_dev` 设 `SIMU_GUI_FEATURE_PROFILE` 后跑 `src/ui/gui/main_window.py`，改代码重跑即可。首次或依赖变化后可加 `-InstallDependencies`。调参 / 改代码不要去跑打包（PyInstaller 全量收集 Qt，与改动量无关，几十秒）。

## Windows exe 打包（仅发布期）

Windows x64 exe 需要在 Windows x64 环境构建，不要在 macOS 上直接用本机 PyInstaller 伪造产物（PySide6/Qt 需要目标平台的 Python wheel、Qt DLL 和 platform plugin）。

```powershell
.\scripts\build_windows_full_release.ps1
.\scripts\build_windows_lite_release.ps1
```

4 个入口（2 个 `run_*_dev` 跑源码调试 + 2 个 `build_*_release` 打包 exe）的差异见 `docs/10-Windows编译入口说明.md`。

GitHub Actions 打包：

- workflow 文件：`.github/workflows/build-windows-exe.yml`
- 触发方式：推送 `main` 相关文件，或手动 `workflow_dispatch`
- 产物：`formation-sim-windows-x64-full` artifact 内含 `编队仿真.exe`，`formation-sim-windows-x64-lite` artifact 内含 `编队仿真-裁剪版.exe`

## macOS app 包本地调试

`编队仿真.app/` 不提交到 git。若本地需要双击 app 调试，可以重新生成或临时同步 app 内代码快照：

```bash
rm -rf '编队仿真.app/Contents/Resources/appsrc/src'
cp -R src '编队仿真.app/Contents/Resources/appsrc/src'
find '编队仿真.app/Contents/Resources/appsrc/src' -type d -name '__pycache__' -prune -exec rm -rf {} +
```

本地同步后可用 offscreen 方式检查 app 内快照（把 `sys.path` 指向 `编队仿真.app/Contents/Resources/appsrc` 后运行 `scripts/check_gui_offscreen.py` 同款检查）。

## 清理生成物

测试后清理 Python 缓存：

```bash
find src tests '编队仿真.app/Contents/Resources/appsrc/src' -type d -name '__pycache__' -prune -exec rm -rf {} +
```
