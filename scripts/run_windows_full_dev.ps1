param(
    [string]$Python = "python",
    [switch]$InstallDependencies
)

# 开发调试入口(全量档)：直接跑源码，秒级迭代，不打包。改代码后重跑即可，无需 PyInstaller。
# 打包成 exe 请用 build_windows_full_release.ps1。功能档固定 full，与全量版 exe 行为一致。
$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
Set-Location $ProjectRoot

if ($InstallDependencies) {
    & $Python -m pip install -r requirements-gui.txt
}

$env:SIMU_GUI_FEATURE_PROFILE = "full"
& $Python src/ui/gui/main_window.py @args
