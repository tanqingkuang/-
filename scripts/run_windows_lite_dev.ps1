param(
    [string]$Python = "python",
    [switch]$InstallDependencies
)

# 开发调试入口(裁剪档)：直接跑源码，秒级迭代，不打包。功能档固定 lite，与裁剪版 exe 的功能门控一致
# (裁剪版对外发布的模块排除只在打包时生效；源码调试下由 SIMU_GUI_FEATURE_PROFILE=lite 控制菜单/功能门控)。
# 打包成 exe 请用 build_windows_lite_release.ps1。
$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
Set-Location $ProjectRoot

if ($InstallDependencies) {
    & $Python -m pip install .
}

$env:SIMU_GUI_FEATURE_PROFILE = "lite"
& $Python src/ui/gui/main_window.py @args
