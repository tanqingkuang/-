param(
    [string]$Python = "python",
    [string]$AppName = "编队仿真-裁剪版"
)

$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
Set-Location $ProjectRoot
$DevSupportDir = Join-Path (Join-Path $ProjectRoot "dist") $AppName
$LiteHook = Join-Path $ProjectRoot "scripts\pyinstaller_hooks\set_lite_feature_profile.py"

& $Python -m pip install --upgrade pip
& $Python -m pip install -r requirements-gui.txt
Remove-Item -LiteralPath $DevSupportDir -Recurse -Force -ErrorAction SilentlyContinue

& $Python -m PyInstaller `
    --noconfirm `
    --clean `
    --windowed `
    --onefile `
    --name $AppName `
    --paths $ProjectRoot `
    --workpath (Join-Path $ProjectRoot "build\lite-release") `
    --runtime-hook $LiteHook `
    --hidden-import src.ui.gui.features.disabled.control_monitor `
    --hidden-import src.ui.gui.features.disabled.data_analysis `
    --hidden-import src.ui.gui.features.disabled.situation3d `
    --exclude-module src.ui.gui.features.full.control_monitor `
    --exclude-module src.ui.gui.features.full.data_analysis `
    --exclude-module src.ui.gui.features.full.situation3d `
    --exclude-module src.ui.gui.live_monitor `
    --exclude-module src.ui.gui.offline_plot `
    --exclude-module src.ui.gui.data_analysis_window `
    --exclude-module src.data.control_effect_analysis `
    --exclude-module src.ui.gui.situation3d `
    --exclude-module PySide6.QtCharts `
    --exclude-module PySide6.QtQml `
    --exclude-module PySide6.QtQuick `
    --exclude-module PySide6.QtQuick3D `
    src/ui/gui/main_window.py

Write-Host "Lite release Windows exe: $ProjectRoot\dist\$AppName.exe"
