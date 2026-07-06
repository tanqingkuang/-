param(
    [string]$Python = "python",
    [string]$AppName = "编队仿真-裁剪版",
    [switch]$InstallDependencies
)

$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
Set-Location $ProjectRoot
$DistRoot = Join-Path $ProjectRoot "dist"
$StagingDist = Join-Path $DistRoot ".lite-dev"
$StagingAppDir = Join-Path $StagingDist $AppName
$StagingExe = Join-Path $StagingAppDir "$AppName.exe"
$StagingSupportDir = Join-Path $StagingAppDir $AppName
$FinalExe = Join-Path $DistRoot "$AppName.exe"
$FinalSupportDir = Join-Path $DistRoot $AppName
$LiteHook = Join-Path $ProjectRoot "scripts\pyinstaller_hooks\set_lite_feature_profile.py"

if ($InstallDependencies) {
    & $Python -m pip install -r requirements-gui.txt
}

& $Python -m PyInstaller `
    --noconfirm `
    --windowed `
    --onedir `
    --contents-directory $AppName `
    --distpath $StagingDist `
    --workpath (Join-Path $ProjectRoot "build\lite-dev") `
    --name $AppName `
    --paths $ProjectRoot `
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

New-Item -ItemType Directory -Path $DistRoot -Force | Out-Null
Remove-Item -LiteralPath $FinalExe -Force -ErrorAction SilentlyContinue
Remove-Item -LiteralPath $FinalSupportDir -Recurse -Force -ErrorAction SilentlyContinue
Move-Item -LiteralPath $StagingExe -Destination $FinalExe
Move-Item -LiteralPath $StagingSupportDir -Destination $FinalSupportDir
Remove-Item -LiteralPath $StagingDist -Recurse -Force -ErrorAction SilentlyContinue

Write-Host "Lite development Windows exe: $FinalExe"
