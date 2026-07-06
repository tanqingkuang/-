param(
    [string]$Python = "python",
    [string]$AppName = "编队仿真",
    [switch]$InstallDependencies
)

$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
Set-Location $ProjectRoot
$DistRoot = Join-Path $ProjectRoot "dist"
$StagingDist = Join-Path $DistRoot ".full-dev"
$StagingAppDir = Join-Path $StagingDist $AppName
$StagingExe = Join-Path $StagingAppDir "$AppName.exe"
$StagingSupportDir = Join-Path $StagingAppDir $AppName
$FinalExe = Join-Path $DistRoot "$AppName.exe"
$FinalSupportDir = Join-Path $DistRoot $AppName
$FullHook = Join-Path $ProjectRoot "scripts\pyinstaller_hooks\set_full_feature_profile.py"

if ($InstallDependencies) {
    & $Python -m pip install -r requirements-gui.txt
}

& $Python -m PyInstaller `
    --noconfirm `
    --windowed `
    --onedir `
    --contents-directory $AppName `
    --distpath $StagingDist `
    --workpath (Join-Path $ProjectRoot "build\full-dev") `
    --name $AppName `
    --paths $ProjectRoot `
    --runtime-hook $FullHook `
    --hidden-import src.ui.gui.features.full.control_monitor `
    --hidden-import src.ui.gui.features.full.data_analysis `
    --hidden-import src.ui.gui.features.full.situation3d `
    --add-data "src/ui/gui/situation3d/qml;src/ui/gui/situation3d/qml" `
    src/ui/gui/main_window.py

New-Item -ItemType Directory -Path $DistRoot -Force | Out-Null
Remove-Item -LiteralPath $FinalExe -Force -ErrorAction SilentlyContinue
Remove-Item -LiteralPath $FinalSupportDir -Recurse -Force -ErrorAction SilentlyContinue
Move-Item -LiteralPath $StagingExe -Destination $FinalExe
Move-Item -LiteralPath $StagingSupportDir -Destination $FinalSupportDir
Remove-Item -LiteralPath $StagingDist -Recurse -Force -ErrorAction SilentlyContinue

Write-Host "Full development Windows exe: $FinalExe"
