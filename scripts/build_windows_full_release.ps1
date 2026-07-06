param(
    [string]$Python = "python",
    [string]$AppName = "编队仿真"
)

$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
Set-Location $ProjectRoot
$DevSupportDir = Join-Path (Join-Path $ProjectRoot "dist") $AppName
$FullHook = Join-Path $ProjectRoot "scripts\pyinstaller_hooks\set_full_feature_profile.py"

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
    --workpath (Join-Path $ProjectRoot "build\full-release") `
    --runtime-hook $FullHook `
    --hidden-import src.ui.gui.features.full.control_monitor `
    --hidden-import src.ui.gui.features.full.data_analysis `
    --hidden-import src.ui.gui.features.full.situation3d `
    --add-data "src/ui/gui/situation3d/qml;src/ui/gui/situation3d/qml" `
    src/ui/gui/main_window.py

Write-Host "Full release Windows exe: $ProjectRoot\dist\$AppName.exe"
