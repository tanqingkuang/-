param(
    [string]$Python = "python",
    [string]$AppName = "编队仿真"
)

$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
Set-Location $ProjectRoot
$DistDir = Join-Path $ProjectRoot "dist"
$DevSupportDir = Join-Path (Join-Path $ProjectRoot "dist") $AppName
$FullHook = Join-Path $ProjectRoot "scripts\pyinstaller_hooks\set_full_feature_profile.py"
$AppIconPath = Join-Path $ProjectRoot "src\ui\gui\assets\app_icon.ico"
$AircraftModelSourceDir = Join-Path $ProjectRoot "src\ui\gui\situation3d\qml\assets"

& $Python -m pip install --upgrade pip
& $Python -m pip install -r requirements-gui.txt
Remove-Item -LiteralPath $DevSupportDir -Recurse -Force -ErrorAction SilentlyContinue
$PySideRoot = @(& $Python -c "from pathlib import Path; import PySide6; print(Path(PySide6.__file__).resolve().parent)")[-1].Trim()
$AssetImportersDir = Join-Path $PySideRoot "plugins\assetimporters"
$GeometryLoadersDir = Join-Path $PySideRoot "plugins\geometryloaders"
$RenderersDir = Join-Path $PySideRoot "plugins\renderers"

& $Python -m PyInstaller `
    --noconfirm `
    --clean `
    --windowed `
    --onefile `
    --name $AppName `
    --icon $AppIconPath `
    --paths $ProjectRoot `
    --workpath (Join-Path $ProjectRoot "build\full-release") `
    --runtime-hook $FullHook `
    --hidden-import src.ui.gui.features.full.control_monitor `
    --hidden-import src.ui.gui.features.full.data_analysis `
    --hidden-import src.ui.gui.features.full.situation3d `
    --add-data "src/ui/gui/assets/app_icon.png;src/ui/gui/assets" `
    --add-data "src/ui/gui/situation3d/qml/Situation3DView.qml;src/ui/gui/situation3d/qml" `
    --add-data "src/ui/gui/situation3d/qml/assets/terrain_detail_normal.png;src/ui/gui/situation3d/qml/assets" `
    --add-data "src/ui/gui/situation3d/qml/assets/terrain_detail_albedo.png;src/ui/gui/situation3d/qml/assets" `
    --add-data "src/ui/gui/situation3d/qml/assets/terrain_loading_guide.png;src/ui/gui/situation3d/qml/assets" `
    --add-binary "$AssetImportersDir;PySide6/plugins/assetimporters" `
    --add-binary "$GeometryLoadersDir;PySide6/plugins/geometryloaders" `
    --add-binary "$RenderersDir;PySide6/plugins/renderers" `
    src/ui/gui/main_window.py

Get-ChildItem -LiteralPath $AircraftModelSourceDir -Filter *.glb |
    ForEach-Object {
        Copy-Item -LiteralPath $_.FullName -Destination $DistDir -Force
    }

Write-Host "Full release Windows exe: $ProjectRoot\dist\$AppName.exe"
Write-Host "External 3D model files: $ProjectRoot\dist\*.glb"
