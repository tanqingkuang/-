param(
    [string]$Python = "python",
    [string]$AppName = "编队仿真"
)

$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
Set-Location $ProjectRoot
$DevSupportDir = Join-Path (Join-Path $ProjectRoot "dist") $AppName

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
    --add-data "src/ui/gui/situation3d/qml;src/ui/gui/situation3d/qml" `
    src/ui/gui/main_window.py

Write-Host "Release Windows exe: $ProjectRoot\dist\$AppName.exe"
