param(
    [string]$Python = "python",
    [string]$AppName = "编队仿真"
)

$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
Set-Location $ProjectRoot

& $Python -m pip install --upgrade pip
& $Python -m pip install -r requirements-gui.txt

& $Python -m PyInstaller `
    --noconfirm `
    --clean `
    --windowed `
    --onefile `
    --name $AppName `
    --paths $ProjectRoot `
    src/ui/gui/main_window.py

Write-Host "Windows exe: $ProjectRoot\dist\$AppName.exe"
