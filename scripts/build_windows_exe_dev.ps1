param(
    [string]$Python = "python",
    [string]$AppName = "编队仿真",
    [switch]$InstallDependencies
)

$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
Set-Location $ProjectRoot
$DistRoot = Join-Path $ProjectRoot "dist"
$StagingDist = Join-Path $DistRoot ".dev"
$StagingAppDir = Join-Path $StagingDist $AppName
$StagingExe = Join-Path $StagingAppDir "$AppName.exe"
$StagingSupportDir = Join-Path $StagingAppDir $AppName
$FinalExe = Join-Path $DistRoot "$AppName.exe"
$FinalSupportDir = Join-Path $DistRoot $AppName

if ($InstallDependencies) {
    & $Python -m pip install -r requirements-gui.txt
}

& $Python -m PyInstaller `
    --noconfirm `
    --windowed `
    --onedir `
    --contents-directory $AppName `
    --distpath $StagingDist `
    --workpath (Join-Path $ProjectRoot "build\dev") `
    --name $AppName `
    --paths $ProjectRoot `
    --add-data "src/ui/gui/situation3d/qml;src/ui/gui/situation3d/qml" `
    src/ui/gui/main_window.py

New-Item -ItemType Directory -Path $DistRoot -Force | Out-Null
Remove-Item -LiteralPath $FinalExe -Force -ErrorAction SilentlyContinue
Remove-Item -LiteralPath $FinalSupportDir -Recurse -Force -ErrorAction SilentlyContinue
Move-Item -LiteralPath $StagingExe -Destination $FinalExe
Move-Item -LiteralPath $StagingSupportDir -Destination $FinalSupportDir
Remove-Item -LiteralPath $StagingDist -Recurse -Force -ErrorAction SilentlyContinue

Write-Host "Development Windows exe: $FinalExe"
