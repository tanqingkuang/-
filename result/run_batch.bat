@echo off
setlocal EnableExtensions
chcp 65001 >nul
set "PYTHONIOENCODING=utf-8"
set "PYTHONUTF8=1"

set "PROJECT_ROOT=%~dp0.."
set "DATA_ROOT=%~dp0simulation_data"
set "VENV_PYTHON=%PROJECT_ROOT%\.venv\Scripts\python.exe"
set "CONFIG_PATH=%PROJECT_ROOT%\configs\rally_demo_5_aircraft.json"
set "PLAYBACK_RATE=20"
set "SEEDS=0 1 2 3 4"

if not "%~1"=="" (
    set "CONFIG_PATH=%~f1"
)
if not "%~2"=="" set "PLAYBACK_RATE=%~2"
if not "%~3"=="" set "SEEDS=%~3"

if not exist "%CONFIG_PATH%" (
    echo [失败] 未找到仿真配置：%CONFIG_PATH%
    pause
    exit /b 1
)

if not exist "%DATA_ROOT%" mkdir "%DATA_ROOT%"
if errorlevel 1 (
    echo [失败] 无法创建数据目录：%DATA_ROOT%
    pause
    exit /b 1
)

set "SOURCE_PYTHON=%VENV_PYTHON%"
if not exist "%SOURCE_PYTHON%" set "SOURCE_PYTHON=python.exe"

echo [批量] 配置：%CONFIG_PATH%
echo [批量] 倍率：%PLAYBACK_RATE%x
echo [批量] seeds：%SEEDS%

for %%S in (%SEEDS%) do (
    echo [启动] seed=%%S
    start "seed-%%S" /min /d "%DATA_ROOT%" "%SOURCE_PYTHON%" "%PROJECT_ROOT%\src\main.py" --config "%CONFIG_PATH%" --rate "%PLAYBACK_RATE%" --seed "%%S"
    if errorlevel 1 (
        echo [失败] 无法启动 seed=%%S
        pause
        exit /b 1
    )
)

echo [已启动] 全部 seed 进程均已发起，结果将写入 result\simulation_data\logs\run-seed-*。
echo [提示] 本脚本不等待子进程结束；请分别查看各进程输出和对应日志。
pause
exit /b 0
