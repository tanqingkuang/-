@echo off
setlocal
chcp 65001 >nul
set "PYTHONIOENCODING=utf-8"
set "PYTHONUTF8=1"

set "PROJECT_ROOT=%~dp0.."
set "DATA_ROOT=%~dp0simulation_data"
set "VENV_PYTHON=%PROJECT_ROOT%\.venv\Scripts\python.exe"
set "CONFIG_PATH=%PROJECT_ROOT%\configs\rally_demo_5_aircraft.json"
set "PLAYBACK_RATE=10"

if not "%~1"=="" (
    set "CONFIG_PATH=%~f1"
)
if not "%~2"=="" set "PLAYBACK_RATE=%~2"

if not exist "%CONFIG_PATH%" (
    echo [失败] 未找到仿真配置：%CONFIG_PATH%
    pause
    exit /b 1
)

set "SOURCE_PYTHON=%VENV_PYTHON%"
if not exist "%SOURCE_PYTHON%" set "SOURCE_PYTHON=python.exe"
"%SOURCE_PYTHON%" -c "import PySide6" >nul 2>&1
if errorlevel 1 (
    echo [失败] 当前 Python 环境缺少 PySide6，无法运行源码验证模式。
    echo 请在已能直接运行本项目的 Python 环境中执行本脚本。
    pause
    exit /b 1
)
echo [运行] 正在以 %PLAYBACK_RATE%x 无界面运行：%CONFIG_PATH%
if not exist "%DATA_ROOT%" mkdir "%DATA_ROOT%"
pushd "%DATA_ROOT%"
"%SOURCE_PYTHON%" "%PROJECT_ROOT%\src\main.py" --config "%CONFIG_PATH%" --rate "%PLAYBACK_RATE%"
set "EXIT_CODE=%ERRORLEVEL%"
popd

if not "%EXIT_CODE%"=="0" (
    echo [失败] 仿真程序退出码：%EXIT_CODE%
    pause
    exit /b %EXIT_CODE%
)

echo [完成] 无界面仿真已结束，数据位于 result\simulation_data\logs。
pause
exit /b 0
