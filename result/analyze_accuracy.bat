@echo off
setlocal
chcp 65001 >nul
set "PYTHONIOENCODING=utf-8"
set "PYTHONUTF8=1"

set "PROJECT_ROOT=%~dp0.."
set "ANALYSIS_ROOT=%~dp0analysis"
set "VENV_PYTHON=%PROJECT_ROOT%\.venv\Scripts\python.exe"
set "SOURCE_PYTHON=%VENV_PYTHON%"
set "SNAPSHOT_FILE=%~f1"

if not exist "%SOURCE_PYTHON%" set "SOURCE_PYTHON=python.exe"
"%SOURCE_PYTHON%" -c "import numpy" >nul 2>&1
if errorlevel 1 (
    echo [失败] 当前 Python 环境缺少 numpy，无法执行控制效果分析。
    pause
    exit /b 1
)

pushd "%PROJECT_ROOT%" >nul
if errorlevel 1 (
    echo [失败] 无法进入项目目录：%PROJECT_ROOT%
    pause
    exit /b 1
)

if "%~1"=="" (
    "%SOURCE_PYTHON%" -m src.data.control_effect_analysis --output-root "%ANALYSIS_ROOT%"
) else (
    "%SOURCE_PYTHON%" -m src.data.control_effect_analysis "%SNAPSHOT_FILE%" --output-root "%ANALYSIS_ROOT%"
)

set "EXIT_CODE=%ERRORLEVEL%"
popd
if not "%EXIT_CODE%"=="0" (
    echo [失败] 控制效果分析退出码：%EXIT_CODE%
    pause
    exit /b %EXIT_CODE%
)

echo [完成] 控制效果指标位于 result\analysis。
pause
exit /b 0
