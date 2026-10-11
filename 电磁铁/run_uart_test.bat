@echo off
chcp 65001 > nul
cd /d "%~dp0"

if exist "%~dp0.venv\Scripts\python.exe" goto use_project_python

echo 当前检测到的串口：
python uart_test.py
if errorlevel 1 py -3 uart_test.py
echo.
set /p PORT=请输入要测试的 COM 号，例如 COM5：
if "%PORT%"=="" (
    echo 未输入 COM 号，测试结束。
    pause
    exit /b 0
)
echo.
python uart_test.py --port %PORT% --sequence
if errorlevel 1 py -3 uart_test.py --port %PORT% --sequence
pause

:use_project_python
set "PYTHON_CMD=%~dp0.venv\Scripts\python.exe"
echo 使用解释器：%PYTHON_CMD%
%PYTHON_CMD% --version
echo 当前检测到的串口：
%PYTHON_CMD% "%~dp0uart_test.py"
echo.
set /p PORT=请输入要测试的 COM 号，例如 COM5：
if "%PORT%"=="" (
    echo 未输入 COM 号，测试结束。
    pause
    exit /b 0
)
echo.
%PYTHON_CMD% "%~dp0uart_test.py" --port %PORT% --sequence
pause
