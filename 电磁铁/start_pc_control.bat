@echo off
chcp 65001 > nul
cd /d "%~dp0"

echo 正在启动上位机...
echo 当前目录：%CD%
echo.

if exist "%~dp0.venv\Scripts\python.exe" goto use_project_python

python --version > nul 2>&1
if not errorlevel 1 goto use_python

py -3 --version > nul 2>&1
if not errorlevel 1 goto use_py

echo 没有找到可用的 Python。
goto failed

:use_project_python
set "PYTHON_CMD=%~dp0.venv\Scripts\python.exe"
goto run

:use_py
set "PYTHON_CMD=py -3"
goto run

:use_python
set "PYTHON_CMD=python"
goto run

:run
echo 使用解释器：%PYTHON_CMD%
%PYTHON_CMD% --version
echo.
%PYTHON_CMD% "%~dp0pc_control.py"
if errorlevel 1 goto failed
goto success

:failed
echo.
echo 上位机启动失败，错误代码：%errorlevel%
echo 请保留此窗口中的错误信息。
pause
exit /b 1

:success
pause
exit /b 0
