@echo off
chcp 65001 > nul
cd /d "%~dp0"

echo 当前目录：%CD%
echo.
echo 检查 Python...

if exist "%~dp0.venv\Scripts\python.exe" goto use_project_python

python --version > nul 2>&1
if not errorlevel 1 goto use_python

py -3 --version > nul 2>&1
if not errorlevel 1 goto use_py

echo.
echo 没有找到可用的 Python。
echo 注意：VS Code 里能运行 Python，不代表 Windows 双击批处理时能找到它。
echo 请在 VS Code 终端执行 python --version，或重新安装 Python 并勾选 Add Python to PATH。
goto failed

:use_project_python
set "PYTHON_CMD=%~dp0.venv\Scripts\python.exe"
goto install

:use_py
set "PYTHON_CMD=py -3"
goto install

:use_python
set "PYTHON_CMD=python"
goto install

:install
echo 使用解释器：%PYTHON_CMD%
%PYTHON_CMD% --version
echo.

if exist "%~dp0serial_compat.py" (
    echo 本项目已包含 Windows 内置串口后备模块。
    echo 即使 pyserial 未安装，上位机也可以运行。
)

%PYTHON_CMD% -c "import serial" > nul 2>&1
if not errorlevel 1 (
    echo pyserial 已经安装，无需重复安装。
    goto success
)

if exist "%~dp0pyserial-3.5-py2.py3-none-any.whl" goto install_local_wheel

echo 正在安装电脑端串口依赖...
echo 使用镜像：清华大学 PyPI 镜像
%PYTHON_CMD% -m pip install --user --default-timeout 60 --retries 2 -i "https://pypi.tuna.tsinghua.edu.cn/simple" -r "%~dp0requirements.txt"

if errorlevel 1 (
    echo.
    echo pyserial 在线安装失败，但这不是上位机运行的阻止性错误。
    echo 上位机将使用 serial_compat.py 访问 Windows 串口。
    goto success
)

echo.
echo 安装完成。
goto success

:failed
echo.
echo 脚本执行失败，错误代码：%errorlevel%
pause
exit /b 1

:success
pause
exit /b 0

:install_local_wheel
echo 找到本地 pyserial 安装包，跳过网络下载。
%PYTHON_CMD% -m pip install --user "%~dp0pyserial-3.5-py2.py3-none-any.whl"
if errorlevel 1 goto failed
echo.
echo pyserial 本地安装完成。
goto success
