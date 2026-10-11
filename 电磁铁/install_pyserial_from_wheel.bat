@echo off
chcp 65001 > nul
cd /d "%~dp0"

set "WHEEL=%~dp0pyserial-3.5-py2.py3-none-any.whl"

if not exist "%WHEEL%" (
    echo 当前目录没有找到：
    echo pyserial-3.5-py2.py3-none-any.whl
    echo.
    echo 请先从官方 PyPI 下载该文件，并放到本目录：
    echo https://pypi.org/project/pyserial/3.5/#files
    echo.
    pause
    exit /b 1
)

python -m pip install --user "%WHEEL%"
if errorlevel 1 (
    echo.
    echo 本地安装失败。
    pause
    exit /b 1
)

echo.
echo pyserial 安装完成。
python -c "import serial; print('pyserial version:', serial.__version__)"
pause
