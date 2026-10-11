# 从这里开始

你现在只需要按下面顺序做：

1. 阅读 `UART_COMPLETE_CHECKLIST.md`。
2. 准备 3.3 V USB-TTL 模块。
3. 在 CubeMX 中配置 USART1。
4. 生成 Keil 工程。
5. 用 `main_uart.c` 替换生成的 `main.c`。
6. 编译并用 ST-Link 烧录。
7. 双击 `start_pc_control.bat` 启动上位机。当前版本包含 Windows 内置串口后备模块，不要求先安装 `pyserial`。

如果希望使用 pyserial，也可以安装电脑端依赖：

```text
双击 install_pc_tools.bat
```

8. 插入 USB-TTL，确认 Windows 中的 COM 号。
9. 双击 `run_uart_test.bat`，输入实际 COM 号并完成四个状态测试。
   也可以在命令行运行：

```text
python uart_test.py --port COM5 --sequence
```

把 `COM5` 换成你的实际 COM 号。

10. 测试通过后，双击 `start_pc_control.bat` 打开图形化上位机。
11. 先用万用表检查 PB0、PB1、PB10。
12. 再接 MOSFET 驱动模块。
13. 最后接独立电源和电磁铁。

## VS Code 里的 Python 和双击批处理

VS Code 左下角选择的 Python 解释器只对 VS Code 的运行按钮和集成终端生效。Windows 资源管理器双击 `.bat` 时，使用的是系统环境里的 `python` 命令，不会自动使用 VS Code 当前选择的虚拟环境。

如果双击窗口一闪而过，请先在 VS Code 终端执行：

```text
python --version
python -c "import sys; print(sys.executable)"
python -m pip install --user -r requirements.txt
python pc_control.py
```

这样可以直接看到真正的错误信息。当前目录的批处理已经改成：

- 优先使用系统中可用的 `python`；
- 找不到时才尝试 `py -3`；
- 安装和启动失败时保留窗口，不再直接闪退。

如果安装日志出现 `SSL EOF`、`Could not fetch URL` 或 `No matching distribution found`，通常是官方 PyPI 连接失败。`install_pc_tools.bat` 会优先使用清华大学 PyPI 镜像；也可以手动执行：

```text
python -m pip install --user -i https://pypi.tuna.tsinghua.edu.cn/simple pyserial
```

如果镜像也无法访问：

1. 打开官方 [pyserial 3.5 文件页面](https://pypi.org/project/pyserial/3.5/#files)。
2. 下载 `pyserial-3.5-py2.py3-none-any.whl`。
3. 把文件放到本目录 `D:\dct\电磁铁`。
4. 双击 `install_pyserial_from_wheel.bat`。

也可以在 VS Code 终端执行：

```text
python -m pip install --user "D:\dct\电磁铁\pyserial-3.5-py2.py3-none-any.whl"
```

如果要从 PowerShell 手动运行批处理，使用：

```text
.\install_pc_tools.bat
.\start_pc_control.bat
```

## 文件选择

- 单片机程序：`main_uart.c`，已经同时复制到 `高电平触发项目/main_uart.c`
- 图形化上位机：`pc_control.py`
- 串口自检：`uart_test.py`
- Windows 串口后备模块：`serial_compat.py`
- CubeMX 配置：`CUBEMX_UART_CHECKLIST.md`
- 接线协议：`UART_PIN_AND_PROTOCOL.md`
- 故障排查：`UART_TROUBLESHOOTING.md`
- 完整勾选清单：`UART_COMPLETE_CHECKLIST.md`

## 最关键的三根线

```text
USB-TTL TXD -> STM32 PA10
USB-TTL RXD -> STM32 PA9
USB-TTL GND -> STM32 GND
```

## 最关键的串口参数

```text
115200 8-N-1
```

## 最关键的状态命令

```text
SET 0  # 3 个吸合
SET 1  # 2 个吸合
SET 2  # 1 个吸合
SET 3  # 0 个吸合
```
