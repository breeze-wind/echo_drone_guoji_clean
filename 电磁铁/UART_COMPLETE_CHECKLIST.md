# STM32 三路电磁铁上位机控制：从零开始完整流程

这份清单按实际执行顺序编写。第一次做串口通信时，不要跳步，也不要一开始就接电磁铁。

## 一、最终要实现的功能

电脑上的上位机通过 USB-TTL 串口模块向 STM32 发送命令，STM32 根据目标状态控制三路电磁铁。

| 单片机状态 | 吸合数量 | 电磁铁 1 | 电磁铁 2 | 电磁铁 3 |
|---:|---:|---|---|---|
| 0 | 3 | 吸合 | 吸合 | 吸合 |
| 1 | 2 | 断开 | 吸合 | 吸合 |
| 2 | 1 | 断开 | 断开 | 吸合 |
| 3 | 0 | 断开 | 断开 | 断开 |

上位机只发送状态编号，不直接操作 PB0、PB1、PB10。

## 二、本目录已经准备好的文件

| 文件 | 用途 |
|---|---|
| `main_uart.c` | 加入串口通信后的单片机完整主程序 |
| `pc_control.py` | Windows 图形化上位机 |
| `uart_test.py` | 不打开图形界面时使用的串口自检程序 |
| `requirements.txt` | Python 串口库依赖 |
| `install_pc_tools.bat` | 双击安装电脑端依赖 |
| `start_pc_control.bat` | 双击启动图形化上位机 |
| `run_uart_test.bat` | 双击列出可用串口 |
| `CUBEMX_UART_CHECKLIST.md` | CubeMX 和 Keil 配置细节 |
| `UART_PIN_AND_PROTOCOL.md` | 接线和通信协议 |
| `UART_TROUBLESHOOTING.md` | 常见故障排查 |

原来的按钮版 `高电平触发项目/main.c` 保留不动，可以作为备份和回退版本。

## 三、你需要准备的硬件

- [ ] STM32F103C8T6 最小系统板
- [ ] ST-Link 下载器
- [ ] 3.3 V TTL 电平 USB-TTL 模块，例如 CP2102、CH340、FT232
- [ ] USB 数据线
- [ ] 三路 MOSFET 电磁铁驱动模块
- [ ] 三个电磁铁
- [ ] 与电磁铁额定电压匹配的独立电源
- [ ] 按钮
- [ ] 杜邦线
- [ ] 万用表
- [ ] 如果 MOSFET 模块没有续流保护，每个电磁铁准备一个续流二极管

### 必须确认的硬件事项

- [ ] USB-TTL 模块是 TTL，不是 RS-232。
- [ ] USB-TTL 的逻辑电平是 3.3 V，不能把 5 V TXD 直接接到 STM32。
- [ ] STM32、USB-TTL、MOSFET 模块的 GND 可以共地。
- [ ] 电磁铁使用独立电源，不从 STM32 的 3.3 V 或 5 V 引脚供电。
- [ ] 电磁铁电源电流足够。
- [ ] 电磁铁线圈有续流二极管或驱动模块自带续流保护。

## 四、第一阶段：先保存原工程

1. [ ] 把当前能正常工作的按钮版工程完整复制一份。
2. [ ] 将备份命名为类似 `Electromagnet_Button_Backup`。
3. [ ] 本次串口版本只在复制后的工程上操作。
4. [ ] 不要删除原来的 `高电平触发项目/main.c`。

串口版本的程序上电后默认从状态 0 开始，即三个电磁铁全部吸合。状态不会自动保存到掉电前的值。

## 五、第二阶段：接 USB-TTL 和 STM32

先不要接三个电磁铁，只连接 STM32 和 USB-TTL。

| USB-TTL 模块 | STM32F103C8T6 |
|---|---|
| TXD | PA10，USART1_RX |
| RXD | PA9，USART1_TX |
| GND | GND |

检查以下事项：

- [ ] TXD 接 RX，RXD 接 TX，不能同名相接。
- [ ] GND 已连接。
- [ ] 没有把 USB-TTL 的 5 V 接到 PA9 或 PA10。
- [ ] 如果 STM32 由 ST-Link 或独立电源供电，不要随意把 USB-TTL 的 VCC 并接进去。
- [ ] STM32 的 PA9、PA10 不再配置为普通 GPIO。

## 六、第三阶段：用 CubeMX 配置串口

打开 STM32CubeMX，建议在原项目复制件上配置。

### 芯片和时钟

- [ ] 芯片选择 `STM32F103C8Tx`
- [ ] `SYS -> Debug` 选择 `Serial Wire`
- [ ] 外部高速时钟使用板上的 8 MHz 晶振
- [ ] 系统时钟配置为 72 MHz

### GPIO

- [ ] PA0：`GPIO_Input`
- [ ] PA0：内部上拉 `Pull-up`
- [ ] PB0：`GPIO_Output`
- [ ] PB1：`GPIO_Output`
- [ ] PB10：`GPIO_Output`
- [ ] 三个电磁铁输出均为推挽输出
- [ ] 三个输出初始电平设为 Low

### USART1

- [ ] PA9：`USART1_TX`
- [ ] PA10：`USART1_RX`
- [ ] `USART1` 模式选择 `Asynchronous`
- [ ] Baud Rate：`115200`
- [ ] Word Length：`8 Bits`
- [ ] Parity：`None`
- [ ] Stop Bits：`1`
- [ ] Hardware Flow Control：`None`
- [ ] 使能 `USART1 global interrupt`

### 生成工程

- [ ] Toolchain 选择你实际使用的 Keil 版本
- [ ] 点击 `Generate Code`
- [ ] 确认生成了 `usart.c` 和 `usart.h`
- [ ] 确认 `main.h` 中能看到 `huart1` 相关声明引用

具体参数也可以查看 `CUBEMX_UART_CHECKLIST.md`。

## 七、第四阶段：替换单片机主程序

1. [ ] 关闭正在编译该工程的 Keil 工程，避免文件占用。
2. [ ] 找到 CubeMX 工程的 `Core/Src/main.c`。
3. [ ] 先把 CubeMX 原始 `main.c` 复制成备份。
4. [ ] 使用本目录的 `main_uart.c` 替换 `Core/Src/main.c`。
5. [ ] 保留 CubeMX 自动生成的 `gpio.c`、`gpio.h`、`usart.c`、`usart.h`。
6. [ ] 不要手动删除 `stm32f1xx_it.c` 中的 USART1 中断函数。
7. [ ] 如果 CubeMX 重新生成代码，可能覆盖 `main.c`，覆盖后需要再次替换。

### 这份程序提供的命令

| 上位机发送 | 单片机行为 |
|---|---|
| `SET 0` | 三个电磁铁吸合 |
| `SET 1` | 两个电磁铁吸合 |
| `SET 2` | 一个电磁铁吸合 |
| `SET 3` | 三个电磁铁全部断开 |
| `GET` | 查询当前状态 |
| `NEXT` | 切换到下一个状态 |

每条命令必须以回车换行结束，即 `\r\n`。本目录的电脑程序会自动添加，不需要手动输入。

## 八、第五阶段：编译和烧录单片机

1. [ ] 用 Keil 打开 `.uvprojx` 工程。
2. [ ] 在 `Options for Target -> Debug` 中选择 `ST-Link Debugger`。
3. [ ] `Settings -> Port` 选择 `SW`。
4. [ ] ST-Link 接线：

| ST-Link | STM32 |
|---|---|
| 3.3V | 3.3V |
| GND | GND |
| SWDIO | PA13 |
| SWCLK | PA14 |

### 找不到 PA13、PA14 时怎么办

不要把 `SWDIO` 和 `SWCLK` 接到 PA9、PA10，也不要随便接到其他 GPIO。

先在开发板上寻找以下任意一种标记：

```text
SWDIO
SWCLK
SWD
DIO
CLK
```

这些标记通常就是：

```text
SWDIO = PA13
SWCLK = PA14
```

有些 STM32F103C8T6 小板没有把 `PA13`、`PA14` 印在排针旁边，而是把它们放在 USB 接口附近的 4 针小接口、焊盘或测试点上。

如果开发板上既没有 `PA13/PA14` 标记，也没有 `SWDIO/SWCLK`、`SWD` 接口：

- [ ] 先暂停接线，不要把 ST-Link 接到未知引脚。
- [ ] 查看开发板背面、USB 接口附近和芯片旁边是否有调试焊盘。
- [ ] 记录开发板的准确型号。
- [ ] 拍一张开发板正面和背面的清晰照片，用于确认接口。

如果确认这块板完全没有 SWD 引脚，可以使用 STM32 的串口 Bootloader 下载程序：

| USB-TTL | STM32 |
|---|---|
| TXD | PA10 / USART1_RX |
| RXD | PA9 / USART1_TX |
| GND | GND |

串口下载需要：

1. `BOOT0` 接 3.3V。
2. 复位或重新上电。
3. 使用 STM32CubeProgrammer 的 UART 模式下载。
4. 下载完成后将 `BOOT0` 接 GND。
5. 再次复位或重新上电，运行用户程序。

串口 Bootloader 下载是 ST-Link 的替代方案，具体步骤见 `UART_BOOTLOADER_PROGRAMMING.md`。上位机运行时仍然使用同一组 PA9、PA10，但此时 `BOOT0` 必须接 GND。

5. [ ] 先只编译，不接电磁铁。
6. [ ] 编译成功后下载程序。
7. [ ] 如果下载失败，先检查 ST-Link 和 `Serial Wire` 配置。

### 常见编译错误

| 错误现象 | 优先检查 |
|---|---|
| 找不到 `usart.h` | CubeMX 没有启用 USART1 或没有重新生成 |
| 找不到 `huart1` | `usart.c/usart.h` 没加入工程 |
| `HAL_UART_Receive_IT` 无效 | HAL 驱动或 USART1 中断配置不完整 |
| 重定义 `SystemClock_Config` | 不要把两个 `main.c` 同时加入 Keil |
| 下载后程序不运行 | 检查时钟配置、供电和复位 |

## 九、第六阶段：安装电脑端软件

电脑端使用 Python 和 `pyserial`。

### 安装 Python

- [ ] Windows 已安装 Python 3
- [ ] 在命令提示符中执行 `python --version`
- [ ] 能显示 Python 版本

如果 `python` 命令不存在，需要重新安装 Python，并在安装界面勾选 `Add Python to PATH`。

### 安装串口库

双击：

```text
install_pc_tools.bat
```

或者在当前目录打开命令提示符执行：

```text
python -m pip install -r requirements.txt
```

- [ ] 安装过程没有报错
- [ ] `pyserial` 安装完成

## 十、第七阶段：确认 Windows 串口号

1. [ ] 插入 USB-TTL 模块。
2. [ ] 打开 Windows“设备管理器”。
3. [ ] 找到“端口（COM 和 LPT）”。
4. [ ] 记录类似 `COM5` 的端口号。
5. [ ] 如果出现黄色感叹号，先安装 USB-TTL 驱动。

运行：

```text
run_uart_test.bat
```

它会先列出当前检测到的串口，然后让你输入 COM 号并自动测试 `SET 0` 到 `SET 3`。

## 十一、第八阶段：先用串口自检程序测试

不要先接电磁铁。使用命令提示符进入本目录，假设串口是 `COM5`。

### 查询状态

```text
python uart_test.py --port COM5
```

预期看到类似：

```text
>> GET
<< OK STATE=0 ACTIVE=3
```

### 依次测试五个状态

```text
python uart_test.py --port COM5 --sequence
```

预期依次收到：

```text
OK STATE=0 ACTIVE=3
OK STATE=1 ACTIVE=2
OK STATE=2 ACTIVE=1
OK STATE=3 ACTIVE=0
```

### 只设置一个状态

```text
python uart_test.py --port COM5 --state 2
```

预期收到：

```text
OK STATE=2 ACTIVE=2
```

### 自检通过标准

- [ ] 能打开 COM 口
- [ ] 能收到 `OK STATE=...`
- [ ] `SET 0` 到 `SET 3` 都能收到正确回复
- [ ] 发送命令时单片机不死机
- [ ] 按住实体按钮时，电脑端仍能发送命令

如果这一阶段不通过，不要接电磁铁，先查看 `UART_TROUBLESHOOTING.md`。

## 十二、第九阶段：使用图形化上位机

双击：

```text
start_pc_control.bat
```

操作顺序：

1. [ ] 在 COM 口下拉框选择 USB-TTL 对应的 COM 口。
2. [ ] 点击“连接”。
3. [ ] 日志中看到 `<< OK STATE=...`。
4. [ ] 点击“3 个吸合”。
5. [ ] 点击“2 个吸合”。
6. [ ] 点击“1 个吸合”。
7. [ ] 点击“0 个吸合”。
9. [ ] 观察状态文字和通信记录。

上位机按钮与命令的对应关系：

| 按钮 | 实际发送 |
|---|---|
| 3 个吸合 | `SET 0` |
| 2 个吸合 | `SET 1` |
| 1 个吸合 | `SET 2` |
| 0 个吸合 | `SET 3` |

## 十三、第十阶段：先测 GPIO，再接驱动模块

使用万用表测三个控制引脚：

| 状态 | PB0 | PB1 | PB10 |
|---:|---|---|---|
| 0 | 高 | 高 | 高 |
| 1 | 低 | 高 | 高 |
| 2 | 低 | 低 | 高 |
| 3 | 低 | 低 | 低 |

- [ ] 状态 0 输出正确
- [ ] 状态 1 输出正确
- [ ] 状态 2 输出正确
- [ ] 状态 3 输出正确
- [ ] 状态 3 输出正确

然后：

- [ ] 接 MOSFET 模块输入，不接电磁铁
- [ ] 确认 STM32 GND 与 MOSFET GND 共地
- [ ] 确认驱动模块指示灯变化正确

## 十四、第十一阶段：最后接电磁铁

只有前面所有测试通过后才执行。

- [ ] 电磁铁使用独立电源
- [ ] 外部电源电压等于电磁铁额定电压
- [ ] 外部电源电流满足三个电磁铁同时吸合的需求
- [ ] STM32 GND、驱动模块 GND、外部电源负极共地
- [ ] 驱动模块输出端没有短路
- [ ] 每个电磁铁都有续流保护
- [ ] 上电时手边有断电开关
- [ ] 先测试单个电磁铁，再测试三个同时吸合

## 十五、完整验收标准

- [ ] 上位机能发现 COM 口
- [ ] 上位机能连接单片机
- [ ] 连接后收到当前状态
- [ ] `SET 0` 到 `SET 3` 全部正确
- [ ] 实体按钮仍然可以切换状态
- [ ] 实体按钮切换后，上位机能收到新状态
- [ ] 上位机与实体按钮交替操作不会死机
- [ ] 三路电磁铁动作与状态表一致
- [ ] 电磁铁释放时 STM32 不重启
- [ ] 串口断开后重新连接仍能查询状态
- [ ] 上电初始化状态符合你的安全要求

## 十六、你必须亲自完成的事项

这些事情需要实际硬件、驱动或 Keil，无法只靠电脑文件代替：

1. [ ] 准备并确认 3.3 V USB-TTL 模块。
2. [ ] 按 TX/RX 交叉方式接线。
3. [ ] 在 CubeMX 配置 USART1。
4. [ ] 生成 Keil 工程。
5. [ ] 用 `main_uart.c` 替换生成的 `main.c`。
6. [ ] 在 Keil 中编译。
7. [ ] 用 ST-Link 烧录。
8. [ ] 在设备管理器确认 COM 号。
9. [ ] 运行 `uart_test.py` 完成无负载通信测试。
10. [ ] 用万用表检查三路 GPIO。
11. [ ] 连接 MOSFET 驱动模块并测试。
12. [ ] 最后连接外部电源和电磁铁。
13. [ ] 完成最终验收表。

除以上硬件、CubeMX、Keil 和现场测试步骤外，本目录中的程序、协议、上位机和文档已经准备好。
