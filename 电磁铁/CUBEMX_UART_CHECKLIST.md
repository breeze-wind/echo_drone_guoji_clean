# CubeMX、Keil 配置清单

## 1. 芯片

选择：

```text
STM32F103C8Tx
```

## 2. SYS

```text
SYS -> Debug -> Serial Wire
```

不要选择完整 JTAG，否则可能占用 PB3、PB4 等调试相关引脚。

## 3. 时钟

如果你的最小系统板有 8 MHz 外部晶振：

```text
RCC -> High Speed Clock -> Crystal/Ceramic Resonator
```

时钟目标：

| 项目 | 设置 |
|---|---:|
| HSE | 8 MHz |
| PLL Source | HSE |
| PLL Mul | x9 |
| SYSCLK | 72 MHz |
| APB1 | /2 |
| APB2 | /1 |

如果你的板子不是 8 MHz 晶振，保留 CubeMX 为该板生成的 `SystemClock_Config()`，不要盲目替换时钟函数。

## 4. GPIO

| 引脚 | CubeMX 配置 | 说明 |
|---|---|---|
| PA0 | GPIO_Input + Pull-up | 按钮按下为低电平 |
| PB0 | GPIO_Output | 电磁铁 1，高电平吸合 |
| PB1 | GPIO_Output | 电磁铁 2，高电平吸合 |
| PB10 | GPIO_Output | 电磁铁 3，高电平吸合 |

三个输出建议：

```text
GPIO mode: Output Push Pull
GPIO Pull-up/Pull-down: No pull-up and no pull-down
Maximum output speed: Low
Initial output level: Low
```

## 5. USART1

| 引脚 | 功能 |
|---|---|
| PA9 | USART1_TX |
| PA10 | USART1_RX |

USART1 参数：

```text
Mode: Asynchronous
Baud Rate: 115200
Word Length: 8 Bits
Parity: None
Stop Bits: 1
Hardware Flow Control: None
Oversampling: 16 Samples
```

在 `NVIC Settings` 中勾选：

```text
USART1 global interrupt
```

## 6. 生成代码后的检查

点击 `Generate Code` 后确认：

- [ ] 有 `usart.c`
- [ ] 有 `usart.h`
- [ ] `main.c` 可以包含 `usart.h`
- [ ] 工程中有 `MX_USART1_UART_Init()`
- [ ] 工程中有 `USART1_IRQHandler()`
- [ ] 工程中有 `HAL_UART_IRQHandler(&huart1)`

## 7. 替换 main.c

使用本目录根目录下的：

```text
main_uart.c
```

替换 CubeMX 工程的：

```text
Core/Src/main.c
```

不要替换以下自动生成文件：

```text
Core/Src/usart.c
Core/Inc/usart.h
Core/Src/gpio.c
Core/Inc/gpio.h
Core/Src/stm32f1xx_it.c
```

## 8. 编译前的检查

- [ ] `main_uart.c` 中 `#include "usart.h"` 存在
- [ ] 程序中调用了 `MX_USART1_UART_Init()`
- [ ] 程序中使用的是 `huart1`
- [ ] USART1 中断已经打开
- [ ] Keil 工程中只有一个 `main.c`
- [ ] `APP_MAGNET_WORK_LEVEL` 为高电平
- [ ] 电磁铁控制引脚与实际接线一致

## 9. Keil 下载

ST-Link 接线：

| ST-Link | STM32 |
|---|---|
| 3.3V | 3.3V |
| GND | GND |
| SWDIO | PA13 |
| SWCLK | PA14 |

如果开发板没有标出 `PA13`、`PA14`，请寻找 `SWDIO/SWCLK`、`DIO/CLK` 或 `SWD` 调试接口。不要把 ST-Link 的 SWD 线接到 PA9、PA10。

如果板子完全没有 SWD 接口，可以改用 USART1 Bootloader 下载。此时使用：

```text
USB-TTL TXD -> PA10
USB-TTL RXD -> PA9
USB-TTL GND -> GND
BOOT0 -> 3.3V 后复位
```

下载完成后必须：

```text
BOOT0 -> GND
复位或重新上电
```

Keil：

```text
Options for Target
  -> Debug
  -> ST-Link Debugger
  -> Settings
  -> Port: SW
```
