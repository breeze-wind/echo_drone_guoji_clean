# 没有 PA13、PA14 时：使用串口 Bootloader 烧录

如果开发板没有暴露 `PA13/PA14`，也没有 `SWDIO/SWCLK` 或 `SWD` 接口，可以使用 STM32F103 自带的系统 Bootloader，通过 USART1 烧录。

这只是 ST-Link 的替代烧录方式。上位机控制电磁铁时，仍然使用 PA9、PA10。

## 1. 接线

| USB-TTL | STM32 |
|---|---|
| TXD | PA10 / USART1_RX |
| RXD | PA9 / USART1_TX |
| GND | GND |

另外准备：

| STM32 | 烧录时 |
|---|---|
| BOOT0 | 接 3.3V |
| NRST | 按一下复位，或重新上电 |

USB-TTL 必须是 3.3 V TTL，不能使用传统 RS-232，也不要把 5 V 接到 STM32 IO。

## 2. 生成烧录文件

1. 在 CubeMX 配置 USART1。
2. 用 `main_uart.c` 替换 CubeMX 工程中的 `main.c`。
3. 在 Keil 中打开：

```text
Options for Target -> Output -> Create HEX File
```

4. 编译工程，生成 `.hex` 文件。

## 3. 进入系统 Bootloader

1. 关闭上位机和其他串口软件。
2. `BOOT0` 接 3.3V。
3. 按一下 `NRST`，或重新上电。
4. 此时 STM32 会等待串口烧录，不会运行你的用户程序。

## 4. 使用 STM32CubeProgrammer

1. 打开 STM32CubeProgrammer。
2. 下载接口选择 `UART`。
3. 选择 USB-TTL 对应的 COM 口。
4. 选择 `115200` 波特率。
5. 系统 Bootloader 的通信格式通常选择：

```text
8 data bits
Even parity
1 stop bit
```

注意：这和用户程序运行时的 `115200 8-N-1` 不同。

6. 点击连接。
7. 选择 Keil 生成的 `.hex` 文件。
8. 点击下载并等待完成。

## 5. 退出系统 Bootloader

烧录完成后必须执行：

1. `BOOT0` 从 3.3V 断开。
2. `BOOT0` 接 GND。
3. 按一下复位或重新上电。
4. 再运行 `uart_test.py` 或 `pc_control.py`。
5. 用户程序串口参数使用 `115200 8-N-1`。

如果忘记把 `BOOT0` 接回 GND，STM32 每次上电都会停留在 Bootloader，上位机不会收到 `READY` 和 `OK STATE=...`。
