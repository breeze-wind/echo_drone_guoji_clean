# CubeMX 配置清单

## 1. 新建工程

1. 打开 STM32CubeMX。
2. 选择芯片：`STM32F103C8Tx`。
3. 新建工程。

## 2. 基础配置

### SYS

`SYS -> Debug` 选择：

```text
Serial Wire
```

这一步很重要，否则后面 Keil 用 ST-Link 下载可能失败。

### RCC

如果你的 STM32F103C8T6 最小系统板上有 8MHz 晶振：

```text
RCC -> High Speed Clock -> Crystal/Ceramic Resonator
```

大多数蓝色 STM32F103C8T6 最小系统板都有 8MHz 外部晶振。

## 3. 时钟配置

进入 `Clock Configuration`：

| 项目 | 设置 |
|---|---|
| HSE | 8 MHz |
| PLL Source | HSE |
| PLL Mul | x9 |
| SYSCLK | 72 MHz |
| APB1 Prescaler | /2 |
| APB2 Prescaler | /1 |

如果你不会调，可以让 CubeMX 自动解决时钟冲突，目标是让 `SYSCLK = 72MHz`。

## 4. GPIO 配置

### 按钮 PA0

把 `PA0-WKUP` 设置为：

```text
GPIO_Input
```

GPIO 参数：

| 参数 | 设置 |
|---|---|
| GPIO Pull-up/Pull-down | Pull-up |
| User Label | KEY |

接法是：按钮一端接 `PA0`，另一端接 `GND`。

按下时 PA0 是低电平，松开时 PA0 是高电平。

### 三路电磁铁控制输出

把下面 3 个引脚设置为 `GPIO_Output`：

| 功能 | 引脚 | User Label | 初始电平 |
|---|---|---|---|
| 电磁铁1控制 | PB0 | MAG1 | Low |
| 电磁铁2控制 | PB1 | MAG2 | Low |
| 电磁铁3控制 | PB10 | MAG3 | Low |

GPIO 参数：

| 参数 | 设置 |
|---|---|
| GPIO output level | Low |
| GPIO mode | Output Push Pull |
| GPIO Pull-up/Pull-down | No pull-up and no pull-down |
| Maximum output speed | Low |

说明：虽然状态 0 最终会让三个电磁铁工作，但初始化电平建议设为 Low，这样上电瞬间更稳。程序启动后会立刻切换到状态 0。

## 5. Project Manager

进入 `Project Manager`：

| 项目 | 设置 |
|---|---|
| Project Name | `Electromagnet_HighLevel` |
| Project Location | 你自己的工程目录 |
| Toolchain / IDE | `MDK-ARM` 或 `MDK-ARM V5` |

然后点击：

```text
GENERATE CODE
```

## 6. 替换 main.c

CubeMX 生成代码后：

1. 打开生成的 Keil 工程。
2. 找到 `Core/Src/main.c`。
3. 用本文件夹里的 `main.c` 替换它。
4. 如果 CubeMX 提示重新生成代码，重新生成后可能覆盖 `main.c`，需要再替换一次，或者只把本代码中 `USER CODE` 区域复制进去。

## 7. 不建议占用的引脚

不要把电磁铁控制脚放在：

| 引脚 | 原因 |
|---|---|
| PA13 | ST-Link SWDIO |
| PA14 | ST-Link SWCLK |
| PB3 / PB4 | 可能与调试/JTAG 功能相关，新手容易踩坑 |

所以本项目推荐使用：

```text
PB0, PB1, PB10
```
