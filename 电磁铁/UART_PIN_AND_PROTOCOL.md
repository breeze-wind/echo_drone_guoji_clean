# 串口接线和通信协议

## 1. 推荐串口方案

电脑 USB 不能直接接 STM32 的 PA9、PA10，需要 USB-TTL 模块完成 USB 和串口电平转换。

推荐模块：

- CP2102 3.3 V
- CH340 3.3 V
- FT232 3.3 V

不要使用带正负电压的传统 RS-232 电平转换器直接接 STM32。

## 2. 接线

| USB-TTL | STM32F103C8T6 |
|---|---|
| TXD | PA10 / USART1_RX |
| RXD | PA9 / USART1_TX |
| GND | GND |

口诀：

```text
发送接接收，接收接发送，地线必须接。
```

如果 USB-TTL 标有 `VCC`：

- STM32 已经由 ST-Link 或独立电源供电时，通常只连接 TXD、RXD、GND。
- 只有确认电源电压和供电方式正确时才连接 VCC。
- 绝对不能把 5 V 接到 STM32 的串口 IO。

## 3. 串口参数

```text
Baud Rate: 115200
Data Bits: 8
Parity: None
Stop Bits: 1
Flow Control: None
```

通常简写为：

```text
115200 8-N-1
```

## 4. 命令格式

每一条命令都以 `\r\n` 结尾。

### 设置状态

```text
SET 0\r\n
SET 1\r\n
SET 2\r\n
SET 3\r\n
```

### 查询状态

```text
GET\r\n
```

### 切换状态

```text
NEXT\r\n
```

## 5. 返回格式

上电后单片机发送：

```text
READY\r\n
OK STATE=0 ACTIVE=3\r\n
```

成功设置状态后返回：

```text
OK STATE=2 ACTIVE=2\r\n
```

字段含义：

- `STATE`：单片机状态，范围 0 到 3。
- `ACTIVE`：当前吸合数量，等于 `3 - STATE`。

错误返回：

```text
ERR COMMAND\r\n
ERR RX_OVERFLOW\r\n
ERR LINE_TOO_LONG\r\n
```

## 6. 为什么要返回状态

上位机发送命令后不能只假设成功，必须等待 STM32 返回：

```text
OK STATE=...
```

这样可以确认：

1. 串口线路正常。
2. 单片机收到了命令。
3. 命令格式正确。
4. 单片机已经更新状态。

实体按钮改变状态时，单片机同样会主动发送状态回复，上位机可以同步显示。
