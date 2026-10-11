import argparse
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    try:
        from serial_compat import list_ports, serial
    except ImportError:
        serial = None
        list_ports = None


BAUDRATE = 115200


def print_ports():
    if list_ports is None:
        print("未安装 pyserial，请先运行 install_pc_tools.bat")
        return

    ports = list(list_ports.comports())
    if not ports:
        print("没有找到串口。请先连接 USB-TTL 模块并安装驱动。")
        return

    print("当前串口：")
    for port in ports:
        description = port.description or "无描述"
        print(f"  {port.device}: {description}")


def read_response(ser, timeout=2.0):
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        line = ser.readline()
        if line:
            return line.decode("ascii", errors="replace").strip()

    return ""


def send_command(ser, command):
    print(f">> {command}")
    ser.write((command + "\r\n").encode("ascii"))
    response = read_response(ser)

    if response:
        print(f"<< {response}")
    else:
        print("<< 超时：没有收到单片机回复")

    return response


def main():
    parser = argparse.ArgumentParser(
        description="STM32 电磁铁 USART1 串口自检工具"
    )
    parser.add_argument(
        "--port",
        help="串口名称，例如 COM5；不填写时只列出串口",
    )
    parser.add_argument(
        "--state",
        type=int,
        choices=range(4),
        help="发送一个目标状态，范围 0~3",
    )
    parser.add_argument(
        "--sequence",
        action="store_true",
        help="依次测试 SET 0 到 SET 3",
    )
    args = parser.parse_args()

    if serial is None:
        print("未安装 pyserial。请先运行：")
        print("  install_pc_tools.bat")
        return 1

    if not args.port:
        print_ports()
        print("\n示例：python uart_test.py --port COM5 --sequence")
        return 0

    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=BAUDRATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.2,
        )
    except serial.SerialException as exc:
        print(f"打开 {args.port} 失败：{exc}")
        return 1

    try:
        time.sleep(0.3)

        # 清理上电时发送的 READY 和初始状态，随后主动查询。
        ser.reset_input_buffer()
        response = send_command(ser, "GET")
        if not response.startswith("OK STATE="):
            print("警告：GET 没有收到预期的 OK STATE 回复。")

        if args.sequence:
            for state in range(4):
                send_command(ser, f"SET {state}")
                time.sleep(0.3)
        elif args.state is not None:
            send_command(ser, f"SET {args.state}")
        else:
            print("未指定 --state 或 --sequence，本次只完成 GET 查询。")
    finally:
        ser.close()

    print("串口测试结束。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
