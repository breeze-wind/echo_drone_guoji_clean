import queue
import re
import threading
import tkinter as tk
from tkinter import messagebox, ttk

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
STATE_LABELS = {
    1: "状态 1：3 只磁铁吸合",
    2: "状态 2：2 只磁铁吸合",
    3: "状态 3：1 只磁铁吸合",
    4: "状态 4：0 只磁铁吸合",
}


class ElectromagnetControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("三路电磁铁控制")
        self.root.geometry("720x520")
        self.root.minsize(640, 440)

        self.serial_port = None
        self.reader_thread = None
        self.stop_event = threading.Event()
        self.message_queue = queue.Queue()

        self.port_var = tk.StringVar()
        self.connection_var = tk.StringVar(value="未连接")
        self.state_var = tk.StringVar(value="当前状态：未知")
        self.help_var = tk.StringVar(
            value="状态 1 = 3 个吸合，状态 4 = 全部断开"
        )

        self._build_ui()
        self.refresh_ports()
        self.root.after(100, self._poll_messages)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def _build_ui(self):
        connection_frame = ttk.LabelFrame(self.root, text="1. 串口连接")
        connection_frame.pack(fill="x", padx=14, pady=(14, 8))

        ttk.Label(connection_frame, text="COM 口").grid(
            row=0, column=0, padx=(12, 5), pady=12
        )

        self.port_combo = ttk.Combobox(
            connection_frame,
            textvariable=self.port_var,
            state="readonly",
            width=18,
        )
        self.port_combo.grid(row=0, column=1, padx=5, pady=12)

        ttk.Button(
            connection_frame,
            text="刷新串口",
            command=self.refresh_ports,
        ).grid(row=0, column=2, padx=5, pady=12)

        self.connect_button = ttk.Button(
            connection_frame,
            text="连接",
            command=self.toggle_connection,
        )
        self.connect_button.grid(row=0, column=3, padx=5, pady=12)

        ttk.Label(
            connection_frame,
            textvariable=self.connection_var,
        ).grid(row=0, column=4, padx=(14, 12), pady=12, sticky="w")

        state_frame = ttk.LabelFrame(self.root, text="2. 选择目标状态")
        state_frame.pack(fill="x", padx=14, pady=8)

        ttk.Label(
            state_frame,
            textvariable=self.state_var,
            font=("Microsoft YaHei UI", 14),
        ).pack(pady=(14, 4))

        ttk.Label(
            state_frame,
            textvariable=self.help_var,
        ).pack(pady=(0, 12))

        button_frame = ttk.Frame(state_frame)
        button_frame.pack(fill="x", padx=12, pady=(0, 12))

        for column, state in enumerate(range(1, 5)):
            button = ttk.Button(
                button_frame,
                text=STATE_LABELS[state],
                command=lambda value=state: self.set_state(value),
            )
            button.grid(row=0, column=column, padx=3, sticky="ew")
            button_frame.columnconfigure(column, weight=1)

        ttk.Button(
            state_frame,
            text="切换到下一状态",
            command=self.next_state,
        ).pack(pady=(0, 14))

        log_frame = ttk.LabelFrame(self.root, text="3. 通信记录")
        log_frame.pack(fill="both", expand=True, padx=14, pady=(8, 14))

        self.log_text = tk.Text(
            log_frame,
            height=10,
            state="disabled",
            wrap="word",
        )
        self.log_text.pack(fill="both", expand=True, padx=8, pady=8)

    def refresh_ports(self):
        if list_ports is None:
            self.port_combo["values"] = ()
            self.connection_var.set("串口功能不可用")
            return

        ports = [port.device for port in list_ports.comports()]
        self.port_combo["values"] = ports

        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        elif not ports:
            self.port_var.set("")
            self.connection_var.set("没有找到串口")

    def toggle_connection(self):
        if self._is_connected():
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        if serial is None:
            messagebox.showerror(
                "串口功能不可用",
                "没有找到 pyserial，也无法加载 Windows 串口后备模块。\n"
                "请确认 serial_compat.py 和本程序在同一目录。",
            )
            return

        port_name = self.port_var.get().strip()
        if not port_name:
            messagebox.showwarning("提示", "请先选择 COM 口")
            return

        try:
            self.serial_port = serial.Serial(
                port=port_name,
                baudrate=BAUDRATE,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.2,
            )
        except serial.SerialException as exc:
            self.serial_port = None
            messagebox.showerror("串口打开失败", str(exc))
            return

        self.stop_event.clear()
        self.reader_thread = threading.Thread(
            target=self._read_loop,
            name="serial-reader",
            daemon=True,
        )
        self.reader_thread.start()

        self.connection_var.set(f"已连接：{port_name}，115200 8-N-1")
        self.connect_button.configure(text="断开")
        self._append_log(f"打开 {port_name}")

        # 查询当前状态，避免界面显示旧状态。
        self._send_line("GET")

    def disconnect(self, write_log=True):
        self.stop_event.set()
        current_port = self.serial_port
        self.serial_port = None

        if current_port is not None:
            try:
                current_port.close()
            except serial.SerialException:
                pass

        self.reader_thread = None
        self.connect_button.configure(text="连接")
        self.connection_var.set("未连接")

        if write_log:
            self._append_log("串口已断开")

    def set_state(self, state):
        if not self._is_connected():
            messagebox.showwarning("提示", "请先连接单片机")
            return

        # Bench-verified mapping: UI/ROS state is the MCU SET value verbatim.
        self._send_line(f"SET {state}")

    def next_state(self):
        if not self._is_connected():
            messagebox.showwarning("提示", "请先连接单片机")
            return

        self._send_line("NEXT")

    def _is_connected(self):
        return (
            self.serial_port is not None
            and self.serial_port.is_open
        )

    def _send_line(self, command):
        if not self._is_connected():
            return

        try:
            self.serial_port.write((command + "\r\n").encode("ascii"))
            self._append_log(f">> {command}")
        except serial.SerialException as exc:
            self._append_log(f"发送失败：{exc}")
            self.disconnect(write_log=False)

    def _read_loop(self):
        while not self.stop_event.is_set():
            current_port = self.serial_port
            if current_port is None:
                return

            try:
                raw_line = current_port.readline()
            except serial.SerialException as exc:
                if not self.stop_event.is_set():
                    self.message_queue.put(("error", str(exc)))
                return

            if raw_line:
                text = raw_line.decode(
                    "ascii",
                    errors="replace",
                ).strip()
                self.message_queue.put(("line", text))

    def _poll_messages(self):
        try:
            while True:
                message_type, message = self.message_queue.get_nowait()
                if message_type == "line":
                    self._handle_line(message)
                else:
                    self._append_log(f"接收失败：{message}")
                    self.disconnect(write_log=False)
        except queue.Empty:
            pass

        self.root.after(100, self._poll_messages)

    def _handle_line(self, line):
        self._append_log(f"<< {line}")

        match = re.search(
            r"STATE=(\d+)\s+ACTIVE=(\d+)",
            line,
        )
        if match is None:
            return

        firmware_state = int(match.group(1))
        state = firmware_state
        protocol_active = int(match.group(2))
        # There are three connected magnets; ACTIVE counts the firmware's four
        # outputs, one of which is unconnected.
        physical_active = max(0, min(3, protocol_active))
        if protocol_active == 4 - firmware_state:
            counter_note = f"MCU ACTIVE={protocol_active}"
        else:
            counter_note = (f"MCU ACTIVE={protocol_active} 与 STATE={state} 不匹配"
                            f"（四路协议预期 {4 - firmware_state}）")
        self.state_var.set(
            f"当前 MCU 状态：{state}，实际 {physical_active} 个磁铁吸合（{counter_note}）"
        )

    def _append_log(self, message):
        self.log_text.configure(state="normal")
        self.log_text.insert("end", message + "\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def close(self):
        self.disconnect(write_log=False)
        self.root.destroy()


def main():
    root = tk.Tk()
    ElectromagnetControlApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
