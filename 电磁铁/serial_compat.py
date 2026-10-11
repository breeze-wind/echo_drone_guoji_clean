"""Small Windows serial-port fallback used when pyserial is unavailable.

This module intentionally implements only the subset used by this project:
115200 8-N-1, write(), readline(), reset_input_buffer(), close(), and
listing COM ports from the Windows serial registry.
"""

import ctypes
import os
import re
import time
import winreg
from ctypes import wintypes


if os.name != "nt":
    raise ImportError("The Windows serial fallback is only available on Windows.")


kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)

GENERIC_READ = 0x80000000
GENERIC_WRITE = 0x40000000
OPEN_EXISTING = 3
INVALID_HANDLE_VALUE = ctypes.c_void_p(-1).value

PURGE_RXCLEAR = 0x0008


class SerialException(Exception):
    pass


class DCB(ctypes.Structure):
    _fields_ = [
        ("DCBlength", wintypes.DWORD),
        ("BaudRate", wintypes.DWORD),
        ("Flags", wintypes.DWORD),
        ("wReserved", wintypes.WORD),
        ("XonLim", wintypes.WORD),
        ("XoffLim", wintypes.WORD),
        ("ByteSize", wintypes.BYTE),
        ("Parity", wintypes.BYTE),
        ("StopBits", wintypes.BYTE),
        ("XonChar", ctypes.c_char),
        ("XoffChar", ctypes.c_char),
        ("ErrorChar", ctypes.c_char),
        ("EofChar", ctypes.c_char),
        ("EvtChar", ctypes.c_char),
        ("wReserved1", wintypes.WORD),
    ]


class COMMTIMEOUTS(ctypes.Structure):
    _fields_ = [
        ("ReadIntervalTimeout", wintypes.DWORD),
        ("ReadTotalTimeoutMultiplier", wintypes.DWORD),
        ("ReadTotalTimeoutConstant", wintypes.DWORD),
        ("WriteTotalTimeoutMultiplier", wintypes.DWORD),
        ("WriteTotalTimeoutConstant", wintypes.DWORD),
    ]


kernel32.CreateFileW.argtypes = [
    wintypes.LPCWSTR,
    wintypes.DWORD,
    wintypes.DWORD,
    wintypes.LPVOID,
    wintypes.DWORD,
    wintypes.DWORD,
    wintypes.HANDLE,
]
kernel32.CreateFileW.restype = wintypes.HANDLE

kernel32.CloseHandle.argtypes = [wintypes.HANDLE]
kernel32.CloseHandle.restype = wintypes.BOOL

kernel32.GetCommState.argtypes = [
    wintypes.HANDLE,
    ctypes.POINTER(DCB),
]
kernel32.GetCommState.restype = wintypes.BOOL

kernel32.SetCommState.argtypes = [
    wintypes.HANDLE,
    ctypes.POINTER(DCB),
]
kernel32.SetCommState.restype = wintypes.BOOL

kernel32.SetCommTimeouts.argtypes = [
    wintypes.HANDLE,
    ctypes.POINTER(COMMTIMEOUTS),
]
kernel32.SetCommTimeouts.restype = wintypes.BOOL

kernel32.SetupComm.argtypes = [
    wintypes.HANDLE,
    wintypes.DWORD,
    wintypes.DWORD,
]
kernel32.SetupComm.restype = wintypes.BOOL

kernel32.ReadFile.argtypes = [
    wintypes.HANDLE,
    wintypes.LPVOID,
    wintypes.DWORD,
    ctypes.POINTER(wintypes.DWORD),
    wintypes.LPVOID,
]
kernel32.ReadFile.restype = wintypes.BOOL

kernel32.WriteFile.argtypes = [
    wintypes.HANDLE,
    wintypes.LPCVOID,
    wintypes.DWORD,
    ctypes.POINTER(wintypes.DWORD),
    wintypes.LPVOID,
]
kernel32.WriteFile.restype = wintypes.BOOL

kernel32.PurgeComm.argtypes = [
    wintypes.HANDLE,
    wintypes.DWORD,
]
kernel32.PurgeComm.restype = wintypes.BOOL


def _last_error(prefix):
    code = ctypes.get_last_error()
    message = ctypes.FormatError(code).strip()
    return SerialException(f"{prefix}（Windows 错误 {code}: {message}）")


class Serial:
    EIGHTBITS = 8
    PARITY_NONE = "N"
    STOPBITS_ONE = 1

    def __init__(
        self,
        port,
        baudrate=115200,
        bytesize=EIGHTBITS,
        parity=PARITY_NONE,
        stopbits=STOPBITS_ONE,
        timeout=0.2,
        **_kwargs,
    ):
        if bytesize != 8 or parity != "N" or stopbits != 1:
            raise SerialException("当前后备串口只支持 8-N-1。")

        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._handle = self._open()
        self.is_open = True

        try:
            self._configure()
        except Exception:
            self.close()
            raise

    def _open(self):
        device = self.port
        if not device.upper().startswith("\\\\.\\"):
            device = "\\\\.\\" + device

        handle = kernel32.CreateFileW(
            device,
            GENERIC_READ | GENERIC_WRITE,
            0,
            None,
            OPEN_EXISTING,
            0,
            None,
        )

        if handle == INVALID_HANDLE_VALUE:
            raise _last_error(f"无法打开串口 {self.port}")

        return handle

    def _configure(self):
        dcb = DCB()
        dcb.DCBlength = ctypes.sizeof(DCB)

        if not kernel32.GetCommState(self._handle, ctypes.byref(dcb)):
            raise _last_error("读取串口参数失败")

        dcb.BaudRate = self.baudrate
        dcb.ByteSize = 8
        dcb.Parity = 0
        dcb.StopBits = 0

        # Binary mode, DTR enabled, RTS enabled, no software/hardware flow.
        dcb.Flags = 1 | (1 << 4) | (1 << 12)
        dcb.XonChar = b"\x11"
        dcb.XoffChar = b"\x13"

        if not kernel32.SetCommState(self._handle, ctypes.byref(dcb)):
            raise _last_error("设置串口参数失败")

        if not kernel32.SetupComm(self._handle, 4096, 4096):
            raise _last_error("设置串口缓冲区失败")

        timeouts = COMMTIMEOUTS()
        timeouts.ReadIntervalTimeout = 50
        timeouts.ReadTotalTimeoutMultiplier = 0
        timeouts.ReadTotalTimeoutConstant = 100
        timeouts.WriteTotalTimeoutMultiplier = 0
        timeouts.WriteTotalTimeoutConstant = 1000

        if not kernel32.SetCommTimeouts(
            self._handle,
            ctypes.byref(timeouts),
        ):
            raise _last_error("设置串口超时失败")

    def write(self, data):
        self._check_open()
        if isinstance(data, str):
            data = data.encode("ascii")

        buffer = ctypes.create_string_buffer(data)
        written = wintypes.DWORD()

        if not kernel32.WriteFile(
            self._handle,
            buffer,
            len(data),
            ctypes.byref(written),
            None,
        ):
            raise _last_error("发送串口数据失败")

        return written.value

    def readline(self):
        self._check_open()
        result = bytearray()
        deadline = None

        if self.timeout is not None:
            deadline = time.monotonic() + self.timeout

        while True:
            if deadline is not None and time.monotonic() >= deadline:
                break

            byte = ctypes.create_string_buffer(1)
            received = wintypes.DWORD()

            if not kernel32.ReadFile(
                self._handle,
                byte,
                1,
                ctypes.byref(received),
                None,
            ):
                raise _last_error("读取串口数据失败")

            if received.value == 0:
                continue

            result.extend(byte.raw[:received.value])
            if byte.raw[:received.value] == b"\n":
                break

        return bytes(result)

    def reset_input_buffer(self):
        self._check_open()
        if not kernel32.PurgeComm(self._handle, PURGE_RXCLEAR):
            raise _last_error("清空串口接收缓冲区失败")

    def close(self):
        if getattr(self, "is_open", False):
            kernel32.CloseHandle(self._handle)
            self.is_open = False

    def _check_open(self):
        if not self.is_open:
            raise SerialException("串口已经关闭。")

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback):
        self.close()

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass


class _SerialModule:
    Serial = Serial
    SerialException = SerialException
    EIGHTBITS = Serial.EIGHTBITS
    PARITY_NONE = Serial.PARITY_NONE
    STOPBITS_ONE = Serial.STOPBITS_ONE


serial = _SerialModule()


class PortInfo:
    def __init__(self, device):
        self.device = device
        self.description = "Windows 串口"
        self.hwid = ""

    def __str__(self):
        return self.device


class _ListPorts:
    @staticmethod
    def comports():
        ports = []

        try:
            key = winreg.OpenKey(
                winreg.HKEY_LOCAL_MACHINE,
                r"HARDWARE\DEVICEMAP\SERIALCOMM",
            )
        except FileNotFoundError:
            return ports

        with key:
            index = 0
            while True:
                try:
                    _name, value, _value_type = winreg.EnumValue(key, index)
                except OSError:
                    break

                ports.append(PortInfo(str(value)))
                index += 1

        def port_number(item):
            match = re.search(r"(\d+)$", item.device)
            return int(match.group(1)) if match else 9999

        return sorted(ports, key=port_number)


list_ports = _ListPorts()
