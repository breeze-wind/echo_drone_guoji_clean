#!/usr/bin/env python3
"""统一串口清单和健康状态发布节点。

本节点只负责“发现和汇报”串口设备，不打开串口、不读写数据、不抢占设备。
实际业务串口仍由各自 owner 使用：飞控由 MAVROS 独占，舵机由 servo_node 打开。
"""

import json
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

import serial.tools.list_ports


class RobotSerialManager(Node):
    """按配置周期扫描串口，并发布统一 JSON 状态。"""

    def __init__(self):
        super().__init__('robot_serial_manager')

        # dry_run=true 时，缺失设备不会被标成硬错误，适合非试验机环境调 launch。
        self.declare_parameter('dry_run', True)
        # 周期扫描会持续更新 /hardware/serial_status，方便 rqt/topic echo 观察。
        self.declare_parameter('scan_period', 1.0)
        # devices 是逻辑设备名列表，具体端口、owner、波特率在 device.<name> 下声明。

        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.scan_period = float(self.get_parameter('scan_period').value)
        self.device_names = list(self.get_parameter('devices').value)
        self.devices = {}

        for name in self.device_names:
            self.devices[name] = self._declare_device(name)

        # 输出统一串口状态，消息体是 JSON 字符串，便于 shell、日志和上位监控解析。
        self.status_pub = self.create_publisher(String, '/hardware/serial_status', 10)
        # 手动重扫入口，用于热插拔 USB 后立即刷新状态。
        self.rescan_srv = self.create_service(
            Trigger, '/hardware/rescan_serials', self.rescan_callback)
        self.scan_timer = self.create_timer(self.scan_period, self.scan_and_publish)
        self.latest_status = {}

        self.get_logger().info(
            'Serial manager started: dry_run=%s devices=%s'
            % (self.dry_run, ','.join(self.device_names))
        )
        self.scan_and_publish()

    def _declare_device(self, name):
        """声明并读取单个逻辑设备的参数。"""
        prefix = 'device.%s' % name
        self.declare_parameter(prefix + '.required', False)
        self.declare_parameter(prefix + '.owner', '')
        self.declare_parameter(prefix + '.port', '')
        self.declare_parameter(prefix + '.baudrate', 0)
        self.declare_parameter(prefix + '.manufacturer', '')

        return {
            'name': name,
            'required': bool(self.get_parameter(prefix + '.required').value),
            'owner': str(self.get_parameter(prefix + '.owner').value),
            'port': str(self.get_parameter(prefix + '.port').value),
            'baudrate': int(self.get_parameter(prefix + '.baudrate').value),
            'manufacturer': str(self.get_parameter(prefix + '.manufacturer').value),
        }

    def rescan_callback(self, request, response):
        """手动重扫服务回调，返回最新串口状态 JSON。"""
        del request
        self.scan_and_publish()
        response.success = True
        response.message = json.dumps(self.latest_status, ensure_ascii=False)
        return response

    def scan_and_publish(self):
        """扫描当前系统串口，生成并发布完整状态。"""
        ports = list(serial.tools.list_ports.comports())
        status = {
            'stamp': time.time(),
            'dry_run': self.dry_run,
            'devices': [],
        }

        for device in self.devices.values():
            status['devices'].append(self._status_for_device(device, ports))

        self.latest_status = status
        msg = String()
        msg.data = json.dumps(status, ensure_ascii=False)
        self.status_pub.publish(msg)

    def _status_for_device(self, device, ports):
        """生成单个逻辑设备的状态字典。"""
        configured_port = device['port']
        configured_exists = bool(configured_port) and os.path.exists(configured_port)
        matched_port = None

        # 优先信任配置端口；如果配置端口不存在，再按 manufacturer 做保底匹配。
        if configured_exists:
            matched_port = self._port_by_path(configured_port, ports)
        elif device['manufacturer']:
            matched_port = self._port_by_manufacturer(device['manufacturer'], ports)

        resolved_port = configured_port if configured_exists else ''
        fallback_used = False
        if matched_port is not None:
            resolved_port = matched_port.device
            fallback_used = not configured_exists

        # 状态含义：
        # available：端口存在或 fallback 匹配成功。
        # dry_run_missing：dry-run 下缺失，仅提示，不阻断台架调试。
        # missing_required：真实模式下必需设备缺失，需要上机前处理。
        if resolved_port:
            state = 'available'
        elif self.dry_run:
            state = 'dry_run_missing'
        elif device['required']:
            state = 'missing_required'
        else:
            state = 'missing_optional'

        return {
            'name': device['name'],
            'owner': device['owner'],
            'required': device['required'],
            'configured_port': configured_port,
            'configured_exists': configured_exists,
            'resolved_port': resolved_port,
            'fallback_used': fallback_used,
            'baudrate': device['baudrate'],
            'manufacturer': device['manufacturer'],
            'state': state,
            'port_info': self._port_info(matched_port),
        }

    @staticmethod
    def _port_by_path(path, ports):
        """按系统枚举出来的真实设备路径匹配端口。"""
        for port in ports:
            if port.device == path:
                return port
        return None

    @staticmethod
    def _port_by_manufacturer(manufacturer, ports):
        """按 USB manufacturer 字段做 fallback 匹配。"""
        for port in ports:
            if port.manufacturer == manufacturer:
                return port
        return None

    @staticmethod
    def _port_info(port):
        """把 pyserial 的 ListPortInfo 转成可 JSON 化的字典。"""
        if port is None:
            return {}
        return {
            'device': port.device,
            'name': port.name,
            'description': port.description,
            'manufacturer': port.manufacturer,
            'hwid': port.hwid,
        }


def main(args=None):
    rclpy.init(args=args)
    node = RobotSerialManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
