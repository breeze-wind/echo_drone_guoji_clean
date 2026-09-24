#!/usr/bin/env python3
import json
import os
import struct
import time

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger

import serial
import serial.tools.list_ports


class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')

        self.declare_parameter('dry_run', True)
        self.declare_parameter('required', True)
        self.declare_parameter('port', '/dev/stm32_servo')
        self.declare_parameter('manufacturer', 'STMicroelectronics')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('command_min', 0)
        self.declare_parameter('command_max', 255)
        self.declare_parameter('initial_command', 0)
        self.declare_parameter('drop_command', 1)
        self.declare_parameter('write_period', 0.1)
        self.declare_parameter('continuous_write', False)
        self.declare_parameter('status_period', 1.0)
        self.declare_parameter('legacy_parameter_name', '/servo/servo')

        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.required = bool(self.get_parameter('required').value)
        self.configured_port = str(self.get_parameter('port').value)
        self.manufacturer = str(self.get_parameter('manufacturer').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.command_min = int(self.get_parameter('command_min').value)
        self.command_max = int(self.get_parameter('command_max').value)
        self.drop_command = int(self.get_parameter('drop_command').value)
        self.write_period = float(self.get_parameter('write_period').value)
        self.continuous_write = bool(self.get_parameter('continuous_write').value)
        self.status_period = float(self.get_parameter('status_period').value)
        self.legacy_parameter_name = str(self.get_parameter('legacy_parameter_name').value)

        initial_command = int(self.get_parameter('initial_command').value)
        self.current_command = self._clamp_command(initial_command)
        self.last_sent_command = None
        self.last_error = ''
        self.last_command_source = 'initial'
        self.serial_port = None
        self.resolved_port = ''
        self.connected = False

        self._declare_legacy_parameter(initial_command)

        self.status_pub = self.create_publisher(String, '/servo/status', 10)
        self.command_sub = self.create_subscription(
            Int32, '/servo/command', self.command_callback, 10)
        self.drop_srv = self.create_service(Trigger, '/servo/drop', self.drop_callback)

        self.add_on_set_parameters_callback(self.parameter_callback)

        self._connect()
        self.write_timer = self.create_timer(self.write_period, self.write_timer_callback)
        self.status_timer = self.create_timer(self.status_period, self.publish_status)

        self.get_logger().info(
            'Servo driver started: dry_run=%s port=%s resolved=%s baudrate=%d'
            % (self.dry_run, self.configured_port, self.resolved_port, self.baudrate)
        )

    def _declare_legacy_parameter(self, initial_command):
        try:
            self.declare_parameter(self.legacy_parameter_name, initial_command)
        except Exception as exc:
            self.last_error = 'failed to declare legacy parameter %s: %s' % (
                self.legacy_parameter_name, exc)
            self.get_logger().warning(self.last_error)

    def _clamp_command(self, command):
        return max(self.command_min, min(self.command_max, int(command)))

    def _validate_command(self, command):
        command = int(command)
        if command < self.command_min or command > self.command_max:
            raise ValueError(
                'servo command %d outside [%d, %d]'
                % (command, self.command_min, self.command_max)
            )
        return command

    def _resolve_port(self):
        if self.configured_port and os.path.exists(self.configured_port):
            return self.configured_port

        if not self.manufacturer:
            return ''

        for port in serial.tools.list_ports.comports():
            if port.manufacturer == self.manufacturer:
                return port.device

        return ''

    def _connect(self):
        self.resolved_port = self._resolve_port()

        if self.dry_run:
            self.connected = False
            self.get_logger().warning(
                'Servo dry-run enabled; serial writes will be logged only.')
            return

        if not self.resolved_port:
            self.last_error = (
                'servo serial port not found: configured=%s manufacturer=%s'
                % (self.configured_port, self.manufacturer)
            )
            self.get_logger().error(self.last_error)
            if self.required:
                raise RuntimeError(self.last_error)
            return

        self.serial_port = serial.Serial(
            port=self.resolved_port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0,
        )
        self.connected = True

    def command_callback(self, msg):
        self.set_command(msg.data, 'topic:/servo/command')

    def drop_callback(self, request, response):
        del request
        ok = self.set_command(self.drop_command, 'service:/servo/drop')
        response.success = bool(ok)
        response.message = 'drop command accepted' if ok else self.last_error
        return response

    def parameter_callback(self, params):
        for param in params:
            if param.name not in (self.legacy_parameter_name, 'servo_command'):
                continue
            if param.type_ != Parameter.Type.INTEGER:
                return SetParametersResult(
                    successful=False,
                    reason='servo command parameter must be integer',
                )
            try:
                self.set_command(param.value, 'parameter:%s' % param.name)
            except ValueError as exc:
                return SetParametersResult(successful=False, reason=str(exc))

        return SetParametersResult(successful=True)

    def set_command(self, command, source):
        try:
            self.current_command = self._validate_command(command)
        except ValueError as exc:
            self.last_error = str(exc)
            self.get_logger().error(self.last_error)
            return False

        self.last_command_source = source
        self.get_logger().info(
            'servo command set to %d from %s' % (self.current_command, source))
        return self.dry_run or self.connected

    def write_timer_callback(self):
        if not self.continuous_write and self.last_sent_command == self.current_command:
            return

        if not self.dry_run and not self.connected:
            return

        self._write_command(self.current_command)

    def _write_command(self, command):
        payload = struct.pack('B', command)
        if self.dry_run:
            self.last_sent_command = command
            self.get_logger().info(
                'servo dry-run write: command=%d payload=%s'
                % (command, payload.hex())
            )
            return

        try:
            self.serial_port.write(payload)
            self.last_sent_command = command
            self.last_error = ''
        except Exception as exc:
            self.connected = False
            self.last_error = 'servo serial write failed: %s' % exc
            self.get_logger().error(self.last_error)

    def publish_status(self):
        msg = String()
        msg.data = json.dumps({
            'stamp': time.time(),
            'dry_run': self.dry_run,
            'required': self.required,
            'connected': self.connected,
            'configured_port': self.configured_port,
            'resolved_port': self.resolved_port,
            'manufacturer': self.manufacturer,
            'baudrate': self.baudrate,
            'current_command': self.current_command,
            'last_sent_command': self.last_sent_command,
            'last_command_source': self.last_command_source,
            'last_error': self.last_error,
        }, ensure_ascii=False)
        self.status_pub.publish(msg)

    def destroy_node(self):
        if self.serial_port is not None:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
