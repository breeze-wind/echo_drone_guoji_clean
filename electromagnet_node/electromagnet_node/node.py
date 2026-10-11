"""Serial driver for the electromagnet firmware's GET / SET n protocol."""

import json
import queue
import re
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

try:
    import serial
except ImportError:  # dry-run remains usable without pyserial
    serial = None


STATE_RE = re.compile(r'\bSTATE=(\d+)\s+ACTIVE=(\d+)\b')


class ElectromagnetNode(Node):
    def __init__(self):
        super().__init__('electromagnet_node')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('command_topic', '/electromagnet/set_state')
        self.declare_parameter('status_topic', '/electromagnet/status')
        self.declare_parameter('status_rate_hz', 2.0)
        self.declare_parameter('ack_timeout_sec', 2.0)
        # Empirical hardware mapping: ROS state N is sent verbatim as MCU SET N.
        # MCU STATE=0 is the startup state; commanded states are 1..4.
        self.declare_parameter('protocol_channel_count', 4)
        self.declare_parameter('cargo_channel_count', 3)

        self.port_name = str(self.get_parameter('port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.ack_timeout = float(self.get_parameter('ack_timeout_sec').value)
        self.protocol_channels = int(self.get_parameter('protocol_channel_count').value)
        self.cargo_channels = int(self.get_parameter('cargo_channel_count').value)
        rate = float(self.get_parameter('status_rate_hz').value)
        if (rate <= 0.0 or self.ack_timeout <= 0.0 or
                self.protocol_channels < self.cargo_channels or self.cargo_channels < 1):
            raise ValueError('invalid status/ACK rate or channel counts')

        self.pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 10)
        self.create_subscription(
            Int32, str(self.get_parameter('command_topic').value), self._command_cb, 10)
        self.events = queue.Queue()
        self.lock = threading.Lock()
        self.serial_port = None
        self.reader = None
        self.stop_event = threading.Event()
        self.connected = False
        self.ready = False
        self.state = None
        self.active = None
        self.reported_active_channels = None
        self.pending_command = None
        self.pending_since = None
        self.last_sent_command = None
        self.ack_sequence = 0
        self.last_error = ''

        if self.dry_run:
            self.get_logger().warning(
                'DRY RUN: serial port will not be opened and no SET command can actuate hardware.')
        else:
            self._open_serial()
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            'Electromagnet node started: port=%s baud=%d dry_run=%s; MCU protocol channels=%d, '
            'cargo channels=%d; ROS state N maps directly to MCU SET N (1..%d); '
            'MCU STATE is reported without offset (startup may report 0).' %
            (self.port_name, self.baudrate, self.dry_run, self.protocol_channels,
             self.cargo_channels, self.cargo_channels + 1))

    def _open_serial(self):
        if serial is None:
            self.last_error = 'pyserial is not installed'
            self.get_logger().error(self.last_error)
            return
        try:
            self.serial_port = serial.Serial(
                port=self.port_name, baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=0.2, write_timeout=1.0)
            self.connected = True
            self.reader = threading.Thread(
                target=self._read_loop, name='electromagnet-serial-reader', daemon=True)
            self.reader.start()
            # Query current firmware state; do not alter outputs on startup.
            self._write_line('GET')
        except Exception as exc:
            self.last_error = 'serial open failed: %s' % exc
            self.connected = False
            self.get_logger().error(self.last_error)

    def _write_line(self, command):
        with self.lock:
            port = self.serial_port
            if port is None or not port.is_open:
                raise RuntimeError('serial connection is not open')
            port.write((command + '\r\n').encode('ascii'))
            port.flush()

    def _read_loop(self):
        port = self.serial_port
        while not self.stop_event.is_set():
            try:
                raw = port.readline()
            except Exception as exc:
                if not self.stop_event.is_set():
                    self.events.put(('error', str(exc)))
                return
            if raw:
                self.events.put(('line', raw.decode('ascii', errors='replace').strip()))

    def _command_cb(self, msg):
        command = int(msg.data)
        if command < 1 or command > self.cargo_channels + 1:
            self.get_logger().error(
                'Rejected invalid public state %d (valid 1..%d)' %
                (command, self.cargo_channels + 1))
            return
        if self.dry_run:
            self.get_logger().warning(
                'DRY RUN: public state %d maps to MCU SET %d; hardware untouched' %
                (command, command))
            return
        if not self.ready:
            self.get_logger().error('Rejected public state %d: MCU state is not ready' % command)
            return
        if self.pending_command is not None:
            if self.pending_command == command:
                # The decision node periodically republishes its desired state.
                # Coalesce duplicates; never queue them behind an outstanding ACK.
                if (self.pending_since is not None and
                        time.monotonic() - self.pending_since < self.ack_timeout):
                    return
                # The ACK deadline has elapsed; let this heartbeat retry the SET.
                self.get_logger().warning(
                    'Retrying public state %d after ACK timeout' % command)
                self.pending_command = None
                self.pending_since = None
            else:
                self.get_logger().error(
                    'Rejected public state %d: state %d is still awaiting ACK' %
                    (command, self.pending_command))
                return
        # Do not short-circuit when the reported state already matches. Bench
        # testing shows the physical release can require repeated identical SETs;
        # each new ROS message must reach the MCU. Pending identical messages above
        # are still coalesced until the in-flight command is acknowledged.
        try:
            self.pending_command = command
            self.pending_since = time.monotonic()
            mcu_command = command
            self._write_line('SET %d' % mcu_command)
            self.get_logger().warning(
                'Sent public state %d (MCU SET %d); waiting for matching ACK' %
                (command, mcu_command))
        except Exception as exc:
            self.pending_command = None
            self.pending_since = None
            self.last_error = 'serial write failed: %s' % exc
            self.connected = self.ready = False
            self.get_logger().error(self.last_error)

    def _tick(self):
        while True:
            try:
                kind, payload = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == 'error':
                self.connected = self.ready = False
                self.last_error = 'serial read failed: %s' % payload
                self.get_logger().error(self.last_error)
                continue
            match = STATE_RE.search(payload)
            if not match:
                if payload.startswith('ERR'):
                    self.last_error = payload
                    self.get_logger().error('Electromagnet MCU: %s' % payload)
                continue
            state, active = int(match.group(1)), int(match.group(2))
            # Empirically verified: SET 1 energizes all three connected magnets,
            # and SET 4 releases all. Preserve MCU STATE verbatim; STATE=0 is
            # an uncommanded startup state and must not masquerade as state 1.
            public_state = state
            active_channel_count = None
            if (0 <= state <= self.protocol_channels and
                    active == self.protocol_channels - state):
                active_channel_count = self.protocol_channels
            elif (0 <= state <= self.cargo_channels and
                    active == self.cargo_channels - state):
                # Also accept the three-connected-output firmware variant.
                active_channel_count = self.cargo_channels
            if (active_channel_count is None or
                    (self.reported_active_channels is not None and
                     active_channel_count != self.reported_active_channels)):
                self.ready = False
                expected_active = [
                    count - state for count in (self.protocol_channels, self.cargo_channels)
                    if 0 <= state <= count]
                self.last_error = (
                    'incompatible MCU status STATE=%d ACTIVE=%d; expected ACTIVE in %s '
                    '(protocol=%d, cargo=%d)' %
                    (state, active, expected_active,
                     self.protocol_channels, self.cargo_channels))
                self.get_logger().error(self.last_error)
                continue
            if self.reported_active_channels is None:
                self.reported_active_channels = active_channel_count
                self.get_logger().info(
                    'Detected MCU ACTIVE convention: %d sequential outputs' %
                    active_channel_count)
            self.state, self.active = public_state, active
            self.connected = self.ready = True
            self.last_error = ''
            if self.pending_command is not None and public_state == self.pending_command:
                self.last_sent_command = self.pending_command
                self.pending_command = None
                self.pending_since = None
                self.ack_sequence += 1
                self.get_logger().info(
                    'MCU ACK: public_state=%d estimated_cargo_magnets_active=%d '
                    'protocol_outputs_active=%d '
                    'ack_sequence=%d' %
                    (public_state, self._cargo_active(public_state), active, self.ack_sequence))
        if (self.pending_command is not None and self.pending_since is not None
                and time.monotonic() - self.pending_since > self.ack_timeout):
            self.last_error = 'ACK timeout waiting for STATE=%d' % self.pending_command
            self.get_logger().error(self.last_error)
            self.pending_command = None
            self.pending_since = None

        status = {
            'connected': self.connected,
            'ready': self.ready,
            'dry_run': self.dry_run,
            'port': self.port_name,
            'state': self.state,
            # ACTIVE is the MCU-reported energized count; expose the bounded
            # connected cargo count separately for mission/status consumers.
            'active': self.active,
            'protocol_active': self.active,
            'cargo_active': (self._cargo_active(self.state)
                             if self.state is not None else None),
            'last_sent_command': self.last_sent_command,
            'ack_sequence': self.ack_sequence,
            'pending_command': self.pending_command,
            'last_error': self.last_error,
        }
        self.pub.publish(String(data=json.dumps(status, separators=(',', ':'))))

    def _cargo_active(self, state):
        """Count connected magnets for UI/ROS/MCU state 1..4."""
        return max(0, min(self.cargo_channels, self.cargo_channels + 1 - int(state)))

    def destroy_node(self):
        self.stop_event.set()
        port = self.serial_port
        self.serial_port = None
        if port is not None:
            try:
                port.close()
            except Exception:
                pass
        if self.reader is not None and self.reader.is_alive():
            self.reader.join(timeout=0.5)
        # Never send a release/reset state on shutdown.
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ElectromagnetNode()
        rclpy.spin(node)
    except (TypeError, ValueError) as exc:
        if node is not None:
            node.get_logger().fatal(str(exc))
        else:
            print('electromagnet node startup failed: %s' % exc)
        return 2
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
