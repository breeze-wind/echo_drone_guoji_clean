#!/usr/bin/env python3
"""隔离验证 MAVROS adapter 的 cmd_vel 转发，不向真实 MAVROS 话题发消息。"""

import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool

from flight_control.mavros_adapter_node import MavrosAdapter


class CmdVelProbe(Node):
    def __init__(self):
        super().__init__('cmd_vel_dry_run_probe')
        self.pose_pub = self.create_publisher(
            TransformStamped, '/cmd_vel_dry_run/pose', 10)
        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel_dry_run/input', 10)
        self.nav_pub = self.create_publisher(
            Bool, '/cmd_vel_dry_run/nav_state', 10)
        self.output_sub = self.create_subscription(
            TwistStamped, '/cmd_vel_dry_run/output', self._output_cb, 10)
        self.latest_output = None
        self.output_count = 0
        self.started_at = time.monotonic()
        self.timer = self.create_timer(0.05, self._publish_inputs)

    def _publish_inputs(self):
        pose = TransformStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'odom'
        pose.child_frame_id = 'livox_raw'
        pose.transform.rotation.w = 1.0
        # At z=0 the configured feedback offsets give 0.31 m FCU height.
        pose.transform.translation.z = 0.0
        self.pose_pub.publish(pose)

        self.nav_pub.publish(Bool(data=True))
        cmd = Twist()
        cmd.linear.x = 0.20
        cmd.linear.y = 0.10
        self.cmd_pub.publish(cmd)

    def _output_cb(self, msg):
        self.latest_output = msg
        self.output_count += 1


def main():
    rclpy.init()
    test_prefix = '/cmd_vel_dry_run'
    overrides = [
        Parameter('dry_run', value=True),
        Parameter('coordinate_mode', value='legacy_ned_compatible'),
        Parameter('allow_rc_arming', value=False),
        Parameter('auto_set_mode', value=False),
        Parameter('hold_heading', value=False),
        Parameter('cruise_height', value=0.7),
        Parameter('height_feedback_z_offset', value=0.39),
        Parameter('fcu_reference_z_offset', value=0.08),
        Parameter('pid_height', value=0.65),
        Parameter('cmd_vel_topic', value=test_prefix + '/input'),
        Parameter('current_pose_topic', value=test_prefix + '/pose'),
        Parameter('nav_state_topic', value=test_prefix + '/nav_state'),
        Parameter('passing_door_state_topic', value=test_prefix + '/door_state'),
        Parameter('target_pose_topic', value=test_prefix + '/target_pose'),
        Parameter('arm_state_topic', value=test_prefix + '/arm_state'),
        Parameter('status_topic', value=test_prefix + '/status'),
        # Deliberately NOT /mavros/mavros/cmd_vel: no FCU can consume test output.
        Parameter('mavros_velocity_setpoint_topic', value=test_prefix + '/output'),
        Parameter('mavros_position_setpoint_topic', value=test_prefix + '/position_output'),
        Parameter('mavros_vision_pose_topic', value=test_prefix + '/vision_output'),
    ]
    adapter = MavrosAdapter(parameter_overrides=overrides)
    probe = CmdVelProbe()
    executor = SingleThreadedExecutor()
    executor.add_node(adapter)
    executor.add_node(probe)

    try:
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)

        output = probe.latest_output
        if output is None:
            raise RuntimeError('adapter published no velocity setpoint')

        # The configured legacy conversion maps input (x=0.2, y=0.1) to
        # MAVROS ENU (-0.1, 0.2); altitude feedback yields vz=+0.2535 m/s.
        expected = (-0.10, 0.20, 0.2535)
        actual = (output.twist.linear.x, output.twist.linear.y,
                  output.twist.linear.z)
        if any(abs(a - e) > 0.02 for a, e in zip(actual, expected)):
            raise RuntimeError('unexpected converted setpoint: %s (expected %s)' %
                               (actual, expected))
        if probe.output_count < 15:
            raise RuntimeError('setpoint stream too slow: %d messages in 3 seconds' %
                               probe.output_count)

        print('PASS: dry_run adapter forwarded cmd_vel to isolated test output')
        print('  source cmd_vel: x=+0.20, y=+0.10 m/s')
        print('  converted ENU: x=%+.3f, y=%+.3f, z=%+.4f m/s' % actual)
        print('  output rate: %.1f Hz (%d messages / 3.0 s)' %
              (probe.output_count / 3.0, probe.output_count))
        print('  output topic: %s/output (not connected to MAVROS/FCU)' % test_prefix)
    except Exception as exc:
        print('FAIL: %s' % exc)
        return 1
    finally:
        executor.remove_node(probe)
        executor.remove_node(adapter)
        probe.destroy_node()
        adapter.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
