#!/usr/bin/env python3
"""只读比较 Nav2 速度、MAVROS setpoint 和 Point-LIO 实测速度。"""

import json
import math
import time

import rclpy
from geometry_msgs.msg import (
    PoseStamped, TransformStamped, Twist, TwistStamped,
)
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from .conversions import (
    LEGACY_COORDINATE_MODE,
    euler_from_quaternion_msg,
)
from .velocity_monitor_math import (
    PoseVelocityEstimator,
    compare_velocity,
    world_to_body,
)


class VelocityMonitor(Node):
    """绝不发布控制命令；只报告各层速度和数据新鲜度。"""

    def __init__(self, *, parameter_overrides=None):
        super().__init__('velocity_monitor',
                         parameter_overrides=parameter_overrides)
        self.pose_topic = self._param('pose_topic', '/robot/current_pose')
        self.cmd_topic = self._param('cmd_vel_topic', '/cmd_vel')
        self.velocity_topic = self._param(
            'velocity_setpoint_topic', '/mavros/mavros/cmd_vel')
        self.position_topic = self._param(
            'position_setpoint_topic', '/mavros/mavros/local')
        self.report_topic = self._param(
            'report_topic', '/flight_control/velocity_comparison')
        self.coordinate_mode = self._param(
            'coordinate_mode', LEGACY_COORDINATE_MODE)
        self.expected_pose_frame = self._param('expected_pose_frame', 'odom')
        self.pose_timeout_sec = float(self._param('pose_timeout_sec', 0.7))
        self.command_timeout_sec = float(
            self._param('command_timeout_sec', 0.7))
        self.print_period_sec = float(self._param('print_period_sec', 1.0))
        window = float(self._param('velocity_window_sec', 0.6))
        if min(self.pose_timeout_sec, self.command_timeout_sec,
               self.print_period_sec, window) <= 0:
            raise ValueError('monitor periods/timeouts must be positive')

        self.estimator = PoseVelocityEstimator(window_sec=window)
        self.pose_received_at = None
        self.yaw = None
        self.pose_frame = None
        self.pose_xy_world = None
        self.command = None
        self.command_received_at = None
        self.sent_velocity = None
        self.sent_velocity_received_at = None
        self.sent_position = None
        self.sent_position_received_at = None

        self.create_subscription(
            TransformStamped, self.pose_topic, self._pose_cb, 10)
        # A best-effort reader can receive either a reliable or best-effort
        # Nav2 publisher. The reverse pairing would silently receive nothing.
        cmd_qos = QoSProfile(
            depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Twist, self.cmd_topic, self._cmd_cb, cmd_qos)
        self.create_subscription(
            TwistStamped, self.velocity_topic, self._sent_velocity_cb, 10)
        self.create_subscription(
            PoseStamped, self.position_topic, self._sent_position_cb, 10)
        self.report_pub = self.create_publisher(String, self.report_topic, 10)
        self.create_timer(self.print_period_sec, self._report)
        self.get_logger().info(
            'Read-only speed comparison: Nav2=%s, pose=%s, '
            'MAVROS velocity=%s, '
            'MAVROS position=%s' %
            (self.cmd_topic, self.pose_topic, self.velocity_topic,
             self.position_topic))

    def _param(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    @staticmethod
    def _age(received_at):
        return None if received_at is None else time.monotonic() - received_at

    def _fresh(self, received_at, timeout):
        age = self._age(received_at)
        return age is not None and age <= timeout

    def _pose_cb(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp == 0.0:
            stamp = self.get_clock().now().nanoseconds * 1e-9
        x = msg.transform.translation.x
        y = msg.transform.translation.y
        frame = msg.header.frame_id
        if frame != self.expected_pose_frame:
            self.pose_frame = frame
            self.estimator.samples.clear()
            return
        if not self.estimator.add(stamp, x, y, frame):
            return
        _, _, yaw = euler_from_quaternion_msg(msg.transform.rotation)
        if not math.isfinite(yaw):
            return
        self.yaw = yaw
        self.pose_frame = frame
        self.pose_xy_world = (x, y)
        self.pose_received_at = time.monotonic()

    def _cmd_cb(self, msg):
        self.command = (float(msg.linear.x), float(msg.linear.y))
        self.command_received_at = time.monotonic()

    def _sent_velocity_cb(self, msg):
        self.sent_velocity = (float(msg.twist.linear.x),
                              float(msg.twist.linear.y),
                              float(msg.twist.linear.z))
        self.sent_velocity_received_at = time.monotonic()

    def _sent_position_cb(self, msg):
        self.sent_position = (float(msg.pose.position.x),
                              float(msg.pose.position.y),
                              float(msg.pose.position.z))
        self.sent_position_received_at = time.monotonic()

    def _report(self):
        pose_fresh = self._fresh(self.pose_received_at, self.pose_timeout_sec)
        cmd_fresh = self._fresh(self.command_received_at,
                                self.command_timeout_sec)
        velocity_fresh = self._fresh(self.sent_velocity_received_at,
                                     self.command_timeout_sec)
        position_fresh = self._fresh(self.sent_position_received_at,
                                     self.command_timeout_sec)
        world_velocity = self.estimator.velocity() if pose_fresh else None
        actual_body = (
            world_to_body(*world_velocity, self.yaw)
            if world_velocity is not None and self.yaw is not None else None)
        desired_body = self.command if cmd_fresh else None
        nav_comparison = (
            compare_velocity(desired_body, actual_body)
            if desired_body is not None and actual_body is not None else None)

        if velocity_fresh and position_fresh:
            mode = 'both_setpoint_types_recent'
        elif velocity_fresh:
            mode = 'velocity_setpoint'
        elif position_fresh:
            mode = 'position_setpoint'
        else:
            mode = 'no_setpoint'

        # map->odom currently has only a z translation. The ENU comparison is
        # useful in this configuration; body-frame comparison above does not
        # depend on that alignment.
        actual_enu = None
        sent_comparison = None
        if world_velocity is not None:
            vx, vy = world_velocity
            actual_enu = ((-vy, vx) if self.coordinate_mode ==
                          LEGACY_COORDINATE_MODE else (vx, vy))
        if velocity_fresh and actual_enu is not None:
            sent_comparison = compare_velocity(
                self.sent_velocity[:2], actual_enu)
        position_error_xy = None
        if position_fresh and pose_fresh and self.pose_xy_world is not None:
            px, py = self.pose_xy_world
            actual_position_enu = (
                (-py, px) if self.coordinate_mode == LEGACY_COORDINATE_MODE
                else (px, py))
            position_error_xy = math.hypot(
                self.sent_position[0] - actual_position_enu[0],
                self.sent_position[1] - actual_position_enu[1])

        report = {
            'stamp': self.get_clock().now().nanoseconds * 1e-9,
            'mode': mode,
            'pose_frame': self.pose_frame,
            'expected_pose_frame': self.expected_pose_frame,
            'pose_age_sec': self._age(self.pose_received_at),
            'cmd_age_sec': self._age(self.command_received_at),
            'velocity_setpoint_age_sec':
                self._age(self.sent_velocity_received_at),
            'position_setpoint_age_sec':
                self._age(self.sent_position_received_at),
            'teb_desired_body_xy_mps': desired_body,
            'actual_body_xy_mps': actual_body,
            'actual_world_xy_mps': world_velocity,
            'teb_vs_actual': nav_comparison,
            'sent_velocity_enu_xyz_mps': (
                self.sent_velocity if velocity_fresh else None),
            'actual_enu_xy_mps': actual_enu,
            'sent_vs_actual': sent_comparison,
            'sent_position_enu_xyz_m': (
                self.sent_position if position_fresh else None),
            'position_target_error_xy_m': position_error_xy,
            'note': ('TEB cmd_vel is a reference only; position mode does '
                     'not send it to PX4. ENU comparison assumes map/odom '
                     'XY aligned.'),
        }
        self.report_pub.publish(String(data=json.dumps(report)))
        if nav_comparison is None:
            self.get_logger().warn(
                '速度比较等待数据：mode=%s pose_age=%s cmd_age=%s pose_frame=%s' %
                (mode, report['pose_age_sec'], report['cmd_age_sec'],
                 self.pose_frame))
            return
        along_ratio = nav_comparison['along_track_ratio']
        along_text = (
            '期望速度接近零' if along_ratio is None else
            '沿轨 %.2f/%.2f m/s (%.0f%%)，横向 %+.2f m/s' %
            (nav_comparison['along_track_mps'],
             nav_comparison['requested_speed_mps'],
             100.0 * along_ratio, nav_comparison['cross_track_mps']))
        position_text = (
            '；位置目标误差=%.3f m' % position_error_xy
            if position_error_xy is not None else '')
        self.get_logger().info(
            '速度比较 [%s] TEB机体系=(%+.2f,%+.2f) 实测=(%+.2f,%+.2f) m/s；'
            '%s；发送速度=%s%s' %
            (mode, desired_body[0], desired_body[1],
             actual_body[0], actual_body[1], along_text,
             ('无（位置模式）' if mode == 'position_setpoint' else
              self.sent_velocity if velocity_fresh else '无'), position_text))


def main(args=None):
    rclpy.init(args=args)
    node = VelocityMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
