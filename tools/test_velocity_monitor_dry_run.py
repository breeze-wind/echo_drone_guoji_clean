#!/usr/bin/env python3
"""隔离输入验证速度比较节点；不会发布到真实 MAVROS 话题。"""

import json
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from flight_control.velocity_monitor_node import VelocityMonitor


class Probe(Node):
    def __init__(self, prefix):
        super().__init__('velocity_monitor_dry_run_probe')
        self.pose_pub = self.create_publisher(
            TransformStamped, prefix + '/pose', 10)
        # Exercise the QoS pairing that a reliable-only monitor would miss.
        cmd_qos = QoSProfile(
            depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.cmd_pub = self.create_publisher(
            Twist, prefix + '/cmd', cmd_qos)
        self.sent_pub = self.create_publisher(
            TwistStamped, prefix + '/sent', 10)
        self.report = None
        self.create_subscription(
            String, prefix + '/report', self._report_cb, 10)
        self.start = time.monotonic()
        self.create_timer(0.05, self._publish)

    def _publish(self):
        elapsed = time.monotonic() - self.start
        pose = TransformStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'odom'
        pose.child_frame_id = 'livox_raw'
        pose.transform.translation.x = 0.1 * elapsed
        pose.transform.rotation.w = 1.0
        self.pose_pub.publish(pose)
        command = Twist()
        command.linear.x = 0.2
        self.cmd_pub.publish(command)
        sent = TwistStamped()
        sent.header.stamp = pose.header.stamp
        sent.twist.linear.y = 0.2  # legacy NED-compatible ENU mapping
        self.sent_pub.publish(sent)

    def _report_cb(self, msg):
        self.report = json.loads(msg.data)


def main():
    rclpy.init()
    prefix = '/velocity_monitor_dry_run'
    parameters = [
        Parameter('pose_topic', value=prefix + '/pose'),
        Parameter('cmd_vel_topic', value=prefix + '/cmd'),
        Parameter('velocity_setpoint_topic', value=prefix + '/sent'),
        Parameter('position_setpoint_topic', value=prefix + '/position'),
        Parameter('report_topic', value=prefix + '/report'),
        Parameter('print_period_sec', value=0.3),
    ]
    monitor = VelocityMonitor(parameter_overrides=parameters)
    probe = Probe(prefix)
    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    executor.add_node(probe)
    try:
        deadline = time.monotonic() + 2.2
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        report = probe.report
        if not report or report['teb_vs_actual'] is None:
            raise RuntimeError('未生成有效比较数据')
        ratio = report['teb_vs_actual']['along_track_ratio']
        if not 0.35 < ratio < 0.65:
            raise RuntimeError('速度比应约为 50%%，实测 %s' % ratio)
        if report['mode'] != 'velocity_setpoint':
            raise RuntimeError('setpoint 模式识别错误: %s' % report['mode'])
        if report['sent_vs_actual'] is None:
            raise RuntimeError('没有发出速度与实测速度的比较')
        print('PASS: 期望 0.20 m/s、实测 0.10 m/s；沿轨速度比 %.0f%%' %
              (ratio * 100.0))
        print('  所有测试话题均在 %s 下，与飞控隔离' % prefix)
        return 0
    except Exception as exc:
        print('FAIL: %s' % exc)
        return 1
    finally:
        executor.remove_node(probe)
        executor.remove_node(monitor)
        probe.destroy_node()
        monitor.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
