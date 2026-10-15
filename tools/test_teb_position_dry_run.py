#!/usr/bin/env python3
"""在隔离话题验证 TEB 位置跟踪；不连接真实 MAVROS/飞控。"""

import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Path
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import StaticTransformBroadcaster

from flight_control.mavros_adapter_node import MavrosAdapter


class Probe(Node):
    def __init__(self, prefix):
        super().__init__('teb_position_dry_run_probe')
        self.pose_pub = self.create_publisher(TransformStamped, prefix + '/pose', 10)
        self.path_pub = self.create_publisher(Path, prefix + '/local_plan', 10)
        self.cmd_pub = self.create_publisher(Twist, prefix + '/cmd_vel', 10)
        self.position_sub = self.create_subscription(
            PoseStamped, prefix + '/position_output', self._position_cb, 10)
        self.velocity_sub = self.create_subscription(
            TwistStamped, prefix + '/velocity_output', self._velocity_cb, 10)
        self.positions = []
        self.velocity_count = 0
        self.publish_plan = True
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.static_tf_pub = StaticTransformBroadcaster(self)
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = self.get_clock().now().to_msg()
        map_to_odom.header.frame_id = 'map'
        map_to_odom.child_frame_id = 'odom'
        map_to_odom.transform.translation.z = 0.39
        map_to_odom.transform.rotation.w = 1.0
        self.static_tf_pub.sendTransform(map_to_odom)
        self.timer = self.create_timer(0.05, self._publish_inputs)

    def _publish_inputs(self):
        stamp = self.get_clock().now().to_msg()
        pose = TransformStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'odom'
        pose.child_frame_id = 'livox_raw'
        pose.transform.translation.x = self.pose_x
        pose.transform.translation.y = self.pose_y
        pose.transform.rotation.w = 1.0
        self.pose_pub.publish(pose)

        if self.publish_plan:
            path = Path()
            path.header.stamp = stamp
            path.header.frame_id = 'map'
            for x in (0.0, 1.0):
                point = PoseStamped()
                point.header = path.header
                point.pose.position.x = x
                point.pose.position.z = 99.0  # TEB visualization time, not altitude.
                point.pose.orientation.w = 1.0
                path.poses.append(point)
            self.path_pub.publish(path)
            cmd = Twist()
            cmd.linear.x = 0.15
            self.cmd_pub.publish(cmd)

    def _position_cb(self, msg):
        self.positions.append(msg)

    def _velocity_cb(self, _msg):
        self.velocity_count += 1


def spin_for(executor, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)


def main():
    rclpy.init()
    prefix = '/teb_position_dry_run'
    overrides = [
        Parameter('dry_run', value=True),
        Parameter('allow_rc_arming', value=False),
        Parameter('auto_set_mode', value=False),
        Parameter('hold_heading', value=False),
        Parameter('teb_position_follow_enabled', value=True),
        Parameter('teb_position_plan_timeout', value=0.5),
        Parameter('cruise_height', value=0.9),
        Parameter('fcu_reference_z_offset', value=0.08),
        Parameter('current_pose_topic', value=prefix + '/pose'),
        Parameter('teb_local_plan_topic', value=prefix + '/local_plan'),
        Parameter('cmd_vel_topic', value=prefix + '/cmd_vel'),
        Parameter('nav_state_topic', value=prefix + '/nav_state'),
        Parameter('passing_door_state_topic', value=prefix + '/door_state'),
        Parameter('target_pose_topic', value=prefix + '/target_pose'),
        Parameter('arm_state_topic', value=prefix + '/arm_state'),
        Parameter('status_topic', value=prefix + '/status'),
        Parameter('mavros_state_topic', value=prefix + '/state'),
        Parameter('mavros_rc_topic', value=prefix + '/rc'),
        Parameter('mavros_arming_service', value=prefix + '/arming'),
        Parameter('mavros_set_mode_service', value=prefix + '/set_mode'),
        Parameter('mavros_velocity_setpoint_topic', value=prefix + '/velocity_output'),
        Parameter('mavros_position_setpoint_topic', value=prefix + '/position_output'),
        Parameter('mavros_vision_pose_topic', value=prefix + '/vision_output'),
    ]
    adapter = MavrosAdapter(parameter_overrides=overrides)
    probe = Probe(prefix)
    executor = SingleThreadedExecutor()
    executor.add_node(adapter)
    executor.add_node(probe)
    try:
        spin_for(executor, 1.3)
        if len(probe.positions) < 8:
            raise RuntimeError('位置 setpoint 数量不足: %d' % len(probe.positions))
        if probe.velocity_count:
            raise RuntimeError('意外发布了 %d 条速度 setpoint' % probe.velocity_count)
        moving = probe.positions[-1]
        # Legacy map XY -> MAVROS ENU (-y, x).
        if not 0.02 < moving.pose.position.y <= 0.13:
            raise RuntimeError('未沿 TEB 路径推进或超前过大: %s' %
                               moving.pose.position.y)
        if abs(moving.pose.position.x) > 0.01:
            raise RuntimeError('Y 转换错误: %s' % moving.pose.position.x)
        if abs(moving.pose.position.z - 0.82) > 0.01:
            raise RuntimeError('高度错误，可能误用了 Path.z: %s' %
                               moving.pose.position.z)

        probe.publish_plan = False
        spin_for(executor, 0.9)
        hold = probe.positions[-1]
        if adapter.last_setpoint_kind != 'teb_position_hold':
            raise RuntimeError('路径过期后未进入位置保持: %s' %
                               adapter.last_setpoint_kind)
        if abs(hold.pose.position.y) > 0.01:
            raise RuntimeError('路径过期后未保持当前位置: %s' %
                               hold.pose.position.y)
        if probe.velocity_count:
            raise RuntimeError('过期后意外发布速度 setpoint')
        print('PASS: TEB 路径 -> 限幅位置 setpoint；路径超时 -> 原地保持')
        print('  位置 setpoint: %d 条；速度 setpoint: %d 条' %
              (len(probe.positions), probe.velocity_count))
        print('  输出话题: %s/position_output（与真实 MAVROS 隔离）' % prefix)
        return 0
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


if __name__ == '__main__':
    raise SystemExit(main())
