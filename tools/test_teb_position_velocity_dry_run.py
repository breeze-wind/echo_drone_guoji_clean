#!/usr/bin/env python3
"""隔离验证 TEB 位置 + 速度前馈；输出只发到测试话题，不连接飞控。"""

import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Path
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

from flight_control.mavros_adapter_node import MavrosAdapter


class Probe(Node):
    def __init__(self, prefix):
        super().__init__('teb_position_velocity_dry_run_probe')
        self.pose_pub = self.create_publisher(
            TransformStamped, prefix + '/pose', 10)
        self.path_pub = self.create_publisher(
            Path, prefix + '/local_plan', 10)
        self.cmd_pub = self.create_publisher(
            Twist, prefix + '/cmd_vel', 10)
        self.raw_setpoints = []
        self.raw_sub = self.create_subscription(
            PositionTarget, prefix + '/raw_output',
            self.raw_setpoints.append, 10)
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.publish_motion = True
        self.static_tf_pub = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.transform.translation.z = 0.39
        transform.transform.rotation.w = 1.0
        self.static_tf_pub.sendTransform(transform)
        self.timer = self.create_timer(0.05, self.publish_inputs)

    def publish_inputs(self):
        stamp = self.get_clock().now().to_msg()
        pose = TransformStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'odom'
        pose.child_frame_id = 'livox_raw'
        pose.transform.translation.x = self.pose_x
        pose.transform.translation.y = self.pose_y
        pose.transform.rotation.w = 1.0
        self.pose_pub.publish(pose)
        if not self.publish_motion:
            return
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = 'map'
        for x in (0.0, 1.0):
            point = PoseStamped()
            point.header = path.header
            point.pose.position.x = x
            # TEB visualization time, not altitude.
            point.pose.position.z = 123.0
            point.pose.orientation.w = 1.0
            path.poses.append(point)
        self.path_pub.publish(path)
        command = Twist()
        command.linear.x = 0.5
        self.cmd_pub.publish(command)


def spin_for(executor, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.05)


def main():
    rclpy.init()
    prefix = '/teb_position_velocity_dry_run'
    overrides = [
        Parameter('dry_run', value=True),
        Parameter('allow_rc_arming', value=False),
        Parameter('auto_set_mode', value=False),
        Parameter('hold_heading', value=False),
        Parameter('teb_position_follow_enabled', value=False),
        Parameter('teb_position_velocity_follow_enabled', value=True),
        Parameter('teb_position_velocity_feedforward_max', value=0.15),
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
        Parameter('mavros_raw_setpoint_topic', value=prefix + '/raw_output'),
        Parameter('mavros_velocity_setpoint_topic',
                  value=prefix + '/velocity_output'),
        Parameter('mavros_position_setpoint_topic',
                  value=prefix + '/position_output'),
        Parameter('mavros_vision_pose_topic',
                  value=prefix + '/vision_output'),
    ]
    adapter = MavrosAdapter(parameter_overrides=overrides)
    probe = Probe(prefix)
    executor = SingleThreadedExecutor()
    executor.add_node(adapter)
    executor.add_node(probe)
    try:
        spin_for(executor, 1.0)
        if len(probe.raw_setpoints) < 5:
            raise RuntimeError('组合 PositionTarget 数量不足')
        target = probe.raw_setpoints[-1]
        if target.coordinate_frame != PositionTarget.FRAME_LOCAL_NED:
            raise RuntimeError('coordinate_frame 不是 LOCAL_NED')
        if target.type_mask & (PositionTarget.IGNORE_PX |
                               PositionTarget.IGNORE_PY |
                               PositionTarget.IGNORE_PZ |
                               PositionTarget.IGNORE_VX |
                               PositionTarget.IGNORE_VY):
            raise RuntimeError('位置或 XY 速度被意外忽略')
        if (abs(target.velocity.x) > 0.01 or
                not (0.02 < target.velocity.y <= 0.151)):
            raise RuntimeError('速度变换/限幅异常: (%.3f, %.3f)' %
                               (target.velocity.x, target.velocity.y))
        if (abs(target.position.x) > 0.01 or
                not (0.01 < target.position.y <= 0.13)):
            raise RuntimeError('位置变换/前瞻异常: (%.3f, %.3f)' %
                               (target.position.x, target.position.y))
        if abs(target.position.z - 0.82) > 0.01:
            raise RuntimeError('高度偏置异常: %.3f' % target.position.z)

        probe.publish_motion = False
        spin_for(executor, 0.8)
        hold = probe.raw_setpoints[-1]
        if abs(hold.velocity.x) > 1e-6 or abs(hold.velocity.y) > 1e-6:
            raise RuntimeError('输入过期后速度前馈未归零')
        if abs(hold.position.x) > 1e-6 or abs(hold.position.y) > 1e-6:
            raise RuntimeError('输入过期后位置目标未保持当前位姿')
        if adapter.last_setpoint_kind != 'teb_position_velocity_hold':
            raise RuntimeError('输入过期后状态错误: %s' % adapter.last_setpoint_kind)
        print('PASS: 单路 raw-local 同时发送位置 + 限幅速度前馈；输入过期后保持/归零')
        print('  隔离输出: %s/raw_output（未连接真实 MAVROS）' % prefix)
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
