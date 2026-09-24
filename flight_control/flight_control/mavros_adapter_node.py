#!/usr/bin/env python3
"""把旧 `/robot/*` 飞控话题桥接到 MAVROS。

仓库内其他节点仍使用比赛旧版话题和坐标轴约定。本节点作为飞控传输边界：
把这些消息转换成 MAVROS 的 vision_pose 和 setpoint 话题，把 MAVROS 解锁
状态回写到 `/robot/arm_state`，并用 dry-run 和参数开关限制解锁/模式服务
调用，方便台架调试。
"""

import json

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, TwistStamped
from mavros_msgs.msg import RCIn, State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .conversions import (
    LEGACY_COORDINATE_MODE,
    MAVROS_ENU_COORDINATE_MODE,
    legacy_nav_velocity,
    legacy_passing_door_velocity,
    legacy_target_position,
    legacy_vision_orientation_to_mavros_enu,
    legacy_vision_pose_position,
    mavros_enu_yaw_for_legacy_ned_yaw,
    quaternion_from_euler_xyzw,
)


class MavrosAdapter(Node):
    """旧 behavior_control 与 MAVROS 之间的兼容适配器。"""

    def __init__(self):
        super().__init__('mavros_adapter')

        # 安全开关和坐标模式最先读取，因为它们决定是否允许调用 MAVROS
        # 服务，以及如何转换坐标轴。
        self.dry_run = self._bool_param('dry_run', True)
        self.coordinate_mode = str(
            self._param('coordinate_mode', LEGACY_COORDINATE_MODE))
        if self.coordinate_mode not in (
                LEGACY_COORDINATE_MODE, MAVROS_ENU_COORDINATE_MODE):
            self.get_logger().warn(
                'Unknown coordinate_mode=%s; using %s'
                % (self.coordinate_mode, LEGACY_COORDINATE_MODE))
            self.coordinate_mode = LEGACY_COORDINATE_MODE

        # frame_id 和高度偏置用于保留旧 pymavlink 数据约定，同时把传输层
        # 切到 MAVROS。
        self.frame_id = str(self._param('frame_id', 'map'))
        self.child_frame_id = str(self._param('child_frame_id', 'livox'))

        self.cruise_height = float(self._param('cruise_height', 0.6))
        self.passing_door_height = float(
            self._param('passing_door_height', 0.6))
        self.height_feedback_z_offset = float(
            self._param('height_feedback_z_offset', 0.39))
        self.fcu_reference_z_offset = float(
            self._param('fcu_reference_z_offset', 0.08))
        self.vision_pose_z_offset = float(
            self._param('vision_pose_z_offset', 0.31))
        self.pid_height = float(self._param('pid_height', 0.65))

        # 定时器和过期门限用于调试：状态持续发布，但位姿或命令过期时
        # 不再向 MAVROS 发布 setpoint。
        self.status_period = float(self._param('status_period', 1.0))
        self.setpoint_republish_period = float(
            self._param('setpoint_republish_period', 0.1))
        self.setpoint_stale_timeout = float(
            self._param('setpoint_stale_timeout', 1.0))
        self.pose_stale_timeout = float(
            self._param('pose_stale_timeout', 1.0))

        # RC 解锁桥是可选的，并且带节流；只有检测到配置的四通道组合后
        # 才会请求 MAVROS 解锁。
        self.allow_rc_arming = self._bool_param('allow_rc_arming', True)
        self.arm_retry_period = float(self._param('arm_retry_period', 1.0))
        self.rc_arm_channels = list(
            self._param('rc_arm_channels', [1, 2, 3, 4]))
        self.rc_arm_low_threshold = int(
            self._param('rc_arm_low_threshold', 1100))
        self.rc_arm_high_threshold = int(
            self._param('rc_arm_high_threshold', 1930))

        # 模式切换默认关闭；确认 setpoint 流稳定后，再手动开启 OFFBOARD
        # 或 GUIDED 模式切换。
        self.auto_set_mode = self._bool_param('auto_set_mode', False)
        self.desired_mode = str(self._param('desired_mode', 'OFFBOARD'))
        self.mode_retry_period = float(
            self._param('mode_retry_period', 1.0))

        # MAVROS 端点保持可配置，因为不同 Foxy MAVROS 安装和命名空间下
        # 话题名可能略有差异。
        self.mavros_state_topic = str(
            self._param('mavros_state_topic', '/mavros/state'))
        self.mavros_rc_topic = str(
            self._param('mavros_rc_topic', '/mavros/rc/in'))
        self.mavros_arming_service = str(
            self._param('mavros_arming_service', '/mavros/cmd/arming'))
        self.mavros_set_mode_service = str(
            self._param('mavros_set_mode_service', '/mavros/set_mode'))
        self.mavros_vision_pose_topic = str(
            self._param('mavros_vision_pose_topic',
                        '/mavros/vision_pose/pose'))
        self.mavros_position_setpoint_topic = str(
            self._param('mavros_position_setpoint_topic',
                        '/mavros/setpoint_position/local'))
        self.mavros_velocity_setpoint_topic = str(
            self._param('mavros_velocity_setpoint_topic',
                        '/mavros/setpoint_velocity/cmd_vel'))

        # behavior_control 和 Point-LIO 使用的旧上下游话题，本适配器负责
        # 保持这些接口稳定。
        current_pose_topic = str(
            self._param('current_pose_topic', '/robot/current_pose'))
        target_pose_topic = str(
            self._param('target_pose_topic', '/robot/target_pose'))
        cmd_vel_topic = str(self._param('cmd_vel_topic', '/cmd_vel'))
        nav_state_topic = str(
            self._param('nav_state_topic', '/robot/nav_state'))
        passing_door_state_topic = str(
            self._param('passing_door_state_topic',
                        '/robot/passing_door_state'))
        turning_state_topic = str(
            self._param('turning_state_topic', '/robot/turning_state'))
        arm_state_topic = str(
            self._param('arm_state_topic', '/robot/arm_state'))
        status_topic = str(
            self._param('status_topic', '/flight_control/status'))

        # 面向 MAVROS 的发布者。
        self.vision_pose_pub = self.create_publisher(
            PoseStamped, self.mavros_vision_pose_topic, 10)
        self.position_setpoint_pub = self.create_publisher(
            PoseStamped, self.mavros_position_setpoint_topic, 10)
        self.velocity_setpoint_pub = self.create_publisher(
            TwistStamped, self.mavros_velocity_setpoint_topic, 10)

        # 面向旧接口和调试输出的发布者。
        self.arm_state_pub = self.create_publisher(Bool, arm_state_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)

        # 来自 MAVROS、Point-LIO、behavior_control 和 Nav2 的订阅。
        self.state_sub = self.create_subscription(
            State, self.mavros_state_topic, self.state_callback, 10)
        self.rc_sub = self.create_subscription(
            RCIn, self.mavros_rc_topic, self.rc_callback, 10)
        self.current_pose_sub = self.create_subscription(
            TransformStamped, current_pose_topic,
            self.current_pose_callback, 10)
        self.target_pose_sub = self.create_subscription(
            TransformStamped, target_pose_topic,
            self.target_pose_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, cmd_vel_topic, self.cmd_vel_callback, 10)
        self.nav_state_sub = self.create_subscription(
            Bool, nav_state_topic, self.nav_state_callback, 10)
        self.passing_door_state_sub = self.create_subscription(
            Bool, passing_door_state_topic,
            self.passing_door_state_callback, 10)
        self.turning_state_sub = self.create_subscription(
            Bool, turning_state_topic, self.turning_state_callback, 10)

        # dry-run 下也创建服务客户端，便于检查可用性，但不允许真正调用。
        self.arming_client = self.create_client(
            CommandBool, self.mavros_arming_service)
        self.set_mode_client = self.create_client(
            SetMode, self.mavros_set_mode_service)

        # 缓存的 MAVROS 状态。
        self.connected = False
        self.armed = False
        self.guided = False
        self.mode = ''
        self.system_status = 0

        # 从 behavior_control 缓存的任务模式开关。
        self.if_nav = True
        self.current_passing_door = False
        self.if_turning = False

        # 最近输入样本及其年龄会发布到 status_topic，便于调试。
        self.current_height_feedback = None
        self.current_fcu_height_feedback = None
        self.last_current_pose_time = None
        self.latest_cmd_vel = None
        self.latest_cmd_vel_time = None
        self.latest_target_pose = None
        self.latest_target_pose_time = None
        self.last_setpoint_kind = 'none'
        self.last_setpoint_time = None

        # 服务调用状态用于避免重复刷屏式请求解锁或切模式。
        self.ready_to_arm = False
        self.arm_request_inflight = False
        self.last_arm_request_time = 0.0
        self.mode_request_inflight = False
        self.last_mode_request_time = 0.0

        # 一个定时器按 MAVROS 友好的频率重发 setpoint，另一个发布紧凑的
        # JSON 调试状态。
        self.status_timer = self.create_timer(
            self.status_period, self.status_timer_callback)
        self.setpoint_timer = self.create_timer(
            self.setpoint_republish_period,
            self.setpoint_timer_callback)

        self.get_logger().info(
            'MAVROS adapter started: dry_run=%s coordinate_mode=%s'
            % (self.dry_run, self.coordinate_mode))

    def _param(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _bool_param(self, name, default):
        return self._as_bool(self._param(name, default))

    @staticmethod
    def _as_bool(value):
        if isinstance(value, str):
            return value.lower() in ('true', '1', 'yes', 'on')
        return bool(value)

    def _now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _age(self, stamp_seconds):
        """返回样本年龄，单位秒；没有样本时返回 None。"""
        if stamp_seconds is None:
            return None
        return self._now_seconds() - stamp_seconds

    def state_callback(self, msg):
        """同步 MAVROS 状态，并把解锁状态发布给旧节点。"""
        self.connected = bool(msg.connected)
        self.armed = bool(msg.armed)
        self.guided = bool(msg.guided)
        self.mode = msg.mode
        self.system_status = int(msg.system_status)

        if self.armed:
            self.ready_to_arm = False
            self.arm_request_inflight = False

        arm_msg = Bool()
        arm_msg.data = self.armed
        self.arm_state_pub.publish(arm_msg)

    def rc_callback(self, msg):
        """把可选 RC 解锁组合转换成带节流的解锁请求。"""
        if not self.allow_rc_arming:
            return

        self.ready_to_arm = self._rc_arm_combo(msg.channels)
        if self.ready_to_arm and not self.armed:
            self._request_arm()

    def current_pose_callback(self, msg):
        """把 Point-LIO 位姿转换坐标后转发到 MAVROS vision_pose。"""
        self.current_height_feedback = (
            msg.transform.translation.z + self.height_feedback_z_offset)
        self.current_fcu_height_feedback = (
            self.current_height_feedback - self.fcu_reference_z_offset)
        self.last_current_pose_time = self._now_seconds()

        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.frame_id

        if self.coordinate_mode == LEGACY_COORDINATE_MODE:
            x, y, z = legacy_vision_pose_position(
                msg.transform.translation.x,
                msg.transform.translation.y,
                msg.transform.translation.z,
                self.vision_pose_z_offset)
            qx, qy, qz, qw = legacy_vision_orientation_to_mavros_enu(
                msg.transform.rotation)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
        else:
            x = msg.transform.translation.x
            y = msg.transform.translation.y
            z = msg.transform.translation.z + self.vision_pose_z_offset
            pose.pose.orientation.x = msg.transform.rotation.x
            pose.pose.orientation.y = msg.transform.rotation.y
            pose.pose.orientation.z = msg.transform.rotation.z
            pose.pose.orientation.w = msg.transform.rotation.w

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        self.vision_pose_pub.publish(pose)

    def cmd_vel_callback(self, msg):
        """缓存 Nav2 速度命令，并在导航模式下立即发布。"""
        self.latest_cmd_vel = msg
        self.latest_cmd_vel_time = self._now_seconds()
        if self.if_nav:
            self._publish_velocity_setpoint()

    def target_pose_callback(self, msg):
        """缓存直接位置目标，并在非 Nav2 模式下立即发布。"""
        self.latest_target_pose = msg
        self.latest_target_pose_time = self._now_seconds()
        if not self.if_nav:
            self._publish_position_setpoint()

    def nav_state_callback(self, msg):
        self.if_nav = bool(msg.data)

    def passing_door_state_callback(self, msg):
        self.current_passing_door = bool(msg.data)

    def turning_state_callback(self, msg):
        self.if_turning = bool(msg.data)

    def setpoint_timer_callback(self):
        """重发最近有效 setpoint，并控制可选 MAVROS 服务调用。"""
        if self.if_nav:
            self._publish_velocity_setpoint()
        else:
            self._publish_position_setpoint()

        if self.auto_set_mode:
            self._request_mode_if_needed()

        if self.ready_to_arm and not self.armed:
            self._request_arm()

    def status_timer_callback(self):
        """发布紧凑 JSON 状态，供台架和飞行日志使用。"""
        arm_msg = Bool()
        arm_msg.data = self.armed
        self.arm_state_pub.publish(arm_msg)

        status = {
            'stamp': self._now_seconds(),
            'dry_run': self.dry_run,
            'coordinate_mode': self.coordinate_mode,
            'connected': self.connected,
            'armed': self.armed,
            'guided': self.guided,
            'mode': self.mode,
            'system_status': self.system_status,
            'nav_state': self.if_nav,
            'passing_door': self.current_passing_door,
            'turning': self.if_turning,
            'ready_to_arm': self.ready_to_arm,
            'arm_request_inflight': self.arm_request_inflight,
            'mode_request_inflight': self.mode_request_inflight,
            'last_current_pose_age': self._age(self.last_current_pose_time),
            'last_cmd_vel_age': self._age(self.latest_cmd_vel_time),
            'last_target_pose_age': self._age(self.latest_target_pose_time),
            'last_setpoint_kind': self.last_setpoint_kind,
            'last_setpoint_age': self._age(self.last_setpoint_time),
            'topics': {
                'vision_pose': self.mavros_vision_pose_topic,
                'position_setpoint': self.mavros_position_setpoint_topic,
                'velocity_setpoint': self.mavros_velocity_setpoint_topic,
                'state': self.mavros_state_topic,
                'rc': self.mavros_rc_topic,
            },
        }
        msg = String()
        msg.data = json.dumps(status, ensure_ascii=True)
        self.status_pub.publish(msg)

    def _publish_velocity_setpoint(self):
        """根据缓存的 `/cmd_vel` 发布 MAVROS 速度 setpoint。"""
        if self.latest_cmd_vel is None:
            return
        if self._is_stale(self.latest_cmd_vel_time,
                          self.setpoint_stale_timeout):
            return
        if self._is_stale(self.last_current_pose_time,
                          self.pose_stale_timeout):
            return

        cmd = self.latest_cmd_vel
        if self.current_passing_door:
            target_height = self.passing_door_height
            current_height = self.current_height_feedback
            if self.coordinate_mode == LEGACY_COORDINATE_MODE:
                vx, vy, vz = legacy_passing_door_velocity(
                    cmd.linear.x, cmd.linear.y, current_height,
                    target_height, self.pid_height)
            else:
                vx = cmd.linear.x
                vy = cmd.linear.y
                vz = self.pid_height * (target_height - current_height)
        else:
            target_height = self.cruise_height
            current_height = self.current_fcu_height_feedback
            if self.coordinate_mode == LEGACY_COORDINATE_MODE:
                vx, vy, vz = legacy_nav_velocity(
                    cmd.linear.x, cmd.linear.y, current_height,
                    target_height, self.pid_height)
            else:
                vx = cmd.linear.x
                vy = cmd.linear.y
                vz = self.pid_height * (target_height - current_height)

        setpoint = TwistStamped()
        setpoint.header.stamp = self.get_clock().now().to_msg()
        setpoint.header.frame_id = self.frame_id
        setpoint.twist.linear.x = vx
        setpoint.twist.linear.y = vy
        setpoint.twist.linear.z = vz
        setpoint.twist.angular.x = 0.0
        setpoint.twist.angular.y = 0.0
        setpoint.twist.angular.z = 0.0
        self.velocity_setpoint_pub.publish(setpoint)
        self._mark_setpoint('velocity')

    def _publish_position_setpoint(self):
        """根据缓存的 `/robot/target_pose` 发布 MAVROS 位置 setpoint。"""
        if self.latest_target_pose is None:
            return
        if self._is_stale(self.latest_target_pose_time,
                          self.setpoint_stale_timeout):
            return

        target = self.latest_target_pose
        if self.coordinate_mode == LEGACY_COORDINATE_MODE:
            x, y, z = legacy_target_position(
                target.transform.translation.x,
                target.transform.translation.y,
                target.transform.translation.z,
                self.fcu_reference_z_offset)
            yaw_ned = 1.57 if self.current_passing_door and self.if_turning \
                else 0.0
            yaw = mavros_enu_yaw_for_legacy_ned_yaw(yaw_ned)
        else:
            x = target.transform.translation.x
            y = target.transform.translation.y
            z = target.transform.translation.z - self.fcu_reference_z_offset
            yaw = 1.57 if self.current_passing_door and self.if_turning \
                else 0.0

        qx, qy, qz, qw = quaternion_from_euler_xyzw(0.0, 0.0, yaw)
        setpoint = PoseStamped()
        setpoint.header.stamp = self.get_clock().now().to_msg()
        setpoint.header.frame_id = self.frame_id
        setpoint.pose.position.x = x
        setpoint.pose.position.y = y
        setpoint.pose.position.z = z
        setpoint.pose.orientation.x = qx
        setpoint.pose.orientation.y = qy
        setpoint.pose.orientation.z = qz
        setpoint.pose.orientation.w = qw
        self.position_setpoint_pub.publish(setpoint)
        self._mark_setpoint('position')

    def _mark_setpoint(self, kind):
        self.last_setpoint_kind = kind
        self.last_setpoint_time = self._now_seconds()

    def _is_stale(self, stamp_seconds, timeout_seconds):
        """判断缓存输入是否已经过期，不应继续驱动飞控。"""
        if stamp_seconds is None:
            return True
        return self._age(stamp_seconds) > timeout_seconds

    def _rc_arm_combo(self, channels):
        """检查配置的四通道高低位 RC 解锁组合。"""
        if len(self.rc_arm_channels) != 4:
            return False
        indexes = [int(ch) - 1 for ch in self.rc_arm_channels]
        if any(index < 0 or index >= len(channels) for index in indexes):
            return False

        values = [int(channels[index]) for index in indexes]
        return (
            values[0] < self.rc_arm_low_threshold
            and values[1] > self.rc_arm_high_threshold
            and values[2] < self.rc_arm_low_threshold
            and values[3] > self.rc_arm_high_threshold
        )

    def _request_arm(self):
        """在 dry-run 和安全门限允许时，通过 MAVROS 请求解锁。"""
        now = self._now_seconds()
        if now - self.last_arm_request_time < self.arm_retry_period:
            return
        self.last_arm_request_time = now

        if self.dry_run:
            self.get_logger().info('dry-run: would call MAVROS arming service')
            return
        if not self.connected:
            self.get_logger().warn(
                'Cannot arm: MAVROS is not connected to the FCU')
            return
        if self.arm_request_inflight:
            return
        if not self.arming_client.wait_for_service(timeout_sec=0.0):
            self.get_logger().warn(
                'Cannot arm: MAVROS arming service is not available')
            return

        request = CommandBool.Request()
        request.value = True
        future = self.arming_client.call_async(request)
        self.arm_request_inflight = True
        future.add_done_callback(self._arm_response_callback)

    def _arm_response_callback(self, future):
        """记录 MAVROS 异步解锁响应。"""
        self.arm_request_inflight = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn('Arming service failed: %s' % exc)
            return

        if response.success:
            self.get_logger().info('MAVROS arming request accepted')
        else:
            self.get_logger().warn(
                'MAVROS arming request rejected: result=%s'
                % response.result)

    def _request_mode_if_needed(self):
        """在已有 setpoint 流后，请求期望的 MAVROS 模式。"""
        if not self.connected or self.mode == self.desired_mode:
            return
        if self.last_setpoint_time is None:
            return

        now = self._now_seconds()
        if now - self.last_mode_request_time < self.mode_retry_period:
            return
        self.last_mode_request_time = now

        if self.dry_run:
            self.get_logger().info(
                'dry-run: would set MAVROS mode to %s' % self.desired_mode)
            return
        if self.mode_request_inflight:
            return
        if not self.set_mode_client.wait_for_service(timeout_sec=0.0):
            self.get_logger().warn(
                'Cannot set mode: MAVROS set_mode service is not available')
            return

        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = self.desired_mode
        future = self.set_mode_client.call_async(request)
        self.mode_request_inflight = True
        future.add_done_callback(self._mode_response_callback)

    def _mode_response_callback(self, future):
        """记录 MAVROS 异步 set_mode 响应。"""
        self.mode_request_inflight = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn('SetMode service failed: %s' % exc)
            return

        if response.mode_sent:
            self.get_logger().info(
                'MAVROS mode request sent: %s' % self.desired_mode)
        else:
            self.get_logger().warn(
                'MAVROS mode request rejected: %s' % self.desired_mode)


def main(args=None):
    rclpy.init(args=args)
    node = MavrosAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
