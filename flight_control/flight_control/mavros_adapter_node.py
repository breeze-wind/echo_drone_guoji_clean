#!/usr/bin/env python3
"""把旧 `/robot/*` 飞控话题桥接到 MAVROS。

仓库内其他节点仍使用比赛旧版话题和坐标轴约定。本节点作为飞控传输边界：
把这些消息转换成 MAVROS 的 vision_pose 和 setpoint 话题，把 MAVROS 解锁
状态回写到 `/robot/arm_state`，并用 dry-run 和参数开关限制解锁/模式服务
调用，方便台架调试。
"""

import json
import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, TwistStamped
from mavros_msgs.msg import PositionTarget, RCIn, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from .conversions import (
    LEGACY_COORDINATE_MODE,
    MAVROS_ENU_COORDINATE_MODE,
    body_velocity_to_map,
    euler_from_quaternion_msg,
    heading_hold_yaw_rate,
    legacy_target_position,
    legacy_vision_orientation_to_mavros_enu,
    legacy_vision_pose_position,
    mavros_enu_yaw_for_legacy_ned_yaw,
    nav2_velocity_to_mavros_enu,
    quaternion_from_euler_xyzw,
)
from .teb_position_tracking import advance_target, preview_on_path
from .wall_escape import (
    door_lateral_direction, escape_direction, grid_obstacles)


class MavrosAdapter(Node):
    """旧 behavior_control 与 MAVROS 之间的兼容适配器。"""

    def __init__(self, *, parameter_overrides=None):
        super().__init__('mavros_adapter', parameter_overrides=parameter_overrides)

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
        self.hold_heading = self._bool_param('hold_heading', True)
        self.heading_hold_kp = float(self._param('heading_hold_kp', 0.8))
        self.heading_hold_max_rate = float(
            self._param('heading_hold_max_rate', 0.12))
        if self.heading_hold_kp < 0.0 or self.heading_hold_max_rate <= 0.0:
            raise ValueError('heading hold gain and max rate must be nonnegative/positive')

        # 位姿过期时停止发布 setpoint；Nav2 速度命令过期时改发零水平速度，
        # 以便空闲/到点后仍维持 Offboard 设定点流。
        self.status_period = float(self._param('status_period', 1.0))
        self.setpoint_republish_period = float(
            self._param('setpoint_republish_period', 0.1))
        self.setpoint_stale_timeout = float(
            self._param('setpoint_stale_timeout', 1.0))
        self.pose_stale_timeout = float(
            self._param('pose_stale_timeout', 1.0))
        # Experimental alternative to forwarding Nav2's cmd_vel. Disabled by
        # default; one position-setpoint stream follows a short TEB preview.
        self.teb_position_follow_enabled = self._bool_param(
            'teb_position_follow_enabled', False)
        # Experimental PX4 position + velocity feed-forward mode. Both fields
        # are sent in one raw-local MAVROS message by this adapter (never two
        # competing setpoint streams).
        self.teb_position_velocity_follow_enabled = self._bool_param(
            'teb_position_velocity_follow_enabled', True)
        self.teb_position_velocity_feedforward_max = float(self._param(
            'teb_position_velocity_feedforward_max', 0.15))
        if (self.teb_position_follow_enabled and
                self.teb_position_velocity_follow_enabled):
            raise ValueError(
                'pure TEB position-follow and position+velocity-follow are mutually exclusive')
        if (self.teb_position_velocity_follow_enabled and
                self.teb_position_velocity_feedforward_max <= 0):
            raise ValueError('TEB velocity feed-forward max must be positive')
        self.teb_local_plan_topic = str(self._param(
            'teb_local_plan_topic', '/local_plan'))
        self.teb_position_lookahead = float(self._param(
            'teb_position_lookahead', 0.12))
        self.teb_position_target_speed = float(self._param(
            'teb_position_target_speed', 0.15))
        self.teb_position_max_lead = float(self._param(
            'teb_position_max_lead', 0.12))
        self.teb_position_max_cross_track = float(self._param(
            'teb_position_max_cross_track', 0.08))
        self.teb_position_plan_timeout = float(self._param(
            'teb_position_plan_timeout', 0.7))
        if self.teb_position_follow_enabled and (
                self.teb_position_lookahead <= 0 or
                self.teb_position_target_speed <= 0 or
                self.teb_position_max_lead <= 0 or
                self.teb_position_max_cross_track <= 0 or
                self.teb_position_plan_timeout <= 0):
            raise ValueError('TEB position-follow limits must all be positive')

        # Optional bounded recovery. The door mode is a very short lateral
        # pulse because it intentionally does not check the opposite wall.
        self.wall_escape_enabled = self._bool_param(
            'wall_escape_enabled', False)
        self.wall_escape_costmap_topic = str(self._param(
            'wall_escape_costmap_topic', '/local_costmap/costmap'))
        self.wall_escape_map_timeout = float(self._param(
            'wall_escape_map_timeout', 0.7))
        self.wall_escape_stall_sec = float(self._param(
            'wall_escape_stall_sec', 2.0))
        self.wall_escape_stall_distance = float(self._param(
            'wall_escape_stall_distance', 0.04))
        self.wall_escape_min_cmd_speed = float(self._param(
            'wall_escape_min_cmd_speed', 0.08))
        self.wall_escape_robot_radius = float(self._param(
            'wall_escape_robot_radius', 0.32))
        self.wall_escape_trigger_clearance = float(self._param(
            'wall_escape_trigger_clearance', 0.10))
        self.wall_escape_step = float(self._param(
            'wall_escape_step', 0.10))
        self.wall_escape_min_gain = float(self._param(
            'wall_escape_min_gain', 0.05))
        self.wall_escape_speed = float(self._param(
            'wall_escape_speed', 0.08))
        self.wall_escape_pulse_sec = float(self._param(
            'wall_escape_pulse_sec', 1.0))
        self.wall_escape_door_speed = float(self._param(
            'wall_escape_door_speed', 0.04))
        self.wall_escape_door_pulse_sec = float(self._param(
            'wall_escape_door_pulse_sec', 0.5))
        self.wall_escape_door_max_pulses = int(self._param(
            'wall_escape_door_max_pulses', 1))
        self.wall_escape_cooldown_sec = float(self._param(
            'wall_escape_cooldown_sec', 5.0))
        wall_escape_limits = (
            self.wall_escape_map_timeout, self.wall_escape_stall_sec,
            self.wall_escape_stall_distance, self.wall_escape_min_cmd_speed,
            self.wall_escape_robot_radius, self.wall_escape_trigger_clearance,
            self.wall_escape_step, self.wall_escape_min_gain,
            self.wall_escape_speed, self.wall_escape_pulse_sec,
            self.wall_escape_door_speed, self.wall_escape_door_pulse_sec,
            self.wall_escape_cooldown_sec)
        if self.wall_escape_enabled and (
                self.teb_position_follow_enabled or
                self.teb_position_velocity_follow_enabled or
                min(wall_escape_limits) <= 0 or
                self.wall_escape_door_max_pulses < 1):
            raise ValueError(
                'wall escape requires velocity mode and positive limits')

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
                        '/mavros/mavros/pose'))
        self.mavros_position_setpoint_topic = str(
            self._param('mavros_position_setpoint_topic',
                        '/mavros/mavros/local'))
        self.mavros_velocity_setpoint_topic = str(
            self._param('mavros_velocity_setpoint_topic',
                        '/mavros/mavros/cmd_vel'))
        self.mavros_raw_setpoint_topic = str(self._param(
            'mavros_raw_setpoint_topic',
            '/mavros/mavros/local'))
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
        self.raw_setpoint_pub = self.create_publisher(
            PositionTarget, self.mavros_raw_setpoint_topic, 10)

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
        self.teb_tf_buffer = None
        self.teb_tf_listener = None
        self.teb_local_plan_sub = None
        if (self.teb_position_follow_enabled or
                self.teb_position_velocity_follow_enabled or
                self.wall_escape_enabled):
            self.teb_tf_buffer = Buffer()
            self.teb_tf_listener = TransformListener(self.teb_tf_buffer, self)
        if (self.teb_position_follow_enabled or
                self.teb_position_velocity_follow_enabled):
            self.teb_local_plan_sub = self.create_subscription(
                Path, self.teb_local_plan_topic, self.teb_local_plan_callback, 10)
        self.wall_escape_costmap_sub = None
        if self.wall_escape_enabled:
            self.wall_escape_costmap_sub = self.create_subscription(
                OccupancyGrid, self.wall_escape_costmap_topic,
                self.wall_escape_costmap_callback, 10)

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

        # 最近输入样本及其年龄会发布到 status_topic，便于调试。
        self.current_height_feedback = None
        self.current_fcu_height_feedback = None
        self.last_current_pose_time = None
        self.latest_cmd_vel = None
        self.latest_cmd_vel_time = None
        self.latest_target_pose = None
        self.latest_target_pose_time = None
        self.current_yaw_enu = None
        self.current_yaw_map = None
        self.hold_yaw_enu = None
        self.last_heading_hold_rate = 0.0
        self.last_setpoint_kind = 'none'
        self.last_setpoint_time = None
        self.current_nav_pose_map_xy = None
        self.latest_teb_path_xy = None
        self.latest_teb_path_time = None
        self.teb_position_target_xy = None
        self.teb_position_holding = True
        self.teb_position_last_update = None
        self.teb_position_cross_track = None
        self.teb_position_last_warning = 0.0
        self.wall_escape_pose_history = deque()
        self.wall_escape_points = None
        self.wall_escape_free_at = None
        self.wall_escape_map_time = None
        self.wall_escape_vector = None
        self.wall_escape_kind = None
        self.wall_escape_active_speed = 0.0
        self.wall_escape_door_pulses = 0
        self.wall_escape_until = 0.0
        self.wall_escape_next_allowed = 0.0

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
            'MAVROS adapter started: dry_run=%s coordinate_mode=%s '
            'teb_position_follow=%s teb_position_velocity_follow=%s'
            % (self.dry_run, self.coordinate_mode,
               self.teb_position_follow_enabled,
               self.teb_position_velocity_follow_enabled))

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
        previous_mode = self.mode
        self.connected = bool(msg.connected)
        self.armed = bool(msg.armed)
        self.guided = bool(msg.guided)
        self.mode = msg.mode
        self.system_status = int(msg.system_status)

        if self.mode != 'OFFBOARD':
            self.hold_yaw_enu = None
        elif previous_mode != 'OFFBOARD':
            self._capture_heading()

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
        if (self.teb_position_follow_enabled or
                self.teb_position_velocity_follow_enabled or
                self.wall_escape_enabled):
            self._update_teb_map_pose(msg)
            if (self.wall_escape_enabled and
                    self.current_nav_pose_map_xy is not None):
                now = time.monotonic()
                self.wall_escape_pose_history.append(
                    (now, *self.current_nav_pose_map_xy))
                while (self.wall_escape_pose_history and
                       now - self.wall_escape_pose_history[0][0] >
                       self.wall_escape_stall_sec + 0.3):
                    self.wall_escape_pose_history.popleft()
        _, _, self.current_yaw_map = euler_from_quaternion_msg(
            msg.transform.rotation)
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
        _, _, self.current_yaw_enu = euler_from_quaternion_msg(
            pose.pose.orientation)
        if self.mode == 'OFFBOARD' and self.hold_yaw_enu is None:
            self._capture_heading()
        self.vision_pose_pub.publish(pose)

    def _update_teb_map_pose(self, msg):
        """Use TF, rather than assuming Point-LIO odom XY equals map XY."""
        source_frame = msg.header.frame_id
        if not source_frame:
            self.current_nav_pose_map_xy = None
            return
        x = msg.transform.translation.x
        y = msg.transform.translation.y
        if source_frame == self.frame_id:
            self.current_nav_pose_map_xy = (x, y)
            return
        try:
            transform = self.teb_tf_buffer.lookup_transform(
                self.frame_id, source_frame, Time())
        except TransformException as exc:
            self.current_nav_pose_map_xy = None
            self._warn_teb_position('No %s <- %s TF: %s' %
                                    (self.frame_id, source_frame, exc))
            return
        _, _, yaw = euler_from_quaternion_msg(transform.transform.rotation)
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        self.current_nav_pose_map_xy = (
            tx + math.cos(yaw) * x - math.sin(yaw) * y,
            ty + math.sin(yaw) * x + math.cos(yaw) * y)

    def _warn_teb_position(self, message):
        now = time.monotonic()
        if now - self.teb_position_last_warning >= 2.0:
            self.get_logger().warn('TEB position follow: ' + message)
            self.teb_position_last_warning = now

    def teb_local_plan_callback(self, msg):
        """Cache the *feasible* TEB local plan; its z is not flight height."""
        if msg.header.frame_id != self.frame_id or len(msg.poses) < 2:
            self.latest_teb_path_xy = None
            self._warn_teb_position(
                'invalid/short local_plan or frame %r (expected %r)' %
                (msg.header.frame_id, self.frame_id))
            return
        points = [(pose.pose.position.x, pose.pose.position.y)
                  for pose in msg.poses]
        if not all(math.isfinite(value) for point in points for value in point):
            self.latest_teb_path_xy = None
            self._warn_teb_position('local_plan has non-finite XY')
            return
        self.latest_teb_path_xy = points
        self.latest_teb_path_time = time.monotonic()

    def wall_escape_costmap_callback(self, msg):
        if msg.header.frame_id != self.frame_id:
            self.wall_escape_points = None
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if (stamp <= 0 or
                abs(self._now_seconds() - stamp) >
                self.wall_escape_map_timeout):
            self.wall_escape_points = None
            return
        try:
            self.wall_escape_points, self.wall_escape_free_at = (
                grid_obstacles(msg))
        except ValueError as exc:
            self.wall_escape_points = None
            self.get_logger().warn('Wall escape costmap rejected: %s' % exc)
            return
        self.wall_escape_map_time = time.monotonic()

    def _wall_escape_override(self, cmd, cmd_is_fresh):
        """Return map-frame escape velocity, or None for Nav2's command."""
        if not self.wall_escape_enabled:
            return None
        now = time.monotonic()
        valid = (self.if_nav and self.armed and self.mode == 'OFFBOARD' and
                 cmd_is_fresh and self.current_nav_pose_map_xy is not None and
                 self.wall_escape_points is not None and
                 self.wall_escape_map_time is not None and
                 now - self.wall_escape_map_time <=
                 self.wall_escape_map_timeout)
        if not valid:
            self.wall_escape_vector = None
            self.wall_escape_kind = None
            return None
        if now < self.wall_escape_until and self.wall_escape_vector is not None:
            checked, kind = self._wall_escape_direction(cmd)
            if (checked is not None and
                    kind == self.wall_escape_kind and
                    checked[0] * self.wall_escape_vector[0] +
                    checked[1] * self.wall_escape_vector[1] > 0.95):
                return tuple(self.wall_escape_active_speed * v
                             for v in self.wall_escape_vector)
            self.wall_escape_vector = None
            self.wall_escape_kind = None
            self.get_logger().warn(
                'Wall escape stopped: obstacle geometry/clearance changed')
            return None
        self.wall_escape_vector = None
        self.wall_escape_kind = None
        if (now < self.wall_escape_next_allowed or
                math.hypot(cmd.linear.x, cmd.linear.y) <
                self.wall_escape_min_cmd_speed):
            return None
        history = self.wall_escape_pose_history
        if (len(history) < 2 or
                now - history[0][0] < self.wall_escape_stall_sec):
            return None
        x0, y0 = history[0][1:]
        if any(math.hypot(x - x0, y - y0) > self.wall_escape_stall_distance
               for _, x, y in history):
            return None
        direction, kind = self._wall_escape_direction(cmd)
        if direction is None:
            return None
        if (kind == 'door_lateral' and
                self.wall_escape_door_pulses >=
                self.wall_escape_door_max_pulses):
            return None
        self.wall_escape_vector = direction
        self.wall_escape_kind = kind
        door_escape = kind == 'door_lateral'
        if door_escape:
            self.wall_escape_door_pulses += 1
        self.wall_escape_active_speed = (
            self.wall_escape_door_speed if door_escape else
            self.wall_escape_speed)
        pulse_sec = (self.wall_escape_door_pulse_sec if door_escape else
                     self.wall_escape_pulse_sec)
        self.wall_escape_until = now + pulse_sec
        self.wall_escape_next_allowed = (
            self.wall_escape_until + self.wall_escape_cooldown_sec)
        self.get_logger().warn(
            'Wall escape (%s): stalled %.1fs; %.2fm/s for %.1fs '
            'toward map (%.2f, %.2f). Single pulse, then cooldown.' %
            (kind, self.wall_escape_stall_sec, self.wall_escape_active_speed,
             pulse_sec, direction[0], direction[1]))
        return tuple(self.wall_escape_active_speed * v for v in direction)

    def _wall_escape_direction(self, cmd):
        x, y = self.current_nav_pose_map_xy
        if self.current_yaw_map is None:
            return None, None
        cmd_x, cmd_y = body_velocity_to_map(
            cmd.linear.x, cmd.linear.y, self.current_yaw_map)
        if self.current_passing_door:
            direction = door_lateral_direction(
                self.wall_escape_points, x, y, cmd_x, cmd_y,
                self.wall_escape_robot_radius,
                self.wall_escape_trigger_clearance,
                self.wall_escape_free_at)
            return direction, 'door_lateral' if direction is not None else None
        direction = escape_direction(
            self.wall_escape_points, x, y, self.wall_escape_robot_radius,
            self.wall_escape_trigger_clearance, self.wall_escape_step,
            self.wall_escape_min_gain, self.wall_escape_free_at)
        if (direction is not None and
                direction[0] * cmd_x + direction[1] * cmd_y >= -1e-3):
            return direction, 'away_from_wall'
        return None, None

    def _capture_heading(self):
        if (not self.hold_heading or self.current_yaw_enu is None or
                self._is_stale(self.last_current_pose_time,
                               self.pose_stale_timeout)):
            return
        self.hold_yaw_enu = self.current_yaw_enu
        self.get_logger().info(
            'Holding OFFBOARD heading at %.1f deg ENU' %
            (self.hold_yaw_enu * 180.0 / 3.141592653589793))

    def cmd_vel_callback(self, msg):
        """缓存 Nav2 速度命令，并在导航模式下立即发布。"""
        self.latest_cmd_vel = msg
        self.latest_cmd_vel_time = self._now_seconds()
        if (self.if_nav and not self.teb_position_follow_enabled and
                not self.teb_position_velocity_follow_enabled):
            self._publish_velocity_setpoint()

    def target_pose_callback(self, msg):
        """缓存直接位置目标，并在非 Nav2 模式下立即发布。"""
        self.latest_target_pose = msg
        self.latest_target_pose_time = self._now_seconds()
        if not self.if_nav:
            self._publish_position_setpoint()

    def nav_state_callback(self, msg):
        if self.if_nav != bool(msg.data):
            self.teb_position_target_xy = None
            self.teb_position_holding = True
            self.latest_teb_path_xy = None
        self.if_nav = bool(msg.data)

    def passing_door_state_callback(self, msg):
        if self.current_passing_door and not bool(msg.data):
            self.wall_escape_door_pulses = 0
        self.current_passing_door = bool(msg.data)

    def setpoint_timer_callback(self):
        """重发最近有效 setpoint，并控制可选 MAVROS 服务调用。"""
        if self.if_nav:
            if self.teb_position_velocity_follow_enabled:
                self._publish_teb_position_velocity_setpoint()
            elif self.teb_position_follow_enabled:
                self._publish_teb_position_setpoint()
            else:
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
            'ready_to_arm': self.ready_to_arm,
            'arm_request_inflight': self.arm_request_inflight,
            'mode_request_inflight': self.mode_request_inflight,
            'last_current_pose_age': self._age(self.last_current_pose_time),
            'last_cmd_vel_age': self._age(self.latest_cmd_vel_time),
            'teb_position_follow_enabled': self.teb_position_follow_enabled,
            'teb_position_velocity_follow_enabled': (
                self.teb_position_velocity_follow_enabled),
            'wall_escape_enabled': self.wall_escape_enabled,
            'wall_escape_active': self.wall_escape_vector is not None,
            'wall_escape_map_age': (
                None if self.wall_escape_map_time is None else
                time.monotonic() - self.wall_escape_map_time),
            'teb_plan_age_monotonic': (
                None if self.latest_teb_path_time is None else
                time.monotonic() - self.latest_teb_path_time),
            'teb_cross_track_m': self.teb_position_cross_track,
            'teb_target_xy_map': self.teb_position_target_xy,
            'last_target_pose_age': self._age(self.latest_target_pose_time),
            'last_setpoint_kind': self.last_setpoint_kind,
            'last_setpoint_age': self._age(self.last_setpoint_time),
            'hold_heading': self.hold_heading,
            'current_yaw_enu': self.current_yaw_enu,
            'current_yaw_map': self.current_yaw_map,
            'hold_yaw_enu': self.hold_yaw_enu,
            'heading_hold_rate': self.last_heading_hold_rate,
            'topics': {
                'vision_pose': self.mavros_vision_pose_topic,
                'position_setpoint': self.mavros_position_setpoint_topic,
                'velocity_setpoint': self.mavros_velocity_setpoint_topic,
                'raw_setpoint': self.mavros_raw_setpoint_topic,
                'state': self.mavros_state_topic,
                'rc': self.mavros_rc_topic,
            },
        }
        msg = String()
        msg.data = json.dumps(status, ensure_ascii=True)
        self.status_pub.publish(msg)

    def _publish_velocity_setpoint(self):
        """转发新鲜 `/cmd_vel`；空闲/过期时发零水平速度保持悬停。"""
        if self._is_stale(self.last_current_pose_time,
                          self.pose_stale_timeout):
            self._note_setpoint_blocked('velocity_pose_stale')
            return

        cmd_is_fresh = (
            self.latest_cmd_vel is not None
            and not self._is_stale(self.latest_cmd_vel_time,
                                   self.setpoint_stale_timeout))
        # Nav2 到点或尚无目标时不能继续沿用最后一条非零速度；使用零速，
        # 同时保留下方已有的高度反馈控制。
        cmd = self.latest_cmd_vel if cmd_is_fresh else Twist()
        if self.current_yaw_map is None:
            self._note_setpoint_blocked('velocity_heading_missing')
            return
        # Nav2/TEB 的 Twist 在 livox 机体系下；MAVROS LOCAL_NED 则接收
        # 局部世界系速度。即使机头保持不转，也须用当前姿态做这一步变换。
        escape_velocity = self._wall_escape_override(cmd, cmd_is_fresh)
        if escape_velocity is None:
            vx, vy = nav2_velocity_to_mavros_enu(
                cmd.linear.x, cmd.linear.y, self.current_yaw_map,
                self.coordinate_mode)
        elif self.coordinate_mode == MAVROS_ENU_COORDINATE_MODE:
            vx, vy = escape_velocity
        else:
            vx, vy = -escape_velocity[1], escape_velocity[0]
        if self.current_passing_door:
            target_height = self.passing_door_height
            current_height = self.current_height_feedback
        else:
            target_height = self.cruise_height
            current_height = self.current_fcu_height_feedback

        vz = self.pid_height * (target_height - current_height)

        setpoint = TwistStamped()
        setpoint.header.stamp = self.get_clock().now().to_msg()
        setpoint.header.frame_id = self.frame_id
        setpoint.twist.linear.x = vx
        setpoint.twist.linear.y = vy
        setpoint.twist.linear.z = vz
        setpoint.twist.angular.x = 0.0
        setpoint.twist.angular.y = 0.0
        yaw_rate = 0.0
        if (self.hold_heading and self.hold_yaw_enu is not None and
              self.current_yaw_enu is not None):
            yaw_rate = heading_hold_yaw_rate(
                self.hold_yaw_enu, self.current_yaw_enu,
                self.heading_hold_kp, self.heading_hold_max_rate)
        setpoint.twist.angular.z = yaw_rate
        self.last_heading_hold_rate = yaw_rate
        self.velocity_setpoint_pub.publish(setpoint)
        self._mark_setpoint(
            'wall_escape' if escape_velocity is not None else
            ('velocity' if cmd_is_fresh else 'velocity_idle_zero'))

    def _publish_teb_position_setpoint(self):
        """Follow a short TEB preview with one slewed position-setpoint stream.

        This experimental mode never publishes a MAVROS velocity setpoint.
        A missing plan, stopped Nav2 command, excessive cross-track error or
        stale plan latches a position hold instead of advancing along the path.
        """
        if self._is_stale(self.last_current_pose_time,
                          self.pose_stale_timeout):
            self._note_setpoint_blocked('teb_position_pose_stale')
            return
        current = self.current_nav_pose_map_xy
        if current is None or not all(math.isfinite(v) for v in current):
            self._note_setpoint_blocked('teb_position_map_pose_missing')
            return

        now = time.monotonic()
        dt = (self.setpoint_republish_period if
              self.teb_position_last_update is None else
              max(0.0, min(0.2, now - self.teb_position_last_update)))
        self.teb_position_last_update = now
        cmd_fresh = (
            self.latest_cmd_vel is not None and
            not self._is_stale(self.latest_cmd_vel_time,
                               self.teb_position_plan_timeout))
        cmd_moving = (cmd_fresh and
                      math.hypot(self.latest_cmd_vel.linear.x,
                                 self.latest_cmd_vel.linear.y) > 0.005)
        plan_fresh = (
            self.latest_teb_path_xy is not None and
            self.latest_teb_path_time is not None and
            now - self.latest_teb_path_time <=
            self.teb_position_plan_timeout)
        preview = None
        if plan_fresh and cmd_moving:
            preview = preview_on_path(
                self.latest_teb_path_xy, current,
                self.teb_position_lookahead)

        can_advance = (preview is not None and
                       preview[1] <= self.teb_position_max_cross_track)
        if can_advance:
            self.teb_position_cross_track = preview[1]
            self.teb_position_holding = False
            self.teb_position_target_xy = advance_target(
                self.teb_position_target_xy, current, preview[0],
                self.teb_position_target_speed * dt,
                self.teb_position_max_lead)
        else:
            if preview is not None:
                self.teb_position_cross_track = preview[1]
                self._warn_teb_position(
                    'cross-track error %.3f m > %.3f m; holding position' %
                    (preview[1], self.teb_position_max_cross_track))
            else:
                self.teb_position_cross_track = None
            if (not self.teb_position_holding or
                    self.teb_position_target_xy is None):
                self.teb_position_target_xy = current
            self.teb_position_holding = True

        height = (self.passing_door_height if self.current_passing_door
                  else self.cruise_height)
        self._send_map_position_setpoint(
            self.teb_position_target_xy[0], self.teb_position_target_xy[1],
            height, 'teb_position' if can_advance else 'teb_position_hold')

    def _publish_teb_position_velocity_setpoint(self):
        """Publish one PX4 raw-local setpoint with position and velocity FF.

        The short-lead position target constrains the path; transformed and
        bounded Nav2 velocity is a feed-forward term. On stale/invalid input,
        hold the current map position and send zero XY feed-forward.
        """
        if self._is_stale(self.last_current_pose_time,
                          self.pose_stale_timeout):
            self._note_setpoint_blocked('teb_hybrid_pose_stale')
            return
        current = self.current_nav_pose_map_xy
        if current is None or not all(math.isfinite(v) for v in current):
            self._note_setpoint_blocked('teb_hybrid_map_pose_missing')
            return

        now = time.monotonic()
        dt = (self.setpoint_republish_period if
              self.teb_position_last_update is None else
              max(0.0, min(0.2, now - self.teb_position_last_update)))
        self.teb_position_last_update = now
        cmd_fresh = (
            self.latest_cmd_vel is not None and
            not self._is_stale(self.latest_cmd_vel_time,
                               self.teb_position_plan_timeout))
        cmd = self.latest_cmd_vel if cmd_fresh else Twist()
        cmd_moving = math.hypot(cmd.linear.x, cmd.linear.y) > 0.005
        plan_fresh = (
            self.latest_teb_path_xy is not None and
            self.latest_teb_path_time is not None and
            now - self.latest_teb_path_time <=
            self.teb_position_plan_timeout)
        preview = None
        if plan_fresh and cmd_moving:
            preview = preview_on_path(
                self.latest_teb_path_xy, current,
                self.teb_position_lookahead)
        can_advance = (preview is not None and
                       preview[1] <= self.teb_position_max_cross_track)

        if can_advance:
            self.teb_position_cross_track = preview[1]
            self.teb_position_holding = False
            self.teb_position_target_xy = advance_target(
                self.teb_position_target_xy, current, preview[0],
                self.teb_position_target_speed * dt,
                self.teb_position_max_lead)
            vx_map, vy_map = body_velocity_to_map(
                cmd.linear.x, cmd.linear.y, self.current_yaw_map)
            speed = math.hypot(vx_map, vy_map)
            max_speed = self.teb_position_velocity_feedforward_max
            if speed > max_speed:
                scale = max_speed / speed
                vx_map *= scale
                vy_map *= scale
        else:
            self.teb_position_cross_track = (
                None if preview is None else preview[1])
            if not self.teb_position_holding or self.teb_position_target_xy is None:
                self.teb_position_target_xy = current
            self.teb_position_holding = True
            vx_map = vy_map = 0.0

        height = (self.passing_door_height if self.current_passing_door
                  else self.cruise_height)
        position = self._map_position_to_mavros_enu(
            self.teb_position_target_xy[0], self.teb_position_target_xy[1],
            height)
        velocity = nav2_velocity_to_mavros_enu(
            vx_map, vy_map, 0.0, self.coordinate_mode)
        # MAVROS raw-local accepts ROS ENU fields and converts them to NED.
        # Keep all position and XY velocity fields active; retain position Z,
        # and intentionally ignore vertical velocity and acceleration.
        setpoint = PositionTarget()
        setpoint.header.stamp = self.get_clock().now().to_msg()
        setpoint.header.frame_id = self.frame_id
        setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        setpoint.type_mask = (
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE)
        (setpoint.position.x, setpoint.position.y,
         setpoint.position.z) = position
        setpoint.velocity.x, setpoint.velocity.y = velocity
        setpoint.velocity.z = 0.0
        yaw = self.current_yaw_enu or 0.0
        if self.hold_yaw_enu is not None:
            yaw = self.hold_yaw_enu
        setpoint.yaw = float(yaw)
        setpoint.yaw_rate = 0.0
        self.raw_setpoint_pub.publish(setpoint)
        self.last_heading_hold_rate = 0.0
        self._mark_setpoint(
            'teb_position_velocity' if can_advance else
            'teb_position_velocity_hold')

    def _publish_position_setpoint(self):
        """根据缓存的 `/robot/target_pose` 发布 MAVROS 位置 setpoint。"""
        if self.latest_target_pose is None:
            self._note_setpoint_blocked('position_target_missing')
            return
        if self._is_stale(self.latest_target_pose_time,
                          self.setpoint_stale_timeout):
            self._note_setpoint_blocked('position_target_stale')
            return

        target = self.latest_target_pose
        self._send_map_position_setpoint(
            target.transform.translation.x,
            target.transform.translation.y,
            target.transform.translation.z, 'position')

    def _send_map_position_setpoint(self, map_x, map_y, map_z, kind):
        """Convert a map position into the existing MAVROS local convention."""
        x, y, z = self._map_position_to_mavros_enu(map_x, map_y, map_z)

        if self.coordinate_mode == LEGACY_COORDINATE_MODE:
            yaw = mavros_enu_yaw_for_legacy_ned_yaw(0.0)
        else:
            yaw = 0.0

        if self.hold_heading and self.current_yaw_enu is not None:
            yaw = (self.hold_yaw_enu if self.hold_yaw_enu is not None
                   else self.current_yaw_enu)

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
        self._mark_setpoint(kind)

    def _map_position_to_mavros_enu(self, map_x, map_y, map_z):
        """Convert a map position to the ROS ENU values expected by MAVROS."""
        if self.coordinate_mode == LEGACY_COORDINATE_MODE:
            x, y, z = legacy_target_position(
                map_x, map_y, map_z, self.fcu_reference_z_offset)
        else:
            x = map_x
            y = map_y
            z = map_z - self.fcu_reference_z_offset
        return x, y, z

    def _mark_setpoint(self, kind):
        if self.last_setpoint_kind.startswith('blocked_'):
            self.get_logger().info(
                'Setpoint stream resumed: %s after %s' %
                (kind, self.last_setpoint_kind))
        self.last_setpoint_kind = kind
        self.last_setpoint_time = self._now_seconds()

    def _note_setpoint_blocked(self, reason):
        kind = 'blocked_' + reason
        if self.last_setpoint_kind != kind:
            self.get_logger().warning(
                'Setpoint stream stopped: %s (pose_age=%s s, target_age=%s s)' %
                (reason, self._age(self.last_current_pose_time),
                 self._age(self.latest_target_pose_time)))
        self.last_setpoint_kind = kind

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
