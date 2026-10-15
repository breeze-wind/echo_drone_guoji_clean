"""独立的蛇形搜索与投递决策节点，不修改也不依赖旧 behavior_control。

整体流程：等待飞手解锁/OFFBOARD并升至搜索高度 → 按航点蛇形搜索 →
确认靶标后导航到投递点并精确对位、下降、投递、爬升 → 继续搜索；
投递次数用尽或搜索航线结束后，导航到穿门起点、降低高度并连续导航到门后终点。

Nav2 负责搜索/投递接近/穿门的平面导航；位置 setpoint 负责起飞、
悬停确认、投递精对准/升降等阶段。飞控解锁和模式切换始终由飞手完成。
"""

import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped, TransformStamped
from mavros_msgs.msg import State as MavrosState
from nav_msgs.msg import Path
from mavros_msgs.srv import SetMode
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import Trigger
import yaml

from coverage_search.route import generate_lawnmower_route
from robot_interfaces.msg import TargetDetectionArray


class CoverageMissionNode(Node):
    # 穿门只向门后终点发送一次导航目标。
    (WAIT_START, WAIT_ARM, SEARCHING, DROP_APPROACH, DROP_ALIGN,
     DROP_DESCEND, DROP_DWELL, DROP_ASCEND, DOOR_APPROACH, DOOR_LOWER,
     DOOR_TEB_SWITCH, DOOR_TRAVERSE, DOOR_DONE, LANDING, FINISHED,
     TARGET_CONFIRM_HOLD, DOOR_LANDING_APPROACH, LANDING_ALIGN) = range(18)

    def __init__(self):
        """声明并读取配置，生成航线，建立 ROS 通信接口及任务初始状态。"""
        super().__init__('coverage_mission_node')
        # 区域、航线与起飞完成后的入口阶段配置。
        self.declare_parameter('corner_a', [])
        self.declare_parameter('corner_b', [])
        self.declare_parameter('lane_spacing', 0.7)
        self.declare_parameter('custom_waypoints_xy', [])
        self.declare_parameter('search_height', 1.0)
        self.declare_parameter('lane_axis', 'auto')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('target_topic', '/target_perception/targets')
        self.declare_parameter('waypoints_topic', '/coverage_search/waypoints')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('start_after_takeoff', 'search')
        self.declare_parameter('start_waypoint_index', 1)
        self.declare_parameter('start_service', '/coverage_mission/start')
        self.declare_parameter('nav_action', '/navigate_to_pose')
        self.declare_parameter('current_pose_topic', '/robot/current_pose')
        self.declare_parameter('pose_source_frame', 'odom')
        self.declare_parameter('arm_state_topic', '/robot/arm_state')
        self.declare_parameter('nav_state_topic', '/robot/nav_state')
        self.declare_parameter('landing_service', '/mavros/set_mode')
        self.declare_parameter('target_confirm_count', 5)
        self.declare_parameter('target_stability_radius', 0.12)
        self.declare_parameter('delivered_target_duplicate_radius', 0.5)
        self.declare_parameter('target_min_confidence', 0.45)
        self.declare_parameter('goal_timeout_sec', 180.0)
        self.declare_parameter('nav_retry_delay_sec', 1.0)
        self.declare_parameter('red_cross_hold_sec', 2.0)
        self.declare_parameter('max_deliveries', 3)
        self.declare_parameter('drop_offsets_xy', [-0.08, -0.08, -0.08, 0.08, 0.08, 0.08])
        self.declare_parameter('drop_commands', [2, 3, 4])
        self.declare_parameter('door_start_xy', [])
        self.declare_parameter('door_waypoints_xy', [])
        self.declare_parameter('door_via_point_tolerance', 0.25)
        self.declare_parameter('teb_via_points_topic', '/via_points')
        self.declare_parameter('door_end_xy', [])
        self.declare_parameter('landing_xy', [])
        self.declare_parameter('door_height', 0.90)
        self.declare_parameter('door_teb_profile_enabled', True)
        self.declare_parameter('door_teb_profile_file', '')
        self.declare_parameter(
            'teb_parameter_service', '/controller_server/set_parameters_atomically')
        self.declare_parameter('passing_door_state_topic', '/robot/passing_door_state')
        self.declare_parameter('drop_height', 0.50)
        self.declare_parameter('pose_z_offset', 0.39)
        self.declare_parameter('pose_stale_timeout_sec', 1.0)
        self.declare_parameter('target_pose_topic', '/robot/target_pose')
        self.declare_parameter('target_pose_child_frame', 'livox')
        self.declare_parameter('align_tolerance', 0.12)
        self.declare_parameter('height_tolerance', 0.10)
        self.declare_parameter('stable_pose_samples', 4)
        self.declare_parameter('actuator_topic', '/electromagnet/set_state')
        self.declare_parameter('actuator_status_topic', '/electromagnet/status')
        self.declare_parameter('actuator_output_enabled', False)
        self.declare_parameter('require_actuator_status', True)
        self.declare_parameter('actuator_status_timeout_sec', 2.0)
        self.declare_parameter('actuator_ack_timeout_sec', 4.0)
        self.declare_parameter('actuator_repeat_period_sec', 1.0)
        self.declare_parameter('drop_dwell_sec', 1.0)
        self.declare_parameter('mavros_state_topic', '/mavros/state')
        self.declare_parameter('require_offboard', True)

        get = self.get_parameter

        def list_parameter(name, *, allow_empty=False):
            value = get(name).value
            # ROS 2 Foxy can expose an explicitly empty YAML sequence ([]) as
            # None for an untyped parameter. Treat it as empty only for
            # optional lists; required geometry gets a useful field name.
            if value is None:
                if allow_empty:
                    return []
                raise ValueError('%s must be a non-empty list' % name)
            if not isinstance(value, (list, tuple)):
                raise ValueError('%s must be a YAML list' % name)
            return list(value)

        self.frame_id = str(get('frame_id').value)
        self.search_height = float(get('search_height').value)
        self.confirm_count = int(get('target_confirm_count').value)
        self.stability_radius = float(get('target_stability_radius').value)
        self.delivered_target_duplicate_radius = float(
            get('delivered_target_duplicate_radius').value)
        self.min_confidence = float(get('target_min_confidence').value)
        self.goal_timeout = float(get('goal_timeout_sec').value)
        self.nav_retry_delay = float(get('nav_retry_delay_sec').value)
        self.red_cross_hold = float(get('red_cross_hold_sec').value)
        self.max_deliveries = int(get('max_deliveries').value)
        self.drop_offsets = [float(v) for v in list_parameter('drop_offsets_xy')]
        self.drop_commands = [int(v) for v in list_parameter('drop_commands')]
        self.door_start = [float(v) for v in list_parameter('door_start_xy')]
        self.door_end = [float(v) for v in list_parameter('door_end_xy')]
        door_waypoint_values = [
            float(v) for v in list_parameter('door_waypoints_xy', allow_empty=True)]
        landing_values = [
            float(v) for v in list_parameter('landing_xy', allow_empty=True)]
        if door_waypoint_values:
            if (len(door_waypoint_values) % 2 != 0 or
                    not all(math.isfinite(v) for v in door_waypoint_values)):
                raise ValueError(
                    'door_waypoints_xy must be finite map-frame XY pairs')
            self.door_waypoints = list(zip(door_waypoint_values[0::2],
                                           door_waypoint_values[1::2]))
        else:
            # Backward compatibility: the old single door_end_xy becomes a
            # one-point ordered traversal route.
            self.door_waypoints = [(self.door_end[0], self.door_end[1])]
        if landing_values:
            if (len(landing_values) != 2 or
                    not all(math.isfinite(v) for v in landing_values)):
                raise ValueError('landing_xy must contain one finite map-frame XY pair')
            self.landing_xy = (landing_values[0], landing_values[1])
        else:
            raise ValueError(
                'landing_xy must explicitly specify a safe map-frame touchdown point')
        self.door_height = float(get('door_height').value)
        self.door_via_point_tolerance = float(
            get('door_via_point_tolerance').value)
        self.teb_via_points_topic = str(get('teb_via_points_topic').value)
        self.door_teb_profile_enabled = bool(
            get('door_teb_profile_enabled').value)
        self.door_teb_profile_file = str(get('door_teb_profile_file').value)
        self.teb_parameter_service = str(get('teb_parameter_service').value)
        self.drop_height = float(get('drop_height').value)
        self.pose_z_offset = float(get('pose_z_offset').value)
        self.pose_source_frame = str(get('pose_source_frame').value)
        self.pose_stale_timeout = float(get('pose_stale_timeout_sec').value)
        self.target_pose_topic = str(get('target_pose_topic').value)
        self.target_pose_child_frame = str(get('target_pose_child_frame').value)
        self.align_tolerance = float(get('align_tolerance').value)
        self.height_tolerance = float(get('height_tolerance').value)
        self.stable_pose_samples = int(get('stable_pose_samples').value)
        self.actuator_output_enabled = bool(get('actuator_output_enabled').value)
        self.require_actuator_status = bool(get('require_actuator_status').value)
        self.actuator_status_timeout = float(get('actuator_status_timeout_sec').value)
        self.actuator_ack_timeout = float(get('actuator_ack_timeout_sec').value)
        self.actuator_repeat_period = float(get('actuator_repeat_period_sec').value)
        self.drop_dwell = float(get('drop_dwell_sec').value)
        self.require_offboard = bool(get('require_offboard').value)
        self.auto_start = bool(get('auto_start').value)
        self.start_after_takeoff = str(get('start_after_takeoff').value).strip().lower()
        self.start_waypoint_index = int(get('start_waypoint_index').value)
        # 检查关键参数长度、数值范围以及配置的任务入口是否合法。
        if (self.confirm_count < 1 or self.stability_radius <= 0.0 or
                self.delivered_target_duplicate_radius <= 0.0 or
                not math.isfinite(self.delivered_target_duplicate_radius) or
                self.max_deliveries < 1 or
                len(self.drop_offsets) != 2 * self.max_deliveries or
                len(self.drop_commands) != self.max_deliveries or
                len(self.door_start) != 2 or
                (not door_waypoint_values and len(self.door_end) != 2) or
                not self.door_waypoints or
                self.drop_height < 0.0 or self.door_height < 0.0 or
                self.stable_pose_samples < 1 or
                self.pose_stale_timeout <= 0.0 or self.nav_retry_delay <= 0.0 or
                self.red_cross_hold <= 0.0 or self.actuator_repeat_period <= 0.0):
            raise ValueError(
                'invalid mission config: need door_start_xy, door route, landing_xy, '
                'the configured delivery offsets/commands, '
                'and positive tolerances/counts')
        if (self.door_via_point_tolerance <= 0.0 or
                not math.isfinite(self.door_via_point_tolerance)):
            raise ValueError('door_via_point_tolerance must be finite and positive')
        if self.start_after_takeoff not in ('search', 'door'):
            raise ValueError("start_after_takeoff must be 'search' or 'door'")
        coordinates_to_check = [self.door_start, *self.door_waypoints,
                                self.landing_xy]
        if self.door_end:
            coordinates_to_check.append(self.door_end)
        if not all(math.isfinite(value) for point in coordinates_to_check
                   for value in point) or not math.isfinite(self.door_height):
            raise ValueError('door coordinates and door_height must be finite')
        if math.hypot(self.door_waypoints[-1][0] - self.door_start[0],
                      self.door_waypoints[-1][1] - self.door_start[1]) < 0.5:
            raise ValueError(
                'door_start_xy and final door_waypoints_xy must be at least 0.5 m apart')
        self.door_teb_parameters = (
            self._load_door_teb_profile(self.door_teb_profile_file)
            if self.door_teb_profile_enabled else {})

        # 如果指定了手工航点，就不生成蛇形线；否则根据矩形角点和行距生成。
        custom_waypoint_values = [
            float(v) for v in list_parameter('custom_waypoints_xy', allow_empty=True)]
        if custom_waypoint_values:
            if (len(custom_waypoint_values) % 2 != 0 or
                    not all(math.isfinite(v) for v in custom_waypoint_values)):
                raise ValueError(
                    'custom_waypoints_xy must be a flat list of finite map-frame XY pairs')
            custom_points = list(zip(custom_waypoint_values[0::2],
                                     custom_waypoint_values[1::2]))
            self.route = []
            for index, (x, y) in enumerate(custom_points):
                # Heading is latched from the aircraft's first valid pose in
                # _pose_cb; route geometry must not imply a turn beforehand.
                self.route.append((x, y, self.search_height, 0.0))
            route_description = 'custom map-frame waypoints'
        else:
            self.route = generate_lawnmower_route(
                list_parameter('corner_a'), list_parameter('corner_b'),
                float(get('lane_spacing').value), self.search_height,
                str(get('lane_axis').value).lower())
            self.route = [(x, y, z, 0.0) for x, y, z, _ in self.route]
            route_description = 'generated serpentine route'
        ax, ay = [float(v) for v in list_parameter('corner_a')]
        bx, by = [float(v) for v in list_parameter('corner_b')]
        self.search_bounds = (min(ax, bx), max(ax, bx), min(ay, by), max(ay, by))
        if not 1 <= self.start_waypoint_index <= len(self.route):
            raise ValueError('start_waypoint_index must be within the route')
        self.get_logger().info('Using %s: %d waypoints in %s; '
                               'cruise altitude must match adapter config.'
                               % (route_description, len(self.route), self.frame_id))
        for i, (x, y, z, yaw) in enumerate(self.route, 1):
            self.get_logger().info(
                'waypoint[%02d] x=%.3f y=%.3f z=%.2f yaw=%.1f deg' %
                (i, x, y, z, math.degrees(yaw)))

        # 内部运行状态、导航异步请求、位姿新鲜度、目标确认和投递计数。
        self.state = self.WAIT_ARM if self.auto_start else self.WAIT_START
        self.armed = False
        self.offboard = False
        self.mavros_connected = False
        self.mavros_mode = ''
        self.current_pose = None
        self.current_pose_time = None
        self.current_altitude = None
        self.fixed_heading_yaw = None
        self.takeoff_xy = None
        self.pose_frame_warning_printed = False
        self.next_index = 0
        self.current_waypoint_index = None
        self.current_nav_goal = None
        self.last_status_print = 0.0
        self.goal_handle = None
        self.goal_started_at = None
        self.goal_request_pending = False
        self.goal_token = 0
        self.goal_kind = None
        self.door_waypoint_index = 0
        self.landing_hold_target = None
        self.nav_retry_at = 0.0
        self.nav_retry_count = 0
        self.cross_hold_until = None
        self.cross_hold_position = None
        self.paused_search_goal = None
        self.pending_confirmed_target = None
        self.after_cancel = None
        self.cancel_in_progress = False
        self.last_nav_wait_log = 0.0
        self.candidate_samples = []
        self.delivered_targets = []
        self.delivery_count = 0
        self.active_target = None
        self.drop_xy = None
        self.drop_started_at = None
        self.actuator_command_sent_at = None
        self.actuator_status = None
        self.actuator_status_received_at = None
        self.expected_ack_sequence = None
        self.last_actuator_repeat_at = None
        self.last_actuator_repeat_state = None
        self.stable_pose_count = 0
        self.landing_requested = False
        # 发布接口：nav_state 切换适配器的导航/位置控制路径；target_pose
        # 是直接位置目标；waypoints 供可视化；actuator_pub 控制电磁铁。
        self.nav_state_pub = self.create_publisher(
            Bool, str(get('nav_state_topic').value), 10)
        self.passing_door_pub = self.create_publisher(
            Bool, str(get('passing_door_state_topic').value), 10)
        self.waypoint_pub = self.create_publisher(
            PoseArray, str(get('waypoints_topic').value), 1)
        self.teb_via_points_pub = self.create_publisher(
            Path, self.teb_via_points_topic, 1)
        self.target_pose_pub = self.create_publisher(
            TransformStamped, self.target_pose_topic, 10)
        self.actuator_pub = self.create_publisher(
            Int32, str(get('actuator_topic').value), 10)
        # 订阅位姿、解锁状态、靶标检测、MAVROS 模式和电磁铁状态。
        self.create_subscription(
            Bool, str(get('arm_state_topic').value), self._arm_cb, 10)
        self.create_subscription(
            TransformStamped, str(get('current_pose_topic').value), self._pose_cb, 10)
        self.create_subscription(
            TargetDetectionArray, str(get('target_topic').value), self._target_cb, 10)
        self.create_subscription(
            MavrosState, str(get('mavros_state_topic').value), self._mavros_state_cb, 10)
        self.create_subscription(
            String, str(get('actuator_status_topic').value), self._actuator_status_cb, 10)
        # 可选的显式启动服务；auto_start=true 时节点启动即等待起飞条件。
        self.start_srv = self.create_service(
            Trigger, str(get('start_service').value), self._start_cb)
        self.nav_client = ActionClient(
            self, NavigateToPose, str(get('nav_action').value))
        self.land_client = self.create_client(
            SetMode, str(get('landing_service').value))
        self.teb_parameter_client = self.create_client(
            SetParametersAtomically, self.teb_parameter_service)
        self.teb_switch_future = None
        self.teb_switch_last_attempt = 0.0
        self.teb_switch_started = None
        self.teb_switch_attempts = 0
        self.timer = self.create_timer(0.1, self._tick)
        self._publish_route()
        if not self.actuator_output_enabled:
            self.get_logger().warning(
                '电磁铁投递输出未启用：命中并对准靶标后只打印投递信号，不会释放货物。')
        if self.auto_start:
            self.get_logger().info(
                '自动启动已启用：获得新鲜位姿后持续发布当前位置上方的巡航高度 setpoint；'
                '飞手手动解锁并切入 OFFBOARD、到达巡航高度后自动发送首个 Nav2 航点；'
                '本节点不会自动解锁或切入 OFFBOARD。')
        else:
            self.get_logger().info(
                '待命。确认区域角点、Nav2、感知、电磁铁与飞控均正常后，调用 '
                '%s 开始；收到新鲜位姿后将发布起飞 setpoint，本节点不会自动解锁或切入 OFFBOARD。'
                % str(get('start_service').value))
        if not door_waypoint_values:
            self.get_logger().warning(
                'door_waypoints_xy is empty; using legacy door_end_xy as a one-point route. '
                'Configure ordered map-frame XY pairs for the full doorway path.')
        self.get_logger().info(
            '起飞检查通过后进入阶段=%s，搜索起始航点=%d。' %
            (self.start_after_takeoff, self.start_waypoint_index))
        self.get_logger().info(
            '穿门顺序航点：start=(%.2f, %.2f)，traverse=%d 点，末点=(%.2f, %.2f)，'
            'landing=(%.2f, %.2f)，height=%.2f m' %
            (self.door_start[0], self.door_start[1], len(self.door_waypoints),
             self.door_waypoints[-1][0], self.door_waypoints[-1][1],
             self.landing_xy[0], self.landing_xy[1], self.door_height))
        self.get_logger().warning(
            '穿门固定航点要求当前 map/odom 与建图时水平对齐；'
            'Point-LIO 从零启动本身不保证全局重定位。门内动态障碍由 Nav2 实时代价地图处理。')

    def _load_door_teb_profile(self, path):
        """读取门洞 TEB 独立参数，只允许动态回调明确支持的数值参数。"""
        if not path:
            raise ValueError('door_teb_profile_file is required')
        if not os.path.isfile(path):
            raise ValueError('door TEB profile does not exist: %s' % path)
        with open(path, 'r') as profile_file:
            document = yaml.safe_load(profile_file) or {}
        parameters = document.get('FollowPath')
        if not isinstance(parameters, dict) or not parameters:
            raise ValueError('door TEB profile must contain a non-empty FollowPath mapping')
        supported = {
            'max_vel_x', 'max_vel_y', 'max_vel_x_backwards', 'max_vel_theta',
            'acc_lim_x', 'acc_lim_y', 'acc_lim_theta', 'min_obstacle_dist',
            'inflation_dist', 'weight_optimaltime', 'weight_obstacle',
            'weight_inflation', 'weight_viapoint', 'weight_acc_lim_x',
            'weight_acc_lim_y', 'weight_acc_lim_theta', 'weight_max_vel_x',
            'weight_max_vel_y', 'weight_max_vel_theta', 'penalty_epsilon',
        }
        unknown = sorted(set(parameters) - supported)
        if unknown:
            raise ValueError('unsupported door TEB parameters: %s' % ', '.join(unknown))
        checked = {}
        for name, value in parameters.items():
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError('door TEB parameter %s must be numeric' % name)
            value = float(value)
            if not math.isfinite(value):
                raise ValueError('door TEB parameter %s must be finite' % name)
            checked['FollowPath.' + name] = value
        return checked

    def _publish_route(self):
        """把当前地图系路线发布为 PoseArray，供 RViz/其他节点查看。"""
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        for x, y, z, yaw in self.route:
            pose = PoseStamped().pose
            pose.position.x, pose.position.y, pose.position.z = x, y, z
            pose.orientation.z = math.sin(yaw / 2.0)
            pose.orientation.w = math.cos(yaw / 2.0)
            msg.poses.append(pose)
        self.waypoint_pub.publish(msg)

    def _start_cb(self, request, response):
        """响应显式开始请求；电磁铁启用时须先确认其在线且可用。"""
        del request
        if self.state != self.WAIT_START:
            response.success = False
            response.message = 'mission is already started or finished'
            return response
        if (self.actuator_output_enabled and self.require_actuator_status
                and not self._actuator_ready()):
            response.success = False
            response.message = (
                'electromagnet must report ready=true, connected=true, dry_run=false; '
                'start electromagnet-real and verify its MCU state first')
            self.get_logger().error(response.message)
            return response
        self.state = self.WAIT_ARM
        response.success = True
        response.message = 'mission accepted; will stream a takeoff setpoint after fresh pose arrives'
        self.get_logger().warning(response.message)
        return response

    def _pose_is_fresh(self):
        """判断是否在超时窗口内收到有效的当前位姿和高度。"""
        return (self.current_pose_time is not None and self.current_pose is not None
                and self.current_altitude is not None
                and time.monotonic() - self.current_pose_time < self.pose_stale_timeout)

    def _state_name(self, state):
        """返回需要显示给操作员的导航阶段名称。"""
        return {
            self.SEARCHING: 'SEARCHING',
            self.DROP_APPROACH: 'DROP_APPROACH',
            self.DOOR_APPROACH: 'DOOR_APPROACH',
            self.DOOR_LOWER: 'DOOR_LOWER',
            self.DOOR_TEB_SWITCH: 'DOOR_TEB_SWITCH',
            self.DOOR_TRAVERSE: 'DOOR_TRAVERSE',
        }.get(state, str(state))

    def _arm_cb(self, msg):
        """缓存飞控适配器发布的解锁状态。"""
        self.armed = bool(msg.data)

    def _pose_cb(self, msg):
        """缓存 LIO 位姿；首次收到时锁定机头方向并换算任务高度。"""
        source_frame = msg.header.frame_id
        if source_frame and source_frame not in (self.frame_id, self.pose_source_frame):
            if not self.pose_frame_warning_printed:
                self.get_logger().warning(
                    'Ignoring /robot/current_pose in frame %s; expected %s or %s' %
                    (source_frame, self.frame_id, self.pose_source_frame))
                self.pose_frame_warning_printed = True
            return
        self.current_pose = msg
        self.current_pose_time = time.monotonic()
        if self.fixed_heading_yaw is None:
            q = msg.transform.rotation
            self.fixed_heading_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            # Search should translate sideways/along lanes without asking the
            # aircraft to point along each segment. Keep every Nav2 goal and
            # direct position target at the heading observed on first pose.
            self.route = [(x, y, z, self.fixed_heading_yaw)
                          for x, y, z, _ in self.route]
            self._publish_route()
            self.get_logger().info(
                '固定机头方向已锁定为初始航向 %.1f deg；搜索、投递和穿门航点均保持该方向。'
                % math.degrees(self.fixed_heading_yaw))
        # Point-LIO publishes the pose in odom. In this setup map->odom is a
        # pure +Z offset (0.39 m), with identical XY axes; use that configured
        # offset for altitude while retaining the matching XY coordinates.
        z_offset = self.pose_z_offset if source_frame == self.pose_source_frame else 0.0
        self.current_altitude = msg.transform.translation.z + z_offset

    def _mavros_state_cb(self, msg):
        """缓存 MAVROS 连接和飞行模式，判断当前是否处于 OFFBOARD。"""
        self.mavros_connected = bool(msg.connected)
        self.mavros_mode = str(msg.mode)
        self.offboard = bool(msg.connected and msg.mode == 'OFFBOARD')

    def _actuator_status_cb(self, msg):
        """解析电磁铁节点的 JSON 状态；格式异常时按无状态处理。"""
        try:
            self.actuator_status = json.loads(msg.data)
            self.actuator_status_received_at = time.monotonic()
        except (TypeError, ValueError):
            self.actuator_status = None
            self.actuator_status_received_at = None

    def _actuator_ready(self):
        """电磁铁必须连接、ready、非 dry-run 且状态消息没有过期。"""
        return bool(self.actuator_status
                    and self.actuator_status.get('ready') is True
                    and self.actuator_status.get('connected') is True
                    and self.actuator_status.get('dry_run') is False
                    and self.actuator_status_received_at is not None
                    and time.monotonic() - self.actuator_status_received_at
                    <= self.actuator_status_timeout)

    def _desired_actuator_state(self):
        """计算应维持的磁铁状态：初始吸附态，投递时用本次投递状态。"""
        if self.state == self.DROP_DWELL and self.delivery_count < len(self.drop_commands):
            return self.drop_commands[self.delivery_count]
        if self.delivery_count > 0:
            return self.drop_commands[min(self.delivery_count - 1, len(self.drop_commands) - 1)]
        return 1

    def _publish_actuator_keepalive(self):
        """周期性重发幂等状态命令，降低单条 ROS 消息丢失的影响。"""
        if not self.actuator_output_enabled:
            return
        if self.require_actuator_status and not self._actuator_ready():
            return

        now = time.monotonic()
        command = self._desired_actuator_state()
        changed = command != self.last_actuator_repeat_state
        due = (self.last_actuator_repeat_at is None or
               now - self.last_actuator_repeat_at >= self.actuator_repeat_period)
        if not changed and not due:
            return

        self.actuator_pub.publish(Int32(data=command))
        self.last_actuator_repeat_at = now
        self.last_actuator_repeat_state = command
        self.get_logger().warning(
            '持续发布电磁铁状态 state=%d（周期 %.2f 秒）' %
            (command, self.actuator_repeat_period))

    def _target_cb(self, msg):
        """筛选有效地图系检测，连续多帧稳定后接受目标并开始投递。"""
        if self.state not in (self.SEARCHING, self.TARGET_CONFIRM_HOLD):
            return
        if msg.header.frame_id != self.frame_id:
            self.get_logger().warning(
                'Ignoring detections in frame %s (expected %s)' %
                (msg.header.frame_id, self.frame_id))
            return
        valid = []
        for detection in msg.detections:
            if (math.isfinite(detection.confidence)
                    and detection.confidence >= self.min_confidence
                    and math.isfinite(detection.ground_position.x)
                    and math.isfinite(detection.ground_position.y)):
                valid.append(detection)
        valid = [item for item in valid
                 if not self._is_delivered_target(
                     item.ground_position.x, item.ground_position.y)]
        if not valid:
            self.candidate_samples.clear()
            self.pending_confirmed_target = None
            return

        red_crosses = [item for item in valid if item.class_id == 0]
        min_x, max_x, min_y, max_y = self.search_bounds
        red_crosses = [item for item in red_crosses
                       if min_x <= item.ground_position.x <= max_x
                       and min_y <= item.ground_position.y <= max_y]
        if (self.state == self.SEARCHING and red_crosses
                and self.delivery_count < self.max_deliveries):
            detection = max(red_crosses, key=lambda item: item.confidence)
            self._begin_red_cross_hold()
        elif self.state == self.TARGET_CONFIRM_HOLD:
            if not red_crosses:
                self.candidate_samples.clear()
                self.pending_confirmed_target = None
                return
            detection = max(red_crosses, key=lambda item: item.confidence)
        else:
            detection = max(valid, key=lambda item: item.confidence)

        sample = (detection.class_id, detection.class_name,
                  detection.ground_position.x, detection.ground_position.y,
                  detection.ground_position.z, detection.confidence)
        if self.candidate_samples and sample[0] != self.candidate_samples[-1][0]:
            self.candidate_samples.clear()
        self.candidate_samples.append(sample)
        self.candidate_samples = self.candidate_samples[-self.confirm_count:]
        if len(self.candidate_samples) < self.confirm_count:
            return
        xs = [s[2] for s in self.candidate_samples]
        ys = [s[3] for s in self.candidate_samples]
        mean_x, mean_y = sum(xs) / len(xs), sum(ys) / len(ys)
        if max(math.hypot(x - mean_x, y - mean_y) for x, y in zip(xs, ys)) > self.stability_radius:
            self.candidate_samples.pop(0)
            self.pending_confirmed_target = None
            return
        best = self.candidate_samples[-1]
        mean_z = sum(s[4] for s in self.candidate_samples) / len(self.candidate_samples)
        target = (mean_x, mean_y, mean_z, best[0], best[1], best[5])
        if self.state == self.TARGET_CONFIRM_HOLD:
            # Keep hovering for the full interval while observations arrive.
            self.pending_confirmed_target = target
            return
        self._accept_confirmed_target(target)

    def _begin_red_cross_hold(self):
        """红十字首次出现时取消搜索目标，保持当前位置并启动确认计时。"""
        pose_fresh = (self.current_pose_time is not None
                      and time.monotonic() - self.current_pose_time < self.pose_stale_timeout
                      and self.current_pose is not None
                      and self.current_altitude is not None)
        if not pose_fresh:
            self.get_logger().warning(
                'Red cross seen, but pose is stale; cannot safely pause for confirmation')
            return
        self.paused_search_goal = self.current_nav_goal
        self.cross_hold_until = time.monotonic() + self.red_cross_hold
        pose = self.current_pose.transform.translation
        self.cross_hold_position = (pose.x, pose.y, self.current_altitude)
        self.pending_confirmed_target = None
        self.candidate_samples.clear()
        self.nav_retry_at = 0.0
        self.state = self.TARGET_CONFIRM_HOLD
        self.nav_state_pub.publish(Bool(data=False))
        self._publish_position_target(*self.cross_hold_position)
        self.get_logger().warning(
            '红十字候选已发现；暂停当前搜索航点，在当前位置悬停 %.1f 秒并确认目标。'
            % self.red_cross_hold)
        self._cancel_goal_then(lambda: None)

    def _resume_search_after_cross_hold(self):
        """红十字未确认成功时，恢复搜索并重新发送暂停前的航点。"""
        if self.paused_search_goal and self.paused_search_goal[0] == 'search':
            self.next_index = max(0, int(self.paused_search_goal[1]) - 1)
        self.paused_search_goal = None
        self.cross_hold_until = None
        self.cross_hold_position = None
        self.pending_confirmed_target = None
        self.candidate_samples.clear()
        self.state = self.SEARCHING
        self.nav_state_pub.publish(Bool(data=True))
        self.get_logger().warning(
            '红十字未能稳定确认；恢复蛇形搜索并重发当前航点。')
        self._send_search_goal()

    def _accept_confirmed_target(self, target):
        """检查区域、重复位置和剩余货物；计算投递偏置并进入接近阶段。"""
        mean_x, mean_y, mean_z, class_id, class_name, confidence = target
        min_x, max_x, min_y, max_y = self.search_bounds
        if not (min_x <= mean_x <= max_x and min_y <= mean_y <= max_y):
            self.get_logger().warning(
                'Ignoring stable detection outside search rectangle: map=(%.3f, %.3f)' %
                (mean_x, mean_y))
            self.candidate_samples.clear()
            self.pending_confirmed_target = None
            return
        if self._is_delivered_target(mean_x, mean_y):
            self.get_logger().info(
                'Ignoring already delivered target at map=(%.3f, %.3f); '
                'duplicate radius=%.2f m' %
                (mean_x, mean_y, self.delivered_target_duplicate_radius))
            self.candidate_samples.clear()
            self.pending_confirmed_target = None
            return
        if self.delivery_count >= self.max_deliveries:
            self.candidate_samples.clear()
            self.pending_confirmed_target = None
            return

        self.active_target = (mean_x, mean_y, mean_z, class_id, class_name, confidence)
        offset_x = self.drop_offsets[2 * self.delivery_count]
        offset_y = self.drop_offsets[2 * self.delivery_count + 1]
        self.drop_xy = (mean_x + offset_x, mean_y + offset_y)
        self.get_logger().warning(
            'CONFIRMED target %d/%d: class=%s id=%d confidence=%.2f '
            'map=(%.3f, %.3f, %.3f); cargo offset=(%.3f, %.3f), '
            'drop point=(%.3f, %.3f)' %
            (self.delivery_count + 1, self.max_deliveries, class_name, class_id,
             confidence, mean_x, mean_y, mean_z, offset_x, offset_y,
             self.drop_xy[0], self.drop_xy[1]))
        self.candidate_samples.clear()
        self.pending_confirmed_target = None
        self.cross_hold_until = None
        self.cross_hold_position = None
        self.state = self.DROP_APPROACH
        self._cancel_goal_then(self._send_drop_goal)

    def _resume_search_after_delivery(self):
        """投递后恢复被靶标确认打断的搜索航点，而不是跳过它。"""
        if self.paused_search_goal and self.paused_search_goal[0] == 'search':
            # 搜索航点在发送时 next_index 已前移；将其退回暂停前的航点。
            self.next_index = max(0, int(self.paused_search_goal[1]) - 1)
            self.get_logger().warning(
                '投递完成；恢复被打断的搜索航点 %d/%d。' %
                (self.next_index + 1, len(self.route)))
        self.paused_search_goal = None
        self.nav_state_pub.publish(Bool(data=True))
        self.state = self.SEARCHING
        self.get_logger().info(
            'Resuming coverage for delivery %d/%d' %
            (self.delivery_count + 1, self.max_deliveries))
        self._send_search_goal()

    def _tick(self):
        """100 ms 状态机主循环：维持输出、检查安全条件并推进任务阶段。"""
        self._publish_actuator_keepalive()
        self._print_mission_status()
        # 起飞阶段持续发当前位置上方的 setpoint；飞手负责解锁和切 OFFBOARD。
        if self.state == self.WAIT_ARM:
            if (self.actuator_output_enabled and self.require_actuator_status
                    and not self._actuator_ready()):
                return
            pose_fresh = (self.current_pose_time is not None
                          and time.monotonic() - self.current_pose_time < self.pose_stale_timeout)
            if pose_fresh and self.current_pose is not None:
                pose = self.current_pose.transform.translation
                if self.takeoff_xy is None:
                    self.takeoff_xy = (pose.x, pose.y)
                    self.get_logger().warning(
                        'Takeoff setpoint latched at current map XY=(%.3f, %.3f), '
                        'cruise height=%.2f m; publishing continuously. '
                        'Pilot may now arm and select OFFBOARD.' %
                        (pose.x, pose.y, self.search_height))
                # PX4 needs a continuous setpoint stream before OFFBOARD. Keep
                # direct position control active during takeoff; never arm or
                # change flight mode automatically here.
                self.nav_state_pub.publish(Bool(data=False))
                self._publish_position_target(
                    self.takeoff_xy[0], self.takeoff_xy[1], self.search_height)
            mode_ok = self.offboard or not self.require_offboard
            height_ok = (self.current_altitude is not None and
                         abs(self.current_altitude - self.search_height) <= self.height_tolerance)
            if self.armed and mode_ok and pose_fresh and height_ok:
                self.stable_pose_count += 1
            else:
                self.stable_pose_count = 0
            if self.stable_pose_count >= self.stable_pose_samples:
                if self.start_after_takeoff == 'door':
                    self._start_door_approach()
                else:
                    self.next_index = self.start_waypoint_index - 1
                    self.state = self.SEARCHING
                    self.get_logger().warning(
                        'Takeoff checks passed; starting search at waypoint %d/%d.' %
                        (self.start_waypoint_index, len(self.route)))
                    self._send_search_goal()
            return
        # 所有飞行任务阶段都监视解锁和 OFFBOARD；丢失时转入 AUTO.LAND 请求。
        active_states = (self.SEARCHING, self.DROP_APPROACH, self.DROP_ALIGN,
                         self.DROP_DESCEND, self.DROP_DWELL, self.DROP_ASCEND,
                         self.DOOR_APPROACH, self.DOOR_LOWER, self.DOOR_TRAVERSE,
                         self.DOOR_TEB_SWITCH, self.DOOR_LANDING_APPROACH,
                         self.DOOR_DONE, self.LANDING_ALIGN,
                         self.TARGET_CONFIRM_HOLD)
        if self.state in active_states:
            if not self.armed or (self.require_offboard and not self.offboard):
                self._begin_landing(
                    'flight state lost: armed=%s MAVROS connected=%s mode=%s '
                    '(OFFBOARD required=%s)' %
                    (self.armed, self.mavros_connected, self.mavros_mode,
                     self.require_offboard))
                return
            # The adapter selects velocity vs. position setpoints with this
            # topic. Repeat it throughout navigation: a single lost transition
            # must not leave the adapter in position mode with an expired goal.
            if self.state in (self.SEARCHING, self.DROP_APPROACH,
                              self.DOOR_APPROACH, self.DOOR_TRAVERSE,
                              self.DOOR_LANDING_APPROACH):
                self.nav_state_pub.publish(Bool(data=True))
            else:
                # Keep direct position-control mode latched through alignment,
                # descent, the entire release dwell, and ascent.
                self.nav_state_pub.publish(Bool(data=False))
            # 穿门模式会让适配器在速度控制时锁定 passing_door_height。
            self.passing_door_pub.publish(Bool(data=self.state in (
                self.DOOR_LOWER, self.DOOR_TEB_SWITCH,
                self.DOOR_TRAVERSE, self.DOOR_LANDING_APPROACH)))
            if (self.goal_started_at and
                    time.monotonic() - self.goal_started_at > self.goal_timeout):
                kind = self.goal_kind
                self.get_logger().error('Navigation goal timed out; holding and retrying')
                self._cancel_goal_then(
                    lambda: self._schedule_nav_retry(kind, 'goal timeout'))
                return
            pose_fresh = (self.current_pose_time is not None
                          and time.monotonic() - self.current_pose_time < self.pose_stale_timeout)
            if self.state == self.DOOR_TRAVERSE and pose_fresh:
                self._update_door_via_progress()
            # 红十字确认期间固定位置悬停，等确认窗口结束后接受目标或恢复搜索。
            if self.state == self.TARGET_CONFIRM_HOLD:
                if not pose_fresh or self.current_pose is None or self.current_altitude is None:
                    self._begin_landing('pose stale during red-cross confirmation hover')
                    return
                self.nav_state_pub.publish(Bool(data=False))
                self._publish_position_target(*self.cross_hold_position)
                if (self.cancel_in_progress or self.cross_hold_until is None
                        or time.monotonic() < self.cross_hold_until):
                    return
                pending = self.pending_confirmed_target
                if pending is not None:
                    self._accept_confirmed_target(pending)
                    if self.state != self.TARGET_CONFIRM_HOLD:
                        return
                self._resume_search_after_cross_hold()
                return
            # Nav2 导航阶段由 action 完成/失败回调推进；失败时在当前位置悬停重试。
            if self.state == self.DOOR_LOWER:
                self.nav_state_pub.publish(Bool(data=False))
                self._publish_position_target(
                    self.door_start[0], self.door_start[1], self.door_height)
                if self._at_position(
                        self.door_start[0], self.door_start[1], self.door_height):
                    self.stable_pose_count += 1
                else:
                    self.stable_pose_count = 0
                if self.stable_pose_count >= self.stable_pose_samples:
                    self.stable_pose_count = 0
                    self.state = self.DOOR_TEB_SWITCH
                    self.teb_switch_started = time.monotonic()
                    self.teb_switch_attempts = 0
                    self.get_logger().warning(
                        '门口已稳定在 %.2f m；保持悬停并切换独立门洞 TEB 参数。' %
                        self.door_height)
                return
            if self.state == self.DOOR_TEB_SWITCH:
                self.nav_state_pub.publish(Bool(data=False))
                self.passing_door_pub.publish(Bool(data=True))
                self._publish_position_target(
                    self.door_start[0], self.door_start[1], self.door_height)
                if not self.door_teb_profile_enabled:
                    self.get_logger().warning(
                        '暂不切换门洞 TEB 参数，沿用普通 TEB；门点作为连续 via-point，'
                        'Nav2 目标为降落点。')
                    self._begin_door_traverse()
                    return
                self._request_door_teb_profile()
                return
            if self.state in (self.SEARCHING, self.DROP_APPROACH,
                              self.DOOR_APPROACH, self.DOOR_TRAVERSE,
                              self.DOOR_LANDING_APPROACH):
                if time.monotonic() < self.nav_retry_at:
                    return
                if (self.goal_handle is None and not self.goal_request_pending
                        and not self.cancel_in_progress):
                    if self.state == self.SEARCHING:
                        self._send_search_goal()
                    elif self.state == self.DROP_APPROACH:
                        self._send_drop_goal()
                    elif self.state == self.DOOR_APPROACH:
                        self._send_door_goal()
                    elif self.state == self.DOOR_LANDING_APPROACH:
                        self._send_landing_goal()
                    else:
                        self._send_door_traverse_goal()
                return
            if not pose_fresh or self.current_pose is None:
                self.get_logger().warning('Pose stale during precision/drop phase; aborting safely')
                self._begin_landing('pose stale during delivery')
                return
            if self.state == self.DOOR_DONE:
                # Nav2's goal has succeeded and will not keep producing a
                # corrective cmd_vel. Refresh the fixed exit pose every tick
                # so the adapter's 1 s target watchdog stays satisfied.
                self._publish_position_target(
                    self.door_end[0], self.door_end[1], self.door_height)
                return
            if self.state == self.LANDING_ALIGN:
                self.nav_state_pub.publish(Bool(data=False))
                if not pose_fresh or self.current_pose is None:
                    self._begin_landing('pose stale while aligning at final landing point')
                    return
                self._publish_position_target(*self.landing_hold_target)
                if self._at_position(*self.landing_hold_target):
                    self.stable_pose_count += 1
                else:
                    self.stable_pose_count = 0
                if self.stable_pose_count >= self.stable_pose_samples:
                    self.state = self.LANDING
                    self.landing_requested = False
                    self.get_logger().warning(
                        '指定降落点已稳定确认 %d 帧；现在请求 PX4 AUTO.LAND。' %
                        self.stable_pose_samples)
                return
            # 到达投递区域后切位置模式，先在搜索高度精确对准，再下降投递。
            if self.state == self.DROP_ALIGN:
                self.nav_state_pub.publish(Bool(data=False))
                self._publish_position_target(self.drop_xy[0], self.drop_xy[1], self.search_height)
                if self._at_position(self.drop_xy[0], self.drop_xy[1], self.search_height):
                    self.stable_pose_count += 1
                else:
                    self.stable_pose_count = 0
                if self.stable_pose_count >= self.stable_pose_samples:
                    self.stable_pose_count = 0
                    self.state = self.DROP_DESCEND
                    self.get_logger().info(
                        'Precisely aligned over drop point; descending to %.2f m' % self.drop_height)
                return
            if self.state == self.DROP_DESCEND:
                self._publish_position_target(self.drop_xy[0], self.drop_xy[1], self.drop_height)
                if self._at_position(self.drop_xy[0], self.drop_xy[1], self.drop_height):
                    self.stable_pose_count += 1
                else:
                    self.stable_pose_count = 0
                if self.stable_pose_count >= self.stable_pose_samples:
                    self.stable_pose_count = 0
                    command = self.drop_commands[self.delivery_count]
                    self.actuator_command_sent_at = time.monotonic()
                    self.state = self.DROP_DWELL
                    target = self.active_target
                    status = self.actuator_status or {}
                    self.expected_ack_sequence = int(status.get('ack_sequence', 0)) + 1
                    if self.actuator_output_enabled:
                        if self.require_actuator_status and not self._actuator_ready():
                            self._begin_landing('electromagnet not ready at drop point')
                            return
                        self.actuator_pub.publish(Int32(data=command))
                        self.get_logger().warning(
                            'At drop height; SENT electromagnet state %d (%d/%d), '
                            'target=%s id=%d map=(%.3f, %.3f)' %
                            (command, self.delivery_count + 1, self.max_deliveries,
                             target[4], target[3], target[0], target[1]))
                    else:
                        self.get_logger().warning(
                            '投递信号（仅打印，未发送电磁铁命令）：state=%d (%d/%d), '
                            'target=%s id=%d map=(%.3f, %.3f)' %
                            (command, self.delivery_count + 1, self.max_deliveries,
                             target[4], target[3], target[0], target[1]))
                return
            if self.state == self.DROP_DWELL:
                # The dwell can last several seconds. Refresh this hold target
                # every tick; otherwise the adapter expires it after 1 s and
                # PX4 can exit OFFBOARD before the release has completed.
                self._publish_position_target(
                    self.drop_xy[0], self.drop_xy[1], self.drop_height)
                command = self.drop_commands[self.delivery_count]
                elapsed = time.monotonic() - self.actuator_command_sent_at
                if self.actuator_output_enabled and self.require_actuator_status:
                    if not self._actuator_ready():
                        self._begin_landing(
                            'electromagnet disconnected, stale, or dry-run during delivery')
                        return
                    acknowledged = (
                        self.actuator_status.get('state') == command
                        and self.actuator_status.get('last_sent_command') == command
                        and int(self.actuator_status.get('ack_sequence', 0))
                        >= self.expected_ack_sequence
                        and self.actuator_status.get('pending_command') is None)
                    if not acknowledged and elapsed > self.actuator_ack_timeout:
                        self._begin_landing('electromagnet command was not acknowledged')
                        return
                    if not acknowledged:
                        return
                if elapsed >= self.drop_dwell:
                    self.delivery_count += 1
                    target_x, target_y = self.active_target[0:2]
                    target_class_id = self.active_target[3]
                    target_class_name = self.active_target[4]
                    self.delivered_targets.append((target_x, target_y))
                    if self.actuator_output_enabled:
                        self.get_logger().warning(
                            'Delivery %d/%d electromagnet state=%d acknowledged: type=%s id=%d '
                            'target=(%.3f, %.3f)' %
                            (self.delivery_count, self.max_deliveries, command,
                             target_class_name, target_class_id, target_x, target_y))
                    else:
                        self.get_logger().warning(
                        'Log-only delivery signal %d/%d recorded: type=%s id=%d '
                            'target=(%.3f, %.3f); no cargo release was actuated' %
                            (self.delivery_count, self.max_deliveries, target_class_name,
                             target_class_id, target_x, target_y))
                    self.state = self.DROP_ASCEND
                    self.stable_pose_count = 0
                return
            if self.state == self.DROP_ASCEND:
                self._publish_position_target(self.drop_xy[0], self.drop_xy[1], self.search_height)
                if self._at_position(self.drop_xy[0], self.drop_xy[1], self.search_height):
                    self.stable_pose_count += 1
                else:
                    self.stable_pose_count = 0
                if self.stable_pose_count >= self.stable_pose_samples:
                    self.stable_pose_count = 0
                    self.drop_started_at = None
                    if self.delivery_count >= self.max_deliveries:
                        self._begin_door_leg()
                    else:
                        self._resume_search_after_delivery()
                return
        if self.state == self.LANDING:
            if self.landing_hold_target is not None:
                self.nav_state_pub.publish(Bool(data=False))
                self.passing_door_pub.publish(Bool(data=False))
                self._publish_position_target(*self.landing_hold_target)
            if not self.landing_requested and self.land_client.service_is_ready():
                request = SetMode.Request()
                request.base_mode = 0
                request.custom_mode = 'AUTO.LAND'
                future = self.land_client.call_async(request)
                self.landing_requested = True
                future.add_done_callback(self._land_result)

    def _at_position(self, x, y, z):
        """判断当前 XY 距离和高度误差是否均进入配置容差。"""
        pose = self.current_pose.transform.translation
        return (math.hypot(pose.x - x, pose.y - y) <= self.align_tolerance
                and self.current_altitude is not None
                and abs(self.current_altitude - z) <= self.height_tolerance)

    def _print_mission_status(self):
        """每秒打印一次任务阶段、航点/目标和投递进度。"""
        now = time.monotonic()
        if now - self.last_status_print < 1.0:
            return
        self.last_status_print = now

        if self.state == self.WAIT_START:
            message = '任务阶段：待命，等待开始指令'
        elif self.state == self.WAIT_ARM:
            actuator_ok = (not self.actuator_output_enabled
                           or not self.require_actuator_status
                           or self._actuator_ready())
            pose_fresh = (self.current_pose_time is not None
                          and now - self.current_pose_time < self.pose_stale_timeout)
            altitude = ('%.2f' % self.current_altitude
                        if pose_fresh and self.current_altitude is not None else '未知')
            if not actuator_ok:
                status = self.actuator_status or {}
                message = ('任务阶段：等待电磁铁就绪；请启动 electromagnet-real，'
                           '检查 MCU 状态（connected=%s ready=%s dry_run=%s）' %
                           (status.get('connected', False), status.get('ready', False),
                            status.get('dry_run', 'unknown')))
            elif not pose_fresh:
                message = ('任务阶段：等待 /robot/current_pose（Point-LIO）；'
                           '尚未锁定起飞位置、未发布起飞 setpoint')
            else:
                message = ('任务阶段：等待提升到搜索高度 %.2f m；当前高度=%s m，'
                           'armed=%s，OFFBOARD=%s' %
                           (self.search_height, altitude, self.armed, self.offboard))
        elif self.state == self.SEARCHING:
            if self.current_nav_goal and self.current_nav_goal[0] == 'search':
                index, x, y = self.current_nav_goal[1:]
                message = ('任务阶段：蛇形搜索，航点 %d/%d，坐标=(%.2f, %.2f)' %
                           (index, len(self.route), x, y))
            else:
                message = '任务阶段：蛇形搜索，等待发送下一个航点'
        elif self.state == self.TARGET_CONFIRM_HOLD:
            remain = (max(0.0, self.cross_hold_until - now)
                      if self.cross_hold_until is not None else self.red_cross_hold)
            count = len(self.candidate_samples)
            message = ('任务阶段：红十字候选悬停确认，剩余 %.1f 秒，确认帧=%d/%d' %
                       (remain, count, self.confirm_count))
        elif self.state in (self.DROP_APPROACH, self.DROP_ALIGN,
                            self.DROP_DESCEND, self.DROP_DWELL, self.DROP_ASCEND):
            target = self.active_target
            if target is not None:
                phase_names = {
                    self.DROP_APPROACH: '发现靶标，前往投递点',
                    self.DROP_ALIGN: '对准靶标',
                    self.DROP_DESCEND: '下降投递',
                    self.DROP_DWELL: '投递处理中',
                    self.DROP_ASCEND: '投递后爬升',
                }
                delivery_number = (self.delivery_count if self.state == self.DROP_ASCEND
                                   else self.delivery_count + 1)
                message = ('任务阶段：%s，靶标=%s id=%d，坐标=(%.2f, %.2f)，'
                           '投递=%d/%d' %
                           (phase_names[self.state], target[4], target[3],
                            target[0], target[1], delivery_number,
                            self.max_deliveries))
            else:
                message = '任务阶段：投递处理，投递=%d/%d' % (
                    self.delivery_count + 1, self.max_deliveries)
        elif self.state == self.DOOR_APPROACH:
            message = ('任务阶段：前往穿门起点，坐标=(%.2f, %.2f)，'
                       '已投递=%d/%d' %
                       (self.door_start[0], self.door_start[1],
                        self.delivery_count, self.max_deliveries))
        elif self.state == self.DOOR_LOWER:
            message = ('任务阶段：穿门前下降至 %.2f m，当前高度=%s m' %
                       (self.door_height,
                        '未知' if self.current_altitude is None else
                        '%.2f' % self.current_altitude))
        elif self.state == self.DOOR_TEB_SWITCH:
            message = ('任务阶段：门口悬停并切换门洞 TEB 参数，等待 Nav2 确认；'
                       '高度=%.2f m' % self.door_height)
        elif self.state == self.DOOR_TRAVERSE:
            if self.door_waypoint_index < len(self.door_waypoints):
                x, y = self.door_waypoints[self.door_waypoint_index]
                message = ('任务阶段：连续穿门，必经点 %d/%d=(%.2f, %.2f)，'
                           '目标降落点=(%.2f, %.2f)，高度=%.2f m' %
                           (self.door_waypoint_index + 1, len(self.door_waypoints),
                            x, y, self.landing_xy[0], self.landing_xy[1],
                            self.door_height))
            else:
                message = ('任务阶段：连续穿门，必经点已全部通过；前往降落点 '
                           '(%.2f, %.2f)，高度=%.2f m' %
                           (self.landing_xy[0], self.landing_xy[1], self.door_height))
        elif self.state == self.DOOR_LANDING_APPROACH:
            message = ('任务阶段：穿门完成，前往指定降落点 (%.2f, %.2f)' %
                       self.landing_xy)
        elif self.state == self.DOOR_DONE:
            message = ('任务阶段：已到指定降落点 (%.2f, %.2f)，准备 AUTO.LAND' %
                       self.landing_xy)
        elif self.state == self.LANDING:
            message = '任务阶段：请求 PX4 AUTO.LAND'
        elif self.state == self.LANDING_ALIGN:
            message = ('任务阶段：位置控制对准指定降落点 (%.2f, %.2f)，稳定帧=%d/%d' %
                       (self.landing_xy[0], self.landing_xy[1],
                        self.stable_pose_count, self.stable_pose_samples))
        else:
            message = '任务阶段：任务结束'
        self.get_logger().info(message)

    def _publish_position_target(self, x, y, z):
        """发布地图系位置目标，姿态始终使用首次位姿锁定的机头方向。"""
        target = TransformStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = self.frame_id
        target.child_frame_id = self.target_pose_child_frame
        target.transform.translation.x = float(x)
        target.transform.translation.y = float(y)
        target.transform.translation.z = float(z)
        yaw = self.fixed_heading_yaw if self.fixed_heading_yaw is not None else 0.0
        target.transform.rotation.z = math.sin(yaw / 2.0)
        target.transform.rotation.w = math.cos(yaw / 2.0)
        self.target_pose_pub.publish(target)

    def _send_search_goal(self):
        """发送当前蛇形搜索航点；航线耗尽时转去穿门起点。"""
        if self.next_index >= len(self.route):
            self._begin_door_leg()
            return
        x, y, _, yaw = self.route[self.next_index]
        self._send_nav_goal('search', x, y, yaw)

    def _is_delivered_target(self, x, y):
        """按地图位置判断是否重复投递，允许不同位置目标属于同一类别。"""
        radius = self.delivered_target_duplicate_radius
        return any(math.hypot(x - delivered_x, y - delivered_y) <= radius
                   for delivered_x, delivered_y in self.delivered_targets)

    def _send_drop_goal(self):
        """用 Nav2 接近计算出的投递 XY；随后由位置环完成精确对位。"""
        if self.drop_xy is None:
            self._begin_landing('drop target unavailable')
            return
        yaw = self.fixed_heading_yaw if self.fixed_heading_yaw is not None else 0.0
        self._send_nav_goal('drop', self.drop_xy[0], self.drop_xy[1], yaw)

    def _send_door_goal(self):
        """穿门第一步：导航到门口起点，之后才开始降低高度。"""
        yaw = self.fixed_heading_yaw if self.fixed_heading_yaw is not None else 0.0
        self._send_nav_goal('door_start', self.door_start[0], self.door_start[1], yaw)

    def _send_door_traverse_goal(self):
        """只向 Nav2 发送最终降落目标，门洞中间点由 TEB via-point 引导。"""
        yaw = self.fixed_heading_yaw if self.fixed_heading_yaw is not None else 0.0
        self._send_nav_goal('door_route', self.landing_xy[0], self.landing_xy[1], yaw)

    def _publish_active_door_via_point(self):
        """只把当前必经点交给 TEB；空 Path 会恢复普通全局路径 via-point。"""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.frame_id
        if self.door_waypoint_index < len(self.door_waypoints):
            x, y = self.door_waypoints[self.door_waypoint_index]
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.teb_via_points_pub.publish(path)

    def _update_door_via_progress(self):
        """进入当前必经点半径后切换下一点，不等 Nav2 到点停车。"""
        if self.door_waypoint_index >= len(self.door_waypoints):
            return
        pose = self.current_pose.transform.translation
        x, y = self.door_waypoints[self.door_waypoint_index]
        distance = math.hypot(pose.x - x, pose.y - y)
        if distance > self.door_via_point_tolerance:
            return
        self.door_waypoint_index += 1
        self._publish_active_door_via_point()
        if self.door_waypoint_index < len(self.door_waypoints):
            next_x, next_y = self.door_waypoints[self.door_waypoint_index]
            self.get_logger().info(
                '已通过门必经点 %d/%d（误差 %.2f m）；继续跟踪下一点 (%.2f, %.2f)，'
                'Nav2 目标仍为降落点。' %
                (self.door_waypoint_index, len(self.door_waypoints), distance,
                 next_x, next_y))
        else:
            self.get_logger().info(
                '已通过全部 %d 个门必经点（末点误差 %.2f m）；继续导航至降落点。' %
                (len(self.door_waypoints), distance))

    def _send_landing_goal(self):
        """在穿门路线完成后导航到单独配置的 AUTO.LAND 落点。"""
        yaw = self.fixed_heading_yaw if self.fixed_heading_yaw is not None else 0.0
        self._send_nav_goal('landing_approach', self.landing_xy[0],
                            self.landing_xy[1], yaw)

    def _request_door_teb_profile(self):
        """原地等待 Nav2 接受门洞 TEB 配置，确认成功后才允许穿门。"""
        if self.teb_switch_future is not None:
            return
        now = time.monotonic()
        if now - self.teb_switch_last_attempt < 1.0:
            return
        if (self.teb_switch_started is not None and
                now - self.teb_switch_started > 12.0):
            self._begin_landing('door TEB profile service unavailable or rejected after 12 seconds')
            return
        if self.teb_switch_attempts >= 3:
            return
        self.teb_switch_last_attempt = now
        if not self.teb_parameter_client.wait_for_service(timeout_sec=0.0):
            if self.teb_switch_attempts == 0:
                self.get_logger().warning(
                    '等待 Nav2 TEB 参数服务 %s；保持在门口悬停。' %
                    self.teb_parameter_service)
            return
        request = SetParametersAtomically.Request()
        request.parameters = [
            Parameter(name=name, value=value).to_parameter_msg()
            for name, value in self.door_teb_parameters.items()]
        self.teb_switch_attempts += 1
        self.get_logger().info(
            '应用门洞 TEB 参数配置（尝试 %d/3）：%s' %
            (self.teb_switch_attempts,
             ', '.join('%s=%.3g' % (name, value)
                       for name, value in self.door_teb_parameters.items())))
        self.teb_switch_future = self.teb_parameter_client.call_async(request)
        self.teb_switch_future.add_done_callback(self._door_teb_profile_result)

    def _door_teb_profile_result(self, future):
        """只在控制器原子确认参数更新后，才提交门后终点 Nav2 目标。"""
        self.teb_switch_future = None
        if self.state != self.DOOR_TEB_SWITCH:
            return
        try:
            result = future.result().result
        except Exception as exc:
            self.get_logger().error('门洞 TEB 参数服务调用失败：%s' % exc)
            return
        if not result.successful:
            self.get_logger().error('Nav2 拒绝门洞 TEB 参数：%s' % result.reason)
            return
        applied = ', '.join(
            '%s=%.3g' % (name, value)
            for name, value in self.door_teb_parameters.items())
        self.get_logger().warning(
            '门洞 TEB 参数已确认生效：%s；门点作为连续 via-point，Nav2 目标为降落点。' %
            applied)
        self._begin_door_traverse()

    def _begin_door_traverse(self):
        """以门洞点作连续 TEB 引导，仅将独立降落点作为 Nav2 goal。"""
        self.state = self.DOOR_TRAVERSE
        self.door_waypoint_index = 0
        self.nav_state_pub.publish(Bool(data=True))
        self.passing_door_pub.publish(Bool(data=True))
        self._publish_active_door_via_point()
        self.get_logger().warning(
            '开始连续穿门：%d 个必经点由 TEB via-point 引导，Nav2 只导航至降落点 '
            '(%.3f, %.3f)；每点通过半径 %.2f m，不在点位停车。' %
            (len(self.door_waypoints), self.landing_xy[0], self.landing_xy[1],
             self.door_via_point_tolerance))
        self._send_door_traverse_goal()

    def _send_nav_goal(self, kind, x, y, yaw):
        """统一构造并发送 Nav2 NavigateToPose 目标，记录异步请求令牌。"""
        if kind == 'search':
            self.current_waypoint_index = self.next_index + 1
            self.current_nav_goal = (kind, self.current_waypoint_index, x, y)
        else:
            self.current_nav_goal = (kind, x, y)
        if self.goal_started_at is None:
            self.goal_started_at = time.monotonic()
        if not self.nav_client.server_is_ready():
            now = time.monotonic()
            if now - self.last_nav_wait_log >= 5.0:
                self.get_logger().warning('Waiting for Nav2 navigate_to_pose action')
                self.last_nav_wait_log = now
            return
        self.nav_state_pub.publish(Bool(data=True))
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        if kind == 'search':
            self.get_logger().info('Sending search waypoint %d/%d: (%.3f, %.3f)' %
                                   (self.next_index + 1, len(self.route), x, y))
        elif kind == 'drop':
            self.get_logger().info('Navigating to drop point: (%.3f, %.3f)' % (x, y))
        elif kind == 'door_waypoint':
            self.get_logger().warning(
                '穿门顺序航点 %d/%d：Nav2 导航至 (%.3f, %.3f)。' %
                (self.door_waypoint_index + 1, len(self.door_waypoints), x, y))
        elif kind == 'door_route':
            self.get_logger().warning(
                '连续穿门 Nav2 最终目标：指定降落点 (%.3f, %.3f)；门点由 TEB via-point 引导。'
                % (x, y))
        elif kind == 'landing_approach':
            self.get_logger().warning(
                '穿门路线结束；Nav2 前往指定降落点 (%.3f, %.3f)。' % (x, y))
        else:
            self.get_logger().info('Navigating to door start: (%.3f, %.3f)' % (x, y))
        send_future = self.nav_client.send_goal_async(goal)
        self.goal_token += 1
        token = self.goal_token
        self.goal_kind = kind
        send_future.add_done_callback(
            lambda future: self._goal_response(future, token, kind))
        self.goal_started_at = time.monotonic()
        self.goal_request_pending = True
        if kind == 'search':
            self.next_index += 1

    def _goal_response(self, future, token, kind):
        """处理 Nav2 是否接受目标；拒绝或请求异常时安排同目标重试。"""
        try:
            handle = future.result()
        except Exception as exc:
            if token == self.goal_token:
                self.goal_request_pending = False
                self._schedule_nav_retry(kind, 'goal request error: %s' % exc)
            else:
                self._run_after_cancel()
            return
        if token != self.goal_token:
            if handle.accepted:
                cancel_future = handle.cancel_goal_async()
                cancel_future.add_done_callback(lambda _: self._run_after_cancel())
            else:
                self._run_after_cancel()
            return
        self.goal_request_pending = False
        if not handle.accepted:
            self.goal_started_at = None
            self._schedule_nav_retry(kind, 'goal rejected')
            return
        self.goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self._goal_result(result, token, kind))

    def _goal_result(self, future, token, kind):
        """处理导航结果并按阶段推进搜索、投递、穿门航点及最终降落。"""
        if token != self.goal_token:
            return
        try:
            status = future.result().status
        except Exception as exc:
            self.get_logger().error('Nav2 result failed: %s' % exc)
            self._schedule_nav_retry(kind, 'goal result error: %s' % exc)
            return
        self.goal_handle = None
        self.goal_started_at = None
        if status != 4:  # GoalStatus.STATUS_SUCCEEDED (Foxy action_msgs)
            self._schedule_nav_retry(kind, 'action aborted (status=%d)' % status)
            return
        self.nav_retry_count = 0
        self.nav_retry_at = 0.0
        if kind == 'search':
            self.get_logger().info('Search waypoint reached')
            if self.state == self.SEARCHING:
                self._send_search_goal()
        elif kind == 'drop':
            self.get_logger().info('Drop vicinity reached; switching to precise position control')
            if self.state == self.DROP_APPROACH:
                self.nav_state_pub.publish(Bool(data=False))
                self.state = self.DROP_ALIGN
                self.stable_pose_count = 0
        elif kind == 'door_start':
            self.get_logger().warning(
                '已到达穿门起点 (%.3f, %.3f)，切位置控制并下降到 %.2f m。' %
                (self.door_start[0], self.door_start[1], self.door_height))
            self.nav_state_pub.publish(Bool(data=False))
            self.state = self.DOOR_LOWER
            self.stable_pose_count = 0
        elif kind == 'door_waypoint':
            self.get_logger().info(
                '穿门航点 %d/%d 已到达。' %
                (self.door_waypoint_index + 1, len(self.door_waypoints)))
            if self.state == self.DOOR_TRAVERSE:
                self.door_waypoint_index += 1
                if self.door_waypoint_index < len(self.door_waypoints):
                    self._send_door_traverse_goal()
                else:
                    self._start_landing_approach()
        elif kind == 'door_route':
            self.get_logger().warning(
                'Nav2 已到达最终降落目标；关闭门洞 via-point 引导，开始落地流程。')
            if self.state == self.DOOR_TRAVERSE:
                if self.door_waypoint_index < len(self.door_waypoints):
                    self._schedule_nav_retry(
                        kind, 'landing goal reached before all door via-points passed')
                else:
                    self._begin_final_landing()
        elif kind == 'landing_approach':
            self.get_logger().warning(
                'Nav2 已到达指定降落点 (%.3f, %.3f)，开始请求 AUTO.LAND。' %
                self.landing_xy)
            if self.state == self.DOOR_LANDING_APPROACH:
                self._begin_final_landing()

    def _schedule_nav_retry(self, kind, reason):
        """确认任务阶段与位姿安全后原地悬停，延迟重试同一个 Nav2 目标。"""
        expected_state = {
            'search': self.SEARCHING,
            'drop': self.DROP_APPROACH,
            'door_start': self.DOOR_APPROACH,
            'door_traverse': self.DOOR_TRAVERSE,
            'door_waypoint': self.DOOR_TRAVERSE,
            'door_route': self.DOOR_TRAVERSE,
            'landing_approach': self.DOOR_LANDING_APPROACH,
        }.get(kind)
        if expected_state is None or self.state != expected_state:
            self._begin_landing('cannot retry navigation outside its mission phase')
            return

        pose_fresh = (self.current_pose_time is not None
                      and time.monotonic() - self.current_pose_time < self.pose_stale_timeout
                      and self.current_pose is not None
                      and self.current_altitude is not None)
        if not pose_fresh:
            self._begin_landing('navigation failed and pose is stale; cannot hold safely')
            return

        if kind == 'search' and self.current_nav_goal:
            # _send_nav_goal advances next_index when dispatching. Restore it
            # so the retry is for the failed waypoint, not the next lane.
            self.next_index = max(0, int(self.current_nav_goal[1]) - 1)

        self.goal_handle = None
        self.goal_request_pending = False
        self.goal_started_at = None
        self.goal_kind = None
        self.nav_retry_count += 1
        self.nav_retry_at = time.monotonic() + self.nav_retry_delay
        self.nav_state_pub.publish(Bool(data=False))
        pose = self.current_pose.transform.translation
        self._publish_position_target(pose.x, pose.y, self.current_altitude)
        if kind == 'search':
            detail = '航点 %d/%d' % (self.next_index + 1, len(self.route))
        else:
            detail = {
                'drop': '投递点',
                'door_start': '穿门起点',
                'door_traverse': '穿门航点',
                'door_waypoint': '穿门航点 %d/%d' % (
                    self.door_waypoint_index + 1, len(self.door_waypoints)),
                'door_route': '穿门路线/最终降落点',
                'landing_approach': '指定降落点',
            }[kind]
        self.get_logger().error(
            'Nav2 %s失败（%s）；已在当前位置悬停，%.1f 秒后重试同一目标（第 %d 次）。'
            % (detail, reason, self.nav_retry_delay, self.nav_retry_count))

    def _cancel_goal_then(self, callback):
        """取消正在执行/等待响应的 Nav2 目标，完成后调用指定后续动作。"""
        was_pending = self.goal_request_pending
        handle = self.goal_handle
        self.goal_token += 1
        self.goal_kind = None
        self.goal_handle = None
        self.goal_request_pending = False
        self.goal_started_at = None
        if handle is not None:
            self.cancel_in_progress = True
            future = handle.cancel_goal_async()
            future.add_done_callback(lambda _: self._cancel_done(callback))
        elif was_pending:
            self.cancel_in_progress = True
            self.after_cancel = callback
        else:
            callback()

    def _cancel_done(self, callback):
        """Nav2 确认取消后执行后续动作；降落/结束状态不再回调。"""
        self.cancel_in_progress = False
        if self.state not in (self.LANDING, self.FINISHED):
            callback()

    def _run_after_cancel(self):
        """执行因导航请求尚未返回而延迟的取消后续动作。"""
        callback = self.after_cancel
        self.after_cancel = None
        self.cancel_in_progress = False
        if callback is not None and self.state not in (self.LANDING, self.FINISHED):
            callback()

    def _begin_door_leg(self):
        """结束搜索/投递阶段，切到导航模式并前往配置的穿门起点。"""
        if self.state in (self.DOOR_APPROACH, self.DOOR_LOWER,
                          self.DOOR_TEB_SWITCH, self.DOOR_TRAVERSE,
                          self.DOOR_LANDING_APPROACH,
                          self.DOOR_DONE, self.LANDING, self.FINISHED):
            return
        self._start_door_approach()

    def _start_door_approach(self):
        """开始向配置的固定门口起点导航。"""
        self.passing_door_pub.publish(Bool(data=False))
        self.goal_started_at = None
        self.state = self.DOOR_APPROACH
        self.nav_state_pub.publish(Bool(data=True))
        self.get_logger().warning(
            '[阶段] 开始穿门：已投递 %d 件；固定门口起点=(%.3f, %.3f)。' %
            (self.delivery_count, *self.door_start))
        self._send_door_goal()

    def _start_landing_approach(self):
        """穿门顺序点全部完成后，先导航到独立的指定降落点。"""
        self.state = self.DOOR_LANDING_APPROACH
        self.goal_started_at = None
        self.nav_retry_at = 0.0
        self.nav_state_pub.publish(Bool(data=True))
        self.passing_door_pub.publish(Bool(data=True))
        self.get_logger().warning(
            '穿门路线已完成；按序点终点=(%.3f, %.3f)，现在前往指定降落点=(%.3f, %.3f)。' %
            (self.door_waypoints[-1][0], self.door_waypoints[-1][1],
             self.landing_xy[0], self.landing_xy[1]))
        self._send_landing_goal()

    def _begin_final_landing(self):
        """到达指定落点后保持该点并请求飞控 AUTO.LAND。"""
        self._publish_active_door_via_point()  # 清除门洞自定义 via-point
        self.state = self.LANDING_ALIGN
        self.goal_started_at = None
        self.goal_kind = None
        self.stable_pose_count = 0
        pose_fresh = (self.current_pose_time is not None
                      and time.monotonic() - self.current_pose_time < self.pose_stale_timeout
                      and self.current_pose is not None)
        self.landing_hold_target = (
            (self.landing_xy[0], self.landing_xy[1], self.door_height)
            if pose_fresh else None)
        self.nav_state_pub.publish(Bool(data=False))
        self.passing_door_pub.publish(Bool(data=False))
        self.get_logger().warning(
            '全流程导航完成：已到指定降落点 (%.3f, %.3f)，切位置控制并稳定确认后才请求 AUTO.LAND。' %
            self.landing_xy)

    def _begin_landing(self, reason):
        """记录触发原因、取消 Nav2 目标，并让主循环请求 PX4 AUTO.LAND。"""
        if self.state in (self.LANDING, self.FINISHED):
            return
        self.state = self.LANDING
        self.landing_hold_target = None
        self.get_logger().error('Mission abort (%s); requesting AUTO.LAND' % reason)
        self.passing_door_pub.publish(Bool(data=False))
        self.landing_requested = False
        if self.goal_handle is not None or self.goal_request_pending:
            self._cancel_goal_then(lambda: None)

    def _land_result(self, future):
        """处理 AUTO.LAND 模式切换结果；失败时允许主循环继续重试。"""
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().warning('AUTO.LAND mode request sent')
                self.nav_state_pub.publish(Bool(data=False))
                self.landing_hold_target = None
                self.state = self.FINISHED
            else:
                self.get_logger().error(
                    'FCU rejected AUTO.LAND; will retry while holding current mode')
                self.landing_requested = False
        except Exception as exc:
            self.get_logger().error('AUTO.LAND request failed: %s; will retry' % exc)
            self.landing_requested = False


def main(args=None):
    """ROS 2 节点入口；确保异常或退出时销毁节点并关闭 rclpy。"""
    rclpy.init(args=args)
    node = None
    try:
        node = CoverageMissionNode()
        rclpy.spin(node)
    except (TypeError, ValueError) as exc:
        if node is not None:
            node.get_logger().fatal(str(exc))
        else:
            print('coverage mission startup failed: %s' % exc)
        return 2
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            # Destroy entities while the node handle is still valid. Without
            # this, Foxy may run ActionClient.__del__ after destroy_node() on
            # Ctrl-C and report InvalidHandle during otherwise-clean shutdown.
            node.nav_client.destroy()
            node.land_client.destroy()
            node.teb_parameter_client.destroy()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
