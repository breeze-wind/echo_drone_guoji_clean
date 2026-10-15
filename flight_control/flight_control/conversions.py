"""旧飞控控制语义到 MAVROS 坐标语义的转换工具。

旧的 pymavlink 节点把近似 NED 的数据直接发给 PX4，并在本地混入了若干
符号和高度偏置。MAVROS 接收 ROS ENU 消息，并在内部转换成 MAVLink。
这里的函数显式做反向转换，让现有 `/robot/*` 生产者保持旧语义，同时把
飞控传输层迁移到 MAVROS。
"""

import math

LEGACY_COORDINATE_MODE = 'legacy_ned_compatible'
MAVROS_ENU_COORDINATE_MODE = 'mavros_enu'


def ned_xyz_to_mavros_enu(x_ned, y_ned, z_ned):
    """返回会被 MAVROS 转回输入 NED 向量的 ROS ENU 向量。"""
    return y_ned, x_ned, -z_ned


def legacy_position_to_mavros_enu(x, y, z):
    """把旧的 x、-y、-z MAVLink 位置约定转换为 MAVROS ENU。"""
    return ned_xyz_to_mavros_enu(x, -y, -z)


def legacy_vision_pose_position(x, y, z, z_offset):
    """把 `/robot/current_pose` 位置转换成 MAVROS vision_pose 使用的 ENU。"""
    return legacy_position_to_mavros_enu(x, y, z + z_offset)


def legacy_target_position(x, y, z, reference_z_offset):
    """把 `/robot/target_pose` 位置转换成 MAVROS 本地位置目标 ENU。"""
    return legacy_position_to_mavros_enu(x, y, z - reference_z_offset)


def body_velocity_to_map(cmd_x, cmd_y, yaw_map):
    """把 Nav2 在机器人机体系下的水平速度转到 map 坐标系。"""
    cos_yaw = math.cos(yaw_map)
    sin_yaw = math.sin(yaw_map)
    return (cos_yaw * cmd_x - sin_yaw * cmd_y,
            sin_yaw * cmd_x + cos_yaw * cmd_y)


def nav2_velocity_to_mavros_enu(cmd_x, cmd_y, yaw_map, coordinate_mode):
    """把机体系 Nav2 XY 速度转成与 vision_pose 一致的 MAVROS ENU。"""
    map_vx, map_vy = body_velocity_to_map(cmd_x, cmd_y, yaw_map)
    if coordinate_mode == MAVROS_ENU_COORDINATE_MODE:
        return map_vx, map_vy
    return -map_vy, map_vx


def mavros_enu_yaw_for_legacy_ned_yaw(yaw_ned):
    """返回会被 MAVROS 转回旧 NED 航向角的 ENU 航向角。"""
    return math.pi / 2.0 - yaw_ned


def heading_hold_yaw_rate(target_yaw, current_yaw, gain, max_rate):
    """用 ENU 航向误差生成有限幅的偏航角速度，正确处理 ±π 环绕。"""
    error = math.atan2(
        math.sin(target_yaw - current_yaw),
        math.cos(target_yaw - current_yaw),
    )
    return max(-max_rate, min(max_rate, gain * error))


def quaternion_from_euler_xyzw(roll, pitch, yaw):
    """由 roll、pitch、yaw 生成 geometry_msgs 使用的四元数组。"""
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5

    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def euler_from_quaternion_msg(quaternion_msg):
    """从 geometry_msgs Quaternion 类对象中解析 roll、pitch、yaw。"""
    x = quaternion_msg.x
    y = quaternion_msg.y
    z = quaternion_msg.z
    w = quaternion_msg.w

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def legacy_vision_orientation_to_mavros_enu(quaternion_msg):
    """把旧视觉姿态转换成 MAVROS 期望的 ENU 姿态。"""
    roll, pitch, yaw = euler_from_quaternion_msg(quaternion_msg)
    return quaternion_from_euler_xyzw(roll, pitch, math.pi / 2.0 + yaw)
