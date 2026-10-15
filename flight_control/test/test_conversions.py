"""旧飞控语义到 MAVROS 坐标契约的可执行检查。

这些测试故意固定旧 pymavlink 的符号和高度偏置。当系统其他部分仍发布旧
`/robot/*` 消息、adapter 负责转换到 MAVROS ENU 话题时，它们就是回归
说明文档。
"""

import math

from flight_control.conversions import (
    LEGACY_COORDINATE_MODE,
    MAVROS_ENU_COORDINATE_MODE,
    body_velocity_to_map,
    heading_hold_yaw_rate,
    legacy_target_position,
    legacy_vision_pose_position,
    mavros_enu_yaw_for_legacy_ned_yaw,
    nav2_velocity_to_mavros_enu,
    ned_xyz_to_mavros_enu,
)


def assert_tuple_close(actual, expected):
    assert len(actual) == len(expected)
    for left, right in zip(actual, expected):
        assert math.isclose(left, right, rel_tol=1e-9, abs_tol=1e-9)


def test_ned_xyz_to_mavros_enu():
    assert_tuple_close(ned_xyz_to_mavros_enu(1.0, 2.0, -3.0),
                       (2.0, 1.0, 3.0))


def test_legacy_vision_pose_position_preserves_old_payload():
    assert_tuple_close(legacy_vision_pose_position(1.0, 2.0, 0.3, 0.31),
                       (-2.0, 1.0, 0.61))


def test_legacy_target_position_preserves_old_payload():
    assert_tuple_close(legacy_target_position(4.0, -2.0, 0.7, 0.08),
                       (2.0, 4.0, 0.62))


def test_nav2_body_velocity_rotates_with_aircraft_heading():
    assert_tuple_close(body_velocity_to_map(0.4, 0.0, 0.0), (0.4, 0.0))
    assert_tuple_close(body_velocity_to_map(0.4, 0.0, math.pi / 2.0),
                       (0.0, 0.4))
    assert_tuple_close(body_velocity_to_map(0.0, 0.4, math.pi / 2.0),
                       (-0.4, 0.0))


def test_nav2_velocity_uses_the_same_map_to_mavros_conversion_at_the_door():
    assert_tuple_close(
        nav2_velocity_to_mavros_enu(0.4, 0.0, 0.0,
                                    LEGACY_COORDINATE_MODE),
        (0.0, 0.4))
    assert_tuple_close(
        nav2_velocity_to_mavros_enu(0.4, 0.0, math.pi / 2.0,
                                    LEGACY_COORDINATE_MODE),
        (-0.4, 0.0))
    assert_tuple_close(
        nav2_velocity_to_mavros_enu(0.4, 0.0, math.pi / 2.0,
                                    MAVROS_ENU_COORDINATE_MODE),
        (0.0, 0.4))


def test_legacy_yaw_is_inverse_of_mavros_ned_conversion():
    assert math.isclose(
        mavros_enu_yaw_for_legacy_ned_yaw(0.0),
        math.pi / 2.0,
        rel_tol=1e-9,
        abs_tol=1e-9,
    )
    assert math.isclose(
        mavros_enu_yaw_for_legacy_ned_yaw(1.57),
        math.pi / 2.0 - 1.57,
        rel_tol=1e-9,
        abs_tol=1e-9,
    )


def test_heading_hold_yaw_rate_wraps_and_limits():
    assert math.isclose(heading_hold_yaw_rate(0.2, 0.1, 0.8, 0.12), 0.08)
    assert math.isclose(heading_hold_yaw_rate(-math.pi + 0.05,
                                               math.pi - 0.05, 0.8, 0.12), 0.08)
    assert heading_hold_yaw_rate(1.0, 0.0, 0.8, 0.12) == 0.12
    assert heading_hold_yaw_rate(0.0, 1.0, 0.8, 0.12) == -0.12
