"""旧飞控语义到 MAVROS 坐标契约的可执行检查。

这些测试故意固定旧 pymavlink 的符号和高度偏置。当系统其他部分仍发布旧
`/robot/*` 消息、adapter 负责转换到 MAVROS ENU 话题时，它们就是回归
说明文档。
"""

import math

from flight_control.conversions import (
    legacy_nav_velocity,
    legacy_passing_door_velocity,
    legacy_target_position,
    legacy_vision_pose_position,
    mavros_enu_yaw_for_legacy_ned_yaw,
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


def test_legacy_nav_velocity_preserves_old_payload():
    assert_tuple_close(legacy_nav_velocity(0.5, -0.2, 0.4, 0.6, 0.65),
                       (0.2, 0.5, 0.13))


def test_legacy_passing_door_velocity_preserves_old_payload():
    assert_tuple_close(
        legacy_passing_door_velocity(0.5, -0.2, 0.7, 0.6, 0.65),
        (0.5, -0.2, -0.065),
    )


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
