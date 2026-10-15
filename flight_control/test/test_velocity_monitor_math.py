import math

from flight_control.velocity_monitor_math import (
    PoseVelocityEstimator,
    compare_velocity,
    world_to_body,
)


def test_pose_regression_estimates_motion_and_body_rotation():
    estimator = PoseVelocityEstimator(window_sec=0.6)
    for index in range(7):
        t = index * 0.1
        estimator.add(t, 0.2 * t, -0.1 * t, 'odom')
    vx, vy = estimator.velocity()
    assert math.isclose(vx, 0.2, abs_tol=1e-9)
    assert math.isclose(vy, -0.1, abs_tol=1e-9)
    bx, by = world_to_body(vx, vy, math.pi / 2)
    assert math.isclose(bx, -0.1, abs_tol=1e-9)
    assert math.isclose(by, -0.2, abs_tol=1e-9)


def test_pose_jump_clears_old_velocity():
    estimator = PoseVelocityEstimator()
    for index in range(4):
        estimator.add(index * 0.1, index * 0.01, 0.0, 'odom')
    assert estimator.velocity() is not None
    estimator.add(0.4, 10.0, 0.0, 'odom')
    assert estimator.velocity() is None


def test_frame_change_and_duplicate_timestamp():
    estimator = PoseVelocityEstimator()
    assert estimator.add(1.0, 0.0, 0.0, 'odom')
    assert not estimator.add(1.0, 1.0, 0.0, 'odom')
    assert len(estimator.samples) == 1
    estimator.add(1.1, 0.0, 0.0, 'map')
    assert len(estimator.samples) == 1
    assert estimator.frame_id == 'map'


def test_along_and_cross_track():
    result = compare_velocity((0.2, 0.0), (0.1, -0.05))
    assert math.isclose(result['along_track_mps'], 0.1)
    assert math.isclose(result['cross_track_mps'], -0.05)
    assert math.isclose(result['along_track_ratio'], 0.5)


def test_idle_has_no_ratio():
    result = compare_velocity((0.0, 0.0), (0.1, 0.0))
    assert result['along_track_ratio'] is None
