"""Pure geometry tests for the opt-in TEB position follower."""

import math

from flight_control.teb_position_tracking import (
    advance_target, preview_on_path,
)


def assert_xy_close(actual, expected):
    assert math.isclose(actual[0], expected[0], abs_tol=1e-9)
    assert math.isclose(actual[1], expected[1], abs_tol=1e-9)


def test_preview_projects_to_nearest_segment_then_advances_along_path():
    target, error = preview_on_path(
        [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)],
        (0.4, 0.05), 0.2)
    assert_xy_close(target, (0.6, 0.0))
    assert math.isclose(error, 0.05, abs_tol=1e-9)


def test_preview_near_corner_follows_polyline_not_a_direct_shortcut():
    target, error = preview_on_path(
        [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)],
        (0.95, 0.0), 0.2)
    assert_xy_close(target, (1.0, 0.15))
    assert math.isclose(error, 0.0, abs_tol=1e-9)


def test_preview_ignores_bad_or_missing_points():
    assert preview_on_path([], (0.0, 0.0), 0.1) is None
    assert preview_on_path([(0.0, 0.0), (math.nan, 0.0)],
                           (0.0, 0.0), 0.1) is None


def test_position_target_slew_and_lead_limit():
    target = advance_target((0.0, 0.0), (0.0, 0.0),
                            (1.0, 0.0), 0.02, 0.12)
    assert_xy_close(target, (0.02, 0.0))
    target = advance_target((0.1, 0.0), (0.0, 0.0),
                            (1.0, 0.0), 0.04, 0.12)
    assert_xy_close(target, (0.12, 0.0))


def test_position_target_resets_when_previous_is_far_from_aircraft():
    target = advance_target((2.0, 0.0), (0.0, 0.0),
                            (1.0, 0.0), 0.02, 0.12)
    assert_xy_close(target, (0.02, 0.0))
