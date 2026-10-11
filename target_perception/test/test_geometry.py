import math

import pytest

from target_perception.geometry import pixel_ray_to_ground


def test_center_ray_intersects_ground_below_camera():
    point = pixel_ray_to_ground(
        320, 240, 500, 500, 320, 240,
        (1.0, 2.0, 2.5), (1.0, 0.0, 0.0, 0.0), 0.0)
    assert point == pytest.approx((1.0, 2.0, 0.0))


def test_pixel_offset_projects_in_expected_map_direction():
    point = pixel_ray_to_ground(
        420, 240, 500, 500, 320, 240,
        (0.0, 0.0, 2.0), (1.0, 0.0, 0.0, 0.0), 0.0)
    assert point == pytest.approx((0.4, 0.0, 0.0))


def test_ray_parallel_to_ground_does_not_localize():
    q = (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))
    point = pixel_ray_to_ground(
        320, 240, 500, 500, 320, 240,
        (0.0, 0.0, 2.0), q, 0.0)
    assert point is None
