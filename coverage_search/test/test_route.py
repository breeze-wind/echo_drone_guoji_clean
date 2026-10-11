import math

import pytest

from coverage_search.route import generate_lawnmower_route


def test_axis_aligned_route_alternates_and_stays_inside_bounds():
    route = generate_lawnmower_route((0.0, 0.0), (5.0, 3.0), 1.0, 2.0)
    assert len(route) == 6
    assert route[0][:3] == pytest.approx((0.0, 0.5, 2.0))
    assert route[1][:3] == pytest.approx((5.0, 0.5, 2.0))
    assert route[2][:3] == pytest.approx((5.0, 1.5, 2.0))
    assert route[3][:3] == pytest.approx((0.0, 1.5, 2.0))
    assert all(0.0 <= p[0] <= 5.0 and 0.0 <= p[1] <= 3.0 for p in route)
    assert route[0][3] == pytest.approx(0.0)
    assert route[1][3] == pytest.approx(math.pi / 2.0)


def test_auto_axis_uses_longer_rectangle_side_for_scan_lanes():
    route = generate_lawnmower_route((0.0, 0.0), (3.0, 6.0), 1.5, 2.2)
    assert route[0][:3] == pytest.approx((0.75, 0.0, 2.2))
    assert route[1][:3] == pytest.approx((0.75, 6.0, 2.2))


def test_reversed_corner_order_is_supported():
    route = generate_lawnmower_route((5.0, 3.0), (0.0, 0.0), 1.0, 2.0)
    assert route[0][:3] == pytest.approx((0.0, 0.5, 2.0))


@pytest.mark.parametrize(
    'corner_a, corner_b, spacing, height, axis',
    [
        ((0.0,), (1.0, 1.0), 1.0, 2.0, 'auto'),
        ((0.0, 0.0), (0.0, 1.0), 1.0, 2.0, 'auto'),
        ((0.0, 0.0), (1.0, 1.0), 0.0, 2.0, 'auto'),
        ((0.0, 0.0), (1.0, 1.0), 1.0, -1.0, 'auto'),
        ((0.0, 0.0), (1.0, 1.0), 1.0, 2.0, 'diagonal'),
    ],
)
def test_invalid_geometry_is_rejected(corner_a, corner_b, spacing, height, axis):
    with pytest.raises(ValueError):
        generate_lawnmower_route(corner_a, corner_b, spacing, height, axis)
