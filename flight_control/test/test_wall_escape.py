import math
from types import SimpleNamespace

from flight_control.wall_escape import (
    door_lateral_direction, escape_direction, grid_obstacles)


def test_open_side_wall_escape():
    direction = escape_direction(
        [(-0.35, 0.0)], 0.0, 0.0, 0.32, 0.10, 0.10, 0.05,
        lambda x, y: abs(x) < 2 and abs(y) < 2)
    assert direction == (1.0, 0.0)


def test_narrow_door_rejects_lateral_push():
    # 80cm door, 45cm square vehicle: moving sideways loses clearance.
    direction = escape_direction(
        [(-0.40, 0.0), (0.40, 0.0)], 0.0, 0.0, 0.32,
        0.10, 0.10, 0.05, lambda x, y: True)
    assert direction is None


def test_narrow_door_uses_lateral_only_without_opposite_wall_check():
    points = [(-0.40, 0.0), (0.40, 0.0)]
    direction = door_lateral_direction(
        points, 0.0, 0.0, 0.0, 0.2, 0.32,
        0.10, lambda x, y: True)
    assert direction == (1.0, -0.0)


def test_door_escape_never_goes_backward():
    # A front wall must not result in a reverse command in the door.
    assert door_lateral_direction(
        [(0.0, 0.35)], 0.0, 0.0, 0.0, 0.2, 0.32,
        0.10, lambda x, y: True) is None


def test_unknown_and_no_obstacle_rejected():
    assert escape_direction([], 0, 0, 0.32, 0.1, 0.1, 0.05,
                            lambda x, y: True) is None
    assert escape_direction([(-0.35, 0)], 0, 0, 0.32, 0.1, 0.1,
                            0.05, lambda x, y: x < 0.03) is None


def test_occupied_grid_and_unknown_cells():
    info = SimpleNamespace(
        width=3, height=2, resolution=0.1,
        origin=SimpleNamespace(
            position=SimpleNamespace(x=1.0, y=2.0),
            orientation=SimpleNamespace(x=0, y=0, z=0, w=1)))
    msg = SimpleNamespace(info=info, data=[0, -1, 100, 0, 0, 0])
    points, free_at = grid_obstacles(msg)
    assert len(points) == 1
    assert math.isclose(points[0][0], 1.25)
    assert free_at(1.05, 2.05)
    assert not free_at(1.15, 2.05)
    assert not free_at(1.25, 2.05)
    assert not free_at(5, 5)
