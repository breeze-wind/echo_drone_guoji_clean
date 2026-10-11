"""Pure geometry helpers for an axis-aligned lawnmower search route."""

import math


def generate_lawnmower_route(corner_a, corner_b, lane_spacing, search_height,
                              lane_axis='auto'):
    """Return ``(x, y, z, yaw)`` poses covering a rectangle.

    ``corner_a`` and ``corner_b`` are opposite corners in the map frame. The
    route follows the longer side by default and alternates direction on each
    lane. The first lane is inset by half a lane spacing so the camera's
    footprint can cover the boundary without placing waypoints on the edge.
    """
    if len(corner_a) != 2 or len(corner_b) != 2:
        raise ValueError('corner_a and corner_b must each contain x and y')
    ax, ay = float(corner_a[0]), float(corner_a[1])
    bx, by = float(corner_b[0]), float(corner_b[1])
    min_x, max_x = sorted((ax, bx))
    min_y, max_y = sorted((ay, by))
    width = max_x - min_x
    height = max_y - min_y
    spacing = float(lane_spacing)
    z = float(search_height)

    if width <= 0.0 or height <= 0.0:
        raise ValueError('rectangle must have non-zero width and height')
    if not math.isfinite(spacing) or spacing <= 0.0:
        raise ValueError('lane_spacing must be a finite positive number')
    if not math.isfinite(z) or z < 0.0:
        raise ValueError('search_height must be finite and non-negative')
    if lane_axis not in ('auto', 'x', 'y'):
        raise ValueError("lane_axis must be 'auto', 'x', or 'y'")

    if lane_axis == 'auto':
        lane_axis = 'x' if width >= height else 'y'

    if lane_axis == 'x':
        scan_length = width
        cross_min, cross_max = min_y, max_y
    else:
        scan_length = height
        cross_min, cross_max = min_x, max_x

    cross_span = cross_max - cross_min
    inset = min(spacing / 2.0, cross_span / 2.0)
    first = cross_min + inset
    last = cross_max - inset
    lane_count = max(1, int(math.floor((last - first) / spacing)) + 1)
    cross_positions = [first + i * spacing for i in range(lane_count)]

    # Center one lane when the search rectangle is narrower than the spacing.
    if lane_count == 1:
        cross_positions[0] = (cross_min + cross_max) / 2.0

    route = []
    along_min, along_max = ((min_x, max_x) if lane_axis == 'x'
                            else (min_y, max_y))
    for lane_index, cross in enumerate(cross_positions):
        forward = lane_index % 2 == 0
        start_along = along_min if forward else along_max
        end_along = along_max if forward else along_min
        if lane_axis == 'x':
            start = (start_along, cross)
            end = (end_along, cross)
        else:
            start = (cross, start_along)
            end = (cross, end_along)

        # Each lane contributes its two ends. Adjacent lanes therefore join
        # at the same side of the rectangle instead of cutting diagonally.
        route.append((start[0], start[1], z))
        route.append((end[0], end[1], z))

    poses = []
    for i, point in enumerate(route):
        if i + 1 < len(route):
            next_point = route[i + 1]
            yaw = math.atan2(next_point[1] - point[1],
                             next_point[0] - point[0])
        elif i > 0:
            previous = route[i - 1]
            yaw = math.atan2(point[1] - previous[1],
                             point[0] - previous[0])
        else:
            yaw = 0.0
        poses.append((point[0], point[1], point[2], yaw))
    return poses
