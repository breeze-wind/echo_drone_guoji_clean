"""Conservative, one-shot wall escape decision for the velocity adapter.

This is a bounded velocity override, never a motor force. The door-only
lateral pulse intentionally ignores the opposite wall and is not a collision
guarantee.
"""

import math


def clearance(points, x, y):
    if not points:
        return math.inf
    return min(math.hypot(x - px, y - py) for px, py in points)


def escape_direction(points, x, y, radius, trigger_clearance,
                     step, min_gain, free_at):
    """Return map-frame unit vector away from nearest wall, or None.

    Every sampled center must remain in known free map cells. The minimum
    clearance cannot deteriorate materially, and the endpoint must improve.
    """
    if not points or not free_at(x, y):
        return None
    nearest = min(points, key=lambda p: (p[0] - x) ** 2 + (p[1] - y) ** 2)
    dx, dy = x - nearest[0], y - nearest[1]
    distance = math.hypot(dx, dy)
    if distance < 1e-6 or distance > radius + trigger_clearance:
        return None
    ux, uy = dx / distance, dy / distance
    initial = clearance(points, x, y)
    samples = max(2, int(math.ceil(step / 0.025)))
    for i in range(1, samples + 1):
        travel = step * i / samples
        px, py = x + ux * travel, y + uy * travel
        if not free_at(px, py) or clearance(points, px, py) < initial - 0.01:
            return None
    if clearance(points, x + ux * step, y + uy * step) < max(
            initial + min_gain, radius + 0.02):
        return None
    return ux, uy


def door_lateral_direction(points, x, y, cmd_x, cmd_y, radius,
                           trigger_clearance, free_at):
    """Push sideways from the nearest wall, never backward along the command.

    Deliberately does not inspect the opposite wall. The caller must use a
    very short pulse because this direction has no swept-clearance guarantee.
    """
    speed = math.hypot(cmd_x, cmd_y)
    if not points or speed < 1e-6 or not free_at(x, y):
        return None
    nearest = min(points, key=lambda p: (p[0] - x) ** 2 + (p[1] - y) ** 2)
    dx, dy = x - nearest[0], y - nearest[1]
    distance = math.hypot(dx, dy)
    if distance < 1e-6 or distance > radius + trigger_clearance:
        return None
    # The perpendicular has zero backward component relative to Nav2's cmd.
    normal_x, normal_y = -cmd_y / speed, cmd_x / speed
    lateral = (dx * normal_x + dy * normal_y) / distance
    if abs(lateral) < 0.3:
        return None
    side = math.copysign(1.0, lateral)
    return side * normal_x, side * normal_y


def grid_obstacles(msg, occupied_threshold=90):
    """Extract occupied cell centers and a known-free center-cell predicate."""
    width = msg.info.width
    height = msg.info.height
    resolution = msg.info.resolution
    origin = msg.info.origin.position
    q = msg.info.origin.orientation
    if (width <= 0 or height <= 0 or resolution <= 0 or
            len(msg.data) != width * height or
            abs(q.x) > 1e-3 or abs(q.y) > 1e-3 or
            abs(q.z) > 1e-3 or abs(abs(q.w) - 1.0) > 1e-3):
        raise ValueError('unsupported or invalid local costmap geometry')
    points = []
    for index, value in enumerate(msg.data):
        if value >= occupied_threshold:
            col, row = index % width, index // width
            points.append((origin.x + (col + 0.5) * resolution,
                           origin.y + (row + 0.5) * resolution))

    def free_at(x, y):
        col = math.floor((x - origin.x) / resolution)
        row = math.floor((y - origin.y) / resolution)
        return (0 <= col < width and 0 <= row < height and
                0 <= msg.data[row * width + col] < occupied_threshold)

    return points, free_at
