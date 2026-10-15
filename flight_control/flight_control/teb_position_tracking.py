"""Small, testable XY helpers for opt-in TEB position-setpoint tracking."""

import math


def preview_on_path(points, position, lookahead):
    """Return (preview_xy, cross_track_error) along a map-frame polyline.

    The nearest projection is found first, then the preview is advanced by
    ``lookahead`` of *arc length*. TEB's Path.z is intentionally ignored by
    the caller: in this repository it is visualization time, not altitude.
    """
    if not points or not all(math.isfinite(v) for p in points for v in p):
        return None
    if not all(math.isfinite(v) for v in position) or lookahead < 0:
        return None

    if len(points) == 1:
        return points[0], math.hypot(points[0][0] - position[0],
                                     points[0][1] - position[1])

    lengths = []
    nearest_distance = math.inf
    nearest_s = 0.0
    running_s = 0.0
    for start, end in zip(points, points[1:]):
        dx, dy = end[0] - start[0], end[1] - start[1]
        length = math.hypot(dx, dy)
        lengths.append(length)
        if length > 0:
            fraction = max(0.0, min(1.0,
                ((position[0] - start[0]) * dx +
                 (position[1] - start[1]) * dy) / (length * length)))
            projection = (start[0] + fraction * dx,
                          start[1] + fraction * dy)
            distance = math.hypot(position[0] - projection[0],
                                  position[1] - projection[1])
            if distance < nearest_distance:
                nearest_distance = distance
                nearest_s = running_s + fraction * length
        running_s += length

    if not math.isfinite(nearest_distance):
        return points[0], math.hypot(points[0][0] - position[0],
                                     points[0][1] - position[1])

    target_s = min(running_s, nearest_s + lookahead)
    running_s = 0.0
    for (start, end), length in zip(zip(points, points[1:]), lengths):
        if length > 0 and target_s <= running_s + length:
            fraction = (target_s - running_s) / length
            return ((start[0] + fraction * (end[0] - start[0]),
                     start[1] + fraction * (end[1] - start[1])),
                    nearest_distance)
        running_s += length
    return points[-1], nearest_distance


def advance_target(previous, position, desired, max_step, max_lead):
    """Slew a position target without letting it outrun the aircraft."""
    if previous is None or math.hypot(previous[0] - position[0],
                                     previous[1] - position[1]) > max_lead:
        previous = position
    if max_step <= 0:
        return previous
    dx, dy = desired[0] - previous[0], desired[1] - previous[1]
    distance = math.hypot(dx, dy)
    if distance > max_step > 0:
        scale = max_step / distance
        dx, dy = dx * scale, dy * scale
    target = (previous[0] + dx, previous[1] + dy)
    lead = math.hypot(target[0] - position[0], target[1] - position[1])
    if lead > max_lead:
        scale = max_lead / lead
        target = (position[0] + scale * (target[0] - position[0]),
                  position[1] + scale * (target[1] - position[1]))
    return target
