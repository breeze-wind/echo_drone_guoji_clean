"""Camera pixel ray projection into a horizontal ground plane."""

import math

import numpy as np


def quaternion_rotate(vector, quaternion_xyzw):
    """Rotate a 3-vector by a normalized ROS-order quaternion."""
    q = np.asarray(quaternion_xyzw, dtype=np.float64)
    norm = np.linalg.norm(q)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('quaternion must be finite and nonzero')
    x, y, z, w = q / norm
    qv = np.array([x, y, z], dtype=np.float64)
    v = np.asarray(vector, dtype=np.float64)
    return v + 2.0 * np.cross(qv, np.cross(qv, v) + w * v)


def pixel_ray_to_ground(u, v, fx, fy, cx, cy, translation, rotation_xyzw,
                        ground_z=0.0):
    """Project a rectified optical-frame pixel to a map-frame z plane.

    ``translation`` and ``rotation_xyzw`` describe map_T_camera_optical.
    Returns ``(x, y, z)`` or None if the ray is parallel or points away from
    the plane.
    """
    vals = (u, v, fx, fy, cx, cy, ground_z)
    if not all(math.isfinite(float(x)) for x in vals):
        raise ValueError('pixel and camera calibration values must be finite')
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError('focal lengths must be positive')

    ray_camera = np.array([(u - cx) / fx, (v - cy) / fy, 1.0],
                          dtype=np.float64)
    ray_map = quaternion_rotate(ray_camera, rotation_xyzw)
    origin = np.asarray(translation, dtype=np.float64)
    if origin.shape != (3,) or not np.all(np.isfinite(origin)):
        raise ValueError('translation must be a finite 3-vector')
    if abs(ray_map[2]) < 1e-9:
        return None
    distance = (float(ground_z) - origin[2]) / ray_map[2]
    if distance <= 0.0:
        return None
    point = origin + distance * ray_map
    return float(point[0]), float(point[1]), float(ground_z)
