"""Project camera-frame target rays onto the map ground plane."""

import copy
import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node

from robot_interfaces.msg import TargetDetectionArray


def _rotate(vector, quaternion):
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9 or not math.isfinite(norm):
        raise ValueError('pose/extrinsic quaternion is invalid')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    qv = (x, y, z)
    cross1 = (qv[1] * vector[2] - qv[2] * vector[1],
              qv[2] * vector[0] - qv[0] * vector[2],
              qv[0] * vector[1] - qv[1] * vector[0])
    cross2 = (qv[1] * cross1[2] - qv[2] * cross1[1],
              qv[2] * cross1[0] - qv[0] * cross1[2],
              qv[0] * cross1[1] - qv[1] * cross1[0])
    return tuple(vector[i] + 2.0 * (w * cross1[i] + cross2[i])
                 for i in range(3))


def _stamp_ns(header):
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


class MapTargetLocalizer(Node):
    def __init__(self):
        super().__init__('map_target_localizer')
        self.declare_parameter('rays_topic', '/target_perception/rays')
        self.declare_parameter('targets_topic', '/target_perception/targets')
        self.declare_parameter('current_pose_topic', '/robot/current_pose')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('pose_source_frame', 'odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('pose_z_offset', 0.39)
        self.declare_parameter('ground_z_map', 0.0)
        self.declare_parameter('pose_sync_tolerance_sec', 0.30)
        self.declare_parameter('pose_wait_timeout_sec', 1.0)
        self.declare_parameter('camera_in_livox_translation', [-0.08, 0.0, -0.20])
        self.declare_parameter('camera_in_livox_rotation_xyzw', [0.0, 1.0, 0.0, 0.0])

        get = self.get_parameter
        self.rays_topic = str(get('rays_topic').value)
        self.targets_topic = str(get('targets_topic').value)
        self.camera_frame = str(get('camera_frame').value)
        self.pose_source_frame = str(get('pose_source_frame').value)
        self.map_frame = str(get('map_frame').value)
        self.pose_z_offset = float(get('pose_z_offset').value)
        self.ground_z_map = float(get('ground_z_map').value)
        self.sync_tolerance = float(get('pose_sync_tolerance_sec').value)
        self.wait_timeout = float(get('pose_wait_timeout_sec').value)
        self.camera_translation = tuple(
            float(v) for v in get('camera_in_livox_translation').value)
        self.camera_rotation = tuple(
            float(v) for v in get('camera_in_livox_rotation_xyzw').value)
        if (len(self.camera_translation) != 3 or len(self.camera_rotation) != 4
                or self.sync_tolerance <= 0.0 or self.wait_timeout <= 0.0):
            raise ValueError('invalid map target localizer configuration')

        self.pose_history = deque(maxlen=300)
        self.pending = deque(maxlen=30)
        self.last_warning_time = 0.0
        self.publisher = self.create_publisher(
            TargetDetectionArray, self.targets_topic, 10)
        self.create_subscription(
            TransformStamped, str(get('current_pose_topic').value),
            self._pose_cb, 50)
        self.create_subscription(
            TargetDetectionArray, self.rays_topic, self._rays_cb, 10)
        self.create_timer(0.02, self._process_pending)
        self.get_logger().info(
            'Map target localization: %s (%s rays) + %s image-time pose -> %s; '
            'livox_T_camera translation=%s rotation_xyzw=%s'
            % (self.rays_topic, self.camera_frame,
               str(get('current_pose_topic').value), self.map_frame,
               self.camera_translation, self.camera_rotation))

    def _pose_cb(self, msg):
        if msg.header.frame_id not in (self.pose_source_frame, self.map_frame):
            return
        self.pose_history.append((_stamp_ns(msg.header), msg))

    def _rays_cb(self, msg):
        if msg.header.frame_id != self.camera_frame:
            self._warn('Ignoring rays in frame %s; expected %s' %
                       (msg.header.frame_id, self.camera_frame))
            return
        self.pending.append((time.monotonic(), msg))

    def _warn(self, message):
        now = time.monotonic()
        if now - self.last_warning_time >= 1.0:
            self.last_warning_time = now
            self.get_logger().warning(message)

    def _pose_at(self, stamp_ns):
        if not self.pose_history:
            return None, None
        pose_stamp, pose = min(
            self.pose_history, key=lambda entry: abs(entry[0] - stamp_ns))
        skew = abs(pose_stamp - stamp_ns) / 1e9
        return (pose, skew) if skew <= self.sync_tolerance else (None, skew)

    def _project(self, detection, pose_msg):
        ray = (float(detection.camera_ray.x), float(detection.camera_ray.y),
               float(detection.camera_ray.z))
        if not all(math.isfinite(v) for v in ray):
            return None
        norm = math.sqrt(sum(v * v for v in ray))
        if norm < 1e-9:
            return None
        ray = tuple(v / norm for v in ray)

        pose = pose_msg.transform
        pose_q = (pose.rotation.x, pose.rotation.y,
                  pose.rotation.z, pose.rotation.w)
        ray_livox = _rotate(ray, self.camera_rotation)
        ray_map = _rotate(ray_livox, pose_q)
        camera_offset = _rotate(self.camera_translation, pose_q)
        z_offset = (self.pose_z_offset
                    if pose_msg.header.frame_id == self.pose_source_frame else 0.0)
        origin = (pose.translation.x + camera_offset[0],
                  pose.translation.y + camera_offset[1],
                  pose.translation.z + z_offset + camera_offset[2])
        if abs(ray_map[2]) < 1e-8:
            return None
        distance = (self.ground_z_map - origin[2]) / ray_map[2]
        if distance <= 0.0:
            return None
        return (origin[0] + distance * ray_map[0],
                origin[1] + distance * ray_map[1], self.ground_z_map)

    def _process_pending(self):
        now = time.monotonic()
        for _ in range(len(self.pending)):
            received_at, rays = self.pending.popleft()
            pose, skew = self._pose_at(_stamp_ns(rays.header))
            if pose is None:
                if now - received_at <= self.wait_timeout:
                    self.pending.append((received_at, rays))
                else:
                    self._warn('Dropping rays: no pose within %.0f ms of image stamp '
                               '(nearest skew=%s)' %
                               (self.sync_tolerance * 1000.0,
                                'unavailable' if skew is None else
                                '%.0f ms' % (skew * 1000.0)))
                continue

            localized = TargetDetectionArray()
            localized.header = copy.deepcopy(rays.header)
            localized.header.frame_id = self.map_frame
            for detection in rays.detections:
                point = self._project(detection, pose)
                if point is None:
                    continue
                mapped = copy.deepcopy(detection)
                mapped.ground_position.x = point[0]
                mapped.ground_position.y = point[1]
                mapped.ground_position.z = point[2]
                localized.detections.append(mapped)
            self.publisher.publish(localized)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MapTargetLocalizer()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError, ValueError) as exc:
        if node is not None and not isinstance(exc, KeyboardInterrupt):
            node.get_logger().error(str(exc))
        elif not isinstance(exc, KeyboardInterrupt):
            print('map_target_localizer startup failed: %s' % exc)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
