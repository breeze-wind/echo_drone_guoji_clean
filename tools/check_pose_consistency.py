#!/usr/bin/env python3
"""Compare Point-LIO current pose with the pose forwarded to MAVROS.

Run after sourcing ROS Foxy and this workspace's install/setup.bash.
The default conversion parameters match flight_control/config/mavros_adapter.yaml.
"""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node

from flight_control.conversions import (
    LEGACY_COORDINATE_MODE,
    MAVROS_ENU_COORDINATE_MODE,
    legacy_vision_orientation_to_mavros_enu,
    legacy_vision_pose_position,
)


def stamp_key(stamp):
    return stamp.sec, stamp.nanosec


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def quaternion_error_deg(expected, actual):
    def normalized(q):
        norm = math.sqrt(sum(value * value for value in q))
        if norm < 1e-12:
            return (0.0, 0.0, 0.0, 1.0)
        return tuple(value / norm for value in q)

    q1 = normalized(expected)
    q2 = normalized(actual)
    dot = abs(sum(a * b for a, b in zip(q1, q2)))
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))


class PoseConsistencyChecker(Node):
    def __init__(self, args):
        super().__init__('pose_consistency_checker')
        self.args = args
        self.sources = {}
        self.vision = {}
        self.source_count = 0
        self.vision_count = 0
        self.paired_count = 0
        self.failed_count = 0
        self.max_position_error = 0.0
        self.max_orientation_error = 0.0
        self.unmatched_source = 0
        self.unmatched_vision = 0
        self.previous_source_position = None
        self.previous_vision_position = None

        self.source_sub = self.create_subscription(
            TransformStamped, args.source_topic, self.source_callback, 50)
        self.vision_sub = self.create_subscription(
            PoseStamped, args.vision_topic, self.vision_callback, 50)

    def source_callback(self, msg):
        self.source_count += 1
        key = stamp_key(msg.header.stamp)
        self.sources[key] = msg
        self.try_pair(key)
        self.trim_pending()

    def vision_callback(self, msg):
        self.vision_count += 1
        key = stamp_key(msg.header.stamp)
        self.vision[key] = msg
        self.try_pair(key)
        self.trim_pending()

    def trim_pending(self):
        # Bound memory if a message from one side is lost or timestamps differ.
        for pending in (self.sources, self.vision):
            while len(pending) > 200:
                pending.pop(next(iter(pending)))

    def try_pair(self, key):
        source = self.sources.get(key)
        vision = self.vision.get(key)
        if source is None or vision is None:
            return

        del self.sources[key]
        del self.vision[key]
        self.paired_count += 1

        p = source.transform.translation
        q = source.transform.rotation
        if self.args.mode == LEGACY_COORDINATE_MODE:
            expected_position = legacy_vision_pose_position(
                p.x, p.y, p.z, self.args.z_offset)
            expected_orientation = legacy_vision_orientation_to_mavros_enu(q)
        else:
            expected_position = (p.x, p.y, p.z + self.args.z_offset)
            expected_orientation = (q.x, q.y, q.z, q.w)

        actual_position = (
            vision.pose.position.x,
            vision.pose.position.y,
            vision.pose.position.z,
        )
        actual_orientation = (
            vision.pose.orientation.x,
            vision.pose.orientation.y,
            vision.pose.orientation.z,
            vision.pose.orientation.w,
        )
        position_delta = tuple(
            actual - expected
            for actual, expected in zip(actual_position, expected_position)
        )
        position_error = math.sqrt(sum(value * value for value in position_delta))
        orientation_error = quaternion_error_deg(
            expected_orientation, actual_orientation)
        self.max_position_error = max(self.max_position_error, position_error)
        self.max_orientation_error = max(
            self.max_orientation_error, orientation_error)

        passed = (
            position_error <= self.args.position_tolerance
            and orientation_error <= self.args.orientation_tolerance_deg
        )
        if not passed:
            self.failed_count += 1

        source_delta = self.delta(self.previous_source_position, (p.x, p.y, p.z))
        vision_delta = self.delta(self.previous_vision_position, actual_position)
        self.previous_source_position = (p.x, p.y, p.z)
        self.previous_vision_position = actual_position

        source_frame = source.header.frame_id or '<empty>'
        vision_frame = vision.header.frame_id or '<empty>'
        status = 'PASS' if passed else 'FAIL'
        print(
            f'#{self.paired_count} {status} '
            f'stamp_skew={abs(stamp_seconds(source.header.stamp) - stamp_seconds(vision.header.stamp))*1000:.3f}ms '
            f'pos_err={position_error:.5f}m rot_err={orientation_error:.4f}deg '
            f'source_frame={source_frame} vision_frame={vision_frame}\n'
            f'  src={self.fmt((p.x, p.y, p.z))} '
            f'expected={self.fmt(expected_position)} '
            f'vision={self.fmt(actual_position)} '
            f'error={self.fmt(position_delta)}\n'
            f'  delta_src={self.fmt(source_delta)} '
            f'delta_vision={self.fmt(vision_delta)}',
            flush=True,
        )

    @staticmethod
    def delta(previous, current):
        if previous is None:
            return (0.0, 0.0, 0.0)
        return tuple(now - before for before, now in zip(previous, current))

    @staticmethod
    def fmt(values):
        return '(' + ', '.join(f'{value:+.4f}' for value in values) + ')'

    def summary(self):
        self.unmatched_source = len(self.sources)
        self.unmatched_vision = len(self.vision)
        print('\nPose consistency summary:')
        print(
            f'  received: source={self.source_count}, vision={self.vision_count}; '
            f'exact timestamp pairs={self.paired_count}')
        print(
            f'  failed pairs={self.failed_count}; '
            f'max position error={self.max_position_error:.5f} m; '
            f'max orientation error={self.max_orientation_error:.4f} deg')
        print(
            f'  unmatched pending: source={self.unmatched_source}, '
            f'vision={self.unmatched_vision}')
        if self.paired_count == 0:
            print('  No exact timestamp pairs. Check that both nodes are running and that stamps are copied through.')
        print(
            '  This verifies adapter math. Physical axis correctness still requires moving the sensor/airframe and checking delta signs.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-topic', default='/robot/current_pose')
    parser.add_argument('--vision-topic', default='/mavros/mavros/pose')
    parser.add_argument(
        '--mode',
        choices=(LEGACY_COORDINATE_MODE, MAVROS_ENU_COORDINATE_MODE),
        default=LEGACY_COORDINATE_MODE,
        help='Must match flight_control/config/mavros_adapter.yaml')
    parser.add_argument('--z-offset', type=float, default=0.31)
    parser.add_argument('--position-tolerance', type=float, default=0.01,
                        help='Maximum Euclidean position error in meters')
    parser.add_argument('--orientation-tolerance-deg', type=float, default=1.0)
    parser.add_argument('--duration-sec', type=float, default=30.0)
    args = parser.parse_args()

    rclpy.init()
    node = PoseConsistencyChecker(args)
    print(
        f'Checking {args.source_topic} -> {args.vision_topic}; '
        f'mode={args.mode}, z_offset={args.z_offset:.3f} m, '
        f'duration={args.duration_sec:.1f} s',
        flush=True,
    )
    print('Move the disarmed aircraft gently by hand to inspect delta signs.', flush=True)

    deadline = time.monotonic() + args.duration_sec
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.destroy_node()
        rclpy.shutdown()

    return 1 if node.paired_count == 0 or node.failed_count else 0


if __name__ == '__main__':
    raise SystemExit(main())
