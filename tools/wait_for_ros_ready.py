#!/usr/bin/env python3
"""Wait in the foreground for a fresh Point-LIO pose or connected MAVROS state."""

import argparse
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from mavros_msgs.msg import State


class ReadyWaiter(Node):
    def __init__(self, kind):
        super().__init__('full2_ready_waiter')
        self.ready = False
        if kind == 'pointlio':
            self.subscription = self.create_subscription(
                TransformStamped, '/robot/current_pose', self._pose_cb, 10)
        else:
            self.subscription = self.create_subscription(
                State, '/mavros/state', self._state_cb, 10)

    def _pose_cb(self, _msg):
        self.ready = True

    def _state_cb(self, msg):
        if msg.connected:
            self.ready = True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('kind', choices=('pointlio', 'mavros'))
    parser.add_argument('timeout_sec', type=float)
    args = parser.parse_args()
    if args.timeout_sec <= 0.0:
        parser.error('timeout_sec must be positive')

    rclpy.init()
    node = ReadyWaiter(args.kind)
    deadline = time.monotonic() + args.timeout_sec
    label = '/robot/current_pose' if args.kind == 'pointlio' else '/mavros/state connected=true'
    next_notice = 10
    try:
        while rclpy.ok() and not node.ready:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                print('TIMEOUT: %s not ready after %.1fs' % (label, args.timeout_sec))
                return 1
            rclpy.spin_once(node, timeout_sec=min(1.0, remaining))
            elapsed = args.timeout_sec - max(0.0, deadline - time.monotonic())
            if not node.ready and elapsed >= next_notice:
                print('Still waiting for %s: %.0f/%.0fs' %
                      (label, elapsed, args.timeout_sec), flush=True)
                next_notice += 10
        if node.ready:
            print('READY: %s' % label)
            return 0
        return 130
    except KeyboardInterrupt:
        print('Interrupted while waiting for %s' % args.kind)
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
