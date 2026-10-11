"""Generate, print, and publish a lawnmower route (preview only)."""

import math

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from coverage_search.route import generate_lawnmower_route


class CoverageWaypointNode(Node):
    def __init__(self):
        super().__init__('coverage_waypoint_generator')
        self.declare_parameter('corner_a', [])
        self.declare_parameter('corner_b', [])
        self.declare_parameter('lane_spacing', 1.4)
        self.declare_parameter('search_height', 2.0)
        self.declare_parameter('lane_axis', 'auto')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('waypoints_topic', '/coverage_search/waypoints')

        corner_a = list(self.get_parameter('corner_a').value)
        corner_b = list(self.get_parameter('corner_b').value)
        lane_spacing = float(self.get_parameter('lane_spacing').value)
        search_height = float(self.get_parameter('search_height').value)
        lane_axis = str(self.get_parameter('lane_axis').value).lower()
        self.frame_id = str(self.get_parameter('frame_id').value)
        topic = str(self.get_parameter('waypoints_topic').value)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(PoseArray, topic, qos)

        try:
            route = generate_lawnmower_route(
                corner_a, corner_b, lane_spacing, search_height, lane_axis)
        except (TypeError, ValueError) as exc:
            self.get_logger().error('Cannot generate route: %s' % exc)
            raise

        message = PoseArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        for index, (x, y, z, yaw) in enumerate(route, start=1):
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation.z = math.sin(yaw / 2.0)
            pose.orientation.w = math.cos(yaw / 2.0)
            message.poses.append(pose)
            self.get_logger().info(
                'waypoint[%02d] frame=%s x=%.3f y=%.3f z=%.3f yaw=%.1f deg'
                % (index, self.frame_id, x, y, z, math.degrees(yaw)))

        self.publisher.publish(message)
        self.get_logger().info(
            'Published %d preview waypoints on %s; this node does not send '
            'navigation goals or control the aircraft.' % (len(route), topic))


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CoverageWaypointNode()
        # Keep the transient-local publisher alive for late subscribers.
        rclpy.spin(node)
    except (TypeError, ValueError):
        return 2
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
