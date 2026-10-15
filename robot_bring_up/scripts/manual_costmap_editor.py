#!/usr/bin/env python3
"""Turn two RViz point clicks into a persistent rectangular costmap edit."""

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from robot_interfaces.msg import CostmapRectEdit
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


class ManualCostmapEditor(Node):
    def __init__(self):
        super().__init__('manual_costmap_editor')
        durable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1000,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.edit_pub = self.create_publisher(
            CostmapRectEdit, '/manual_costmap/edit_rect', durable_qos)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/manual_costmap/rectangles', durable_qos)
        self.pending = {'add': None, 'erase': None}
        self.markers = []
        self.operation_stack = []
        self.undo_service = self.create_service(
            Trigger, '/manual_costmap/undo', self.undo_callback)
        self.create_subscription(
            PointStamped, '/manual_costmap/add_click',
            lambda msg: self.on_click('add', msg), 10)
        self.create_subscription(
            PointStamped, '/manual_costmap/erase_click',
            lambda msg: self.on_click('erase', msg), 10)
        self.get_logger().info(
            'RViz rectangle editor ready: choose Add Rectangle or Erase Rectangle; '
            'click opposite corners. Use the RViz Undo button to revert the last rectangle. '
            'Fixed frame must be map.')

    def on_click(self, mode, msg):
        if msg.header.frame_id and msg.header.frame_id != 'map':
            self.get_logger().error(
                'Rejected click in frame %s; RViz Fixed Frame must be map' % msg.header.frame_id)
            self.pending[mode] = None
            return
        point = (float(msg.point.x), float(msg.point.y))
        if self.pending[mode] is None:
            self.pending[mode] = point
            self.get_logger().info(
                '%s rectangle: first corner (%.2f, %.2f); click opposite corner' %
                (mode, point[0], point[1]))
            return

        first = self.pending[mode]
        self.pending[mode] = None
        min_x, max_x = sorted((first[0], point[0]))
        min_y, max_y = sorted((first[1], point[1]))
        if max_x - min_x < 0.02 or max_y - min_y < 0.02:
            self.get_logger().warning('Rectangle must be at least 2 cm wide and high; discarded')
            return

        edit = CostmapRectEdit()
        edit.header.stamp = self.get_clock().now().to_msg()
        edit.header.frame_id = 'map'
        edit.corner_a.x = min_x
        edit.corner_a.y = min_y
        edit.corner_b.x = max_x
        edit.corner_b.y = max_y
        edit.operation = CostmapRectEdit.ADD_OBSTACLE if mode == 'add' else CostmapRectEdit.FORCE_FREE
        self.edit_pub.publish(edit)
        marker = self.publish_marker(mode, min_x, min_y, max_x, max_y)
        self.operation_stack.append(marker)
        self.get_logger().warning(
            '%s rectangle applied: [%.2f, %.2f] - [%.2f, %.2f] map' %
            ('OBSTACLE' if mode == 'add' else 'FORCED FREE', min_x, min_y, max_x, max_y))

    def publish_marker(self, mode, min_x, min_y, max_x, max_y):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'manual_costmap_%s' % mode
        marker.id = len(self.markers)
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = (min_x + max_x) / 2.0
        marker.pose.position.y = (min_y + max_y) / 2.0
        # Slightly raise later edits so the most recent overlapping color remains visible.
        marker.pose.position.z = 0.015 + marker.id * 0.001
        marker.pose.orientation.w = 1.0
        marker.scale.x = max_x - min_x
        marker.scale.y = max_y - min_y
        marker.scale.z = 0.02
        marker.color.r = 1.0 if mode == 'add' else 0.1
        marker.color.g = 0.1 if mode == 'add' else 0.9
        marker.color.b = 0.1
        marker.color.a = 0.35
        self.markers.append(marker)
        array = MarkerArray()
        array.markers = self.markers
        self.marker_pub.publish(array)
        return marker

    def undo_callback(self, request, response):
        del request
        if not self.operation_stack:
            response.success = False
            response.message = 'There are no manual rectangles to undo.'
            return response

        marker = self.operation_stack.pop()
        edit = CostmapRectEdit()
        edit.header.stamp = self.get_clock().now().to_msg()
        edit.header.frame_id = 'map'
        edit.operation = CostmapRectEdit.UNDO
        self.edit_pub.publish(edit)

        delete_marker = Marker()
        delete_marker.header.frame_id = 'map'
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = marker.ns
        delete_marker.id = marker.id
        delete_marker.action = Marker.DELETE
        self.markers = [m for m in self.markers if (m.ns, m.id) != (marker.ns, marker.id)]
        array = MarkerArray()
        array.markers = [delete_marker] + self.markers
        self.marker_pub.publish(array)

        response.success = True
        response.message = 'Undid last rectangle; %d edits remain.' % len(self.operation_stack)
        self.get_logger().warning(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ManualCostmapEditor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
