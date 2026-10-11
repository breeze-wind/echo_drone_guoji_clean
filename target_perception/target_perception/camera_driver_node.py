"""OpenCV V4L2 camera source publishing ROS Image and CameraInfo messages."""

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class CameraDriverNode(Node):
    def __init__(self):
        super().__init__('target_camera')
        self.declare_parameter('device_id', 1)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('fourcc', 'MJPG')
        self.declare_parameter('fx', 499.6183)
        self.declare_parameter('fy', 501.6010)
        self.declare_parameter('cx', 279.6429)
        self.declare_parameter('cy', 217.2060)
        self.declare_parameter('calibration_width', 640)
        self.declare_parameter('calibration_height', 480)
        self.declare_parameter(
            'distortion',
            [0.019251, -0.039608, 0.001383, -0.007422, -0.013079])

        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        if self.width <= 0 or self.height <= 0 or self.fps <= 0.0:
            raise ValueError('camera width, height and fps must be positive')

        cap = cv2.VideoCapture(
            int(self.get_parameter('device_id').value), cv2.CAP_V4L2)
        fourcc = str(self.get_parameter('fourcc').value)
        if len(fourcc) == 4:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            cap.release()
            raise RuntimeError(
                'cannot open camera device %s'
                % self.get_parameter('device_id').value)
        self.capture = cap
        actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(
            Image, str(self.get_parameter('image_topic').value),
            qos_profile_sensor_data)
        self.info_publisher = self.create_publisher(
            CameraInfo, str(self.get_parameter('camera_info_topic').value),
            qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / self.fps, self.publish_frame)
        self.get_logger().info(
            'Camera opened device=%s requested=%dx%d@%.1fHz actual=%dx%d@%.1fHz frame=%s'
            % (self.get_parameter('device_id').value,
               self.width, self.height, self.fps,
               actual_width, actual_height, actual_fps, self.frame_id))

    def _camera_info(self, stamp, width, height):
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.width = width
        info.height = height

        calibration_width = float(
            self.get_parameter('calibration_width').value)
        calibration_height = float(
            self.get_parameter('calibration_height').value)
        if calibration_width <= 0.0 or calibration_height <= 0.0:
            raise ValueError('calibration dimensions must be positive')
        sx = width / calibration_width
        sy = height / calibration_height
        fx = float(self.get_parameter('fx').value) * sx
        fy = float(self.get_parameter('fy').value) * sy
        cx = float(self.get_parameter('cx').value) * sx
        cy = float(self.get_parameter('cy').value) * sy
        info.distortion_model = 'plumb_bob'
        info.d = [float(value) for value in
                  self.get_parameter('distortion').value]
        info.k = [fx, 0.0, cx,
                  0.0, fy, cy,
                  0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0,
                  0.0, fy, cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        return info

    def publish_frame(self):
        ok, frame = self.capture.read()
        if not ok:
            self.get_logger().warning('camera frame read failed')
            return
        stamp = self.get_clock().now().to_msg()
        image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        image.header.stamp = stamp
        image.header.frame_id = self.frame_id
        info = self._camera_info(stamp, frame.shape[1], frame.shape[0])
        self.image_publisher.publish(image)
        self.info_publisher.publish(info)

    def destroy_node(self):
        if hasattr(self, 'capture'):
            self.capture.release()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraDriverNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError, ValueError) as exc:
        if node is not None and not isinstance(exc, KeyboardInterrupt):
            node.get_logger().error(str(exc))
        elif not isinstance(exc, KeyboardInterrupt):
            print('target_camera startup failed: %s' % exc)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
