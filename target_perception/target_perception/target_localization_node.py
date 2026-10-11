"""YOLO target detection and camera-frame target-ray ROS 2 node."""

import math
import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from robot_interfaces.msg import TargetDetection, TargetDetectionArray


CLASS_NAMES = {
    0: '红十字', 1: '黑H', 2: '营帐', 3: '车',
    4: '堡垒', 5: '坦克', 6: '桥',
}


def refine_circle(frame, bbox, minimum_score=0.70, pad=6):
    """Fit the dominant approximately circular contour inside a YOLO box."""
    height, width = frame.shape[:2]
    x1, y1, x2, y2 = bbox
    rx1, ry1 = max(0, x1 - pad), max(0, y1 - pad)
    rx2, ry2 = min(width, x2 + pad), min(height, y2 + pad)
    if rx2 - rx1 < 8 or ry2 - ry1 < 8:
        return None

    roi = frame[ry1:ry2, rx1:rx2]
    gray = cv2.GaussianBlur(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY), (3, 3), 0)
    radius_guess = (((x2 - x1) + (y2 - y1)) * 0.5) * 0.5
    center_roi = (roi.shape[1] * 0.5, roi.shape[0] * 0.5)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    _, threshold = cv2.threshold(
        gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    best = None

    for mask in (threshold, 255 - threshold):
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 50:
                continue
            moments = cv2.moments(contour)
            if moments['m00'] < 1e-6:
                continue
            cmx = moments['m10'] / moments['m00']
            cmy = moments['m01'] / moments['m00']
            if math.hypot(cmx - center_roi[0], cmy - center_roi[1]) > radius_guess * 1.2:
                continue
            _, enclosing_radius = cv2.minEnclosingCircle(contour)
            if enclosing_radius < 3:
                continue
            score = area / (math.pi * enclosing_radius * enclosing_radius)
            if score < minimum_score:
                continue
            points = contour.reshape(-1, 2).astype(np.float64)
            if len(points) < 8:
                continue
            matrix = np.c_[2 * points[:, 0], 2 * points[:, 1],
                           np.ones(len(points))]
            rhs = points[:, 0] ** 2 + points[:, 1] ** 2
            solution, *_ = np.linalg.lstsq(matrix, rhs, rcond=None)
            fit_x, fit_y = solution[0], solution[1]
            fit_radius = math.sqrt(max(0.0, solution[2] + fit_x ** 2 + fit_y ** 2))
            if not (3 < fit_radius < radius_guess * 1.5):
                continue
            if math.hypot(fit_x - center_roi[0], fit_y - center_roi[1]) > radius_guess * 0.8:
                continue
            if best is None or score > best[3]:
                best = (rx1 + fit_x, ry1 + fit_y, fit_radius, float(score))
    return best


class TargetLocalizationNode(Node):
    def __init__(self):
        super().__init__('target_localization')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('detections_topic', '/target_perception/rays')
        self.declare_parameter('debug_image_topic', '/target_perception/debug_image')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('show_window', True)
        self.declare_parameter('camera_frame', '')
        self.declare_parameter('model_path', '/home/echo/Downloads/best.pt')
        self.declare_parameter('confidence', 0.55)
        self.declare_parameter('iou', 0.5)
        self.declare_parameter('image_size', 640)
        self.declare_parameter('device', '0')
        self.declare_parameter('half_precision', True)
        self.declare_parameter('minimum_circle_score', 0.70)
        self.declare_parameter('maximum_rate_hz', 10.0)
        self.declare_parameter('fx', 499.6183)
        self.declare_parameter('fy', 501.6010)
        self.declare_parameter('cx', 279.6429)
        self.declare_parameter('cy', 217.2060)
        self.declare_parameter('calibration_width', 640)
        self.declare_parameter('calibration_height', 480)
        self.declare_parameter(
            'distortion', [0.019251, -0.039608, 0.001383, -0.007422, -0.013079])

        self.camera_frame_override = str(self.get_parameter('camera_frame').value)
        self.publish_debug = bool(self.get_parameter('publish_debug_image').value)
        self.show_window = bool(self.get_parameter('show_window').value)
        self.window_created = False
        if self.show_window and not os.environ.get('DISPLAY'):
            self.show_window = False
            self.get_logger().warn(
                'DISPLAY is not set; OpenCV UI disabled (ROS debug image remains enabled)')
        self.maximum_rate_hz = float(self.get_parameter('maximum_rate_hz').value)
        self.minimum_circle_score = float(
            self.get_parameter('minimum_circle_score').value)

        model_path = os.path.expanduser(str(self.get_parameter('model_path').value))
        if not os.path.isfile(model_path):
            raise RuntimeError('YOLO model does not exist: %s' % model_path)
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                'ultralytics is required in the ROS Python environment') from exc
        try:
            from target_perception.cuda_nms_compat import install_cuda_nms_fallback
            install_cuda_nms_fallback(self.get_logger())
        except Exception as exc:
            raise RuntimeError('Could not initialize TorchVision NMS compatibility: %s'
                               % exc) from exc
        self.model = YOLO(model_path)
        self.confidence = float(self.get_parameter('confidence').value)
        self.iou = float(self.get_parameter('iou').value)
        self.image_size = int(self.get_parameter('image_size').value)
        self.device = str(self.get_parameter('device').value)
        self.half_precision = bool(self.get_parameter('half_precision').value)

        self.bridge = CvBridge()
        self.camera_info = None
        self.camera_matrix = None
        self.distortion = None
        self.undistort_maps = None
        self.map_size = None
        self.last_inference_time = 0.0
        self.last_log_time = 0.0
        self.last_frame_time = 0.0
        self.smoothed_fps = 0.0

        self.publisher = self.create_publisher(
            TargetDetectionArray,
            str(self.get_parameter('detections_topic').value), 10)
        self.debug_publisher = None
        if self.publish_debug:
            self.debug_publisher = self.create_publisher(
                Image, str(self.get_parameter('debug_image_topic').value), 1)

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self.camera_info_callback,
            qos_profile_sensor_data)
        self.image_sub = self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self.image_callback,
            qos_profile_sensor_data)
        self.get_logger().info(
            'YOLO ready model=%s image=%s camera_info=%s output=%s camera_frame=%s'
            % (model_path, self.get_parameter('image_topic').value,
               self.get_parameter('camera_info_topic').value,
               self.get_parameter('detections_topic').value,
               self.camera_frame_override or 'image/CameraInfo frame_id'))

    def camera_info_callback(self, message):
        if message.k[0] <= 0.0 or message.k[4] <= 0.0:
            return
        self.camera_info = message
        self.camera_matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
        self.distortion = np.asarray(message.d, dtype=np.float64)
        if self.distortion.size == 0:
            self.distortion = np.zeros(5, dtype=np.float64)

    def _calibration_for_size(self, width, height):
        if self.camera_info is not None:
            info = self.camera_info
            matrix = self.camera_matrix.copy()
            distortion = self.distortion.copy()
            ref_width, ref_height = info.width, info.height
        else:
            matrix = np.array([
                [float(self.get_parameter('fx').value), 0.0,
                 float(self.get_parameter('cx').value)],
                [0.0, float(self.get_parameter('fy').value),
                 float(self.get_parameter('cy').value)],
                [0.0, 0.0, 1.0]], dtype=np.float64)
            distortion = np.asarray(
                self.get_parameter('distortion').value, dtype=np.float64)
            ref_width = int(self.get_parameter('calibration_width').value)
            ref_height = int(self.get_parameter('calibration_height').value)
        if ref_width <= 0 or ref_height <= 0:
            raise ValueError('invalid camera calibration resolution')
        sx, sy = width / float(ref_width), height / float(ref_height)
        matrix[0, :] *= sx
        matrix[1, :] *= sy
        matrix[2, 2] = 1.0
        return matrix, distortion

    def _undistort(self, frame):
        height, width = frame.shape[:2]
        if self.map_size != (width, height):
            matrix, distortion = self._calibration_for_size(width, height)
            self.camera_matrix = matrix
            self.distortion = distortion
            self.undistort_maps = cv2.initUndistortRectifyMap(
                matrix, distortion, None, matrix, (width, height), cv2.CV_32FC1)
            self.map_size = (width, height)
        return cv2.remap(frame, self.undistort_maps[0], self.undistort_maps[1],
                         cv2.INTER_LINEAR)

    def image_callback(self, message):
        now = time.monotonic()
        if self.maximum_rate_hz > 0.0 and now - self.last_inference_time < 1.0 / self.maximum_rate_hz:
            return
        self.last_inference_time = now
        callback_start = time.monotonic()
        try:
            raw = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            frame = self._undistort(raw)
        except Exception as exc:
            self.get_logger().error('Could not decode/calibrate image: %s' % exc)
            return

        camera_frame = self.camera_frame_override
        if not camera_frame and self.camera_info is not None:
            camera_frame = self.camera_info.header.frame_id
        if not camera_frame:
            camera_frame = message.header.frame_id
        if not camera_frame:
            self.get_logger().warn('Image has no optical frame_id; skipping localization')
            return

        fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
        cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
        yolo_start = time.monotonic()
        try:
            predictions = self.model.predict(
                source=frame,
                conf=self.confidence,
                iou=self.iou,
                imgsz=self.image_size,
                device=self.device,
                half=self.half_precision,
                verbose=False)
        except Exception as exc:
            self.get_logger().error('YOLO inference failed: %s' % exc)
            return
        yolo_elapsed = time.monotonic() - yolo_start

        output = TargetDetectionArray()
        output.header = message.header
        # Pixels/rays remain in the calibrated camera frame. The mission node
        # combines this with the image-time vehicle pose and static extrinsic.
        output.header.frame_id = camera_frame
        annotated = (frame.copy()
                     if self.debug_publisher is not None or self.show_window
                     else None)
        ui_targets = []
        for result in predictions:
            if result.boxes is None:
                continue
            for box in result.boxes:
                x1, y1, x2, y2 = [int(round(v)) for v in
                                  box.xyxy[0].detach().cpu().tolist()]
                class_id = int(box.cls[0].detach().cpu().item())
                confidence = float(box.conf[0].detach().cpu().item())
                if class_id not in CLASS_NAMES:
                    continue
                bbox = (x1, y1, x2, y2)
                refined = refine_circle(frame, bbox, self.minimum_circle_score)
                if refined is None:
                    pixel_x, pixel_y = (x1 + x2) * 0.5, (y1 + y2) * 0.5
                    radius = 0.0
                    fit_score = 0.0
                    method = 'bbox_center'
                else:
                    pixel_x, pixel_y, radius, fit_score = refined
                    method = 'circle_fit'
                ray = ((pixel_x - cx) / fx, (pixel_y - cy) / fy, 1.0)
                ray_norm = math.sqrt(sum(component * component for component in ray))
                ray = tuple(component / ray_norm for component in ray)

                ui_targets.append((bbox, pixel_x, pixel_y, radius, confidence,
                                   class_id, method, ray))

                detection = TargetDetection()
                detection.class_id = class_id
                detection.class_name = CLASS_NAMES[class_id]
                detection.confidence = confidence
                detection.pixel_x = float(pixel_x)
                detection.pixel_y = float(pixel_y)
                detection.circle_radius_px = float(radius)
                detection.circle_fit_score = float(fit_score)
                detection.camera_ray.x = ray[0]
                detection.camera_ray.y = ray[1]
                detection.camera_ray.z = ray[2]
                detection.localization_method = method
                output.detections.append(detection)

        if annotated is not None:
            frame_dt = (callback_start - self.last_frame_time
                        if self.last_frame_time else 0.0)
            self.last_frame_time = callback_start
            instant_fps = 1.0 / frame_dt if frame_dt > 0.0 else 0.0
            self.smoothed_fps = (
                0.9 * self.smoothed_fps + 0.1 * instant_fps
                if self.smoothed_fps > 0.0 else instant_fps)
            for (bbox, pixel_x, pixel_y, radius, confidence, class_id,
                 method, ray) in ui_targets:
                x1, y1, x2, y2 = bbox
                color = (0, 200, 0)
                cv2.rectangle(annotated, (x1, y1), (x2, y2),
                              (150, 150, 150), 1)
                if method == 'circle_fit':
                    cv2.circle(annotated,
                               (int(round(pixel_x)), int(round(pixel_y))),
                               int(round(radius)), color, 2)
                cv2.drawMarker(annotated,
                               (int(round(pixel_x)), int(round(pixel_y))),
                               (0, 0, 255), cv2.MARKER_CROSS, 12, 2)
                source = 'circle' if method == 'circle_fit' else 'bbox'
                label = '%s %.2f [%s]' % (
                    CLASS_NAMES[class_id], confidence, source)
                label += ' ray=(%+.2f,%+.2f,%+.2f)' % ray
                (text_width, text_height), _ = cv2.getTextSize(
                    label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                label_top = max(0, y1 - text_height - 6)
                cv2.rectangle(annotated, (x1, label_top),
                              (x1 + text_width + 6, y1), color, -1)
                cv2.putText(annotated, label,
                            (x1 + 3, max(text_height + 2, y1 - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (255, 255, 255), 1)
            cv2.putText(
                annotated,
                'FPS=%.1f  YOLO=%.0fms  Targets=%d' % (
                    self.smoothed_fps, yolo_elapsed * 1000.0, len(ui_targets)),
                (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)
            if self.show_window:
                try:
                    if not self.window_created:
                        cv2.namedWindow('YOLO Circle Localization',
                                        cv2.WINDOW_NORMAL)
                        self.window_created = True
                    cv2.imshow('YOLO Circle Localization', annotated)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        cv2.destroyWindow('YOLO Circle Localization')
                        self.show_window = False
                        self.window_created = False
                except cv2.error as exc:
                    self.get_logger().error(
                        'OpenCV UI unavailable; continuing headless: %s' % exc)
                    self.show_window = False

        self.publisher.publish(output)
        if annotated is not None:
            debug_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            debug_msg.header = message.header
            self.debug_publisher.publish(debug_msg)
        if output.detections and now - self.last_log_time >= 1.0:
            self.last_log_time = now
            for detection in output.detections:
                self.get_logger().info(
                    '靶标 %s id=%d conf=%.2f pixel=(%.1f, %.1f) '
                    'camera_ray=(%+.3f, %+.3f, %+.3f) via=%s'
                    % (detection.class_name, detection.class_id,
                       detection.confidence, detection.pixel_x,
                       detection.pixel_y, detection.camera_ray.x,
                       detection.camera_ray.y, detection.camera_ray.z,
                       detection.localization_method))

    def destroy_node(self):
        if self.window_created:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TargetLocalizationNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError, ValueError) as exc:
        if node is not None and not isinstance(exc, KeyboardInterrupt):
            node.get_logger().error(str(exc))
        elif not isinstance(exc, KeyboardInterrupt):
            print('target_localization startup failed: %s' % exc)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
