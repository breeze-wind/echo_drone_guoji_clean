"""只读速度诊断使用的位姿微分与速度误差计算。"""

import math
from collections import deque


class PoseVelocityEstimator:
    """在同一里程计坐标系中，对短窗口位姿做线性回归估速。"""

    def __init__(self, window_sec=0.6, min_span_sec=0.2, max_speed_mps=3.0):
        self.window_sec = float(window_sec)
        self.min_span_sec = float(min_span_sec)
        self.max_speed_mps = float(max_speed_mps)
        self.samples = deque()
        self.frame_id = None

    def add(self, stamp_sec, x, y, frame_id):
        if not frame_id or not all(map(math.isfinite, (stamp_sec, x, y))):
            self.samples.clear()
            return False
        if self.frame_id != frame_id:
            self.samples.clear()
            self.frame_id = frame_id
        if self.samples:
            previous_t, previous_x, previous_y = self.samples[-1]
            dt = stamp_sec - previous_t
            if dt <= 0:
                # Duplicate stamps are common with odometry; ignore those.
                if dt == 0:
                    return False
                self.samples.clear()
            elif (dt > 1.0 or
                  math.hypot(x - previous_x, y - previous_y) / dt >
                  self.max_speed_mps):
                # Never interpret a Point-LIO reset/jump as real airspeed.
                self.samples.clear()
        self.samples.append((stamp_sec, x, y))
        while (self.samples and
               stamp_sec - self.samples[0][0] > self.window_sec):
            self.samples.popleft()
        return True

    def velocity(self):
        if len(self.samples) < 3:
            return None
        span = self.samples[-1][0] - self.samples[0][0]
        if span < self.min_span_sec:
            return None
        t0 = self.samples[0][0]
        time_values = [sample[0] - t0 for sample in self.samples]
        mean_t = sum(time_values) / len(time_values)
        variance_t = sum((t - mean_t) ** 2 for t in time_values)
        if variance_t <= 1e-9:
            return None
        mean_x = sum(sample[1] for sample in self.samples) / len(self.samples)
        mean_y = sum(sample[2] for sample in self.samples) / len(self.samples)
        vx = sum((t - mean_t) * (sample[1] - mean_x)
                 for t, sample in zip(time_values, self.samples)) / variance_t
        vy = sum((t - mean_t) * (sample[2] - mean_y)
                 for t, sample in zip(time_values, self.samples)) / variance_t
        return vx, vy


def world_to_body(vx, vy, yaw):
    """绕 z 轴逆转 yaw，得到机器人机体系 x/y 速度。"""
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return cosine * vx + sine * vy, -sine * vx + cosine * vy


def compare_velocity(desired, actual, moving_threshold=0.03):
    """计算实际沿期望方向的速度、横向漂移及向量误差。"""
    requested_speed = math.hypot(*desired)
    actual_speed = math.hypot(*actual)
    result = {
        'requested_speed_mps': requested_speed,
        'actual_speed_mps': actual_speed,
        'error_xy_mps': [desired[0] - actual[0],
                         desired[1] - actual[1]],
        'error_norm_mps': math.hypot(desired[0] - actual[0],
                                     desired[1] - actual[1]),
        'along_track_mps': None,
        'cross_track_mps': None,
        'along_track_ratio': None,
    }
    if requested_speed >= moving_threshold:
        ux, uy = desired[0] / requested_speed, desired[1] / requested_speed
        along = actual[0] * ux + actual[1] * uy
        result['along_track_mps'] = along
        result['cross_track_mps'] = -actual[0] * uy + actual[1] * ux
        result['along_track_ratio'] = along / requested_speed
    return result
