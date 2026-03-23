#!/usr/bin/env python3
"""
Requirement 5: Traffic Rules
"""

from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from yolo_msgs.msg import DetectionArray


class TrafficSignController(Node):
    def __init__(self) -> None:
        super().__init__('traffic_sign_controller')

        self.declare_parameter('input_cmd_topic', '/manual_cmd_vel')
        self.declare_parameter('output_cmd_topic', '/atlas/cmd_vel')
        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter('status_topic', '/traffic_sign/status')
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('detection_timeout', 1.0)
        self.declare_parameter('min_confidence', 0.50)
        self.declare_parameter('min_box_width', 40.0)
        self.declare_parameter('stop_duration', 3.0)
        self.declare_parameter('slow_speed_limit', 0.08)
        self.declare_parameter('fast_speed_limit', 0.60)
        self.declare_parameter('default_speed_limit', 0.50)
        self.declare_parameter('only_react_to_centered_signs', True)
        self.declare_parameter('center_tolerance_pixels', 160.0)

        input_cmd_topic = self.get_parameter('input_cmd_topic').value
        output_cmd_topic = self.get_parameter('output_cmd_topic').value
        detections_topic = self.get_parameter('detections_topic').value
        status_topic = self.get_parameter('status_topic').value
        control_rate_hz = float(self.get_parameter('control_rate_hz').value)

        self.detection_timeout = float(self.get_parameter('detection_timeout').value)
        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.min_box_width = float(self.get_parameter('min_box_width').value)
        self.stop_duration = float(self.get_parameter('stop_duration').value)
        self.slow_speed_limit = float(self.get_parameter('slow_speed_limit').value)
        self.fast_speed_limit = float(self.get_parameter('fast_speed_limit').value)
        self.default_speed_limit = float(self.get_parameter('default_speed_limit').value)
        self.only_centered = bool(self.get_parameter('only_react_to_centered_signs').value)
        self.center_tolerance_pixels = float(self.get_parameter('center_tolerance_pixels').value)

        self.cmd_sub = self.create_subscription(Twist, input_cmd_topic, self.cmd_cb, 10)
        self.det_sub = self.create_subscription(DetectionArray, detections_topic, self.detections_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, output_cmd_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.timer = self.create_timer(1.0 / max(control_rate_hz, 1.0), self.timer_cb)

        self.last_manual_cmd = Twist()
        self.active_sign = 'none'
        self.last_sign_stamp = self.get_clock().now()
        self.stop_until = self.get_clock().now()
        self.last_stop_trigger_time = self.get_clock().now()
        self.stop_cooldown = 4.0
        self.last_status_sent = ''

        self.get_logger().info('TrafficSignController started')
        self.get_logger().info(f'Listening for manual commands on: {input_cmd_topic}')
        self.get_logger().info(f'Publishing filtered commands to: {output_cmd_topic}')
        self.get_logger().info(f'Listening for detections on: {detections_topic}')

    def cmd_cb(self, msg: Twist) -> None:
        self.last_manual_cmd = msg

    def detections_cb(self, msg: DetectionArray) -> None:
        best_label = None
        best_score = -1.0
        best_width = 0.0
        best_center_offset = float('inf')

        for det in msg.detections:
            label = self._extract_label(det)
            score = self._extract_score(det)
            width, center_x = self._extract_box_info(det)

            if label not in {'stop_sign', 'slow_sign', 'fast_sign'}:
                continue
            if score < self.min_confidence:
                continue
            if width < self.min_box_width:
                continue

            center_offset = abs(center_x)
            if self.only_centered and center_offset > self.center_tolerance_pixels:
                continue

            rank = (score, width, -center_offset)
            current = (best_score, best_width, -best_center_offset)
            if rank > current:
                best_label = label
                best_score = score
                best_width = width
                best_center_offset = center_offset

        if best_label is None:
            return

        now = self.get_clock().now()
        self.active_sign = best_label
        self.last_sign_stamp = now

        if best_label == 'stop_sign':
            time_since_last_stop = (now - self.last_stop_trigger_time).nanoseconds / 1e9
            if now >= self.stop_until and time_since_last_stop > self.stop_cooldown:
                self.stop_until = now + rclpy.duration.Duration(seconds=self.stop_duration)
                self.last_stop_trigger_time = now

    def timer_cb(self) -> None:
        now = self.get_clock().now()

        if (now - self.last_sign_stamp).nanoseconds / 1e9 > self.detection_timeout:
            self.active_sign = 'none'

        output = Twist()
        output.linear.x = self.last_manual_cmd.linear.x
        output.linear.y = self.last_manual_cmd.linear.y
        output.linear.z = self.last_manual_cmd.linear.z
        output.angular.x = self.last_manual_cmd.angular.x
        output.angular.y = self.last_manual_cmd.angular.y
        output.angular.z = self.last_manual_cmd.angular.z

        status = 'NORMAL'

        if now < self.stop_until:
            output = Twist()
            status = 'STOP'
        elif self.active_sign == 'slow_sign':
            if output.linear.x > 0.0:
                output.linear.x = min(output.linear.x, self.slow_speed_limit)
            status = 'SLOW'
        elif self.active_sign == 'fast_sign':
            if output.linear.x > 0.0:
                output.linear.x = self.fast_speed_limit
            status = 'FAST'
        else:
            if output.linear.x > 0.0:
                output.linear.x = min(output.linear.x, self.default_speed_limit)

        self.cmd_pub.publish(output)
        self._publish_status(status)

    def _publish_status(self, status: str) -> None:
        if status == self.last_status_sent:
            return
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.last_status_sent = status
        self.get_logger().info(f'Traffic rule state: {status}')

    def _extract_label(self, det) -> Optional[str]:
        for attr in ['class_name', 'name', 'label']:
            value = getattr(det, attr, None)
            if isinstance(value, str) and value:
                return value.strip()

        cls = getattr(det, 'class_id', None)
        if isinstance(cls, str):
            return cls.strip()
        return None

    def _extract_score(self, det) -> float:
        for attr in ['score', 'confidence', 'probability']:
            value = getattr(det, attr, None)
            if isinstance(value, (float, int)):
                return float(value)
        return 1.0

    def _extract_box_info(self, det) -> Tuple[float, float]:
        bbox = getattr(det, 'bbox', None)
        if bbox is None:
            return 0.0, 0.0

        width = 0.0
        center_x = 0.0
        image_width = 0.0

        size = getattr(bbox, 'size', None)
        if size is not None:
            width = float(getattr(size, 'x', 0.0))
            image_width = float(getattr(size, 'image_width', 0.0) or 0.0)

        center = getattr(bbox, 'center', None)
        if center is not None:
            position = getattr(center, 'position', None)
            if position is not None:
                center_x = float(getattr(position, 'x', 0.0))
            else:
                center_x = float(getattr(center, 'x', 0.0))

        if image_width > 0.0:
            center_x = center_x - (image_width / 2.0)

        return width, center_x


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrafficSignController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
