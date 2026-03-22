#!/usr/bin/env python3
"""
Requirement 7: Landmark Database
This node subscribes to YOLO detections and robot odometry to build a landmark database.
"""

import os
import yaml
import math
from typing import Optional, Dict

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from yolo_msgs.msg import DetectionArray


class LandmarkDatabase(Node):
    """ROS2 node to maintain a database of detected landmarks."""

    # Map YOLO class names to display names for the log
    CLASS_DISPLAY_NAMES = {
        'orange': 'oranges',
        'tree': 'trees',
        'vehicle': 'cars',
        'car': 'cars',
        'stop_sign': 'stop_signs',
        'fast_sign': 'fast_signs',
        'slow_sign': 'slow_signs',
    }

    # classes counted as landmarks
    LANDMARK_CLASSES = {'orange', 'tree', 'vehicle', 'car'}

    def __init__(self) -> None:
        super().__init__('landmark_database')

        # parameters
        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter('odom_topic', '/atlas/odom_ground_truth')
        self.declare_parameter('status_topic', '/landmark_database/status')
        self.declare_parameter('log_file', 'object_log.log')
        self.declare_parameter('yaml_file', 'landmark_database.yaml')
        self.declare_parameter('min_confidence', 0.50)
        self.declare_parameter('min_box_width', 40.0)
        self.declare_parameter('detection_cooldown', 2.0)
        self.declare_parameter('track_all_classes', False)

        detections_topic = self.get_parameter('detections_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        status_topic = self.get_parameter('status_topic').value
        self.log_file_path = self.get_parameter('log_file').value
        self.yaml_file_path = self.get_parameter('yaml_file').value
        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.min_box_width = float(self.get_parameter('min_box_width').value)
        self.detection_cooldown = float(self.get_parameter('detection_cooldown').value)
        self.track_all_classes = bool(self.get_parameter('track_all_classes').value)

        # state
        self.object_counts: Dict[str, int] = {}
        self.last_log_time: Dict[str, float] = {}
        self.class_poses: Dict[str, dict] = {}
        self.current_odom: Optional[Odometry] = None

        # load existing database from YAML if it exists
        self._load_yaml_database()

        # subscribers
        self.det_sub = self.create_subscription(
            DetectionArray, detections_topic, self.detections_cb, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_cb, 10
        )

        # pubishers
        self.status_pub = self.create_publisher(String, status_topic, 10)

        # publish periodic status
        self.create_timer(5.0, self.publish_status)

        self.get_logger().info('LandmarkDatabase started')
        self.get_logger().info(f'  Detections topic: {detections_topic}')
        self.get_logger().info(f'  Odometry topic:   {odom_topic}')
        self.get_logger().info(f'  Log file:         {self.log_file_path}')
        self.get_logger().info(f'  YAML file:        {self.yaml_file_path}')

    # callbacks
    def odom_cb(self, msg: Odometry) -> None:
        # stores the latest odometry
        self.current_odom = msg

    def detections_cb(self, msg: DetectionArray) -> None:
        # process any incoming YOLO detections
        if self.current_odom is None:
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9

        # count detections per class in this frame
        frame_counts: Dict[str, int] = {}
        for det in msg.detections:
            label = self._extract_label(det)
            score = self._extract_score(det)
            width = self._extract_box_width(det)

            if label is None:
                continue
            if score < self.min_confidence:
                continue
            if width < self.min_box_width:
                continue

            # normalise the class name
            normalised = label.lower().strip()

            # only track landmark classes unless track_all_classes is True
            if not self.track_all_classes and normalised not in self.LANDMARK_CLASSES:
                continue

            frame_counts[normalised] = frame_counts.get(normalised, 0) + 1

        # update counts and log for each detected class
        for class_name, count in frame_counts.items():
            # store the highest count seen for this class
            self.object_counts[class_name] = max(
                self.object_counts.get(class_name, 0), count
            )

            # cooldown check to avoid writing the same class every frame
            last_time = self.last_log_time.get(class_name, 0.0)
            if (now_sec - last_time) < self.detection_cooldown:
                continue

            self.last_log_time[class_name] = now_sec

            # write to log file
            self._write_log_entry(class_name, self.object_counts[class_name])

            # save to YAML file
            self._save_yaml_database()

            self.get_logger().info(
                f'Landmark update: {self._display_name(class_name)} = '
                f'{self.object_counts[class_name]}'
            )

    # output for log file 

    def _write_log_entry(self, class_name: str, total_count: int) -> None:
        # store the latest pose per class and rewrite the summary log
        pos = self.current_odom.pose.pose.position
        ori = self.current_odom.pose.pose.orientation

        # Store the latest pose for this class
        self.class_poses[class_name] = {
            'pos': (pos.x, pos.y, pos.z),
            'ori': (ori.x, ori.y, ori.z, ori.w),
        }

        # rewrite the entire file with one line per class
        try:
            with open(self.log_file_path, 'w') as f:
                for cname in sorted(self.object_counts.keys()):
                    if cname not in self.class_poses:
                        continue
                    p = self.class_poses[cname]['pos']
                    o = self.class_poses[cname]['ori']
                    display = self._display_name(cname)
                    count = self.object_counts[cname]
                    line = (
                        f'Number of {display} detected: {count} ; '
                        f'Robot odometry:'
                        f'Position(x={p[0]:.2f}, y={p[1]:.2f}, z={p[2]:.1f}), '
                        f'Orientation(x={o[0]:.1f}, y={o[1]:.1f}, '
                        f'z={o[2]:.4f}, w={o[3]:.4f})'
                    )
                    f.write(line + '\n')
        except IOError as e:
            self.get_logger().error(f'Failed to write log: {e}')

    # YAML persistence
    def _save_yaml_database(self) -> None:
        # save the current landmark database to a YAML file
        data = {
            'landmark_database': {
                'total_counts': dict(self.object_counts),
            }
        }

        # add current robot pose if available
        if self.current_odom is not None:
            pos = self.current_odom.pose.pose.position
            ori = self.current_odom.pose.pose.orientation
            data['landmark_database']['last_robot_pose'] = {
                'position': {'x': float(pos.x), 'y': float(pos.y), 'z': float(pos.z)},
                'orientation': {
                    'x': float(ori.x), 'y': float(ori.y),
                    'z': float(ori.z), 'w': float(ori.w),
                },
            }

        try:
            with open(self.yaml_file_path, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)
        except IOError as e:
            self.get_logger().error(f'Failed to save YAML: {e}')

    def _load_yaml_database(self) -> None:
        # load landmark counts from YAML if the file already exists
        if not os.path.exists(self.yaml_file_path):
            return
        try:
            with open(self.yaml_file_path, 'r') as f:
                data = yaml.safe_load(f)
            if data and 'landmark_database' in data:
                counts = data['landmark_database'].get('total_counts', {})
                self.object_counts = {k: int(v) for k, v in counts.items()}
                self.get_logger().info(
                    f'Loaded existing database: {self.object_counts}'
                )
        except Exception as e:
            self.get_logger().warn(f'Could not load YAML database: {e}')

    # status publishing
    def publish_status(self) -> None:
        # publish the current landmark counts
        if not self.object_counts:
            return
        parts = []
        for class_name, count in sorted(self.object_counts.items()):
            parts.append(f'{self._display_name(class_name)}={count}')
        msg = String()
        msg.data = ' | '.join(parts)
        self.status_pub.publish(msg)

    # helpers
    def _display_name(self, class_name: str) -> str:
        # convert the internal class name to display name
        return self.CLASS_DISPLAY_NAMES.get(class_name, class_name)

    def _extract_label(self, det) -> Optional[str]:
        # extract class label from a detection message
        for attr in ['class_name', 'name', 'label']:
            value = getattr(det, attr, None)
            if isinstance(value, str) and value:
                return value.strip()
        cls = getattr(det, 'class_id', None)
        if isinstance(cls, str):
            return cls.strip()
        return None

    def _extract_score(self, det) -> float:
        # extract confidence score from a detection message
        for attr in ['score', 'confidence', 'probability']:
            value = getattr(det, attr, None)
            if isinstance(value, (float, int)):
                return float(value)
        return 1.0

    def _extract_box_width(self, det) -> float:
        # extract bounding box width from a detection message
        bbox = getattr(det, 'bbox', None)
        if bbox is None:
            return 0.0
        size = getattr(bbox, 'size', None)
        if size is not None:
            return float(getattr(size, 'x', 0.0))
        return 0.0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LandmarkDatabase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
