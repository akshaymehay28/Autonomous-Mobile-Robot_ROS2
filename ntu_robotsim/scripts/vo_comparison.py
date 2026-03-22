#!/usr/bin/env python3
"""
Visual Odometry Comparison Node for Requirement 9
Compares VO output against ground-truth odometry and logs any errors.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
import math
import csv
from datetime import datetime


class VOComparison(Node):

    def __init__(self):
        super().__init__('vo_comparison')

        self.declare_parameter('vo_odom_topic', '/vo/odom')
        self.declare_parameter('gt_odom_topic', '/atlas/odom_ground_truth')
        self.declare_parameter('log_file', 'vo_comparison.log')
        self.declare_parameter('csv_file', 'vo_comparison.csv')
        self.declare_parameter('log_interval', 2.0)

        vo_topic = self.get_parameter('vo_odom_topic').value
        gt_topic = self.get_parameter('gt_odom_topic').value
        self.log_file = self.get_parameter('log_file').value
        self.csv_file = self.get_parameter('csv_file').value
        log_interval = self.get_parameter('log_interval').value

        self.latest_vo = None
        self.latest_gt = None
        self.vo_received = False
        self.gt_received = False
        self.start_time = None
        self.error_samples = []

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        self.vo_sub = self.create_subscription(
            Odometry, vo_topic, self.vo_callback, reliable_qos)
        self.gt_sub = self.create_subscription(
            Odometry, gt_topic, self.gt_callback, reliable_qos)
        self.timer = self.create_timer(log_interval, self.compare_and_log)

        with open(self.log_file, 'w') as f:
            f.write(f"VO Comparison Log - Started {datetime.now().isoformat()}\n")
            f.write(f"VO topic: {vo_topic}\nGT topic: {gt_topic}\n")
            f.write("-" * 70 + "\n")

        with open(self.csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'time_s', 'gt_x', 'gt_y', 'gt_yaw',
                'vo_x', 'vo_y', 'vo_yaw',
                'error_x', 'error_y', 'error_dist', 'error_yaw_deg'])

        self.get_logger().info(
            f"VO Comparison started.\n  VO: {vo_topic}\n  GT: {gt_topic}\n"
            f"  Log: {self.log_file}\n  CSV: {self.csv_file}")

    def vo_callback(self, msg):
        self.latest_vo = msg
        if not self.vo_received:
            self.vo_received = True
            self.get_logger().info("First VO odometry message received!")

    def gt_callback(self, msg):
        self.latest_gt = msg
        if not self.gt_received:
            self.gt_received = True
            self.get_logger().info("First ground-truth odometry received!")

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def compare_and_log(self):
        if not self.vo_received:
            self.get_logger().warn(
                "Waiting for VO odometry... Check: ros2 topic echo /vo/odom --once")
            return
        if not self.gt_received:
            self.get_logger().warn("Waiting for ground-truth odometry...")
            return

        if self.start_time is None:
            self.start_time = self.get_clock().now()

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        gt_x = self.latest_gt.pose.pose.position.x
        gt_y = self.latest_gt.pose.pose.position.y
        gt_yaw = self.quaternion_to_yaw(self.latest_gt.pose.pose.orientation)

        vo_x = self.latest_vo.pose.pose.position.x
        vo_y = self.latest_vo.pose.pose.position.y
        vo_yaw = self.quaternion_to_yaw(self.latest_vo.pose.pose.orientation)

        err_x = vo_x - gt_x
        err_y = vo_y - gt_y
        err_dist = math.sqrt(err_x ** 2 + err_y ** 2)
        err_yaw = math.degrees(vo_yaw - gt_yaw)
        while err_yaw > 180: err_yaw -= 360
        while err_yaw < -180: err_yaw += 360

        self.error_samples.append(err_dist)

        self.get_logger().info(
            f"[{elapsed:.1f}s] GT=({gt_x:.2f},{gt_y:.2f}) "
            f"VO=({vo_x:.2f},{vo_y:.2f}) "
            f"Error: {err_dist:.3f}m, {err_yaw:.1f}deg")

        with open(self.log_file, 'a') as f:
            f.write(
                f"[{elapsed:8.1f}s] "
                f"GT=({gt_x:7.3f}, {gt_y:7.3f}, {math.degrees(gt_yaw):7.1f}deg) "
                f"VO=({vo_x:7.3f}, {vo_y:7.3f}, {math.degrees(vo_yaw):7.1f}deg) "
                f"Err={err_dist:.4f}m {err_yaw:.1f}deg\n")

        with open(self.csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                f"{elapsed:.2f}", f"{gt_x:.4f}", f"{gt_y:.4f}",
                f"{math.degrees(gt_yaw):.2f}",
                f"{vo_x:.4f}", f"{vo_y:.4f}", f"{math.degrees(vo_yaw):.2f}",
                f"{err_x:.4f}", f"{err_y:.4f}", f"{err_dist:.4f}",
                f"{err_yaw:.2f}"])

        if len(self.error_samples) % 10 == 0 and len(self.error_samples) > 0:
            avg_err = sum(self.error_samples) / len(self.error_samples)
            max_err = max(self.error_samples)
            self.get_logger().info(
                f"--- Summary ({len(self.error_samples)} samples): "
                f"Avg={avg_err:.4f}m, Max={max_err:.4f}m ---")
            with open(self.log_file, 'a') as f:
                f.write(
                    f"--- Summary ({len(self.error_samples)} samples): "
                    f"Avg={avg_err:.4f}m, Max={max_err:.4f}m ---\n")


def main(args=None):
    rclpy.init(args=args)
    node = VOComparison()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.error_samples:
            avg = sum(node.error_samples) / len(node.error_samples)
            mx = max(node.error_samples)
            node.get_logger().info(
                f"\nFINAL: {len(node.error_samples)} samples, "
                f"Avg={avg:.4f}m, Max={mx:.4f}m")
            with open(node.log_file, 'a') as f:
                f.write(
                    f"\nFINAL: {len(node.error_samples)} samples, "
                    f"Avg={avg:.4f}m, Max={mx:.4f}m\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
