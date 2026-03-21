import rclpy
from rclpy.node import Node
from yolo_msgs.msg import DetectionArray
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
import numpy as np
import struct

class GoalPublisherNode(Node):
    def __init__(self):
        super().__init__('goal_publisher_node')

        self.goal_sequence = ['orange', 'tree', 'vehicle', 'stop_sign']
        self.detected_goals = {}
        self.current_target_index = 0
        self.nav2_goal_index = 0
        self.all_goals_found = False
        self.first_goal_published = False
        self.robot_pose = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.detection_subscriber = self.create_subscription(
            DetectionArray, '/yolo/tracking', self.detection_callback, 10)

        self.odom_subscriber = self.create_subscription(
            Odometry, '/atlas/odom_ground_truth', self.odom_callback, 10)

        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.goal_reached_subscriber = self.create_subscription(
            Bool, '/goal_reached', self.goal_reached_callback, 10)

        self.nav2_check_timer = self.create_timer(2.0, self.check_nav2)

        self.get_logger().info(f'Ready. Drive near each landmark in order: {self.goal_sequence}')

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def check_nav2(self):
        if self.first_goal_published or not self.all_goals_found:
            return
        service_names = [s[0] for s in self.get_service_names_and_types()]
        if any('navigate_to_pose' in s for s in service_names):
            self.get_logger().info('Navigation system ready. Starting in 5 seconds...')
            self.create_timer(5.0, self.publish_first_goal_once)
        else:
            self.get_logger().info('Waiting for navigation system...', throttle_duration_sec=4.0)

    def publish_first_goal_once(self):
        if not self.first_goal_published:
            self.first_goal_published = True
            self.publish_next_goal()

    def goal_reached_callback(self, msg):
        if msg.data:
            self.nav2_goal_index += 1
            if self.nav2_goal_index < len(self.goal_sequence):
                self.get_logger().info(f'Reached destination. Moving to: {self.goal_sequence[self.nav2_goal_index]}')
                self.publish_next_goal()
            else:
                self.get_logger().info('Maze complete. All destinations visited.')

    def detection_callback(self, msg):
        if self.all_goals_found or self.robot_pose is None:
            return
        if self.current_target_index >= len(self.goal_sequence):
            return

        current_target = self.goal_sequence[self.current_target_index]

        for detection in msg.detections:
            if detection.class_name == current_target:
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = self.get_clock().now().to_msg()
                goal_pose.pose.position.x = self.robot_pose.position.x
                goal_pose.pose.position.y = self.robot_pose.position.y
                goal_pose.pose.position.z = 0.0
                goal_pose.pose.orientation = self.robot_pose.orientation

                self.detected_goals[current_target] = goal_pose
                self.current_target_index += 1

                self.get_logger().info(
                    f'Landmark {self.current_target_index}/{len(self.goal_sequence)} recorded: '
                    f'{current_target} at [{goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f}]'
                )

                if self.current_target_index < len(self.goal_sequence):
                    self.get_logger().info(f'Next landmark: {self.goal_sequence[self.current_target_index]}')
                else:
                    self.all_goals_found = True
                    self.get_logger().info('All landmarks recorded. Return to start and launch navigation.')
                return

    def publish_next_goal(self):
        if self.nav2_goal_index >= len(self.goal_sequence):
            return
        label = self.goal_sequence[self.nav2_goal_index]
        goal_pose = self.detected_goals[label]
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        self.goal_publisher.publish(goal_pose)
        self.get_logger().info(
            f'Navigating to {label} [{self.nav2_goal_index + 1}/{len(self.goal_sequence)}] '
            f'at [{goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f}]'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
