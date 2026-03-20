import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import random

class ExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer_node')
        self.get_logger().info('Explorer Node started - searching for vehicle...')

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/atlas/cmd_vel',
            10)

        self.pointcloud_subscriber = self.create_subscription(
            PointCloud2,
            '/atlas/velodyne_points',
            self.pointcloud_callback,
            10)

        self.goal_found = False
        self.turning = False
        self.turn_count = 0
        self.turn_direction = 1
        self.front_distance = 999  # How far ahead is the closest obstacle

        self.timer = self.create_timer(0.1, self.explore)

    def pointcloud_callback(self, msg):
        try:
            points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            # Only care about points at robot body height, directly ahead in narrow corridor
            front_points = [p[0] for p in points if 0.1 < p[0] and -0.2 < p[1] < 0.2 and 0.0 < p[2] < 1.0]
            if front_points:
                self.front_distance = min(front_points)
            else:
                self.front_distance = 999
        except Exception as e:
            self.get_logger().warn(f'Pointcloud error: {e}')

    def explore(self):
        if self.goal_found:
            return

        twist = Twist()

        if self.front_distance < 0.5 or self.turning:
            twist.linear.x = 0.0
            twist.angular.z = 2.0 * self.turn_direction
            self.turning = True
            self.turn_count += 1
            if self.turn_count > 10:
                self.turning = False
                self.turn_count = 0
                self.turn_direction = random.choice([1, -1])
        else:
            twist.linear.x = 2.5
            twist.angular.z = 0.0

        self.cmd_vel_publisher.publish(twist)

    def stop_robot(self):
        self.goal_found = True
        twist = Twist()
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
