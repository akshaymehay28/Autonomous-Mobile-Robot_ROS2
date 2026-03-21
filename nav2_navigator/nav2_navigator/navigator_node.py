import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class NavigatorNode(Node):
    def __init__(self):
        super().__init__('navigator_node')
        self.get_logger().info('Navigation node ready.')

        self.navigator = BasicNavigator()
        self.navigating = False

        self.goal_subscriber = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.goal_reached_publisher = self.create_publisher(Bool, '/goal_reached', 10)
        self.timer = self.create_timer(0.5, self.check_navigation)
        self.goal_active = False

    def goal_callback(self, msg):
        if self.navigating:
            return
        self.get_logger().info(
            f'New destination: [{msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}]'
        )
        self.navigating = True
        self.goal_active = True
        self.navigator.goToPose(msg)

    def check_navigation(self):
        if not self.goal_active:
            return

        if not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(
                    f'Distance remaining: {feedback.distance_remaining:.2f}m',
                    throttle_duration_sec=2.0
                )
            return

        result = self.navigator.getResult()
        reached = Bool()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Destination reached.')
            reached.data = True
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('Navigation cancelled.')
            reached.data = False
        elif result == TaskResult.FAILED:
            self.get_logger().error('Navigation failed.')
            reached.data = False

        self.goal_reached_publisher.publish(reached)
        self.navigating = False
        self.goal_active = False


def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
