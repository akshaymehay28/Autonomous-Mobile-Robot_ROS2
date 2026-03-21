import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

class MapRelayNode(Node):
    def __init__(self):
        super().__init__('map_relay_node')

        # Subscribe to /projected_map with volatile QoS
        sub_qos = QoSProfile(depth=1)
        sub_qos.durability = DurabilityPolicy.VOLATILE
        sub_qos.reliability = ReliabilityPolicy.RELIABLE

        # Publish to /map with transient local QoS (what Nav2 expects)
        pub_qos = QoSProfile(depth=1)
        pub_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        pub_qos.reliability = ReliabilityPolicy.RELIABLE

        self.publisher = self.create_publisher(OccupancyGrid, '/map', pub_qos)
        self.subscriber = self.create_subscription(
            OccupancyGrid,
            '/projected_map',
            self.callback,
            sub_qos)

        self.get_logger().info('Map relay started: /projected_map -> /map')

    def callback(self, msg):
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
