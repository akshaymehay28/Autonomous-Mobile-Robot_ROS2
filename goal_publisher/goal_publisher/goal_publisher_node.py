import rclpy
from rclpy.node import Node
from yolo_msgs.msg import DetectionArray
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, PointStamped
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs
import numpy as np
import struct

class GoalPublisherNode(Node):
    def __init__(self):
        super().__init__('goal_publisher_node')

        self.goal_sequence = ['orange', 'tree', 'vehicle', 'stop_sign']
        self.current_goal_index = 0
        self.map_frame = 'map'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.point_cloud_data = None

        self.detection_subscriber = self.create_subscription(
            DetectionArray,
            '/yolo/tracking',
            self.detection_callback,
            10)

        self.pointcloud_subscriber = self.create_subscription(
            PointCloud2,
            '/atlas/rgbd_camera/points',
            self.pointcloud_callback,
            10)

        self.goal_publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10)

        self.get_logger().info(f'Goal detection ready. Looking for: {self.goal_sequence[0]}')

    def pointcloud_callback(self, msg):
        self.point_cloud_data = msg

    def detection_callback(self, msg):
        if self.current_goal_index >= len(self.goal_sequence):
            return

        if self.point_cloud_data is None:
            return

        current_target = self.goal_sequence[self.current_goal_index]

        for detection in msg.detections:
            if detection.class_name == current_target:
                center_x = int(detection.bbox.center.position.x)
                center_y = int(detection.bbox.center.position.y)
                bbox_w = int(detection.bbox.size.x)
                bbox_h = int(detection.bbox.size.y)

                point_3d = self.get_3d_point_bbox(center_x, center_y, bbox_w, bbox_h)
                if point_3d is None:
                    continue

                self.publish_goal(point_3d, msg.header, current_target)
                return

    def get_point_at(self, x, y):
        pc = self.point_cloud_data
        if x < 0 or y < 0 or x >= pc.width or y >= pc.height:
            return None
        offset = y * pc.row_step + x * pc.point_step
        try:
            px, py, pz = struct.unpack_from('fff', bytes(pc.data), offset)
            if not (np.isnan(px) or np.isnan(py) or np.isnan(pz)) and pz > 0:
                return (px, py, pz)
        except Exception:
            pass
        return None

    def get_3d_point_bbox(self, cx, cy, bw, bh):
        width = self.point_cloud_data.width
        height = self.point_cloud_data.height

        x_start = max(0, cx - bw // 2)
        x_end = min(width - 1, cx + bw // 2)
        y_start = max(0, cy - bh // 2)
        y_end = min(height - 1, cy + bh // 2)

        valid_points = []
        step = 10
        for x in range(x_start, x_end, step):
            for y in range(y_start, y_end, step):
                pt = self.get_point_at(x, y)
                if pt is not None:
                    valid_points.append(pt)

        if valid_points:
            pts = np.array(valid_points)
            return np.median(pts, axis=0)

        return None

    def publish_goal(self, point_3d, header, label):
        try:
            point_in_camera_frame = PointStamped()
            point_in_camera_frame.header = header
            point_in_camera_frame.point.x = float(point_3d[0])
            point_in_camera_frame.point.y = float(point_3d[1])
            point_in_camera_frame.point.z = float(point_3d[2])

            if not self.tf_buffer.can_transform(self.map_frame, header.frame_id,
                                                header.stamp,
                                                rclpy.duration.Duration(seconds=0.5)):
                return

            transformed_point = self.tf_buffer.transform(
                point_in_camera_frame, self.map_frame,
                rclpy.duration.Duration(seconds=0.5))

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = self.map_frame
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = transformed_point.point.x
            goal_pose.pose.position.y = transformed_point.point.y
            goal_pose.pose.position.z = 0.0
            goal_pose.pose.orientation.w = 1.0

            self.goal_publisher.publish(goal_pose)

            self.get_logger().info(
                f'Goal {self.current_goal_index + 1}/{len(self.goal_sequence)}: '
                f'{label} detected at [{goal_pose.pose.position.x:.2f}, '
                f'{goal_pose.pose.position.y:.2f}]'
            )

            self.current_goal_index += 1
            if self.current_goal_index < len(self.goal_sequence):
                self.get_logger().info(f'Next target: {self.goal_sequence[self.current_goal_index]}')
            else:
                self.get_logger().info('All goals reached. Sequence complete.')

        except TransformException as e:
            self.get_logger().error(f'Transform error: {e}')
        except Exception as e:
            self.get_logger().error(f'Error: {e}')


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
