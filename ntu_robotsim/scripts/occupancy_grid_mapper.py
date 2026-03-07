#!/usr/bin/env python3
"""
Occupancy Grid Mapper Node
COMP30271 Cognitive Computing Coursework - Requirement 1: Basic Mapping
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2

import numpy as np
import struct
import math


class OccupancyGridMapper(Node):

    def __init__(self):
        super().__init__('occupancy_grid_mapper')
        self.declare_parameter('map_resolution', 0.1)
        self.declare_parameter('map_width', 120)
        self.declare_parameter('map_height', 120)
        self.declare_parameter('map_origin_x', -6.0)
        self.declare_parameter('map_origin_y', -6.0)
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('log_odds_occupied', 0.85)
        self.declare_parameter('log_odds_free', -0.15)
        self.declare_parameter('log_odds_max', 5.0)
        self.declare_parameter('log_odds_min', -5.0)

        self.resolution = self.get_parameter('map_resolution').value
        self.width = self.get_parameter('map_width').value
        self.height = self.get_parameter('map_height').value
        self.origin_x = self.get_parameter('map_origin_x').value
        self.origin_y = self.get_parameter('map_origin_y').value
        publish_rate = self.get_parameter('publish_rate').value
        self.log_odds_occ = self.get_parameter('log_odds_occupied').value
        self.log_odds_free = self.get_parameter('log_odds_free').value
        self.log_odds_max = self.get_parameter('log_odds_max').value
        self.log_odds_min = self.get_parameter('log_odds_min').value

        self.log_odds_grid = np.zeros((self.height, self.width), dtype=np.float64)
        self.current_pose = None
        self.pose_received = False
        self.frame_count = 0
        self.last_map_x = None
        self.last_map_y = None
        self.last_map_yaw = None
        self.get_logger().info(
            f'Occupancy Grid Mapper initialised:\n'
            f'  Resolution: {self.resolution} m/cell\n'
            f'  Grid size: {self.width} x {self.height} cells\n'
            f'  Origin: ({self.origin_x}, {self.origin_y})\n'
            f'  Publish rate: {publish_rate} Hz'
        )

        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.odom_sub = self.create_subscription(
            Odometry, '/atlas/odom_ground_truth',
            self.odom_callback, odom_qos)
        self.get_logger().info('Subscribed to /atlas/odom_ground_truth')

        self.pc_sub = self.create_subscription(
            PointCloud2, '/atlas/rgbd_camera/points',
            self.pointcloud_callback, sensor_qos)
        self.get_logger().info('Subscribed to /atlas/rgbd_camera/points')

        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self.get_logger().info('Publishing occupancy grid on /map')

        self.timer = self.create_timer(1.0 / publish_rate, self.publish_map)
        self.get_logger().info('Occupancy Grid Mapper node is ready.')

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        if not self.pose_received:
            self.pose_received = True
            self.get_logger().info(
                f'First pose received: '
                f'x={self.current_pose.position.x:.2f}, '
                f'y={self.current_pose.position.y:.2f}')
    def pointcloud_callback(self, msg):
        if not self.pose_received:
            return

        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        robot_yaw = self.quaternion_to_yaw(self.current_pose.orientation)

        # Only update map when robot has moved or turned enough
        if self.last_map_x is not None:
            dist = math.sqrt((robot_x - self.last_map_x)**2 + (robot_y - self.last_map_y)**2)
            yaw_diff = abs(robot_yaw - self.last_map_yaw)
            if yaw_diff > math.pi:
                yaw_diff = 2 * math.pi - yaw_diff
            if dist < 0.05 and yaw_diff < 0.1:
                return

        self.last_map_x = robot_x
        self.last_map_y = robot_y
        self.last_map_yaw = robot_yaw
        robot_z = self.current_pose.position.z

        robot_gx, robot_gy = self.world_to_grid(robot_x, robot_y)
        if not self.in_bounds(robot_gx, robot_gy):
            return

        points = self.parse_pointcloud2(msg)
        if points is None or len(points) == 0:
            return

        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)

        # Camera height above ground (base z + camera mount offset)
        camera_height = robot_z + 0.19

        for point in points:
            px, py, pz = point

            if math.isnan(px) or math.isnan(py) or math.isnan(pz):
                continue
            if math.isinf(px) or math.isinf(py) or math.isinf(pz):
                continue

            # RGB-D camera frame: x=right, y=down, z=forward
            forward = pz
            lateral = px
            vertical = -py  # flip y since y is down in camera frame

            distance = math.sqrt(forward * forward + lateral * lateral)
            if distance < 0.15 or distance > 10.0:
                continue

            # Calculate world z height of this point
            world_z = camera_height + vertical

            # Only keep points at wall height (filter floor and ceiling)
            if world_z < 0.1 or world_z > 0.8:
                continue

            world_px = robot_x + (forward * cos_yaw - lateral * sin_yaw)
            world_py = robot_y + (forward * sin_yaw + lateral * cos_yaw)

            point_gx, point_gy = self.world_to_grid(world_px, world_py)

            self.ray_trace(robot_gx, robot_gy, point_gx, point_gy)

            if self.in_bounds(point_gx, point_gy):
                self.log_odds_grid[point_gy, point_gx] = min(
                    self.log_odds_grid[point_gy, point_gx] + self.log_odds_occ,
                    self.log_odds_max)
    def publish_map(self):
        if not self.pose_received:
            return

        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'map'
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height
        grid_msg.info.origin.position.x = self.origin_x
        grid_msg.info.origin.position.y = self.origin_y
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0

        grid_data = np.full(self.height * self.width, -1, dtype=np.int8)

        for y in range(self.height):
            for x in range(self.width):
                lo = self.log_odds_grid[y, x]
                if abs(lo) < 0.01:
                    grid_data[y * self.width + x] = -1
                else:
                    probability = 1.0 - (1.0 / (1.0 + math.exp(lo)))
                    grid_data[y * self.width + x] = int(probability * 100.0)

        grid_msg.data = grid_data.tolist()
        self.map_pub.publish(grid_msg)

    def ray_trace(self, x0, y0, x1, y1):
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        cx, cy = x0, y0
        while True:
            if cx == x1 and cy == y1:
                break
            if self.in_bounds(cx, cy):
                self.log_odds_grid[cy, cx] = max(
                    self.log_odds_grid[cy, cx] + self.log_odds_free,
                    self.log_odds_min)
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                cx += sx
            if e2 < dx:
                err += dx
                cy += sy

    def world_to_grid(self, world_x, world_y):
        grid_x = int((world_x - self.origin_x) / self.resolution)
        grid_y = int((world_y - self.origin_y) / self.resolution)
        return grid_x, grid_y

    def in_bounds(self, gx, gy):
        return 0 <= gx < self.width and 0 <= gy < self.height

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def parse_pointcloud2(self, msg):
        try:
            field_offsets = {}
            for field in msg.fields:
                if field.name in ('x', 'y', 'z'):
                    field_offsets[field.name] = field.offset
            if not all(k in field_offsets for k in ('x', 'y', 'z')):
                return None
            points = []
            point_step = msg.point_step
            data = msg.data
            for i in range(msg.width * msg.height):
                offset = i * point_step
                x = struct.unpack_from('f', data, offset + field_offsets['x'])[0]
                y = struct.unpack_from('f', data, offset + field_offsets['y'])[0]
                z = struct.unpack_from('f', data, offset + field_offsets['z'])[0]
                points.append((x, y, z))
            return points
        except Exception as e:
            self.get_logger().error(f'Error parsing PointCloud2: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
