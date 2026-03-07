"""
Occupancy Grid Mapping Launch File (OctoMap-based)
COMP30271 Coursework - Requirement 1: Basic Mapping
Uses OctoMap server to build a 3D map and project it to 2D.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock')

    # Node 1: Odometry to TF (same as Lab 7)
    odom_to_tf_node = Node(
        package='odom_to_tf_ros2',
        executable='odom_to_tf',
        name='odom_to_tf',
        output='screen',
        parameters=[
            {'odom_topic': '/atlas/odom_ground_truth'},
            {'use_sim_time': use_sim_time},
        ]
    )

    # Node 2: Static TF — base_link to camera
    static_tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_camera',
        arguments=[
            '0.1', '0', '0.19',
            '0', '0', '0', '1',
            'atlas/base_link', 'atlas/realsense'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Node 3: Static TF — base_link to laser
    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_laser',
        arguments=[
            '0', '0', '0.28',
            '0', '0', '0', '1',
            'atlas/base_link', 'atlas/base_laser'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Node 4: OctoMap Server with ground filtering
    octomap_server_node = Node(
        package='octomap_server2',
        executable='octomap_server',
        output='screen',
        remappings=[
            ('cloud_in', '/atlas/rgbd_camera/points'),
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
            'resolution': 0.10,
            'frame_id': 'map',
            'base_frame_id': 'atlas/base_link',
            'height_map': True,
            'colored_map': True,
            'color_factor': 0.8,
            'compress_map': True,
            'incremental_2D_projection': True,
            'publish_free_space': False,
            'filter_ground': True,
            'filter_speckles': True,
            'ground_filter/distance': 0.06,
            'ground_filter/angle': 0.30,
            'ground_filter/plane_distance': 0.20,
            'pointcloud_min_z': 0.12,
            'sensor_model/max_range': 8.0,
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        odom_to_tf_node,
        static_tf_base_to_camera,
        static_tf_base_to_laser,
        octomap_server_node,
    ])
