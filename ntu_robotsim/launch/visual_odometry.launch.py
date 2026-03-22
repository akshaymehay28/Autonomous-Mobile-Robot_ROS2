"""
Visual Odometry Launch File for Requirement 9
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock'
    )

    static_tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_camera_vo',
        arguments=[
            '0.1', '0', '0.19',
            '0', '0', '0', '1',
            'vo_base_link',
            'atlas/realsense'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    rgbd_odometry_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'vo_base_link',
            'odom_frame_id': 'vo_odom',
            'publish_tf': True,
            'wait_for_transform': 0.5,
            'approx_sync': True,
            'approx_sync_max_interval': 0.5,
            'queue_size': 10,
            'topic_queue_size': 10,
            'qos': 1,
            'qos_camera_info': 1,
            'Reg/Strategy': '1',
            'Odom/Strategy': '1',
            'Odom/ResetCountdown': '10',
            'Odom/GuessSmoothingDelay': '0',
            'Icp/MaxCorrespondenceDistance': '0.15',
            'Icp/VoxelSize': '0.05',
            'Icp/PointToPlane': 'true',
            'Icp/Iterations': '30',
            'Icp/MaxTranslation': '0.5',
            'Vis/MaxDepth': '8.0',
        }],
        remappings=[
            ('rgb/image', '/atlas/rgbd_camera/image'),
            ('depth/image', '/atlas/rgbd_camera/depth_image'),
            ('rgb/camera_info', '/atlas/rgbd_camera/camera_info'),
            ('odom', '/vo/odom'),
        ]
    )

    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'vo_base_link',
            'odom_frame_id': 'vo_odom',
            'subscribe_depth': True,
            'subscribe_odom_info': True,
            'approx_sync': True,
            'qos': 1,
            'qos_camera_info': 1,
        }],
        remappings=[
            ('rgb/image', '/atlas/rgbd_camera/image'),
            ('depth/image', '/atlas/rgbd_camera/depth_image'),
            ('rgb/camera_info', '/atlas/rgbd_camera/camera_info'),
            ('odom', '/vo/odom'),
        ]
    )

    vo_comparison_node = Node(
        package='ntu_robotsim',
        executable='vo_comparison.py',
        name='vo_comparison',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'vo_odom_topic': '/vo/odom',
            'gt_odom_topic': '/atlas/odom_ground_truth',
            'log_file': 'vo_comparison.log',
            'csv_file': 'vo_comparison.csv',
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        static_tf_base_to_camera,
        rgbd_odometry_node,
        rtabmap_viz_node,
        vo_comparison_node,
    ])
