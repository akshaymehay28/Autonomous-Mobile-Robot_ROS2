"""
Landmark Database Launch File for Requirement 7
Launches YOLO object detection alongside the landmark database node.
The landmark database listens for oranges, trees, vehicles and logs them with robot odometry to object_log.log.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='yolov8s.pt',
        description='Path to YOLO weights (use best.pt for custom model)'
    )

    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cpu',
        description='Inference device: cpu or cuda:0'
    )

    threshold_arg = DeclareLaunchArgument(
        'threshold',
        default_value='0.5',
        description='Minimum detection confidence'
    )

    input_image_topic_arg = DeclareLaunchArgument(
        'input_image_topic',
        default_value='/atlas/rgbd_camera/image',
        description='Robot RGB camera topic'
    )

    log_file_arg = DeclareLaunchArgument(
        'log_file',
        default_value='object_log.log',
        description='Path to the object detection log file'
    )

    yaml_file_arg = DeclareLaunchArgument(
        'yaml_file',
        default_value='landmark_database.yaml',
        description='Path to the YAML persistent database file'
    )

    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ntu_robotsim'), '/launch/object_detection.launch.py'
        ]),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'device': LaunchConfiguration('device'),
            'threshold': LaunchConfiguration('threshold'),
            'input_image_topic': LaunchConfiguration('input_image_topic'),
        }.items()
    )

    landmark_db_node = Node(
        package='ntu_robotsim',
        executable='landmark_database.py',
        name='landmark_database',
        output='screen',
        parameters=[{
            'detections_topic': '/yolo/detections',
            'odom_topic': '/atlas/odom_ground_truth',
            'status_topic': '/landmark_db/status',
            'log_file': LaunchConfiguration('log_file'),
            'yaml_file': LaunchConfiguration('yaml_file'),
            'min_confidence': 0.50,
            'min_box_width': 40.0,
        }]
    )

    return LaunchDescription([
        model_arg,
        device_arg,
        threshold_arg,
        input_image_topic_arg,
        log_file_arg,
        yaml_file_arg,
        yolo_launch,
        landmark_db_node,
    ])
