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
        description='Path to YOLO weights. Use best.pt for coursework traffic signs.'
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

    stop_duration_arg = DeclareLaunchArgument(
        'stop_duration',
        default_value='3.0',
        description='How long to stop after detecting a stop sign'
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

    controller_node = Node(
        package='ntu_robotsim',
        executable='traffic_sign_controller.py',
        name='traffic_sign_controller',
        output='screen',
        parameters=[{
            'input_cmd_topic': '/manual_cmd_vel',
            'output_cmd_topic': '/atlas/cmd_vel',
            'detections_topic': '/yolo/detections',
            'status_topic': '/traffic_sign/status',
            'stop_duration': LaunchConfiguration('stop_duration'),
            'slow_speed_limit': 0.08,
            'fast_speed_limit': 0.60,
            'default_speed_limit': 0.50,
            'detection_timeout': 1.0,
            'min_confidence': 0.50,
            'min_box_width': 40.0,
            'only_react_to_centered_signs': True,
            'center_tolerance_pixels': 160.0,
        }]
    )

    return LaunchDescription([
        model_arg,
        device_arg,
        threshold_arg,
        input_image_topic_arg,
        stop_duration_arg,
        yolo_launch,
        controller_node,
    ])
