"""
Object Detection Launch File
==============================
COMP30271 Cognitive Computing Coursework - Requirement 3: Object Detection

Launches YOLO object detection using yolo_ros, following the same
approach as Lab 5 Section 2.3.

This launch file starts the YOLO detection node pointed at the
coursework robot's RGB camera topic. It supports both pre-trained
COCO weights (detects stop signs) and custom trained weights
(detects all coursework classes: stop_sign, fast_sign, slow_sign,
orange, vehicle, tree).

Launch AFTER the simulation and robot are running:
    Terminal 1: ros2 launch ntu_robotsim cwmaze.launch.py
    Terminal 2: ros2 launch ntu_robotsim single_robot_sim.launch.py
    Terminal 3: ros2 launch ntu_robotsim object_detection.launch.py

To use a custom trained model instead of COCO:
    ros2 launch ntu_robotsim object_detection.launch.py \
        model:=$HOME/ros2_ws/custom_models/best.pt
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ─────────────────────────────────────────────────────────────
    # Launch Arguments
    # ─────────────────────────────────────────────────────────────
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='yolov8s.pt',
        description='Path to YOLO model weights. Use yolov8s.pt for '
                    'pre-trained COCO or path to custom best.pt'
    )

    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cpu',
        description='Inference device: cpu or cuda:0'
    )

    threshold_arg = DeclareLaunchArgument(
        'threshold',
        default_value='0.5',
        description='Minimum confidence threshold for detections'
    )

    input_image_topic_arg = DeclareLaunchArgument(
        'input_image_topic',
        default_value='/atlas/rgbd_camera/image',
        description='Camera image topic from the robot'
    )

    # ─────────────────────────────────────────────────────────────
    # YOLO Detection Node (via yolo_bringup, same as Lab 5)
    # ─────────────────────────────────────────────────────────────
    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('yolo_bringup'), '/launch/yolo.launch.py'
        ]),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'device': LaunchConfiguration('device'),
            'threshold': LaunchConfiguration('threshold'),
            'input_image_topic': LaunchConfiguration('input_image_topic'),
        }.items()
    )

    return LaunchDescription([
        model_arg,
        device_arg,
        threshold_arg,
        input_image_topic_arg,
        yolo_launch,
    ])
