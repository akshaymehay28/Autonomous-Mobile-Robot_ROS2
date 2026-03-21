from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_params = os.path.join(
        get_package_share_directory('nav2_navigator'),
        'config', 'nav2_params.yaml'
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': nav2_params,
            'map_subscribe_transient_local': 'true',
        }.items()
    )

    navigator = Node(
        package='nav2_navigator',
        executable='navigator_node',
        name='navigator_node',
        output='screen'
    )

    map_relay = Node(
        package='nav2_navigator',
        executable='map_relay_node',
        name='map_relay_node',
        output='screen'
    )

    # Relay cmd_vel to atlas/cmd_vel
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        arguments=['/cmd_vel', '/atlas/cmd_vel'],
        output='screen'
    )

    return LaunchDescription([
        map_relay,
        cmd_vel_relay,
        nav2,
        navigator,
    ])
