# NTU_COMP30271_CW_RobotSim
NTU COMP30271 Robot Simulation for CW using ROS 2 Humble and Gazebo Fortress

ros2 launch ntu_robotsim cwmaze.launch.py

ros2 launch ntu_robotsim single_robot_sim.launch.py

source ~/ros2_ws/install/setup.bash

ros2 launch ntu_robotsim occupancy_grid_mapping.launch.py

source ~/ros2_ws/install/setup.bash

source ~/ros2_ws/install/setup.bash
ros2 launch ntu_robotsim object_detection.launch.py \
    model:=$HOME/ros2_ws/custom_models/best.pt

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/atlas/cmd_vel


source ~/ros2_ws/install/setup.bash

rviz2
