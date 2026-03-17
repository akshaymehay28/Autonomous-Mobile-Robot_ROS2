# NTU_COMP30271_CW_RobotSim
NTU COMP30271 Robot Simulation for CW using ROS 2 Humble and Gazebo Fortress

terminal 1 
ros2 launch ntu_robotsim cwmaze.launch.py

terminal 2
ros2 launch ntu_robotsim single_robot_sim.launch.py

terminal 3
source ~/ros2_ws/install/setup.bash

ros2 launch ntu_robotsim occupancy_grid_mapping.launch.py

terminal 4
source ~/ros2_ws/install/setup.bash

source ~/ros2_ws/install/setup.bash
ros2 launch ntu_robotsim object_detection.launch.py \
    model:=$HOME/ros2_ws/custom_models/best.pt
    
terminal 5
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/atlas/cmd_vel

terminal 6
source ~/ros2_ws/install/setup.bash

rviz2
