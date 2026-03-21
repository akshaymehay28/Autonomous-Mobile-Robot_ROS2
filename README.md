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
ros2 launch ntu_robotsim object_detection.launch.py \
    model:=$HOME/ros2_ws/custom_models/best.pt
    
terminal 5
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/atlas/cmd_vel

terminal 6
source ~/ros2_ws/install/setup.bash
rviz2

Terminal 7 (Task 4 - Goal Position Detection):

source ~/ros2_ws/install/setup.bash
ros2 launch goal_publisher goal_publisher.launch.py

Note on Task 4:

- With teleop (Terminal 5), drive the robot near each landmark in order: orange, tree, vehicle, stop sign. Which also builds the occupancy map before launching navigation
- Terminal 7 will confirm each position is recorded with coordinates
- Once all 4 landmarks are recorded, drive the robot back to the starting position

Terminal 8 (Task 6 - Autonomous Navigation):

source ~/ros2_ws/install/setup.bash
ros2 launch nav2_navigator nav2_navigator.launch.py

Note on Task 6:

- Launch Terminal 8 after all landmarks are detected and robot is at start
- The robot will automatically navigate to each landmark in sequence
- Nav2 installation is required

  
In RViz: set Fixed Frame to map, add Map on /projected_map, add MarkerArray on /occupied_cells_vis_array, add Image on /yolo/dbg_image.

------------------------------------------------------------------------------------------
Terminal 1
ros2 launch ntu_robotsim cwmaze.launch.py

Terminal 2
ros2 launch ntu_robotsim single_robot_sim.launch.py

Terminal 3
source ~/ros2_ws/install/setup.bash
ros2 launch ntu_robotsim occupancy_grid_mapping.launch.py

Terminal 4
source ~/ros2_ws/install/setup.bash
ros2 launch ntu_robotsim traffic_rules.launch.py model:=/home/ntu-user/ros2_ws/src/cognitive_groupwork/custom_models/best.pt

Terminal 5
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/manual_cmd_vel

Terminal 6
source ~/ros2_ws/install/setup.bash
rviz2

Terminal 7 (Requirement 7)
source ~/ros2_ws/install/setup.bash
ros2 run ntu_robotsim landmark_database.py --ros-args \
    -p detections_topic:=/yolo/detections \
    -p odom_topic:=/atlas/odom_ground_truth \
    -p log_file:=object_log.log \
    -p yaml_file:=landmark_database.yaml

Terminal 8 (Testing Requirement 7)
cat ~/ros2_ws/object_log.log
