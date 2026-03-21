# NTU_COMP30271_CW_RobotSim
NTU COMP30271 Robot Simulation for CW using ROS 2 Humble and Gazebo Fortress

## Terminal 1 - Simulation world
ros2 launch ntu_robotsim cwmaze.launch.py

## Termnial 2 - Spawn robot
ros2 launch ntu_robotsim single_robot_sim.launch.py

## Terminal 3 - Requirement 1: Occupancy Grid Mapping
source ~/ros2_ws/install/setup.bash
ros2 launch ntu_robotsim occupancy_grid_mapping.launch.py

## Terminal 4 - Requirements 3 & 5: Object detection and Traffic rules
source ~/ros2_ws/install/setup.bash
ros2 launch ntu_robotsim traffic_rules.launch.py \
    model:=/home/ntu-user/ros2_ws/src/cognitive_groupwork/custom_models/best.pt

## Terminal 5 - Teleop Control
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/manual_cmd_vel

## Terminal 6 - RViz Visualisation
source ~/ros2_ws/install/setup.bash
rviz2

In RViz: set Fixed Frame to map, add Map on /projected_map, add MarkerArray on /occupied_cells_vis_array, add Image on /yolo/dbg_image.

## Terminal 7 - Requirements 7 & 8: Landmark database and Object counting
source ~/ros2_ws/install/setup.bash
ros2 run ntu_robotsim landmark_database.py --ros-args \
    -p detections_topic:=/yolo/detections \
    -p odom_topic:=/atlas/odom_ground_truth \
    -p log_file:=object_log.log \
    -p yaml_file:=landmark_database.yaml

## Terminal 8 - Requirement 4: Goal Position Detection
source ~/ros2_ws/install/setup.bash
ros2 launch goal_publisher goal_publisher.launch.py

## Terminal 9 - Requirement 6: Autonomous Navigation
source ~/ros2_ws/install/setup.bash
ros2 launch nav2_navigator nav2_navigator.launch.py

## Terminal 10 - Testing requirement outputs
### Requirement 5 — Traffic sign status
ros2 topic echo /traffic_sign/status
### Requirements 7 & 8 — Landmark database log
cat ~/ros2_ws/object_log.log
### Requirement 7 — YAML database
cat ~/ros2_ws/landmark_database.yaml

## Instructions:
1. Launch Terminals 1–7 in order and wait for each to initialise before launching the next.
2. Using teleop (Terminal 5), drive the robot near each landmark in order: orange, tree, vehicle, stop sign. This builds the occupancy map and populates the landmark database.
3. Terminal 7 will log detected objects to object_log.log. Verify with: cat ~/ros2_ws/object_log.log
4. Terminal 8 (Goal Publisher): launch after driving near landmarks — it will confirm each position is recorded with coordinates.
5. Once all landmarks are recorded, drive the robot back to the starting position.
6. Terminal 9 (Autonomous Navigation): launch after all landmarks are detected and the robot is at start. The robot will automatically navigate to each landmark in sequence.
