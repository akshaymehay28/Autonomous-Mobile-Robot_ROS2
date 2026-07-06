# Autonomous Mobile Robot - ROS 2 Navigation, Mapping and Perception

An autonomous mobile robot built with ROS 2 Humble and simulated in Gazebo Fortress. The robot explores a maze environment, builds an occupancy grid map from point cloud data, detects and classifies objects with a custom-trained YOLO model, obeys traffic signs, records discovered landmarks in a database, and then autonomously navigates back to each landmark using Nav2.

## Capabilities

**Occupancy grid mapping** - a custom mapping node (`occupancy_grid_mapper.py`) converts depth-camera point clouds and odometry into a live occupancy grid, published for RViz and used downstream by Nav2. OctoMap is integrated for 3D mapping support.

**Object detection** - a YOLO model trained on custom classes for this environment (oranges, trees, vehicles, stop signs; weights in `custom_models/best.pt`) runs against the robot's camera feed and publishes detections.

**Traffic rules** - `traffic_sign_controller.py` sits between the velocity command sources and the robot, monitoring YOLO detections and enforcing stop-sign behaviour by gating `cmd_vel`.

**Landmark database** - `landmark_database.py` fuses YOLO detections with odometry to estimate world coordinates for each discovered object, de-duplicates repeat sightings, and persists the result to YAML and a human-readable log.

**Autonomous navigation** - once landmarks are recorded, `goal_publisher` publishes each landmark position as a navigation goal and `nav2_navigator` drives the robot to every one in sequence using the Nav2 stack (`BasicNavigator`), reporting goal completion.

**Visual odometry** - a separate RTAB-Map pipeline estimates the robot's trajectory from camera data alone and logs a live comparison against ground-truth odometry (`vo_comparison.py`).

## Repository layout

```
ntu_robotsim/     simulation worlds, robot model, launch files, mapping /
                  perception / traffic scripts
goal_publisher/   landmark goal publishing and exploration nodes
nav2_navigator/   Nav2 integration: navigator and map relay nodes
custom_models/    trained YOLO weights (best.pt)
octomap2/         OctoMap server and PCL support packages
odom_to_tf_ros2/  odometry-to-TF broadcaster
```

## Running the simulation

Requires ROS 2 Humble, Gazebo Fortress and a built colcon workspace. Launch in order, waiting for each to initialise:

```bash
ros2 launch ntu_robotsim cwmaze.launch.py              # simulation world
ros2 launch ntu_robotsim single_robot_sim.launch.py    # spawn robot
ros2 launch ntu_robotsim occupancy_grid_mapping.launch.py
ros2 launch ntu_robotsim traffic_rules.launch.py model:=<path to best.pt>
ros2 run ntu_robotsim landmark_database.py
ros2 launch goal_publisher goal_publisher.launch.py
ros2 launch nav2_navigator nav2_navigator.launch.py    # autonomous phase
```

Teleop for the exploration phase:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/manual_cmd_vel
```

For the visual odometry pipeline (run separately from the above):

```bash
sudo apt install ros-humble-rtabmap-ros
ros2 launch ntu_robotsim visual_odometry.launch.py
```

## Tech stack

ROS 2 Humble, Gazebo Fortress, Nav2, YOLO, RTAB-Map, OctoMap, PCL, Python, C++, RViz
