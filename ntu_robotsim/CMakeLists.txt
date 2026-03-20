cmake_minimum_required(VERSION 3.5)

project(ntu_robotsim)

find_package(ament_cmake REQUIRED)

# Install directories (config, models, worlds, launch are existing)
install(DIRECTORY
  config
  models
  worlds
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts as executable nodes
install(PROGRAMS
  scripts/occupancy_grid_mapper.py
  DESTINATION lib/${PROJECT_NAME}
)

# Setup the environmental hooks for the Gazebo resource path
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_resource_path.dsv.in")

ament_package()
