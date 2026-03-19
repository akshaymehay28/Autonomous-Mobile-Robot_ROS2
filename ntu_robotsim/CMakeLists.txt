cmake_minimum_required(VERSION 3.5)

project(ntu_robotsim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  models
  worlds
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/occupancy_grid_mapper.py
  scripts/traffic_sign_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_resource_path.dsv.in")

ament_package()
