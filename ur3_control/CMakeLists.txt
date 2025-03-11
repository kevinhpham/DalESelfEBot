cmake_minimum_required(VERSION 3.8)
project(ur3_control)

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)

# Ensure include directories
include_directories(
  src
  ${rclcpp_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${nlohmann_json_INCLUDE_DIRS}
)

# Define executable
add_executable(spline_follower src/spline_follower.cpp)

# Link dependencies
ament_target_dependencies(spline_follower 
  rclcpp 
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_core
  moveit_visual_tools
  Eigen3 
  nlohmann_json
)

# Install the executable
install(TARGETS spline_follower
  DESTINATION lib/${PROJECT_NAME}
)

# Install the config directory (JSON files)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install the models directory (including ur3e.urdf)
install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()