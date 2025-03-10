cmake_minimum_required(VERSION 3.8)
project(ur3_control)

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

# Ensure include directories
include_directories(
  src
  ${rclcpp_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${nlohmann_json_INCLUDE_DIRS}
)

# Build the IK solver as a library
add_library(ik_solver_kdl src/ik_solver_kdl.cpp)
target_link_libraries(ik_solver_kdl orocos-kdl)
ament_target_dependencies(ik_solver_kdl rclcpp kdl_parser orocos_kdl Eigen3)

# Unit test for IK Solver
ament_add_gtest(test_ik_solver_kdl test/test_ik_solver_kdl.cpp)
target_link_libraries(test_ik_solver_kdl ik_solver_kdl)
target_include_directories(test_ik_solver_kdl PRIVATE src)
ament_target_dependencies(test_ik_solver_kdl rclcpp kdl_parser orocos_kdl Eigen3)

# Install library
install(TARGETS ik_solver_kdl
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_package()