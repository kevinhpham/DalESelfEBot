cmake_minimum_required(VERSION 3.8)
project(ur3_localisation)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Executables
add_executable(localisation_node src/localisation_node.cpp)
ament_target_dependencies(localisation_node rclcpp std_srvs geometry_msgs yaml-cpp Eigen3 visualization_msgs tf2_ros)

# Link YAML-CPP explicitly
target_link_libraries(localisation_node yaml-cpp Eigen3::Eigen)

install(TARGETS
  localisation_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
