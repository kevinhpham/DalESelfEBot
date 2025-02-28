cmake_minimum_required(VERSION 3.8)
project(ur3_localisation)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

# Executables
add_executable(localisation_node src/localisation_node.cpp)
ament_target_dependencies(localisation_node rclcpp std_srvs geometry_msgs yaml-cpp sensor_msgs Eigen3)

add_executable(draw_rectangle src/draw_rectangle.cpp)
ament_target_dependencies(draw_rectangle rclcpp std_srvs geometry_msgs yaml-cpp)

# Link YAML-CPP explicitly
target_link_libraries(localisation_node yaml-cpp Eigen3::Eigen)
target_link_libraries(draw_rectangle yaml-cpp)

install(TARGETS
  localisation_node
  draw_rectangle
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
