cmake_minimum_required(VERSION 3.8)
project(agv_sensor_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Fusion monitor node
add_executable(fusion_monitor_node src/fusion_monitor_node.cpp)
ament_target_dependencies(fusion_monitor_node
  rclcpp nav_msgs geometry_msgs diagnostic_msgs)
target_compile_features(fusion_monitor_node PUBLIC cxx_std_17)

# Covariance override node (fixes zero-covariance from sim)
add_executable(covariance_override_node src/covariance_override_node.cpp)
ament_target_dependencies(covariance_override_node
  rclcpp nav_msgs sensor_msgs)
target_compile_features(covariance_override_node PUBLIC cxx_std_17)

install(TARGETS fusion_monitor_node covariance_override_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
