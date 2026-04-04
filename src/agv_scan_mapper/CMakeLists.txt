cmake_minimum_required(VERSION 3.8)
project(agv_scan_mapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(scan_grid_mapper_node src/scan_grid_mapper_node.cpp)
find_package(std_msgs REQUIRED)

ament_target_dependencies(scan_grid_mapper_node
  rclcpp nav_msgs sensor_msgs std_srvs std_msgs geometry_msgs tf2_ros)
target_compile_features(scan_grid_mapper_node PUBLIC cxx_std_17)

install(TARGETS scan_grid_mapper_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
