cmake_minimum_required(VERSION 3.8)
project(agv_markers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Werror)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# isaac_ros_apriltag_interfaces may not be installed — build conditionally
find_package(isaac_ros_apriltag_interfaces QUIET)

if(isaac_ros_apriltag_interfaces_FOUND)
  add_executable(marker_correction_node src/marker_correction_node.cpp)
  ament_target_dependencies(marker_correction_node
    rclcpp geometry_msgs std_msgs tf2 tf2_geometry_msgs
    isaac_ros_apriltag_interfaces)
  install(TARGETS marker_correction_node DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "isaac_ros_apriltag_interfaces not found — marker_correction_node will not be built")
endif()

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_marker_lookup test/test_marker_lookup.cpp)
  ament_target_dependencies(test_marker_lookup rclcpp)
endif()

ament_package()
