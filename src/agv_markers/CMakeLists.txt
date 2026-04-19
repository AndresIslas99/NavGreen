cmake_minimum_required(VERSION 3.8)
project(agv_markers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Werror)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(marker_correction_node src/marker_correction_node.cpp)
ament_target_dependencies(marker_correction_node
  rclcpp geometry_msgs sensor_msgs nav_msgs std_msgs tf2_ros apriltag_msgs robot_localization)
target_link_libraries(marker_correction_node ${OpenCV_LIBS})

install(TARGETS marker_correction_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_marker_lookup test/test_marker_lookup.cpp)
  ament_target_dependencies(test_marker_lookup rclcpp)
endif()

ament_package()
