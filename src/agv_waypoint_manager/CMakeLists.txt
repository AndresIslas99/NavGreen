cmake_minimum_required(VERSION 3.8)
project(agv_waypoint_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Werror)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(agv_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(waypoint_manager_node src/waypoint_manager_node.cpp)
ament_target_dependencies(waypoint_manager_node
  rclcpp rclcpp_action agv_interfaces nav2_msgs geometry_msgs)

install(TARGETS waypoint_manager_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
# launch/ exists only on commissioned robots (untracked); OPTIONAL keeps
# fresh clones and CI building.
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch OPTIONAL)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mission_persistence test/test_mission_persistence.cpp)
  ament_target_dependencies(test_mission_persistence rclcpp)
endif()

ament_package()
