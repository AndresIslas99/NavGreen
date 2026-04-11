cmake_minimum_required(VERSION 3.8)
project(agv_localization_init)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Werror)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(isaac_ros_visual_slam_interfaces REQUIRED)
find_package(robot_localization REQUIRED)

add_executable(auto_init_orchestrator_node
  src/auto_init_orchestrator_node.cpp)
ament_target_dependencies(auto_init_orchestrator_node
  rclcpp std_msgs std_srvs geometry_msgs nav_msgs
  isaac_ros_visual_slam_interfaces
  robot_localization)

install(TARGETS auto_init_orchestrator_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
