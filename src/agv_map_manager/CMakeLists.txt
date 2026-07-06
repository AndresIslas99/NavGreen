cmake_minimum_required(VERSION 3.8)
project(agv_map_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Werror)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(agv_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(isaac_ros_visual_slam_interfaces REQUIRED)

add_executable(map_manager_node src/map_manager_node.cpp)
ament_target_dependencies(map_manager_node
  rclcpp agv_interfaces nav_msgs nav2_msgs std_msgs std_srvs
  isaac_ros_visual_slam_interfaces)
target_include_directories(map_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

install(TARGETS map_manager_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_zone_persistence test/test_zone_persistence.cpp)
  ament_target_dependencies(test_zone_persistence rclcpp)
  ament_add_gtest(test_name_validation test/test_name_validation.cpp)
  target_include_directories(test_name_validation PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
endif()

ament_package()
