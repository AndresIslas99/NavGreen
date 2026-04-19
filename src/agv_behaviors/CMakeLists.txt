cmake_minimum_required(VERSION 3.8)
project(agv_behaviors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Werror)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(agv_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(behavior_executor_node src/behavior_executor_node.cpp)
ament_target_dependencies(behavior_executor_node
  rclcpp rclcpp_action behaviortree_cpp_v3 nav2_msgs geometry_msgs agv_interfaces std_msgs)

install(TARGETS behavior_executor_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY trees/ DESTINATION share/${PROJECT_NAME}/trees)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_bt_loading test/test_bt_loading.cpp)
  ament_target_dependencies(test_bt_loading rclcpp behaviortree_cpp_v3)
endif()

ament_package()
