cmake_minimum_required(VERSION 3.8)
project(agv_mode_arbiter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(agv_interfaces REQUIRED)

add_executable(mode_arbiter_node src/mode_arbiter_node.cpp)
ament_target_dependencies(mode_arbiter_node
  rclcpp geometry_msgs nav_msgs std_msgs agv_interfaces)
target_compile_features(mode_arbiter_node PUBLIC cxx_std_17)
target_include_directories(mode_arbiter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

install(TARGETS mode_arbiter_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mode_fsm test/test_mode_fsm.cpp)
  target_include_directories(test_mode_fsm PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_compile_features(test_mode_fsm PUBLIC cxx_std_17)
endif()

ament_package()
