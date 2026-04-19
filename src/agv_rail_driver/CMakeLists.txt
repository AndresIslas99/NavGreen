cmake_minimum_required(VERSION 3.8)
project(agv_rail_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(rail_driver_node src/rail_driver_node.cpp)
ament_target_dependencies(rail_driver_node
  rclcpp nav_msgs geometry_msgs std_msgs std_srvs)
target_compile_features(rail_driver_node PUBLIC cxx_std_17)
target_include_directories(rail_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

install(TARGETS rail_driver_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_rail_controller test/test_rail_controller.cpp)
  target_include_directories(test_rail_controller PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_compile_features(test_rail_controller PUBLIC cxx_std_17)
endif()

ament_package()
