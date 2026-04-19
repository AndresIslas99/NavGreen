cmake_minimum_required(VERSION 3.8)
project(agv_rail_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(rail_detector_node src/rail_detector_node.cpp)
ament_target_dependencies(rail_detector_node
  rclcpp sensor_msgs geometry_msgs std_msgs)
target_compile_features(rail_detector_node PUBLIC cxx_std_17)
target_include_directories(rail_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

install(TARGETS rail_detector_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_rail_ransac test/test_rail_ransac.cpp)
  target_include_directories(test_rail_ransac PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_compile_features(test_rail_ransac PUBLIC cxx_std_17)
endif()

ament_package()
