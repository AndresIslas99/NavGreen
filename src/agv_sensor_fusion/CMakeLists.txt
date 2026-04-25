cmake_minimum_required(VERSION 3.8)
project(agv_sensor_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(std_msgs REQUIRED)

# Fusion monitor node
add_executable(fusion_monitor_node src/fusion_monitor_node.cpp)
ament_target_dependencies(fusion_monitor_node
  rclcpp nav_msgs geometry_msgs diagnostic_msgs sensor_msgs std_msgs)
target_compile_features(fusion_monitor_node PUBLIC cxx_std_17)

# Covariance override node (fixes zero-covariance from sim)
add_executable(covariance_override_node src/covariance_override_node.cpp)
ament_target_dependencies(covariance_override_node
  rclcpp nav_msgs sensor_msgs)
target_compile_features(covariance_override_node PUBLIC cxx_std_17)

# IMU vibration filter (Butterworth low-pass on gyro + accel)
add_executable(imu_filter_node src/imu_filter_node.cpp)
target_include_directories(imu_filter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(imu_filter_node rclcpp sensor_msgs)
target_compile_features(imu_filter_node PUBLIC cxx_std_17)

# Wheel-slip detector (Phase 2 of the diff-drive calibration plan).
# Pure-logic library + node wrapper. The library is also used by the
# gtest under test/.
add_library(wheel_slip_detector_lib STATIC src/wheel_slip_detector.cpp)
target_include_directories(wheel_slip_detector_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(wheel_slip_detector_lib PUBLIC cxx_std_17)

add_executable(wheel_slip_detector_node src/wheel_slip_detector_node.cpp)
target_link_libraries(wheel_slip_detector_node wheel_slip_detector_lib)
target_include_directories(wheel_slip_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(wheel_slip_detector_node
  rclcpp nav_msgs sensor_msgs std_msgs)
target_compile_features(wheel_slip_detector_node PUBLIC cxx_std_17)

# Caster-aware dwell advisor (Phase 4, advisory variant).
add_library(caster_dwell_advisor_lib STATIC src/caster_dwell_advisor.cpp)
target_include_directories(caster_dwell_advisor_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(caster_dwell_advisor_lib PUBLIC cxx_std_17)

add_executable(caster_dwell_advisor_node src/caster_dwell_advisor_node.cpp)
target_link_libraries(caster_dwell_advisor_node caster_dwell_advisor_lib)
target_include_directories(caster_dwell_advisor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(caster_dwell_advisor_node
  rclcpp geometry_msgs nav_msgs std_msgs)
target_compile_features(caster_dwell_advisor_node PUBLIC cxx_std_17)

install(TARGETS fusion_monitor_node covariance_override_node imu_filter_node wheel_slip_detector_node caster_dwell_advisor_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_sensor_health test/test_sensor_health.cpp)
  target_include_directories(test_sensor_health PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_compile_features(test_sensor_health PUBLIC cxx_std_17)

  ament_add_gtest(test_wheel_slip_detector test/test_wheel_slip_detector.cpp)
  target_link_libraries(test_wheel_slip_detector wheel_slip_detector_lib)
  target_compile_features(test_wheel_slip_detector PUBLIC cxx_std_17)

  ament_add_gtest(test_caster_dwell_advisor test/test_caster_dwell_advisor.cpp)
  target_link_libraries(test_caster_dwell_advisor caster_dwell_advisor_lib)
  target_compile_features(test_caster_dwell_advisor PUBLIC cxx_std_17)
endif()

ament_package()
