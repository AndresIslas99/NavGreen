cmake_minimum_required(VERSION 3.8)
project(agv_odrive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# -- Main executable --
add_executable(odrive_can_node
  src/main.cpp
  src/odrive_can_node.cpp
  src/can_socket.cpp
)

target_include_directories(odrive_can_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(odrive_can_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)

install(TARGETS odrive_can_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(PROGRAMS
  scripts/odrive_can_node.py
  scripts/odrive_gui.py
  scripts/validate.py
  scripts/simple_test.py
  scripts/calibrate_odom.py
  DESTINATION lib/${PROJECT_NAME}
)

# -- Tests --
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_include_directories(test_kinematics PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  ament_add_gtest(test_odom_integration test/test_odom_integration.cpp)
  target_include_directories(test_odom_integration PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
endif()

ament_package()
