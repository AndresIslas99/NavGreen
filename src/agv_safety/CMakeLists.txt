cmake_minimum_required(VERSION 3.8)
project(agv_safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(agv_interfaces REQUIRED)

include_directories(include)

# -- safety_supervisor executable --
add_executable(safety_supervisor_node
  src/safety_supervisor_main.cpp
  src/safety_supervisor.cpp
)
ament_target_dependencies(safety_supervisor_node
  rclcpp
  std_msgs
  agv_interfaces
)

# -- cmd_vel_gate executable --
add_executable(cmd_vel_gate_node
  src/cmd_vel_gate_main.cpp
  src/cmd_vel_gate.cpp
)
ament_target_dependencies(cmd_vel_gate_node
  rclcpp
  std_msgs
  geometry_msgs
  agv_interfaces
)

install(TARGETS
  safety_supervisor_node
  cmd_vel_gate_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_gtest(test_supervisor_logic
    test/test_supervisor_logic.cpp
    src/safety_supervisor.cpp
  )
  target_include_directories(test_supervisor_logic PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_target_dependencies(test_supervisor_logic
    rclcpp
    std_msgs
    agv_interfaces
  )

  ament_add_gtest(test_gate_logic
    test/test_gate_logic.cpp
    src/cmd_vel_gate.cpp
  )
  target_include_directories(test_gate_logic PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_target_dependencies(test_gate_logic
    rclcpp
    std_msgs
    geometry_msgs
    agv_interfaces
  )

  # launch_testing — exemplar for Gap 4 of docs/architectural_gaps.md
  add_launch_test(test/launch/test_gate_smoke.launch.py)
endif()

ament_package()
