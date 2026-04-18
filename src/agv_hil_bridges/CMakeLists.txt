cmake_minimum_required(VERSION 3.8)
project(agv_hil_bridges)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/joint_states_to_wheel_odom.py
  scripts/vslam_fallback_relay.py
  scripts/gt_to_wheel_odom.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
