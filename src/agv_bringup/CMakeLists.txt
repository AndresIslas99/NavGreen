cmake_minimum_required(VERSION 3.8)
project(agv_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(PROGRAMS
  scripts/agv_start.sh
  scripts/field_test.py
  scripts/caster_tuning.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
