cmake_minimum_required(VERSION 3.8)
project(agv_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(PROGRAMS
  scripts/agv_start.sh
  scripts/agv_healthcheck.sh
  scripts/agv_watchdog.sh
  scripts/field_test.py
  scripts/caster_tuning.py
  DESTINATION lib/${PROJECT_NAME})

# systemd unit files installed to share/ so the install.sh helper can find
# them via `ros2 pkg prefix agv_bringup` — install.sh itself still copies to
# /etc/systemd/system/ (requires sudo).
install(DIRECTORY systemd/
  DESTINATION share/${PROJECT_NAME}/systemd
  USE_SOURCE_PERMISSIONS)

ament_package()
