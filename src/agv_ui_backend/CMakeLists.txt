cmake_minimum_required(VERSION 3.8)
project(agv_ui_backend)

find_package(ament_cmake REQUIRED)

install(DIRECTORY static/ DESTINATION share/${PROJECT_NAME}/static)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# Install TypeScript backend wrapper as a ROS2 executable
install(PROGRAMS scripts/teleop_backend.sh
        DESTINATION lib/${PROJECT_NAME}
        RENAME teleop_backend)

ament_package()
