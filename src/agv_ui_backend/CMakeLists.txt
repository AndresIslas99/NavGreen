cmake_minimum_required(VERSION 3.8)
project(agv_ui_backend)

find_package(ament_cmake REQUIRED)

install(DIRECTORY static/ DESTINATION share/${PROJECT_NAME}/static)

# Install the compiled TypeScript output when it exists (run `npm ci && npm
# run build` in this package first). OPTIONAL keeps colcon builds working on
# a fresh checkout where dist/ has not been generated yet. Note: running the
# backend from the install space also requires node_modules/ to be
# provisioned at share/agv_ui_backend/node_modules (see scripts/teleop_backend.sh).
install(DIRECTORY dist/ DESTINATION share/${PROJECT_NAME}/dist OPTIONAL)

# Install TypeScript backend wrapper as a ROS2 executable
install(PROGRAMS scripts/teleop_backend.sh
        DESTINATION lib/${PROJECT_NAME}
        RENAME teleop_backend)

ament_package()
