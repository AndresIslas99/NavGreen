cmake_minimum_required(VERSION 3.8)
project(agv_sim)

# Pure ament_cmake install package: launch files, world, controllers config and
# the sim URDF. No compiled targets (all simulation compute is Gazebo's).
find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  worlds
  config
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
