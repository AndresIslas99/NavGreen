cmake_minimum_required(VERSION 3.8)
project(agv_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY maps/ DESTINATION share/${PROJECT_NAME}/maps)

ament_package()
