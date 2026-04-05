cmake_minimum_required(VERSION 3.8)
project(agv_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_urdf_valid test/test_urdf_valid.py)
endif()

ament_package()
