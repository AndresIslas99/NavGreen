cmake_minimum_required(VERSION 3.8)
project(agv_integration_tests)

find_package(ament_cmake REQUIRED)

install(DIRECTORY test/ DESTINATION share/${PROJECT_NAME}/test)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_service_availability test/test_service_availability.py)
  ament_add_pytest_test(test_topic_availability test/test_topic_availability.py)
endif()

ament_package()
