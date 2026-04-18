cmake_minimum_required(VERSION 3.8)
project(agv_integration_tests)

find_package(ament_cmake REQUIRED)

install(DIRECTORY test/ DESTINATION share/${PROJECT_NAME}/test)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_service_availability test/test_service_availability.py)
  ament_add_pytest_test(test_topic_availability test/test_topic_availability.py)
  ament_add_pytest_test(test_e_stop test/test_e_stop.py)
  ament_add_pytest_test(test_tf_tree test/test_tf_tree.py)
  ament_add_pytest_test(test_ekf_frames test/test_ekf_frames.py)
  # HIL tests — skip themselves when SIM_API_HOST / ROS_DOMAIN_ID=42 are not set.
  # Timeout bumped — each waypoint may take up to 120 s on Nav2.
  ament_add_pytest_test(test_nav2_probe test/test_nav2_probe.py
    TIMEOUT 180)
  ament_add_pytest_test(test_waypoint_precision test/test_waypoint_precision.py
    TIMEOUT 3600)
  ament_add_pytest_test(test_full_flow test/test_full_flow.py
    TIMEOUT 5400)
endif()

ament_package()
