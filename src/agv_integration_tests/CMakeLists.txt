cmake_minimum_required(VERSION 3.8)
project(agv_integration_tests)

find_package(ament_cmake REQUIRED)

install(DIRECTORY test/ DESTINATION share/${PROJECT_NAME}/test)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Stack-required tests — skip themselves unless AGV_STACK_TEST=1 is set
  # with the full stack running (agv_full.launch.py); assert hard otherwise.
  ament_add_pytest_test(test_service_availability test/test_service_availability.py)
  ament_add_pytest_test(test_topic_availability test/test_topic_availability.py)
  ament_add_pytest_test(test_e_stop test/test_e_stop.py)
  ament_add_pytest_test(test_tf_tree test/test_tf_tree.py)
  ament_add_pytest_test(test_ekf_frames test/test_ekf_frames.py)
  # Phase 2 — ROS-free oracle unit tests (always run).
  ament_add_pytest_test(test_mode_transitions_oracle
    test/test_mode_transitions_oracle.py)
  # Phase 2 G — dispatch router pure-logic tests.
  ament_add_pytest_test(test_dispatch_logic
    test/test_dispatch_logic.py)
  # Round 44 Q4 — harness oracle parsers (ROS-free).
  ament_add_pytest_test(test_harness_oracles
    test/test_harness_oracles.py)
  # Round 44 Q5 — iteration analysis generator (ROS-free).
  ament_add_pytest_test(test_iteration_report
    test/test_iteration_report.py)
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
