# agv_integration_tests

Python-based integration test suite that validates system-level behavior:
service availability, topic availability, and E-stop propagation.
Requires a running ROS2 stack for execution.

## Test Files

1. **test_e_stop.py** — Verifies E-stop signal propagation and motor response
2. **test_service_availability.py** — Checks 7 required services are available
3. **test_topic_availability.py** — Checks 13+ required topics are publishing

## Required Services Validated

```
/agv/map_manager/save_map
/agv/map_manager/load_map
/agv/map_manager/update_zone
/agv/waypoint_manager/save
/agv/waypoint_manager/list
/agv/waypoint_manager/execute
/agv/navigate_to_pose/_action/get_result
```

## Required Topics Validated

```
/agv/wheel_odom, /agv/joint_states, /agv/motor_state
/agv/drive_debug, /agv/cmd_vel, /agv/e_stop, /agv/scan
/agv/odometry/local, /agv/odometry/global
/agv/map, /agv/plan
/tf, /tf_static
/visual_slam/tracking/odometry
```

## How to Run

Requires full stack running (e.g., via `agv_full.launch.py`):
```bash
colcon test --packages-select agv_integration_tests
colcon test-result --verbose
```

## Dependencies

- pytest, rclpy, subprocess (ros2 CLI)

## Improvement Opportunities

- Add pipeline integration tests (odometry -> EKF -> navigation end-to-end)
- Add latency tests (E-stop response time < 200ms per acceptance.yaml)
- Add TF tree completeness test (all required frames within 5s)
- Add Nav2 lifecycle state validation test
- Increase coverage beyond basic availability checks
