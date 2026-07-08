# agv_integration_tests

Python-based integration test suite that validates system-level behavior.
Tests fall into three tiers with explicit skip semantics — no test in
this package passes silently against a broken or absent stack.

## Test tiers

### 1. ROS-free oracle tests — always run (CI-safe, no stack needed)

- **test_mode_transitions_oracle.py** — mode-transition recorder/subsequence oracles
- **test_dispatch_logic.py** — dispatch router pure-logic tests
- **test_harness_oracles.py** — harness oracle parsers
- **test_iteration_report.py** — iteration analysis generator

### 2. Stack-required tests — skip unless `AGV_STACK_TEST=1`

Require the full stack running (`agv_full.launch.py`). They skip with an
explicit message when `AGV_STACK_TEST=1` is not set, and assert hard when
it is. Namespace comes from `AGV_NAMESPACE` (default `agv`).

- **test_e_stop.py** — publishes `/{ns}/e_stop` and asserts the ODrive
  driver acknowledges (drive_debug `e_stop:true`, zeroed wheel targets)
  within the acceptance.yaml budget plus a 10 Hz observation allowance,
  then that measured wheel speed reaches zero. Releases the e-stop only
  if the test latched it itself.
- **test_service_availability.py** — asserts 7 required services exist
- **test_topic_availability.py** — asserts 14 required topics exist
- **test_tf_tree.py** — asserts required TF frames appear within 5 s
- **test_ekf_frames.py** — asserts odom->base_link and map->odom exist
  (dual EKF frame ownership, via a tf2_ros buffer)

### 3. HIL tests — skip without `SIM_API_HOST` + `ROS_DOMAIN_ID=42`

See `docs/validation/RUNBOOK_lan_hil.md`.

- **test_nav2_probe.py** — 90 s Nav2 closed-loop smoke test
- **test_waypoint_precision.py** — 20-waypoint precision gate
- **test_full_flow.py** — full mapping/localization/navigation flow

## How to run

Oracle tests only (what CI runs — no stack needed):
```bash
colcon test --packages-select agv_integration_tests
colcon test-result --verbose
```

Stack-required tests (operator machine, stack running):
```bash
AGV_STACK_TEST=1 colcon test --packages-select agv_integration_tests
```

## Dependencies

- pytest, rclpy, std_msgs, tf2_ros (Python), subprocess (ros2 CLI)

## Improvement Opportunities

- Add pipeline integration tests (odometry -> EKF -> navigation end-to-end)
- Hardware-instrumented E-stop latency measurement (the software test
  observes through a 10 Hz debug topic; the strict 0.2 s command-to-stop
  gate needs external timing)
- Add Nav2 lifecycle state validation test
- Increase coverage beyond basic availability checks
