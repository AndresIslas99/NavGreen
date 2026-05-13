# Phase 1 future_work

Items detected by the agent during Phase 1 sub-phases that are
out-of-scope for the current sub-phase. Capture here instead of
remediating inline (Sub-fase 1.1 §0 rule "scope estricto").

---

## Captured during Sub-fase 1.1.a

### specs/interfaces.yaml coverage gap

`verify_topic_types.py` reports 69 topic declarations in `src/agv_*`
on topics that don't appear in `specs/interfaces.yaml`. These are
not type mismatches but spec-coverage gaps. Sample (full list via
`VERBOSE=1 python3 tools/verify_specs/verify_topic_types.py`):

- `/agv/rail_driver/goal` (PoseStamped, mode_arbiter publishes,
  rail_driver subscribes — purely internal, but cross-package)
- `/agv/clear_map` (Bool, scan_grid_mapper)
- `/agv/zed/pose_with_covariance`, `/agv/zed/pose/status` (consumed
  by auto_init_orchestrator from the ZED wrapper)
- `/agv/detections` (apriltag_msgs from external pkg)
- `/agv/markers/registry_reload` (Empty trigger)
- `/agv/behavior_executor/status`
- `/agv/safety_status` (cmd_vel_gate / safety_supervisor)
- `/agv/motor_state`, `/agv/drive_debug` (ODrive published)
- `/agv/motor_enable` (Bool consumed by ODrive)
- `/slam/diagnostics` (test-only)
- Several `/agv/cmd_vel_*` variants

Action: dedicated cleanup sprint to expand specs/interfaces.yaml.
Not Phase-1-blocking; the type-matching verifier already covers the
65 topics that ARE in spec.

### TypeScript / rclnodejs not covered by verify_topic_types.py

`verify_topic_types.py` v1 only parses C++. `rclnodejs` (used by
`agv_ui_backend`) uses runtime IDL strings, so the catastrophic
silent-drop mode of C++ template subs doesn't apply identically.
But there are still ways to mis-declare a subscription. A v2 could
parse `.ts` files for `node.createSubscription(...)` patterns.

### ODrive vel_limit_tolerance asymmetry (G5 leftover)

LEFT vel_limit_tolerance=1.30 vs RIGHT=1.20. Not a control gain
(it's a safety margin), so out of G5 scope. Match-up recommended
during a future ODrive housekeeping window.

### G6 joint_states publisher bug

`src/agv_odrive/src/odrive_can_node.cpp:494-497` publishes
`msg.position = left_.position * 2π` (motor radians) but names the
joints `left_wheel_joint` and `right_wheel_joint` (URDF wheel
joints). The division by `gear_ratio_` is missing, so RViz spins
the wheels 10× too fast cosmetically. `wheel_odom` computation is
unaffected (uses correct math). One-line fix, deferred.

### HIGH-04-09 empirical re-validation

The HIGH-04-09 fix (commit `0428b39`, imu0_config orientation row
→ [F,F,F]) is `CLOSED-VERIFIED-CODE`. Empirical re-validation of
the post-fix yaw delta (target < 0.5° vs the +2.4° baseline from
Section-0 F4) was deferred per operator direction to move forward
to Phase 1. The first hardware USB-reboot test during Phase 1
operations will close the verdict to `CLOSED-VERIFIED-HW`.

---

## Capture rules

Add an entry here when:
- The current sub-phase explicitly forbids touching some code area.
- A non-blocking improvement is observed mid-task.
- The fix risk exceeds the value at this point in the schedule.

Don't capture here when:
- The current sub-phase prompt asks for it (do the work).
- The item is a runtime bug actively breaking the system (raise,
  don't queue).
