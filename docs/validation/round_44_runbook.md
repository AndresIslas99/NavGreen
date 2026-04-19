# Round 44 — Rail entry / exit / inter-rail validation

## Purpose

Validate in HIL the three scenarios the earlier rounds left untested after
Phase 2 (J/K/M) merged:

1. Rail entries at **off-center aisles** (y = ±2.2).
2. The **RAIL_EXIT flow end-to-end** (RAIL_DRIVE → RAIL_EXIT → CORRIDOR_NAV),
   including the arbiter-published push goal 1.5 m past the exit AprilTag.
3. A **rail-to-rail transition** using an intermediate gap waypoint
   (lane-change + FRONT entry at a different aisle).

## Preconditions

- Jetson branch ahead of `main` with Stage M1 (arbiter aisle-side geometry)
  + Stage N (waypoints_tagged_v3.yaml) + Stage O (rail_exit dispatch) built.
- Unit tests green:
  - `colcon test --packages-select agv_mode_arbiter` → 22 FSM + 4
    rail_exit_geometry = 26/26.
  - `colcon test --packages-select agv_integration_tests --ctest-args -R
    dispatch_logic` → 17/17.
  - `bash tools/verify_specs/all.sh` → 0 BLOCKING, 0 warnings.
- Sim host running with `agv-greenhouse-sim` commit ≥ 3d44cec, publishing
  ZED stereo + IMU + joint_states + ground truth.
- Cross-machine discovery via CycloneDDS (`agv_start.sh` generates the
  runtime XML).

## Launch commands

Jetson:

```bash
cd /home/orza/ros2_ws
colcon build --packages-select agv_mode_arbiter agv_integration_tests
source install/setup.bash

ros2 launch agv_bringup agv_hil_full.launch.py \
  map:=/home/orza/agv_data/maps/greenhouse_v2.yaml \
  cuvslam_in_hil:=false use_gt_odom:=true enable_wheel_odom_bridge:=false
```

`auto_approach` stays false — the dispatch harness calls `rail_approach`
explicitly. The new `rail_exit` dispatch uses the FSM shortcut path
combined with M1's aisle-side geometry, so no extra flag is needed.

## Run the suite

```bash
AGV_WAYPOINTS_YAML=waypoints_tagged_v3.yaml AGV_MIN_WAYPOINTS=16 \
  AGV_TEST_AUTO_RESTART=0 \
  python3 -u -m pytest -q -s \
    src/agv_integration_tests/test/test_waypoint_precision.py
```

Expected runtime: ~8 minutes (16 waypoints, 20–30 s avg each with teleport
+ sync + drive).

## Per-bucket gates

| Bucket        | Waypoints                    | Threshold                                                               |
|---------------|------------------------------|-------------------------------------------------------------------------|
| nav2          | wp01, wp02, wp03, wp10, wp15 | mean err_xy ≤ 0.15 m, 0 collisions                                      |
| rail_approach | wp04, wp07, wp11, wp12, wp16 | final lat err ≤ 0.02 m, yaw err ≤ 0.017 rad                             |
| rail_drive    | wp05, wp06, wp08, wp09       | lat drift peak ≤ 0.05 m, 0 collisions, no blocked_lateral/misaligned    |
| rail_exit     | wp13, wp14                   | modes include `rail_exit`; final err_xy (vs exit_goal) ≤ 0.15 m; angular_z always 0; 0 collisions |
| Overall       | 16                           | ≥ 14/16 SUCCESS                                                         |

## Report

`report.json` lands at
`${AGV_DATA_DIR:-$HOME/agv_data}/sim_episodes/precision_run_{ts}/report.json`.
Per-waypoint entries now carry `dispatch_used ∈ {nav2, rail_approach,
rail_drive, rail_exit}`. Append the summary block + per-bucket verdict to
this runbook once the round completes. Flag any bucket that misses its
gate; re-run only after root-causing.

## Known limitations

- Each waypoint teleports; wp15 + wp16 validate the rail-to-rail flow as
  two legs, not one continuous trajectory. A future round can chain
  them with the sim `/reset` disabled for adjacent legs.
- `rail_exit` dispatch relies on the FSM shortcut + M1 geometry. If the
  brain stack is launched without `mode_arbiter`, wp13/wp14 hang on
  `rail_exit` mode transition and time out at 20 × 1.5 = 30 s.
- `auto_approach=true` is NOT covered by this round. The dispatch harness
  drives every scenario explicitly.
