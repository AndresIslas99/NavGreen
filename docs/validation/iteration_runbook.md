# Iteration runbook — oracle-driven HIL validation

## Goal

Converge on `waypoints_tagged_v3.yaml` by running the precision suite in
short iterations. Each iteration consumes **all** sim-side oracles
(visible_markers, obstacles, localization_error, episode_summary, events)
plus the sim_api metrics surface, and generates an **analysis markdown**
that tells you exactly what to fix next. Two consecutive clean iterations
= acceptance.

## Pre-flight

Jetson:

```bash
cd ~/ros2_ws
colcon build --packages-select \
  agv_mode_arbiter agv_rail_driver agv_rail_detector agv_integration_tests
source install/setup.bash
bash tools/verify_specs/all.sh   # must be 0 BLOCKING, 0 warnings
```

Unit tests (run at least once per iteration before the HIL sweep):

```bash
colcon test --packages-select \
  agv_mode_arbiter agv_rail_driver agv_rail_detector agv_integration_tests
# Expect: 22 (mode_fsm) + 4 (rail_exit_geometry) + 19 (rail_controller)
# + 9 (rail_ransac) + 17 (dispatch_logic) + 11 (harness_oracles)
# + 4 (iteration_report) = 86 green.
```

Sim host (coordinated manually):

```bash
# on the sim PC:
ros2 launch agv_sim_validation isaac_hil.launch.py \
  validation:=true enable_api:=true
# make sure Isaac Sim is in Play; check /sim/telemetry RTF ≥ 0.4.
```

## Per-iteration loop

```bash
export ROS_DOMAIN_ID=42
export SIM_API_HOST=<sim_pc_ip>           # no default — fails safe
export AGV_DATA_DIR=$HOME/agv_data        # writable dir for reports
export AGV_WAYPOINTS_YAML=waypoints_tagged_v3.yaml
export AGV_MIN_WAYPOINTS=16
export AGV_TEST_AUTO_RESTART=0            # keep deterministic during iteration

# 1. Brain stack
ros2 launch agv_bringup agv_hil_full.launch.py \
  map:=$HOME/agv_data/maps/greenhouse_v2.yaml \
  cuvslam_in_hil:=false use_gt_odom:=true \
  enable_wheel_odom_bridge:=false

# 2. Precision sweep (in another shell on the Jetson)
python3 -u -m pytest -q -s \
  src/agv_integration_tests/test/test_waypoint_precision.py

# 3. Analysis generator — points at the newest run via latest/ symlink
python3 src/agv_integration_tests/scripts/iteration_report.py \
  $AGV_DATA_DIR/sim_episodes/latest/report.json \
  $AGV_DATA_DIR/sim_episodes/prev/report.json   # optional, for deltas
```

`iteration_report.py` writes
`$AGV_DATA_DIR/sim_episodes/latest/iteration_<N>_analysis.md`. Read it
top-to-bottom; the "Next iteration" section names the top rule to address.

## Decision tree

| Analysis finding | Action |
|---|---|
| All bucket verdicts pass + all gates ✓ | Re-run **once more**. Two consecutive clean runs → **DONE**. |
| Single rail_drive peak > 5 cm | Tune `agv_rail_driver/config/rail_driver_params.yaml` (`kP`, `speed_max_mps`). Rebuild rail_driver only. |
| rail_approach visible-marker miss | Regression in ZED mount / URDF / approach geometry. Check `agv_description/config/robot_params.yaml` and the `rail_approach` params. |
| rail_exit missing from modes | Arbiter FSM did not receive `rail_driver_state=="driving"`. Ensure `/agv/rail_driver/state` is publishing and `mode_arbiter` is alive. |
| drift event AND NAV_TIMEOUT | EKF cold-start or wheel_odom covariance drift. Bump `initial_estimate_covariance` in `ekf_global.yaml`; run again without other changes. |
| watchdog_recovery fired | Physics ejection — usually a collision. Inspect the saved snapshot + events JSON for the contact. |
| collision event | Blocker. Do NOT re-run until the root cause is fixed and a unit-level regression is added. |

Apply **one** smallest diff per iteration. Commit with message
`iter-N: <one-line>`. Re-run.

## Archival

After each iteration, append one line to
`docs/validation/iteration_log.md`:

```
## iter-<N> (<YYYY-MM-DD>)
- report: sim_episodes/precision_run_<ts>/report.json
- decision: <e.g. "bump ekf_global initial_estimate_covariance.yaw 0.01 → 0.05">
- delta vs iter-<N-1>: rail_drive ↑ (2/4 → 4/4), others →
```

`$AGV_DATA_DIR/sim_episodes/latest/` is an auto-updated symlink to the
most recent run. Keep `prev/` (manual symlink) pointing at the previous
iteration if you want the generator to emit Δ columns.

## Acceptance

All of the following must be true in TWO consecutive iterations:

- `summary.success_rate` ≥ 0.95
- `summary.collision_count` == 0
- `summary.p95_err_xy_m` ≤ 0.10, `max_err_xy_m` ≤ 0.15
- `summary.p95_err_yaw_rad` ≤ 0.25
- Per-bucket: nav2 mean ≤ 0.15 m, rail_approach lat ≤ 0.02 m + yaw ≤ 0.017 rad,
  rail_drive peak_lat ≤ 0.05 m + 0 collisions, rail_exit has `rail_exit`
  in modes + final clearance ≥ 1 m
- `iteration_<N>_analysis.md` shows "All gates passed" in both runs

When achieved, merge the branch and close the iteration log.
