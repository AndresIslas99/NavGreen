# Iteration loop — autonomous HIL tuning for the 10 cm gate

This file is the runbook for an LLM agent (or a human on autopilot) that
wants to make `test_waypoint_precision` pass. It assumes the sim host is
running with Isaac in Play, and the Jetson stack is up per
`docs/validation/RUNBOOK_lan_hil.md`.

The loop never reboots Isaac. Manual Play is unavoidable; we preserve the
session across rounds by using `POST /reset` on the sim_api instead of
restarting the sim.

## Invocation

```bash
# External driver that does not require claude to stay resident:
claude --loop "corre test_waypoint_precision hasta p95≤0.10, ajusta con \
  docs/validation/iteration_loop.md, máx 5 rondas, si no converge escala"
```

The agent's toolbox:

- `http://${SIM_API_HOST}:8090/` — reset, goal, state, events, metrics, snapshot.
- `${AGV_DATA_DIR}/sim_episodes/*/report.json` — memory between rounds.
- `src/agv_navigation/config/nav2_hil_overrides.yaml` — durable tuning handle.
- `ros2 param set /<node> <param> <value>` — ephemeral tuning handle (reverts on relaunch).
- `/agv/sim/events` — the oracle of failure category.

## The loop

```
round = 0
max_rounds = 5

while round < max_rounds:
    result = run_test("test_waypoint_precision")
    # result has: p95_err_xy_m, max_err_xy_m, success_rate, collision_count,
    #             per-waypoint errors, events timeline.

    if result.p95_err_xy_m <= 0.10 \
       and result.max_err_xy_m <= 0.15 \
       and result.success_rate >= 0.95 \
       and result.collision_count == 0:
        stop SUCCESS

    diagnosis = classify(result)
    action = pick_action(diagnosis)   # from the table below
    apply(action)                     # ros2 param set (ephemeral) OR edit YAML (durable)

    POST /reset {x: 0, y: 0, yaw: 0}  # bring sim to a known pose before next round
    round += 1

stop HUMAN_REVIEW
report: {rounds, per-round (diagnosis, action, metrics), final report.json dir}
```

## Diagnosis table — classify the dominant failure mode

Look at the dominant category across the 20 waypoints. If a single waypoint
fails oddly, treat that one as a flake and re-run before tuning.

| Category | Signal | Typical cause |
|---|---|---|
| `collision_with_known` | 1+ `collision` events on `/agv/sim/events` | costmap inflation too loose OR footprint wrong OR scan_grid_mapper blind spot |
| `oscillation_near_goal` | waypoint SUCCEEDED but err_xy is 0.11–0.15 on many waypoints; Nav2 log shows "rotating in place" within 1 m of goal | MPPI samples cannot resolve sub-cell correction; controller_frequency too low; `xy_goal_tolerance` vs MPPI horizon mismatch |
| `path_blocked_no_obstacle` | ABORTED with `no_valid_path`; GT shows free space on the planned line | inflation_radius too large OR scan_grid_mapper adding phantom obstacles |
| `localization_jump` | `brain_drift > 0.3 m` on adjacent timestamps; `/agv/sim/localization_error` spikes | AprilTag registration error OR wheel-odom reset not absorbed OR cuVSLAM relocalized to a wrong keyframe |
| `goal_timeout_progressing` | NAV_TIMEOUT but the robot was still making progress (distance to goal decreasing) when timer expired | planning cycle too slow OR controller speed cap too tight |
| `goal_timeout_stalled` | NAV_TIMEOUT, robot motion < 0.02 m in last 10 s | `cmd_vel_safe` gated off by collision_monitor or safety_supervisor; log `/agv/collision_monitor_state` |
| `reset_timeout` | RESET_TIMEOUT status from the test | sim_isaac_handler not responsive or crashed; auto-restart (category `physics_corrupted`) |
| `physics_corrupted` | any of: `RESET_TIMEOUT`, `gt_pose == null` for ≥3 consecutive samples, brain drift > 10 m/s (impossible), `/clock` Hz drops to 0 | Isaac Kit crashed, supervisor still up but the current session is broken |
| `goal_no_motion` | NavigateToPose accepted, BT running, but `gt_delta < 0.05 m` after 30 s with goal active | brain est_pose drifted from GT → Nav2 thinks it already arrived OR RotationShim stuck OR cmd_vel_safe gated off downstream of MPPI |
| `wheel_odom_silent` | `/agv/wheel_odom` pub count ≥1 but `ros2 topic hz` 0 messages; ekf_local stuck at SetPose state | `agv_hil_bridges/joint_states_to_wheel_odom` not running OR `/agv/joint_states` from sim silent |
| `scan_silent` | `/agv/scan` pub count ≥1 but `hz` 0 msgs; local_costmap empty; SLAM map 100 % free | `/agv/zed/point_cloud/cloud_registered` RELIABLE pub vs BEST_EFFORT sub — check pointcloud_to_laserscan QoS override applied |

## Action table — what to tune

Prefer ephemeral changes (`ros2 param set`) first to bisect quickly. Only
durably edit YAML when the ephemeral change confirms the diagnosis.

| Diagnosis | Parameter | Change | File | Risk |
|---|---|---|---|---|
| `oscillation_near_goal` | `general_goal_checker.xy_goal_tolerance` | 0.10 → 0.12 (temporary, document gap) | `nav2_hil_overrides.yaml` | violates gate until diagnosed — use as a bisection aid only |
| `oscillation_near_goal` | `controller_frequency` | 10.0 → 15.0 | `nav2_hil_overrides.yaml` controller_server | sim rate-limited; may cause desync |
| `oscillation_near_goal` | MPPI `vx_std` / `wz_std` | ↓ 0.1 | `nav2_params.yaml` FollowPath | slower adaptation in open space |
| `oscillation_near_goal` | MPPI `PathAlignCritic.cost_weight` | ↑ 2.0–5.0 | `nav2_params.yaml` | over-weights path over forward progress |
| `path_blocked_no_obstacle` | `inflation_layer.inflation_radius` | 0.55 → 0.45 | `nav2_hil_overrides.yaml` both costmaps | tighter margin on real hardware |
| `path_blocked_no_obstacle` | `obstacle_layer.obstacle_max_range` | 2.5 → 2.0 | `nav2_hil_overrides.yaml` | less lookahead; may miss distant walls |
| `collision_with_known` | `inflation_layer.inflation_radius` | 0.55 → 0.65 | `nav2_hil_overrides.yaml` | corridors may become untraversable |
| `collision_with_known` | `footprint` | widen front by +0.05 m | `nav2_params.yaml` | changes base geometry assumption |
| `localization_jump` | `ekf_global` `pose0 rejection_threshold` | tighten 3.0 → 2.5 | `ekf_global.yaml` | more rejections; risk of drift in featureless zones |
| `localization_jump` | `marker_correction.relocalization_threshold` | 2.0 → 1.5 | `agv_markers` params | more hard resets; careful |
| `goal_timeout_progressing` | `SmacPlanner2D.max_planning_time` | 2.0 → 2.5 | `nav2_params.yaml` | slower replan on dynamic obstacles |
| `goal_timeout_progressing` | MPPI `vx_max` | 0.25 → 0.3 | `nav2_params.yaml` (production impact — be careful) | exceeds safety chain L4 envelope |
| `goal_timeout_stalled` | inspect `/agv/collision_monitor_state` + `/agv/safety/status` | — | — | find the gate blocking `cmd_vel_safe`, fix root cause |
| `physics_corrupted` | — | `POST http://$SIM_API_HOST:8090/sim/restart` then poll `GET /state` until `gt_pose != null` (≤90 s) | supervisor script on sim host | budget 2/run, cooldown 120 s between restarts; 3rd need → escalate |
| `reset_timeout` (isolated) | — | treat as `physics_corrupted` and restart | — | if a reset_timeout happens without other corruption signals and the restart budget is spent, escalate |
| `wheel_odom_silent` | check `/agv/joint_states` hz first; if sim is emitting, restart `joint_states_to_wheel_odom` (`pkill -f joint_states_to_wheel_odom` — supervisor re-launches via launch file) | `agv_hil_bridges/joint_states_to_wheel_odom` — single-source-of-truth for HIL wheel_odom | if /agv/joint_states also silent, escalate (sim-side) |
| `scan_silent` | verify QoS override in agv_hil_full.launch.py for pointcloud_to_laserscan: `qos_overrides./agv/zed/point_cloud/cloud_registered.subscription.reliability: reliable` — then relaunch brain | `agv_bringup/launch/agv_hil_full.launch.py` pointcloud_to_laserscan Node block | if sim stops publishing pointcloud, escalate |
| `goal_no_motion` | (A) inspect `ros2 topic hz /agv/cmd_vel` (B) inspect `ros2 topic echo /agv/drive_debug --once` — left/right_target non-zero? (C) call `/agv/set_pose` to sync brain to GT pose | pre-goal sync helper (`_sync_brain_to_gt` in `test_waypoint_precision.py`) — ALREADY runs before each waypoint. If it didn't help, diagnose Nav2 controller state | (A)+(B) silent → `ros2 lifecycle set /agv/controller_server deactivate && configure && activate`; (C) worked once → keep running |

## Commands the agent uses

```bash
# Ephemeral param set — fastest bisection
ros2 param set /controller_server FollowPath.PathAlignCritic.cost_weight 14.0

# Durable — survives relaunch
sed -i 's/inflation_radius: 0.55/inflation_radius: 0.45/' \
    src/agv_navigation/config/nav2_hil_overrides.yaml
# Then, if Nav2 was running: restart controller_server + planner_server via
#   ros2 lifecycle set /controller_server deactivate/configure/activate
# OR rebuild+relaunch if the operator prefers clean state.

# Sim reset between rounds
curl -sS -X POST -H 'Content-Type: application/json' \
    -d '{"x": 0, "y": 0, "yaw": 0}' \
    http://$SIM_API_HOST:8090/reset

# Next round
colcon test --packages-select agv_integration_tests \
    --ctest-args -R test_waypoint_precision --event-handlers console_direct+

# Human escalation — dump all 5 rounds' metadata
ls -lt $AGV_DATA_DIR/sim_episodes/precision_run_* | head -5
for r in $(ls -1t $AGV_DATA_DIR/sim_episodes/precision_run_* | head -5); do
    echo "=== $r ==="
    jq '.summary' $r/report.json
done
```

## Auto-restart policy (self-heal via `POST /sim/restart`)

The loop may call `POST http://$SIM_API_HOST:8090/sim/restart` when a
round's report contains signals consistent with `physics_corrupted`. This
is the only autonomous interaction with the sim host lifecycle — every
other tune is a parameter change inside the brain stack.

- **Budget**: max **2 restarts per run** (not per round — per the whole
  5-round loop).
- **Cooldown**: **120 s** between consecutive restart attempts. The
  supervisor typically needs 30–60 s to bring Isaac back; the extra margin
  protects against rebooting before the previous boot settled.
- **Readiness wait**: after `POST /sim/restart`, poll `GET /state` until
  `gt_pose != null` (timeout 90 s). Then re-verify:
  - `ros2 topic hz /clock` > 0 Hz
  - `/navigate_to_pose` action server available (ActionClient.wait_for_server ≤ 10 s)
  - brain-side EKF is publishing on `/agv/odometry/global`
- **Logging**: every restart appends `{ts_unix, reason, waypoint_id, success}`
  to `restart_events` inside the precision run's `report.json`. The
  escalation handler reads these to decide if the failure pattern looks
  structural vs transient.
- **Hard cap**: the **3rd** restart in a single run is a refusal — the
  loop must escalate to human without attempting it.

The harness-level mirror of this policy lives in
`src/agv_integration_tests/test/test_waypoint_precision.py` under the
`AGV_TEST_AUTO_RESTART` env flag (default off, so CI is deterministic).
Enable only when driving from the loop:

```bash
export AGV_TEST_AUTO_RESTART=1
export AGV_TEST_AUTO_RESTART_MAX=2         # default
export AGV_TEST_AUTO_RESTART_COOLDOWN_S=120 # default
```

## Escalation criteria

Stop and ask a human when any of these holds:

- The 3rd auto-restart would be needed within a single run (budget = 2).
- 5 rounds elapsed AND `restart_count ≥ 2` AND the gate still fails —
  self-heal budget kept getting consumed without the metric converging;
  something structural is wrong.
- Any round introduces new `collision_with_known` failures on waypoints
  that previously passed — you may have pushed the inflation radius past
  a corridor bottleneck; revert the last change.
- `p95_err_xy_m` is not monotonically decreasing across rounds — tuning
  is oscillating; the diagnosis is wrong.
- 5 rounds elapsed without meeting the gate even without any restarts
  (pure tune-space exhaustion).

Escalation payload:

- All `report.json` files under `${AGV_DATA_DIR}/sim_episodes/precision_run_*/`
  for the current session.
- `restart_events` lists from each report (if any).
- Diff of `nav2_hil_overrides.yaml` vs the committed version.
- The last sim-host log: `ssh orza@$SIM_HOST 'tail -500 ~/.ros/log/latest/*.log'`
  (useful if `physics_corrupted` fired — reveals what crashed Isaac).

## What the loop does NOT fix

- Hardware blind zone (30 cm in front of ZED) — documented in
  `src/agv_navigation/CLAUDE.md`. Not tunable in software.
- USD staleness — if the sim host shows wheel_radius mismatch vs the Jetson's URDF,
  the agent cannot regenerate the USD. Escalate.
- DDS peer list — if `ros2 topic list` shows no `/agv/sim/*`, the agent must
  exit with instructions to edit `cyclonedds.xml` on the sim host, not try to
  tune Nav2.
- Bugs in the waypoint_manager's localization bypass — known gap, separate PR.

The loop is a local-search optimizer on the Nav2 parameter subspace, not a general fix-everything tool.
