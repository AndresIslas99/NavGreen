# Baseline Protocol — diff-drive odometry characterization

This is the reproducible procedure for measuring the AGV's wheel
odometry against AprilTag ground truth. Every change to the odometry
stack (calibration, slip detector, EKF config) MUST be evaluated by
running this protocol before and after, and committing the resulting
CSV under `tools/calib_runs/`. The accumulated history lives in
`docs/calibration/history.md`.

## Why this baseline

`/agv/wheel_odom` is a *predictor*, not a measurement of ground truth.
The session of 2026-04-25 confirmed that diff-drive with passive
caster wheels exhibits a **systematic bias of ~20%** between
wheel-encoder distance and AprilTag-measured distance, with significant
asymmetry between forward and reverse motion. The bias has multiple
sources (wheel radius, track width, caster slip on smooth floor, …)
and it is **dishonest to attribute it to a single parameter** without
controlled measurements.

This protocol is the controlled measurement. It does NOT pretend to
fix the odometry — it makes the bias *legible* so we can decide
whether a code change actually improves it.

## Hardware setup

- AGV running the production stack via `agv.service` (or manually via
  `agv_start.sh`), with the Nav2 + safety chain active.
- An AprilTag from the **tag36h11** family, **20 cm** outer side
  (border included, matching `apriltag_node`'s `size: 0.2`), placed
  flat on the floor in front of the robot.
- A clear corridor of at least **2 m forward and 2 m reverse** from
  the robot's starting pose so the operator can drive without
  collisions.
- Lighting that produces consistent tag detection. Validate with
  `python3 tools/calib_apriltag_probe.py --duration 5` before
  starting; expect ≥ 30 hits/3 s with `decision_margin > 50` and
  `hamming = 0`.

## Required pre-checks

```bash
# 1. Stack is up and motors are armed
curl -s http://localhost:8090/api/status | python3 -m json.tool | \
  grep -E "robot_state|motors_armed|e_stop|mode"
#    expect: motors_armed=true, e_stop=false, mode=teleop

# 2. The dashboard is open in another tab so the e-stop button is
#    reachable. The dashboard's stale-command watchdog publishing zeros
#    is fine for this protocol because the operator drives by joystick
#    (which keeps lastCmdTime fresh).

# 3. AprilTag detector parameter `detector.decimate` is 1.0 (NOT 2.0
#    default). Verify:
ros2 param get /agv/apriltag_node detector.decimate
#    if it says 2.0, the launch parameter override is broken — see
#    src/agv_bringup/launch/agv_full.launch.py:487 history.
```

## Running a session

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///tmp/agv_cyclonedds_runtime.xml

python3 tools/calib_diff_drive_baseline.py \
    --surface "polished ceramic"       \
    --payload-kg 0                     \
    --label "post-fase-2"              \
    --record-bag                       \
    --id 12 --tag-size 0.20
```

The operator drives the robot with the joystick from the dashboard.
Each "arranque-y-paro" (motion-then-stop) cycle is captured as one
leg. Aim for at least **20 useful legs** with a mix of:

- 5+ short forward (~5–10 cm)
- 5+ short reverse (~5–10 cm)
- 5+ medium (15–30 cm) in either direction
- 3+ direction reversals (forward → reverse without long pause)

Stop the script with `Ctrl+C`. The CSV is written under
`tools/calib_runs/baseline_<ISO>_<label>.csv` and the bag (if
`--record-bag`) under `tools/calib_runs/bag_<ISO>/`.

## Reading the CSV

The CSV header records session metadata (surface, payload, git SHA,
session timestamp). The data rows have one entry per useful leg with:

- `forward_proxy` — robot forward delta from the AprilTag observation,
  in meters. **This is the ground truth.**
- `dodom_distance` — wheel-encoder integrated distance, signed.
- `error_distance = forward_proxy - dodom_distance`.

The headline metric is **median(dodom_distance / forward_proxy)** over
legs with `|forward_proxy| ≥ 5 cm` (smaller legs are dominated by
solvePnP pixel noise). A value of 1.000 means perfect odometry; today
the AGV measures ~1.20 (odometry over-reports by 20%).

## When to invalidate a session

Discard the run if any of:

- Bus voltage drops below 24 V at any point — undervoltage protection
  on the ODrive starts limiting torque non-deterministically.
- Tag visibility drops (operator drove out of the camera FOV). The
  script flags lost-tag legs but if more than 30% of legs are lost,
  the geometry is wrong; reposition the tag.
- The dashboard's collision_monitor goes into STOP repeatedly. If the
  operator is provoking it, call the run a *safety stress test* and
  not a baseline.
- The Jetson load average goes above 15 for sustained periods (check
  `top` in another terminal). Heavy load distorts cmd_vel timing.

## Frozen baselines

| Date       | Surface          | Payload | N legs | median Δodom/Δtag | std  | Notes                                    |
|------------|------------------|---------|--------|--------------------|------|------------------------------------------|
| 2026-04-25 | polished ceramic | 0 kg    | 30 (14 large) | **1.205** (+20.5%) | 0.13 | Original baseline. Cause: caster slip. See `docs/calibration/history.md`. |

Add new rows here only after running this protocol against the
current `main` branch with no in-flight changes. Use `git log` to
identify the SHA.
