# RUNBOOK — Jetson side of the LAN HIL validation loop

This file is the Jetson counterpart to the `RUNBOOK.md` in the sim repo
(`agv-greenhouse-sim`). The sim-side runbook covers how to boot Isaac,
the overlay, and `sim_api`. This one covers what the Jetson operator
does to consume that overlay and run the validation harness.

**Assumption:** you are SSH'd into the Jetson, the sim host is a separate
Linux PC on the LAN, and DDS crosses that LAN via Cyclone with a unicast
peer list configured on the sim side.

## 1. Network topology

| Host | Role | ROS distro | Interfaces |
|---|---|---|---|
| Jetson Orin (this host) | brain: Nav2, EKF, map_manager, waypoint_manager | Jazzy | USB eth `192.168.55.1` / WiFi `JETSON-LAN-IP` |
| Sim host (PC Linux + GPU) | Isaac Sim, overlay, sim_api, foxglove | Humble | USB eth `192.168.55.100` / WiFi `SIM-HOST-LAN-IP` |

**Prefer the USB connection** (`192.168.55.x`). It is stable, not affected
by the greenhouse WiFi, and already wired into the sim repo's
`cyclonedds.xml` peer list.

**SSH to the sim host from the Jetson:**

```bash
ssh orza@192.168.55.100   # USB — preferred
ssh orza@SIM-HOST-LAN-IP    # WiFi — fallback
```

Use key-based auth (ssh-copy-id).

## 2. Cross-distro DDS boundary — the rules

1. **`ROS_DOMAIN_ID=42` is mandatory** on any Jetson terminal that needs
   to see `/agv/sim/*` topics. The Jetson `.bashrc` defaults to 0. The
   sim host's launch (`isaac_hil.launch.py`) forces Domain 42 internally
   — if you don't export it here, `ros2 topic list` shows nothing from the
   sim.
2. **Only standard ROS message types cross the LAN**. Humble↔Jazzy IDL
   compatibility is guaranteed for `std_msgs`, `geometry_msgs`, `nav_msgs`,
   `nav2_msgs`, `sensor_msgs` but NOT for `agv_interfaces/*` or any other
   custom package. The overlay is designed around this constraint — do not
   introduce a custom-typed topic under `/agv/sim/*` without building the
   IDL on both distros first.
3. **Cyclone peer list** lives in the sim repo at `cyclonedds.xml`. It
   contains both Jetson IPs (USB and WiFi). If either IP changes (DHCP
   lease roll, new router), edit the peer list on the sim host and
   restart the sim-side terminals. Static IP via DHCP reservation on the
   greenhouse router is the production fix.

## 3. Prerequisites before you start the harness

Run these once per day or after any sim host restart:

```bash
# A. Verify the sim host is reachable
ping -c 2 192.168.55.100                    # expect <1 ms RTT

# B. Verify the sim host's Cyclone knows this Jetson
ssh orza@192.168.55.100 'grep -c "192.168.55.1\|JETSON-LAN-IP" \
    /home/orza/agv-sim/cyclonedds.xml'       # expect >= 1

# C. Verify the sim host's USD is not stale (see sim RUNBOOK §2 note)
ssh orza@192.168.55.100 'ls -la /home/orza/agv-sim/src/agv_isaac_sim/worlds/greenhouse_with_robot.usd'
# If mtime is older than the last build_greenhouse_usd.py change, regenerate
# on the sim host:
#   isaacsim --exec src/agv_isaac_sim/scripts/build_greenhouse_usd.py
#   isaacsim --exec src/agv_isaac_sim/scripts/import_robot_usd.py
```

## 4. Encendido — strict order

Order matters. If you launch Nav2 before `/clock` publishes, it hangs
waiting for sim time and never recovers.

**Sim host T1 — Isaac Sim + in-Kit handler (auto-play, supervised):**

```bash
ssh orza@192.168.55.100
cd ~/agv-sim
./run_isaac_supervised.sh     # default since 2026-04-16: auto-play + crash-restart
# Expected console lines (auto-play — no manual Play needed):
#   [AGV] Opened greenhouse_with_robot.usd
#   [AGV][auto-play] scheduled (disable with AGV_AUTO_PLAY=0)
#   [AGV][handler] PhysxContactReportAPI applied to N prims   (N > 0)
#   [AGV][auto-play] simulation started automatically
#   [INFO] [sim_isaac_handler]: sim_isaac_handler ready
#
# Legacy manual-Play flow (fallback):
#   AGV_AUTO_PLAY=0 ./run_isaac_supervised.sh   # then press Play in viewport
#   ./run_isaac_sim.sh                          # no supervisor, no auto-play
```

The supervisor relaunches Isaac if it crashes OR on `POST :8090/sim/restart`
from the Jetson. Typical restart downtime is 30–60 s — the LLM agent must
poll `GET /state` until `gt_pose` is non-null before resuming a test run
(see §6.1 below).

**Sim host T2 — overlay + sim_api + foxglove:**

```bash
ssh orza@192.168.55.100
cd ~/agv-sim && source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch agv_isaac_sim isaac_hil.launch.py validation:=true enable_api:=true
```

**Jetson — brain stack:**

```bash
# New Jetson terminal — the launch sets its own ROS_DOMAIN_ID via env.
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch agv_bringup agv_hil_full.launch.py map:=<map_name>
```

## 5. Verificación del link DDS — 6 chequeos

Nueva terminal en la Jetson:

```bash
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=42                       # CRITICAL — default is 0

# 1. /clock is flowing (sim is in Play)
ros2 topic hz /clock                          # expect > 0 Hz

# 2. The 9 /agv/sim/* topics are discoverable
ros2 topic list | grep /agv/sim/              # expect 8–9 lines

# 3. Ground truth pose frame is correct
ros2 topic echo /agv/sim/ground_truth/pose --once | grep frame_id
# expect "frame_id: map"    — if you see "world", update specs and/or
# ask the sim-side team to add a world→map identity TF on publish.

# 4. Obstacle catalogue is latched and has all 11 entries
ros2 topic echo /agv/sim/ground_truth/obstacles --once | python3 -c "
import sys, json
data = json.loads(sys.stdin.read().split('data: ',1)[1])
print(f\"obstacles: {len(data['obstacles'])}\")"          # expect 11

# 5. sim_api HTTP reachable from brain
curl -s http://192.168.55.100:8090/state | head -c 400    # JSON snapshot
curl -s http://192.168.55.100:8090/metrics | head -c 200  # JSON totals

# 6. Cross-distro IDL sanity
ros2 interface show geometry_msgs/msg/PoseStamped | head -5
# Must resolve cleanly. If it errors, your workspace build is out of sync.

# 7. Brain-side HIL bridges producing downstream topics (post 2026-04-17)
ros2 topic hz /agv/joint_states    # ≥30 Hz — sim encoder emulation
ros2 topic hz /agv/wheel_odom      # ~50 Hz — joint_states_to_wheel_odom (Jetson)
ros2 topic hz /agv/scan            # ~10 Hz — pointcloud_to_laserscan (Jetson)
ros2 topic hz /visual_slam/tracking/odometry  # 10-30 Hz cuVSLAM, or 50 Hz if fallback
```

All 9 checks must be OK before running the harness.

## 6.1. LLM remote control — `/sim/*` endpoints

The autonomous iteration loop (`docs/validation/iteration_loop.md`) uses
these HTTP endpoints for self-heal without SSH, rclpy, or DDS. All three
land on the sim_api FastAPI at port 8090 of the sim host.

```bash
SIM=http://192.168.55.100:8090   # sim host IP (USB preferred)

# Pause the timeline (physics + /clock frozen, Isaac process stays alive)
curl -sS -X POST $SIM/sim/stop

# Resume the timeline (idempotent if already playing)
curl -sS -X POST $SIM/sim/play

# Hard restart: kills Isaac; supervisor relaunches it with auto-play.
# The endpoint returns immediately — Isaac is NOT ready yet when it does.
curl -sS -X POST $SIM/sim/restart

# Post-restart readiness poll: gt_pose is the primary signal.
until curl -sS $SIM/state | jq -e '.gt_pose != null' > /dev/null; do
    sleep 3
done
# Additional sanity check before resuming tests:
ros2 action list | grep /navigate_to_pose    # must be present after brain re-settles
```

Budget recommendation for automated loops: **max 2 restarts per run**,
**cooldown 120 s** between restarts. A third restart within one run is a
human-review signal, not another retry — the supervisor may be failing
silently (e.g. USD corruption) and a human needs to read the sim-side logs.

## 4.1 — Topic flow post 2026-04-17 sim refactor

Commit `3d44cec` in `agv-greenhouse-sim` removed every topic that the
Jetson brain handles on the real robot. The sim is now a pure
hardware-emulator + oracle. The brain must therefore run:

- `agv_hil_bridges/joint_states_to_wheel_odom` — integrates
  `/agv/joint_states` (sim encoder emulation) into `/agv/wheel_odom`.
- `pointcloud_to_laserscan` — consumes
  `/agv/zed/point_cloud/cloud_registered` (RELIABLE QoS!) and publishes
  `/agv/scan`.
- `cuVSLAM` via `agv_slam.launch.py` (default in HIL: on) — consumes
  `/agv/zed/left|right/*` + `/agv/imu/data`, publishes
  `/visual_slam/tracking/odometry`.
- Fallback `agv_hil_bridges/vslam_fallback_relay` — only when
  `cuvslam_in_hil:=false`, relays wheel_odom as a coarse vslam signal.

ASCII-ish flow:

```
SIM HOST                                     JETSON
─────────────────                            ─────────────────
Isaac (encoder)    /agv/joint_states  ─▶  joint_states_to_wheel_odom
                                            └─▶ /agv/wheel_odom ─▶ covariance_override ─▶ ekf_local
Isaac (BMI088)     /agv/imu/data      ─▶  covariance_override ─▶ ekf_local, ekf_global
Isaac Replicator   /agv/zed/point_cloud/cloud_registered
                   (RELIABLE)         ─▶  pointcloud_to_laserscan ─▶ /agv/scan ─▶ slam_toolbox + costmap
Isaac stereo + IMU /agv/zed/left|right/* ─▶ cuVSLAM ─▶ /visual_slam/tracking/odometry ─▶ covariance_override ─▶ ekf_global
sim_api (:8090)    /agv/sim/*         ─▶  test_waypoint_precision + iteration_loop
```

Skip chain: if `cuvslam_in_hil:=false`, `vslam_fallback_relay` consumes
`/agv/wheel_odom` and publishes `/visual_slam/tracking/odometry` with
inflated covariance so ekf_global still gets odom1.

## 6.3 — Drive scale caveat + EKF pre-goal sync

Observed in sim runs post-2026-04-16 drive-fix: the chassis translates at
**~15 % of the commanded linear velocity** (cmd 0.3 m/s → real ~0.045 m/s).
Root cause is probably PhysX ground friction / caster drag; signs and
directions are correct, so Nav2 closed-loop works but takes ~6× longer.

Runtime compensation (ephemeral — no config change):

```bash
# Bump MPPI/controller max velocity so actual chassis speed reaches ~0.25 m/s.
ros2 param set /agv/controller_server FollowPath.vx_max 1.5
ros2 param set /agv/controller_server FollowPath.wz_max 1.5
# Mapping tolerance (cover area, not precision):
ros2 param set /agv/controller_server general_goal_checker.xy_goal_tolerance 0.40
# Precision tolerance (the gate):
ros2 param set /agv/controller_server general_goal_checker.xy_goal_tolerance 0.10
```

Persistent form (when the factor stabilizes between sessions): add
`FollowPath.vx_max: 1.5` block to
`src/agv_navigation/config/nav2_hil_overrides.yaml` with a comment
"compensates for ~15 % effective velocity in Isaac HIL".

### EKF brain ↔ GT pre-goal sync

Symptom without this step: Nav2 goal accepted → BT advances → cmd_vel
emits non-zero → but `gt_delta < 0.05 m` in 30 s. Diagnosis: the brain's
`est_pose` is 4+ m off GT (cold-start `Path C` never relocalized), so
MPPI thinks it already arrived. `test_waypoint_precision.py` now calls
`_sync_brain_to_gt()` — a `/agv/set_pose` service call with the current
GT pose — after every `/reset` and before every NavigateToPose. No user
action needed when running the pytest harness.

Manual sync when poking live from a terminal:

```bash
ros2 service call /agv/set_pose robot_localization/srv/SetPose \
  "{pose: {header: {frame_id: 'map'}, pose: {pose: {position: {x: 5.5, y: 0.0}, \
     orientation: {w: 1.0}}, covariance: [0.01, 0,0,0,0,0, 0,0.01, \
     0,0,0,0, 0,0,1.0, 0,0,0, 0,0,0,1.0, 0,0, 0,0,0,0,1.0, 0, \
     0,0,0,0,0,0.01]}}}"
```

Budget: the sync can be ignored by `ekf_global` if `wheel_odom` is
pushing conflicting updates at high confidence. If `est_pose` remains
> 0.2 m off GT 0.5 s after the service call, fall back to
`ros2 service call /agv/localization/reinitialize std_srvs/srv/Trigger`.

## 6. Running the harness

```bash
# Precision test only (20 waypoints, measures terminal error)
cd ~/ros2_ws && source install/setup.bash
export ROS_DOMAIN_ID=42
export SIM_API_HOST=192.168.55.100            # required — no default in code
colcon test --packages-select agv_integration_tests \
    --ctest-args -R test_waypoint_precision \
    --event-handlers console_direct+

# Full flow (mapping → save → load → precision)
colcon test --packages-select agv_integration_tests \
    --ctest-args -R test_full_flow \
    --event-handlers console_direct+

# Map fidelity CLI (run after a Save Map cycle)
python3 src/agv_integration_tests/scripts/map_diff.py \
    --pgm $AGV_DATA_DIR/maps/greenhouse_v1.pgm \
    --yaml $AGV_DATA_DIR/maps/greenhouse_v1.yaml \
    --obstacles-topic /agv/sim/ground_truth/obstacles
```

Reports land in `${AGV_DATA_DIR}/sim_episodes/<run>/report.json` (or
`summary.json` for full-flow) on the Jetson. Rosbags for the episodes
live on the sim host at `/tmp/agv_runs/`. For a postmortem of a failed
run, grab the bag before the sim host reboots (that path is volatile):

```bash
ssh orza@192.168.55.100 'rsync -av /tmp/agv_runs/ ~/agv_runs_archive/$(date +%F)/'
```

## 7. Apagado limpio

Inverse of encendido:

1. Ctrl+C the Jetson launch.
2. Ctrl+C Sim host T2 (overlay + sim_api).
3. Close the Isaac viewport (X) or Ctrl+C T1. The in-Kit handler dies
   with the Kit process.

If you just want to restart the harness between runs, do NOT reboot the
sim — use `POST /reset` on the sim_api instead. Each full restart costs
the manual "Play" step (see §4 T1), which the LLM agent cannot automate.

## 8. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `ros2 topic list` shows no `/agv/sim/*` on Jetson | `ROS_DOMAIN_ID` not 42 | `export ROS_DOMAIN_ID=42` in the terminal |
| `/clock` silent | Isaac not in Play, or sim host RTF dropped to 0 | press Play in Isaac viewport; check CPU/GPU on sim host |
| `curl http://192.168.55.100:8090/state` times out | USB ethernet down OR sim T2 not started | ping sim host; check `ip addr show` both sides; restart T2 |
| `ros2 interface show` fails for `PoseStamped` | Jetson workspace not built / not sourced | `colcon build` + `source install/setup.bash` |
| Test `test_waypoint_precision` hangs on reset | sim handler didn't boot (no `[AGV][handler] ready` in T1 console) | restart T1 `./run_isaac_sim.sh` + Play |
| Test fails with collision on known-clear waypoint | costmap inflation too tight, or localization jumped | see `docs/validation/iteration_loop.md` |
| p95 err_xy hovers around 0.11 m | `xy_goal_tolerance` still at 0.15 m | confirm `nav2_hil_overrides.yaml` was loaded; `ros2 param get /controller_server general_goal_checker.xy_goal_tolerance` → 0.1 |
| sim host RTF collapses / GT pose NaN / physics frozen | Isaac crashed or PhysX lost a rigid body | `curl -X POST http://192.168.55.100:8090/sim/restart`; poll `GET /state` until `gt_pose` non-null (≤60 s); resume. If 2 restarts in a row fail, escalate to human — USD or supervisor may be broken |

## 9. References

- `specs/interfaces.yaml` — authoritative declaration of `/agv/sim/*`
- `specs/acceptance.yaml#hil_validation` — the gate this runbook validates
- `specs/launch_sequence.yaml#hil_simulation.sim_host_companion` — sim host boot procedure cross-ref
- Sim repo `RUNBOOK.md` — the origin document for the sim-side operations
- Sim repo `TOPIC_CONTRACT.md` — authoritative schema of each `/agv/sim/*` payload
- `docs/validation/iteration_loop.md` — how to diagnose and re-tune when the gate fails
