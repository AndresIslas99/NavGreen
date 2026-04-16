# agv_navigation

Nav2 autonomous navigation configuration. Provides path planning (SmacPlanner2D),
trajectory following (MPPI controller), and two-stage collision monitoring.
No custom C++ nodes — this package is configuration, launch, and a custom BT XML.

## Nodes (all from Nav2)

- **bt_navigator** — Behavior tree navigation coordinator (uses custom forward-only BT)
- **planner_server** — SmacPlanner2D global path planning (5 Hz, max 2.0s)
- **controller_server** — MPPI local trajectory following (20 Hz, forward-only motion model)
- **behavior_server** — Recovery behaviors: spin, wait (backup loaded but never invoked)
- **velocity_smoother** — cmd_vel smoothing
- **collision_monitor** — Two-stage obstacle protection (stop + slowdown zones)
- **map_server** — Static map serving
- **lifecycle_manager** — Manages lifecycle of all Nav2 nodes (including collision_monitor)

## nvblox is NOT in the costmap pipeline

`nvblox_node` runs for visualization, mesh generation, and ground plane estimation,
but **`nvblox_layer` is intentionally removed from both `local_costmap` and
`global_costmap` plugins**. The reason is structural and incompatible with our
hardware:

- nvblox builds a TSDF from the forward-facing ZED depth camera.
- The camera **never sees the cells directly under or behind the robot**.
- Once nvblox marks a cell as occupied (e.g., from looking at a plant ahead),
  it has no observation that can clear it later — the camera looked elsewhere
  by the time the robot reached that cell.
- Result: the robot drives over cells that nvblox still considers occupied.
  The next planning attempt fails with `Starting point in lethal space! Cannot
  create feasible plan` because nvblox_layer reports the robot's current cell
  as lethal/inflated.
- This breaks **any** scenario that requires backtracking through previously
  visited corridors — exactly the greenhouse aisle workflow.

The 2D obstacle pipeline relies on:
- `static_layer` (loaded map walls)
- `voxel_layer` with `scan` source from `pointcloud_to_laserscan` (live sensor
  obstacles, properly raytrace-cleared as the robot moves)
- `inflation_layer` (single inflation around lethal cells)

This is sufficient for ground-plane navigation in the greenhouse. nvblox is kept
for what it does well (3D mesh, ESDF for visualization, ground plane) but its
TSDF semantics conflict with rolling-window costmap clearing.

## Safety chain — defense in depth (post-collision audit)

The collision_monitor is the **last line of software defense**, but it has known
gaps that cannot be closed without additional hardware. The current architecture
implements 5 layers of independent safety:

| Layer | Mechanism | What it catches | What it misses |
|---|---|---|---|
| L1 | Costmap inflation_radius (1.0m) | Known map obstacles + planned path safety | Dynamic obstacles not on map |
| L2 | collision_monitor with **two sources** (scan + pointcloud) | 2D laser obstacles AND 3D depth obstacles in the polygon | Anything < 30cm from ZED lens |
| L3 | stop_zone polygon = footprint + 20cm front | Reactive stop within physical stopping distance | Objects detected too late |
| L4 | vx_max capped at 0.25 m/s | Bounded kinetic energy and stopping distance | High-speed approach to obstacle |
| L5 | Backend watchdog of `collision_monitor_state` topic | Silent failure of the safety chain (lifecycle crash, deactivation, source stall) | Bugs in the watchdog itself |

**Math behind L3**: At vx_max=0.25 m/s, max_decel=1.0 m/s², stop distance =
v²/(2a) = 3.1 cm. Add ~3cm reaction latency (collision_monitor cycle + comm) and
~14cm safety margin → 20cm forward extent of stop_zone is sized for safe braking.

### THE HARDWARE GAP — 30cm blind zone in front of the camera

**This cannot be fixed in software.** The ZED 2i has `min_depth: 0.30m` per
hardware spec. It returns NaN for any pixel closer than 30cm from the lens.

In the robot frame:
- ZED is mounted at `base_link + (0.70, 0, -0.055)`
- Min depth 0.30m → camera "sees" only from x=1.00m forward
- Robot front edge is at x=0.50m
- **Blind zone: 50cm in front of the physical robot front**

If the operator places ANY object within 50cm of the front of the robot, NO
software safety can detect it. The robot will only stop when:
- The object enters the robot's side of view (impossible, the ZED looks forward)
- A different sensor (none today) detects it
- Physical collision

**Required hardware additions** (priority HIGH for production deployment):

1. **2D LIDAR (recommended)**: RPLIDAR A1/A2 mounted at 10-20cm above the floor.
   Minimum range 15cm, 360° coverage. Resolves the front blind zone AND adds
   lateral/rear safety. Cost ~$100-300 USD.
2. **Bumper switches**: mechanical contacts along the front bumper, wired
   directly to the e-stop circuit on the ODrive (bypasses software entirely).
   Defense of last resort.
3. **Time-of-flight array**: 3-5 VL53L1X sensors on a microcontroller (ESP32),
   publishing `/agv/proximity` and triggering `/agv/e_stop` when reading < 20cm.

Until hardware is added, **document this limitation in operator training**: the
robot can collide if an obstacle is placed within 50cm of the front while the
robot is moving toward it.

### The 8 audit gaps and their disposition

| # | Gap | Class | Status |
|---|---|---|---|
| 1 | ZED 30cm hardware blind zone | hardware | **OPEN** — needs additional sensor |
| 2 | `range_min: 0.3` redundant with hardware | param redundancy | accepted (no software fix possible) |
| 3 | collision_monitor only had `scan` source | architecture | **FIXED**: added `pointcloud_source` |
| 4 | stop_zone front margin too small (5cm) | physics vs geometry | **FIXED**: extended to 20cm |
| 5 | source_timeout: 2.0s | timing | **FIXED**: reduced to 0.5s |
| 6 | No watchdog of collision_monitor liveness | lifecycle | **FIXED**: backend watchdog + nav goal gate + dashboard pill |
| 7 | min_height: 0.03m drops floor obstacles | param | **FIXED**: lowered to 0.01m |
| 8 | max_points: 3 misses small objects | threshold | **FIXED**: set to 1 (any single point triggers stop) |

## Forward-only motion (hardware constraint)

The robot has a single front-facing ZED 2i camera and the collision_monitor stop/slowdown
polygons extend almost entirely forward. There is **no rear obstacle perception**.
Reverse motion is structurally unsafe and is forbidden in three places:

1. **MPPI** sample space: `vx_min: 0.0` (no negative samples)
2. **MPPI** critic weights: `PreferForwardCritic.cost_weight: 18.0` (dominates `PathAlignCritic`)
3. **Custom BT**: `behavior_trees/navigate_to_pose_forward_only.xml` removes the `BackUp`
   recovery action from the RoundRobin. The bt_navigator loads this via the
   `default_nav_to_pose_bt_xml` parameter, set in `launch/navigation.launch.py`.

The `backup` plugin is **kept loaded** in `behavior_server.behavior_plugins` so the
default `navigate_through_poses` BT (which references it) can still resolve action
clients during bring-up. The plugin is harmless because no operational BT in our stack
calls it.

## Auto-localization via cuVSLAM + AprilTag (primary) with manual fallback

After evaluating AMCL, SLAM Toolbox localization mode, cuVSLAM relocalization,
ICP, and hybrid combinations for our specific use case (Jetson Orin + greenhouse
with repetitive crop rows + dual EKF + AprilTag), we chose the **cuVSLAM +
AprilTag hybrid**. Rationale:

- **AMCL is structurally incompatible**. It publishes `map→odom` which
  conflicts with our `ekf_global`, and its 2D scan matching collapses in
  greenhouse aisles where rows look identical.
- **SLAM Toolbox localization** is compatible but CPU-bound (Ceres), uses
  local optimization (risky in ambiguous geometry), and requires a parallel
  save workflow (pose graph + occupancy grid).
- **cuVSLAM is GPU-accelerated on Jetson** and uses 256-D visual descriptors
  that distinguish visually-identical-looking crop rows via foliage texture,
  lighting micro-variations, and RANSAC-refined feature matching.
- **AprilTag provides absolute ground truth** for the initial pose hint,
  making the visual relocalization robust even in featureless zones.

### The orchestrator: `agv_localization_init`

A new package, see its own CLAUDE.md. A C++17 node named
`auto_init_orchestrator` coordinates the sequence:

1. Listens for `/agv/maps/loaded` events from `map_manager_node`
2. Loads the matching cuVSLAM keyframe database via `/visual_slam/load_map`
3. Waits up to 10s for an AprilTag detection on `/agv/marker_pose`
4. If tag detected → uses it as pose hint; if not → falls back to
   last-known pose from `{map_name}_meta.json` on disk
5. Calls `/visual_slam/localize_in_map(folder, pose_hint)` with retries
6. Publishes `/agv/localization/state` as JSON with action:
   `INITIALIZING | LOCALIZED | DEGRADED | FAILED`

### The dashboard LOC pill

`agv_ui_backend` subscribes to `/agv/localization/state` purely for display.
The TopBar shows a LOC pill mirroring the orchestrator's current state:
- `LOCALIZED` → green pill, AprilTag-anchored
- `DEGRADED` → amber pill, visual only / last-known pose
- `INITIALIZING` → blue spinner pill, cascade running
- `FAILED` → red pill, all automatic paths exhausted

**Nav goals are NOT gated on localization state.** The orchestrator owns
localization end-to-end; there is no manual "set initial pose" modal.
Recovery from FAILED: operator drives via teleop toward a visible
AprilTag and calls `/agv/localization/reinitialize` (std_srvs/Trigger).

### Save workflow (per mapping session)

When the operator clicks "Save Map" in the dashboard:
1. `map_manager_node::on_save_map` runs `map_saver_cli` → `mapname.pgm` + `mapname.yaml`
2. On success, it also calls `/visual_slam/save_map` → `mapname_cuvslam/`
   (keyframe database for future relocalization)
3. Both artifacts end up in `~/agv_data/maps/`

A map without the `*_cuvslam/` folder falls back to AprilTag absolute
pose (Path B) or last-known-pose JSON (Path C). If none of those are
available the orchestrator reports `FAILED` and the LOC pill goes red;
recovery requires driving to an AprilTag and calling
`/agv/localization/reinitialize`.

### Load workflow (per nav session)

1. Operator selects a map, `POST /api/maps/load` → `map_manager_node::on_load_map`
2. `map_manager_node` calls `nav2_msgs/srv/LoadMap` as before for the occupancy grid
3. On success, `map_manager_node` publishes `/agv/maps/loaded` (std_msgs/String)
4. `auto_init_orchestrator` receives the event and runs its sequence (above)
5. State transitions propagate through `/agv/localization/state` to the
   dashboard pill and backend action guards

### Recovery path

There is no manual `/api/maps/set_initial_pose` REST endpoint. The
orchestrator calls `robot_localization/srv/SetPose` itself from its
internal cascade (Path B AprilTag or Path C last-known). If the whole
cascade fails (FAILED), recovery is: drive to an AprilTag via teleop
and call `/agv/localization/reinitialize` (std_srvs/Trigger).

## SLAM Toolbox mode (deferred decision)

`config/slam_toolbox_localization.yaml` is in `mode: mapping` (not `localization`)
during nav. SLAM Toolbox builds its own map continuously and publishes it to `/map`
(absolute topic, separate from `/agv/map` published by `map_server`). Loop closures
re-optimize the pose graph but **do not feed back into ekf_global**.

This is a known suboptimality. The cleaner alternative would be:
- SLAM Toolbox in `mode: localization` with a pre-built serialized pose graph
  (`*_posegraph.data`) loaded from disk
- Optionally feed `/agv/slam_toolbox/pose` to `ekf_global` as a `pose1` source

Pending validation in a separate session — requires a workflow change for map saving
(use `slam_toolbox/serialize_map` service instead of `map_saver_cli`).

## Topics

**Subscribed:**
- `/agv/scan` (LaserScan) — Primary obstacle source (from pointcloud_to_laserscan)
- `/agv/zed/point_cloud/cloud_registered` (PointCloud2) — Volumetric obstacles via nvblox
- `odometry/global` (Odometry) — Robot pose from dual EKF

**Published:**
- `cmd_vel_smoothed` (Twist) — Smoothed commands
- `cmd_vel_safe` (Twist) — Safe commands after collision check (final output to odrive)
- `collision_monitor_state` (String) — Current collision zones status
- Costmaps: local (3m x 3m rolling) and global (full map)

## Action

- `/navigate_to_pose` (NavigateToPose) — Canonical goal dispatch interface

## Key Configuration (Tunable Parameters)

| Parameter | Value | File |
|-----------|-------|------|
| `vx_max` (MPPI) | 0.4 m/s | nav2_params.yaml |
| `vx_min` (MPPI) | **0.0** (forward-only) | nav2_params.yaml |
| `wz_max` (MPPI) | 1.5 rad/s | nav2_params.yaml |
| `time_steps` × `model_dt` (MPPI horizon) | 32 × 0.05 = 1.6 s | nav2_params.yaml |
| `batch_size` (MPPI samples) | 1000 | nav2_params.yaml |
| `goal_tolerance` | xy=0.15m, yaw=0.25 rad | nav2_params.yaml |
| `max_planning_time` | 2.0s | nav2_params.yaml |
| `controller_frequency` | 20 Hz | nav2_params.yaml |
| **Stop zone** | footprint + 5cm | collision_monitor.yaml |
| **Slowdown zone** | footprint + 25cm, 30% speed | collision_monitor.yaml |

**Robot footprint** (meters): `[[0.50, 0.37], [0.50, -0.37], [-0.30, -0.37], [-0.30, 0.37]]`

## Configuration Files

- `config/nav2_params.yaml` — Core Nav2 config (planner, MPPI controller, costmaps, behaviors, custom BT path)
- `config/collision_monitor.yaml` — Stop/slowdown zone polygons (flat double_array format)
- `config/velocity_smoother.yaml` — cmd_vel smoothing parameters
- `config/slam_toolbox.yaml` — SLAM in mapping mode
- `config/slam_toolbox_localization.yaml` — SLAM in mapping mode (despite name; see "SLAM Toolbox mode" above)
- `config/nav2_hil_overrides.yaml` — HIL-specific parameter overrides
- `behavior_trees/navigate_to_pose_forward_only.xml` — Custom BT without BackUp recovery
- `launch/navigation.launch.py` — Full Nav2 bringup with lifecycle management

## Collision Monitor Pipeline

```
cmd_vel -> velocity_smoother -> cmd_vel_smoothed -> collision_monitor -> cmd_vel_safe
```

- Stop zone: immediate halt if obstacle inside footprint + 5cm
- Slowdown zone: 30% speed reduction if obstacle within footprint + 25cm
- Source: `/agv/scan` only (not depth, to avoid false positives)

### HIL mode override

When `navigation.launch.py` is invoked with `hil_mode:=true` (via
`agv_full.launch.py hil_mode:=true`), an extra params file
`config/collision_monitor_hil_overrides.yaml` is layered on top of the base
config. Its only effect is to override `observation_sources` to
`["scan_source"]` — that is, it drops `pointcloud_source`.

Rationale: in HIL the raw ZED point cloud is published over the WiFi network
by the sim PC at ~180 Mbps. The Jetson cannot sustain two consumers of that
stream (collision_monitor's pointcloud_source AND pointcloud_to_laserscan)
without starving the small BEST_EFFORT `/agv/scan` topic and tripping
`safety_supervisor` on "silent: /agv/scan". Dropping the 3D source in HIL
keeps the bandwidth within budget. Production behavior is untouched — the
override file is only loaded when the `hil_mode` flag is true.

## Dependencies

- nav2_bringup, nav2_bt_navigator, nav2_controller, nav2_planner, nav2_behaviors,
  nav2_collision_monitor, nav2_velocity_smoother, nav2_lifecycle_manager

## Improvement Opportunities

- Document which parameters are tunable for field commissioning vs fixed architectural choices
- Add field-calibration checklist for costmap and controller tuning
- Add Nav2 parameter validation (current config relies on Nav2 defaults for unset values)
- Consider adding costmap filter for keepout/speed zones (currently zones are UI-only)
