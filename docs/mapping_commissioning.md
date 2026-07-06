# Mapping Commissioning Procedure

## Overview

This document describes how to produce a **gold-standard map** of the greenhouse
(or office commissioning space) for Nav2 autonomous navigation. The procedure
saves a coordinated set of **four sidecar artifacts** — the 2D occupancy grid,
the cuVSLAM keyframe database, the ZED Area Memory landmark file, and a
last-known-pose metadata file. All four are required for the `auto_init_orchestrator`
cold-start cascade (Path A0, Path A, Path B) to work reliably on subsequent boots.

> **⚠ Field validation status:** This procedure is documented from the
> `agv_map_manager::map_manager_node::on_save_map()` implementation in
> `src/agv_map_manager/src/map_manager_node.cpp:180-218` and must be executed
> against the real robot to produce a validated `corridor_v1` map. Until that
> test is run, treat the checklist at the end as a dry-run guide.

## Why four sidecars

| Sidecar | Purpose | Consumed by |
|---------|---------|-------------|
| `{name}.pgm` + `{name}.yaml` | 2D occupancy grid | Nav2 `map_server` → `global_costmap.static_layer` |
| `{name}_cuvslam/` (directory of keyframes) | Visual feature database for relocalization | Isaac ROS Visual SLAM via `/visual_slam/load_map` → `/visual_slam/localize_in_map` (Path A) |
| `{name}.area` | ZED SDK Area Memory landmark database | ZED wrapper at startup (Path A0) via `pos_tracking.area_memory_db_path` and the landing-pad swap done by `map_manager::swap_area_memory_for_map()` |
| `{name}_meta.json` | Last-known pose at map-save time | `auto_init_orchestrator` Path B fallback when no AprilTag is seen and Area Memory has not converged |

If any sidecar is missing, the orchestrator falls through to the next path.
If all four fail, the LOC pill goes red (`FAILED`) and the operator must
re-initialize manually by teleoperating near a tag and calling
`/agv/localization/reinitialize`.

## Pipeline

```
ZED 2i ──┬─ point_cloud ── pointcloud_to_laserscan ── /agv/scan ──┐
         │                                                        │
         ├─ RGB + depth ─── cuVSLAM ── /visual_slam/tracking/odometry
         │                                                        │
         └─ IMU ───────────┐                                      │
                           │                                      │
                       imu_filter                                 │
                           │                                      │
                    ekf_local ─ odom → base_link                  │
                                                                  │
                                  scan_grid_mapper ── /agv/live_map
                                                                  │
                                                                  ▼
                                  ┌─────────────────────────────────┐
                                  │   Operator clicks "Save Map"    │
                                  │       via dashboard             │
                                  └───────────────┬─────────────────┘
                                                  │
                                                  ▼
                         ┌──────── agv/map_manager/save_map ───────┐
                         │                                          │
                         │  1. map_saver_cli → .pgm + .yaml        │
                         │  2. /visual_slam/save_map → _cuvslam/   │
                         │  3. /agv/zed/save_area_memory → .area   │
                         │     (with landing-pad copy)             │
                         │  4. /agv/localization/save_last_known_pose
                         │     → _meta.json                        │
                         │                                          │
                         └──────────────────────────────────────────┘
```

## Prerequisites

- Jetson booted, CAN bus up, ODrive responding
- `ros2 node list` shows at least: `zed_node`, `visual_slam`, `ekf_local`,
  `ekf_global`, `scan_grid_mapper`, `map_manager`, `auto_init_orchestrator`
- ZED 2i pointing forward, nothing within the 30 cm blind zone
- Floor clean and well lit — cuVSLAM relies on visual features and the ZED
  Area Memory needs stable landmarks

## Step 1 — Launch mapping mode

```bash
# Real hardware (no Nav2, no safety chain gated by has_map — teleop only)
ros2 launch agv_bringup agv_mapping.launch.py

# HIL (simulation) — provides sensor inputs through Gazebo
ros2 launch agv_bringup agv_hil_full.launch.py
```

`agv_mapping.launch.py` starts cuVSLAM (with TF publishing via the normal
override rules), SLAM Toolbox in mapping mode, `scan_grid_mapper`, the ZED
wrapper with Area Memory enabled, `map_manager`, and the dashboard. It does
**not** launch Nav2 or the safety chain — operator uses teleop only.

## Step 2 — Verify all four subsystems are alive

```bash
# cuVSLAM: must publish odometry continuously
ros2 topic hz /visual_slam/tracking/odometry
# Expected: ~30 Hz with low jitter

# ZED Area Memory: check pos_tracking is enabled with area_memory=true
ros2 param get /agv/zed pos_tracking.area_memory
ros2 param get /agv/zed pos_tracking.area_memory_db_path
# Expected: True, and the path should point to the landing pad (not empty)

# scan_grid_mapper: must publish /agv/live_map
ros2 topic echo /agv/live_map --once --no-arr | head -5

# map_manager: save service must be discoverable
ros2 service list | grep /agv/map_manager
# Expected: /agv/map_manager/save_map, /agv/map_manager/load_map,
#           /agv/map_manager/update_zone
```

If any of these fail, do not proceed — fix the underlying subsystem first.

## Step 3 — Drive the full area

Coverage protocol (derived from cuVSLAM feature-tracking and Area Memory
landmark density recommendations):

- **Speed: 0.3–0.5 m/s.** Faster produces motion blur that degrades feature
  matching in cuVSLAM and drops Area Memory landmark quality.
- **Angular velocity: ≤ 1.0 rad/s.** Fast rotations are the single biggest
  cause of visual SLAM tracking loss.
- **Cover every corridor in BOTH directions.** The ZED has 110° forward FOV;
  features observed only once from one side lose their depth estimate on
  the return pass.
- **Return to previously mapped areas every 20–50 m of travel** to produce
  loop closures. Loop closures are what correct accumulated drift in the
  pose graph.
- **End at the starting position** so the final map has a closed global loop.
- Avoid dynamic activity (people walking, doors opening) during the run.
  cuVSLAM and the occupancy grid both assume static geometry.

Drive via dashboard joystick (`http://agv.local:8090/`, **Map** tab) or:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/agv/cmd_vel
```

Watch the live scan accumulation in the dashboard as you drive. Dark areas
are marked occupied, light areas are free, gray is unexplored.

## Step 4 — Save the map via the service (NOT map_saver_cli)

```bash
ros2 service call /agv/map_manager/save_map agv_interfaces/srv/SaveMap \
  "{name: 'corridor_v1'}"
```

Or click **Save Map** in the dashboard and enter the name.

**Do not call `map_saver_cli` directly.** It will produce only the `.pgm`
and `.yaml` and skip the cuVSLAM, Area Memory, and meta-pose sidecars.
The orchestrator cold-start will fail on the resulting incomplete map.

The service orchestrates the full chain. Behavior:

1. `map_saver_cli` writes `corridor_v1.pgm` + `corridor_v1.yaml`
   (synchronous, blocks the response).
2. On success, three non-blocking async calls fire in parallel:
   - `/visual_slam/save_map` → `corridor_v1_cuvslam/` directory
   - `/agv/zed/save_area_memory` → flushes landing pad, which triggers
     `copy_landing_pad_to_per_map` → `corridor_v1.area`
   - `/agv/localization/save_last_known_pose` → `corridor_v1_meta.json`
3. The service returns `success: true` **as soon as step 1 completes**.
   The three sidecars land a few hundred ms later. Verify them in step 5.

Name constraints (from `map_manager_node.cpp:226-230`): no `/`, no `..`,
not empty, not the literal `default_empty`.

## Step 5 — Verify all four sidecars

```bash
ls -la $AGV_DATA_DIR/maps/ | grep corridor_v1
```

Expected output:

```
-rw-r--r-- corridor_v1.pgm          # binary occupancy grid image
-rw-r--r-- corridor_v1.yaml         # nav2 map metadata (resolution, origin)
-rw-r--r-- corridor_v1.area         # >500 KB typical (depends on area size)
drwxr-xr-x corridor_v1_cuvslam/     # non-empty directory of keyframes
-rw-r--r-- corridor_v1_meta.json    # last-known pose in map frame
```

Quick sanity checks:

```bash
# .pgm should render cleanly (no double walls, no drift trails)
file $AGV_DATA_DIR/maps/corridor_v1.pgm
xdg-open $AGV_DATA_DIR/maps/corridor_v1.pgm  # optional, requires DE

# .yaml has resolution: 0.05 and a sane origin
cat $AGV_DATA_DIR/maps/corridor_v1.yaml

# cuVSLAM directory is not empty
ls -la $AGV_DATA_DIR/maps/corridor_v1_cuvslam/
# Expected: keyframe files, graph data — at minimum 100 KB total

# .area file is non-trivial size — empty means SDK rejected the save
du -b $AGV_DATA_DIR/maps/corridor_v1.area
# Expected: > 500 KB for a reasonable office

# _meta.json has a valid pose
cat $AGV_DATA_DIR/maps/corridor_v1_meta.json
# Expected: JSON with position (x, y, z) and orientation (quaternion)
```

If any sidecar is missing or empty, check the `map_manager` logs for the
relevant WARN/ERROR lines. Common causes:
- `cuVSLAM /visual_slam/save_map not available` → the Visual SLAM node
  is not running or was launched without `enable_localization_n_mapping: true`
- `ZED Area Memory save reported failure` → `pos_tracking.area_memory` is
  off in the wrapper config, or the area memory SDK rejected the save
  (usually because tracking state was never `OK`)
- `orchestrator wrote … refused pose save` → the orchestrator's safety
  guards (never-called or implausible-distance) rejected the save. The
  orchestrator will retry on its periodic 180 s tick.

## Step 6 — Reload test (the real acceptance gate)

Verifies that a cold-start from this map actually works end-to-end, with
and without AprilTags.

```bash
# Test A: with AprilTags enabled (default) — exercises Path A (tag hint)
ros2 launch agv_bringup agv_full.launch.py \
  map:=$AGV_DATA_DIR/maps/corridor_v1.yaml

# In another terminal, within 30 s:
ros2 topic echo /agv/localization/state --once
# Expected: LOCALIZED or DEGRADED (not FAILED)
```

```bash
# Test B: with AprilTags DISABLED — exercises Path A0 then Path B
ros2 launch agv_bringup agv_full.launch.py \
  map:=$AGV_DATA_DIR/maps/corridor_v1.yaml \
  enable_markers:=false

ros2 topic echo /agv/localization/state --once
# Expected: DEGRADED (Path B used last-known pose from _meta.json)
```

Both cases must complete within 30 s. `FAILED` on test B means:
- `_meta.json` is missing, malformed, or the pose is implausible
- OR the orchestrator's `marker_wait_timeout_s` is set too high and the
  launch timer runs out before Path B kicks in (check `auto_init_params.yaml`)

## Parameters reference

The scan_grid_mapper (used during the map-building phase, not navigation):

| Parameter | Default | Description |
|-----------|---------|-------------|
| resolution | 0.05 m | Grid cell size (matches Nav2 costmap) |
| width × height | 400 × 400 | Grid size in cells (20 m × 20 m) |
| origin_x, origin_y | -10.0 | Grid origin in meters |
| max_range | 8.0 m | Ignore scan rays beyond this |
| l_occupied | 0.85 | Log-odds increase for occupied evidence |
| l_free | -0.4 | Log-odds decrease for free evidence |

The map_manager save chain:

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `map_dir` | `$AGV_DATA_DIR/maps` (launch arg) | Directory for all sidecars |
| `cuvslam_enabled` | `true` | Gates the `_cuvslam/` save step |
| `zed_area_save_enabled` | `true` | Gates the `.area` save step |
| `zed_area_landing_path` | `$AGV_DATA_DIR/maps/.current.area` | Transient path the ZED wrapper reads on reload |
| `area_memory_autosave_period_s` | `180` | Background auto-save interval |

## Troubleshooting

| Symptom | Likely cause | Check |
|---------|--------------|-------|
| `/agv/live_map` all gray | Scan has no rays or robot not moving | `ros2 topic hz /agv/scan` |
| `/agv/live_map` all black | Ground plane detected as obstacle | `pointcloud_to_laserscan` min_height/max_height params |
| `corridor_v1.pgm` shows drift trails (ghosted walls) | cuVSLAM lost tracking mid-run | `ros2 topic echo /visual_slam/tracking/vo_status`, redo that section slower |
| `corridor_v1.area` missing or 0 bytes | SDK rejected save — tracking never reached `OK` | `ros2 topic echo /agv/zed/pose/status`, drive more through feature-rich areas |
| `corridor_v1_cuvslam/` empty | `/visual_slam/save_map` service failed | `ros2 service list \| grep visual_slam`, check node logs |
| `corridor_v1_meta.json` missing | Orchestrator not running or its pose guards refused the save | `ros2 node list \| grep auto_init`, check orchestrator logs |
| Reload test A: `FAILED` | No matching AprilTag visible at startup pose | Drive to a known tag, call `/agv/localization/reinitialize` |
| Reload test B: `FAILED` | `_meta.json` pose is stale or implausible | Re-save and confirm the orchestrator logs "wrote corridor_v1_meta.json" |
| Reload test B: takes > 30 s | `marker_wait_timeout_s` too high | Reduce in `auto_init_params.yaml` (currently 10 s) |

## Related reading

- [src/agv_map_manager/CLAUDE.md](../src/agv_map_manager/CLAUDE.md) — package responsibilities and the full save chain
- [src/agv_localization_init/CLAUDE.md](../src/agv_localization_init/CLAUDE.md) — orchestrator cascade (Path A0/A/B/C) details
- [src/agv_bringup/CLAUDE.md](../src/agv_bringup/CLAUDE.md) — launch startup DAG at t=7.0 s
- [specs/launch_sequence.yaml](../specs/launch_sequence.yaml) — authoritative startup order and conditions
- [specs/persistence.yaml](../specs/persistence.yaml) — which artifact each component owns
