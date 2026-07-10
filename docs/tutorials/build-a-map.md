# Map a greenhouse

Before NavGreen can navigate autonomously, you commission a **gold-standard
map** of the site: teleoperate the robot through every corridor while the
SLAM pipeline builds an occupancy grid, then save it together with the
relocalization sidecars the cold-start cascade needs. This page is the
condensed version of that workflow; the full procedure with every check and
failure mode lives in the
[mapping commissioning runbook](../mapping_commissioning.md).

!!! warning "Hardware and vendor SDKs required"
    Mapping runs on the **real robot**. It needs the ZED 2i camera (ZED SDK),
    the ODrive/CAN drivetrain, and the external `agv_slam` overlay package
    (NVIDIA Isaac ROS cuVSLAM pipeline), which is **not in this repository**
    and must be cloned and built separately on the Jetson.
    `agv_mapping.launch.py` fails fast at `t=0` with an actionable error if
    `agv_slam` is missing. You cannot build a map in the
    [drivetrain-only Gazebo sim](drive-in-simulation.md) — it has no sensors.

## What a saved map contains

A NavGreen map is not just a `.pgm`. Saving `corridor_v1` produces **four
coordinated artifacts** under `$AGV_DATA_DIR/maps/`:

| Artifact | Purpose |
|----------|---------|
| `corridor_v1.pgm` + `corridor_v1.yaml` | 2D occupancy grid for Nav2's `map_server` / static costmap layer |
| `corridor_v1_cuvslam/` | cuVSLAM keyframe database for visual relocalization |
| `corridor_v1.area` | ZED SDK Area Memory landmark database |
| `corridor_v1_meta.json` | Last-known pose at save time (final fallback) |

The `auto_init_orchestrator` cold-start cascade tries these in order on every
boot; each missing sidecar removes one relocalization path. Writers and
readers of every artifact are specified in
[`specs/persistence.yaml`](https://github.com/AndresIslas99/NavGreen/blob/main/specs/persistence.yaml).

## Step 1 — Launch mapping mode

```bash
ros2 launch agv_bringup agv_mapping.launch.py
```

Mapping mode is its own launch file — **no Nav2, no dual EKF, no safety
chain**; cuVSLAM owns the TF tree and you drive by teleop only. It starts the
robot description, the ODrive CAN node (motor control + wheel odometry),
`pointcloud_to_laserscan` (ZED point cloud → `/agv/scan`), the camera image
server (port 8091), `scan_grid_mapper` (live occupancy grid on
`/agv/live_map`), the external `agv_slam` pipeline (delayed 3 s), and the
[operator backend](operator-dashboard.md) on port 8090 (delayed 5 s).

Open the dashboard at `http://<jetson>:8090/dashboard` and switch to the
**Map** view to watch the grid grow as you drive.

## Step 2 — Preflight checks

Do not start driving until all subsystems are alive:

```bash
# cuVSLAM publishes visual odometry continuously (~30 Hz)
ros2 topic hz /visual_slam/tracking/odometry

# the live grid is being published
ros2 topic echo /agv/live_map --once --no-arr | head -5

# the four-sidecar save service is discoverable
ros2 service list | grep /agv/map_manager
```

If any of these fail, fix the underlying subsystem first — the
[full runbook](../mapping_commissioning.md) has the complete check list and
diagnostics.

## Step 3 — Drive the mapping run

The commissioning protocol (canonical in the
[project spec](https://github.com/AndresIslas99/NavGreen/blob/main/specs/project.yaml)
and the root
[CLAUDE.md](https://github.com/AndresIslas99/NavGreen/blob/main/CLAUDE.md)):

- **Speed 0.3–0.5 m/s.** Faster causes motion blur that degrades cuVSLAM
  feature matching and Area Memory landmark quality.
- **Rotate slowly (≤ 1.0 rad/s).** Fast turns are the top cause of visual
  tracking loss.
- **Cover every operational corridor in both directions.** The camera's
  forward FOV means features seen from only one side lose their depth
  estimate on the return pass.
- **Revisit mapped areas every 20–50 m** to generate loop closures, and **end
  at your starting position** so the run closes a global loop.
- **Map when dynamic activity is minimal** — no people walking, no doors
  opening. Both cuVSLAM and the occupancy grid assume static geometry.

Drive with the dashboard joystick (**Operate** view), or with the keyboard:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/agv/cmd_vel
```

Greenhouse-specific caution: crop rows are visually repetitive and lighting
changes across the day — prefer stable mid-day light, and redo any section
where the dashboard's SLAM-quality indicator degrades.

## Step 4 — Save the map

Save through the `map_manager` service — it orchestrates all four artifacts:

```bash
ros2 service call /agv/map_manager/save_map agv_interfaces/srv/SaveMap \
  "{name: 'corridor_v1'}"
```

Map names must be **1–64 characters of `[A-Za-z0-9_-]`** — anything else
(slashes, dots, spaces, quotes) is rejected with an `Invalid map name` error.
The service returns `success: true` as soon as the 2D grid is written; the
three sidecars land asynchronously a moment later.

!!! warning "The dashboard's Save Map button currently saves less"
    Known drift, documented in
    [`specs/interfaces.yaml`](https://github.com/AndresIslas99/NavGreen/blob/main/specs/interfaces.yaml):
    the dashboard's `POST /api/maps/save` does **not** call this service — it
    runs nav2's `map_saver_cli` directly, producing only the `.pgm` + `.yaml`
    pair with **no relocalization sidecars**. The same is true of calling
    `map_saver_cli` by hand. Until the two save paths are reconciled, use the
    service call above for commissioning saves.

## Step 5 — Verify all four artifacts

```bash
ls -la $AGV_DATA_DIR/maps/ | grep corridor_v1
```

You should see the `.pgm`, `.yaml`, a non-trivial `.area` file, a non-empty
`_cuvslam/` directory, and `_meta.json`. (`AGV_DATA_DIR` is the single
canonical data root — default `/home/orza/agv_data` on the robot, per
[`specs/persistence.yaml`](https://github.com/AndresIslas99/NavGreen/blob/main/specs/persistence.yaml).)
If a sidecar is missing or empty, check the `map_manager` logs; the
[runbook's troubleshooting table](../mapping_commissioning.md#troubleshooting)
maps each symptom to its cause.

## Step 6 — The real acceptance gate: reload it

A map is only commissioned once a cold start from it localizes:

```bash
ros2 launch agv_bringup agv_full.launch.py \
  map:=$AGV_DATA_DIR/maps/corridor_v1.yaml

# in another terminal, within 30 s:
ros2 topic echo /agv/localization/state --once
# Expected: LOCALIZED or DEGRADED — not FAILED
```

The full runbook adds a second reload test with AprilTags disabled to exercise
every fallback path — run both before calling the map done.

## Where to go next

- [Mapping commissioning runbook](../mapping_commissioning.md) — the complete
  procedure: pipeline diagram, parameters, all checks and failure modes.
- [Operator runbook](../operator_runbook.md) — day-to-day operation once a
  map exists.
- [`src/agv_map_manager/CLAUDE.md`](https://github.com/AndresIslas99/NavGreen/blob/main/src/agv_map_manager/CLAUDE.md)
  — the save/load chain in detail.
