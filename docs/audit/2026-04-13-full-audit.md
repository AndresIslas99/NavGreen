# AGV Workspace — Full Audit (2026-04-13)

This document is the forensic audit of the AGV greenhouse robot workspace at
`/home/orza/ros2_ws`. It consolidates findings from three independent Explore
agents plus direct verification. The intent is to capture the **implicit
behavior** of the system — the things that are true of the running stack but
not written down in any single file — so that the next phases (specs, enforcement,
restructuring) can make them explicit and machine-checkable.

**Audience**: humans and AI agents who need to understand "what actually happens
when you reboot this robot" without reading every file in the workspace.

**Status**: Draft 1 — will be iterated as Fase 2 specs reveal contradictions.

**How this document was produced**:
- 3 Explore agents ran in parallel, each with a narrow focus (modes+launch,
  documentation+conventions, persistence+contracts).
- Each agent produced a report with file:line citations.
- This document consolidates and organizes those reports into 9 sections,
  adds cross-references, and flags drift + bugs that emerged during the
  cross-check.

---

## Section 1 — Matrix of operation modes

The system has **three independent layers of "mode"**, none of which are
documented together. A complete description of a running state requires
specifying all three simultaneously.

### Layer 1 — systemd environment (`AGV_MODE`)
Set in [/etc/systemd/system/agv.service](/etc/systemd/system/agv.service) via
`Environment=AGV_MODE=...`. Read by
[src/agv_bringup/scripts/agv_start.sh:47](src/agv_bringup/scripts/agv_start.sh#L47).

| Value | Launch file | Purpose |
|---|---|---|
| `real` (default) | `agv_full.launch.py` | Production. Map resolution via `AGV_MAP` → `~/.agv/last_map` → `default_empty.yaml`. |
| `mapping` | `agv_mapping.launch.py` | SLAM + teleop only. Different TF ownership (cuVSLAM owns odom→base). No Nav2, no EKF dual. |
| `hil` | `agv_hil_full.launch.py` | HIL simulation. Requires explicit map. |

Switching layer 1 requires a `systemctl restart agv.service` — the launch files
are different processes.

### Layer 2 — launch file condition (`has_map`)
Defined in [src/agv_bringup/launch/agv_full.launch.py:63](src/agv_bringup/launch/agv_full.launch.py#L63)
as `has_map = PythonExpression(["'", map_yaml, "' != ''"])`. Gates whether
Nav2 and the safety chain come up.

| has_map | ODrive topic | Nav2 | Safety gate | Implications |
|---|---|---|---|---|
| `true` | `/agv/cmd_vel_safe` (via gate, remap at line 93) | running | running (but buggy — see Section 6) | Full production chain |
| `false` | `/agv/cmd_vel` (direct, remap at line 108) | NOT running | NOT running | Mapping-first: teleop goes direct to ODrive, no collision check |

**Critical non-obvious behavior**: in `real` mode without a saved map, `agv_start.sh`
resolves `MAP=default_empty.yaml` (20m×20m all-free grid) at line 89. That makes
`has_map=true` **always** in `real` mode, so Nav2 and the safety chain always
come up. The "mapping-first" branch of layer 2 is only exercised when
someone launches manually without the default or boots in `mapping` mode.

### Layer 3 — backend runtime mode (`currentMode`)
Declared in [src/agv_ui_backend/src/app_deps.ts](src/agv_ui_backend/src/app_deps.ts)
as `'teleop' | 'mapping' | 'nav'`. Changed at runtime by dashboard
button or WebSocket message; no restart required.

| currentMode | cmd_vel limits | Nav goals | Dashboard buttons |
|---|---|---|---|
| `teleop` | 0.5 m/s lin, 0.5 rad/s ang | rejected | teleop + save + load |
| `mapping` | 0.4 m/s lin, 0.2 rad/s ang (stricter for cuVSLAM) | rejected | mapping + save |
| `nav` | — (cmd_vel from backend blocked) | allowed if motors armed + collision_monitor_state fresh | nav + mission |

Transitions at [src/agv_ui_backend/src/index.ts:810-821](src/agv_ui_backend/src/index.ts#L810-L821).
The backend **does not verify** that layer 1/2 are compatible with the
requested mode — a user can request `nav` mode while the system is in
`AGV_MODE=mapping` and get silently broken behavior.

### Layer 4 — state machine (`RobotState`) — **derived, not authoritative**
Enum with 9 values at [src/agv_ui_backend/src/state_machine.ts:5-14](src/agv_ui_backend/src/state_machine.ts#L5-L14):
`offline`, `idle`, `ready`, `mapping`, `navigating`, `executing_mission`,
`blocked`, `e_stop`, `fault`. Derived from sensor data, not set by any
caller. Used only to gate dashboard button visibility — it has no authority
over the other three layers.

### The full matrix (condensed)

Not every combination is valid. The valid production combinations are:

| Layer 1 | Layer 2 | Layer 3 | Layer 4 | What actually runs |
|---|---|---|---|---|
| `real` | `has_map=true` (via default_empty or saved) | `teleop` | `idle` → `ready` | Full stack, teleop through safety chain |
| `real` | `has_map=true` | `mapping` | `mapping` | Full stack, slower teleop, cuVSLAM tracking |
| `real` | `has_map=true` | `nav` | `navigating` / `executing_mission` | Full stack, goals dispatched to Nav2 |
| `real` | `has_map=false` | `teleop` | `ready` | Mapping-first: direct teleop, no Nav2, no safety |
| `mapping` | N/A | any | `mapping` | cuVSLAM-TF-on mode, SLAM building map |
| `hil` | `has_map=true` | any | simulated | Sim overrides |

**This table does not exist anywhere in the repository today.** Section 2 of the
spec work (Fase 2) will materialize it in `specs/state_machine.yaml`.

---

## Section 2 — Launch sequence DAG

The following DAG describes the chronological startup of `agv_full.launch.py`
(the `real` mode entry point). Each node is cited with file:line. Times are
relative to `launch_ros` start (t=0).

```
t=0 ── robot_state_publisher ──┐
                                │ [agv_full.launch.py:73-82]
t=0 ── odrive_can_node ─────────┤ cmd_vel_topic remap conditional on has_map
                                │ [agv_full.launch.py:84-115]
t=0 ── pointcloud_to_laserscan ─┤ subscribes /agv/zed/point_cloud/cloud_registered
                                │ [agv_full.launch.py:116-138]
t=0 ── image_server ────────────┤ MJPEG on :8091
                                │ [agv_full.launch.py:140-154]
t=0 ── scan_grid_mapper ────────┘ publishes /agv/live_map
                                  [agv_full.launch.py:156-169]

t=3.0 ── cuVSLAM + nvblox (via agv_slam.launch.py include)
         [agv_full.launch.py:172-185]
         slam_params_override: cuvslam_greenhouse.yaml (TF DISABLED via /**: key)
         Downstream: /visual_slam/tracking/odometry for ekf_global

t=3.5 ── imu_filter
         [agv_full.launch.py:188-209]
         Butterworth 2nd order on /agv/zed/imu/data → imu/filtered
         Must start BEFORE EKF or the filter has no history buffer

t=4.0 ── ekf_local + ekf_global + fusion_monitor
         [agv_full.launch.py:211-223]
         (via agv_sensor_fusion/launch/fusion.launch.py)
         ekf_local: odom→base_link @ 50Hz
         ekf_global: map→odom @ 10Hz

t=4.5 ── factor_graph
         [agv_full.launch.py:225-243]
         Parallel mode, publish_tf=false (validation only)

t=5.0 ── slam_toolbox_localization   [agv_full.launch.py:245-267]
         Async mode, transform_publish_period=0.0 (TF disabled)

t=5.0 ── map_manager + waypoint_manager
         [agv_full.launch.py:269-296]
         Subscribes /agv/maps/loaded (transient_local)
         Publishes /agv/current_map (transient_local)

t=6.0 ── Nav2 stack   [agv_full.launch.py:298-315]
         (via agv_navigation/launch/navigation.launch.py)
         condition: IfCondition(has_map)
         Includes: map_server, controller_server (MPPI), planner_server
                  (SmacPlanner2D), behavior_server, bt_navigator,
                  velocity_smoother, collision_monitor, lifecycle_manager

t=6.5 ── agv_safety stack   [agv_full.launch.py:317-335]
         (via agv_safety/launch/safety.launch.py)
         condition: IfCondition(has_map)
         Includes: safety_supervisor_node, cmd_vel_gate_node
         *** BUG: header of safety.launch.py still says "NOT yet wired".
             cmd_vel_gate has 0.5s startup watchdog that blocks teleop. ***

t=7.0 ── AprilTag + marker_correction + rail_approach + auto_init_orchestrator
         [agv_full.launch.py:337-409]
         condition: IfCondition(enable_markers), default=true

t=7.0 ── behavior_executor   [agv_full.launch.py:411-425]
         condition: IfCondition(enable_behaviors), default=false
         (never exercised in practice)

t=8.0 ── teleop_server (agv_ui_backend)
         [agv_full.launch.py:427-455]
         additional_env: AGV_BOOT_MAP_NAME, AGV_DATA_DIR, AGV_PORT, AGV_NAMESPACE
         Publishes /agv/maps/loaded ~10s after its own start
```

**Dependencies not enforced by timing alone** (drift-prone):
- Nav2 `lifecycle_manager_navigation` transitions nodes through
  `configure → activate`. If `collision_monitor` or `velocity_smoother` fails
  to reach `active`, no error bubbles up to systemd. Partial Nav2 bringup is
  invisible.
- `teleop_server` publishes `/agv/maps/loaded` with `AGV_BOOT_MAP_NAME` ~10s
  after its own start. If the map_server hasn't actually loaded the map by
  then (Nav2 still in `configure`), the orchestrator starts relocalizing
  against an empty grid.
- `safety_supervisor` must be publishing `SafetyStatus` before `cmd_vel_gate`
  watchdog fires at 0.5s. There is no start-order guarantee inside
  `safety.launch.py` — they launch in parallel.

**Unreachable launch files** (dead code):
- [src/agv_bringup/launch/agv_modular.launch.py](src/agv_bringup/launch/agv_modular.launch.py) — documented but never invoked by `agv_start.sh`
- [src/agv_bringup/launch/nav.launch.py](src/agv_bringup/launch/nav.launch.py) — sub-component of `agv_modular`, also unused
- [src/agv_bringup/launch/hardware.launch.py](src/agv_bringup/launch/hardware.launch.py) — ditto
- [src/agv_bringup/launch/dashboard.launch.py](src/agv_bringup/launch/dashboard.launch.py) — ditto
- [src/agv_bringup/launch/perception.launch.py](src/agv_bringup/launch/perception.launch.py) — ditto
- `agv_teleop.launch.py`, `agv_robot_core.launch.py`, `agv_fusion.launch.py`,
  `agv_ekf_local_test.launch.py`, `agv_hil.launch.py` — listed in CLAUDE.md
  table but not called from the boot path.

---

## Section 3 — Persistence inventory

Every file/folder written by the runtime stack. Derived from reports of
agent 3 + grep of `fs::rename`, `std::ofstream`, `map_saver_cli`, etc.

| Path | Owner (file:line) | Readers | Format | Lifecycle |
|---|---|---|---|---|
| `~/agv_data/maps/<X>.yaml` | `map_manager_node.cpp:143` (via `map_saver_cli`) | `nav2_map_server`, `agv_ui_backend/routes/maps.ts` | Nav2 map YAML | Persistent |
| `~/agv_data/maps/<X>.pgm` | `map_manager_node.cpp:143` (via `map_saver_cli`) | `agv_ui_backend` (image conversion) | PGM 8-bit grayscale | Persistent |
| `~/agv_data/maps/<X>.area` | `map_manager_node.cpp:515-517` (copy from landing pad) | `map_manager_node::swap_area_memory_for_map` | ZED SDK binary | Persistent per-map |
| `~/agv_data/maps/.current.area` | ZED wrapper on `save_area_memory` service; `map_manager_node:267` on map load | ZED wrapper `startPosTracking()` | ZED SDK binary | Session landing pad |
| `~/agv_data/maps/<X>_cuvslam/` | `map_manager_node::save_cuvslam_map` → `/visual_slam/save_map` | `auto_init_orchestrator` → `/visual_slam/load_map` | cuVSLAM keyframe DB | Persistent per-map |
| `~/agv_data/maps/<X>_meta.json` | `auto_init_orchestrator_node` on shutdown (BUG: not on Save Map) | `auto_init_orchestrator_node` as last-known-pose fallback | JSON `{x, y, theta, saved_at}` | Persistent per-map |
| `~/agv_data/maps/zones.json` | `map_manager_node::on_update_zone` (line 214+) | (Nav2 costmap filter integration — future) | JSONL (one zone per line) | Persistent |
| `~/.agv/last_map` | `map_manager_node::persist_last_map` (line 585, atomic tmp+rename) | `agv_start.sh:79` on boot | Plain text, single line | Persistent |
| `~/agv_data/events.jsonl` | `agv_ui_backend` EventLog | `agv_ui_backend` REST `/api/events`, dashboard | JSONL | Rolling, 30 days |
| `~/agv_data/telemetry.db` | `agv_ui_backend` TelemetryStore | `agv_ui_backend` REST `/api/analytics` | SQLite3 | Rolling, 30 days |
| `~/agv_data/users.json` | `agv_ui_backend` AuthManager | AuthManager (JWT validation) | JSON | Persistent |
| `~/agv_data/apriltags.json` | `agv_ui_backend` AprilTagManager | marker_correction_node (via re-trigger), dashboard | JSON | Persistent |
| `/mnt/ssd/sessions/` | `agv_slam/scripts/session_recorder.py` | operator (offline) | rosbag2 + SVO2 + TUM | Session |
| `/mnt/ssd/slam_logs/` | `agv_slam/src/pipeline_watchdog_node.cpp` | operator (offline) | text logs | Rolling |
| `~/.ros/log/` | ROS2 default | `journalctl` / operator | launch logs | Rolling |
| `/tmp/launch_params_*` | `ros2 launch` framework | Child processes | YAML | Ephemeral |

**Canonical base dir**: `/home/orza/agv_data/` for application artifacts,
`/mnt/ssd/*` for session recordings. **BUT**: `/mnt/ssd` is not a real mount
point — it is a bare directory on the root filesystem (`/dev/nvme0n1p1`). The
name is misleading; a future real SSD mount would shadow existing files.

**Symmetry anomaly**: `*_meta.json` is only written on orchestrator shutdown,
not on explicit Save Map. This is a latent bug (documented in Section 6).

---

## Section 4 — Critical topic contracts

Topics that cross package boundaries. Type and QoS must match between
publisher and subscribers or communication silently fails.

### Velocity command chain
| Topic | Publisher | Subscribers | Type | QoS |
|---|---|---|---|---|
| `/agv/cmd_vel` | `teleop_server` ([index.ts:167](src/agv_ui_backend/src/index.ts#L167)), `/teleop` (rogue — dev PC) | `velocity_smoother` (Nav2), or `odrive_can_node` (mapping-first) | `geometry_msgs/Twist` | reliable keep_last(10) |
| `/agv/cmd_vel_smoothed` | `velocity_smoother` | `collision_monitor` | Twist | reliable(10) |
| `/agv/cmd_vel_collision_safe` | `collision_monitor` ([nav2 config](src/agv_navigation/config/collision_monitor.yaml#L26)) | `cmd_vel_gate` ([remap in safety.launch.py:55](src/agv_safety/launch/safety.launch.py#L55)) | Twist | reliable(10) |
| `/agv/cmd_vel_safe` | `cmd_vel_gate` (output remap) | `odrive_can_node` ([agv_full.launch.py:93](src/agv_bringup/launch/agv_full.launch.py#L93)) | Twist | reliable(10) |

**Bug**: `cmd_vel_gate` is launched but its startup watchdog blocks the chain
(see Section 6, bug #1).

### Odometry + TF sources
| Topic | Publisher | Subscribers | Type | QoS |
|---|---|---|---|---|
| `/agv/wheel_odom` | `odrive_can_node` ([odrive_can_node.cpp:102](src/agv_odrive/src/odrive_can_node.cpp#L102)) | `ekf_local` | `nav_msgs/Odometry` | best_effort(10) |
| `/visual_slam/tracking/odometry` | cuVSLAM | `ekf_global` (differential mode) | `nav_msgs/Odometry` | best_effort |
| `/agv/imu/filtered` | `imu_filter_node` | `ekf_local`, `ekf_global` | `sensor_msgs/Imu` | best_effort |
| `/agv/odometry/local` | `ekf_local` | `agv_ui_backend`, other consumers | `nav_msgs/Odometry` | best_effort |
| `/agv/odometry/global` | `ekf_global` | `agv_ui_backend`, `fusion_monitor`, Nav2 | `nav_msgs/Odometry` | best_effort |

**TF ownership** (must be mutually exclusive):
- `map → odom`: `ekf_global` ONLY. cuVSLAM TF disabled via `cuvslam_greenhouse.yaml` (`/**:` key). SLAM Toolbox TF disabled via `transform_publish_period: 0.0`.
- `odom → base_link`: `ekf_local` ONLY.
- `base_link → *`: `robot_state_publisher` (URDF static).
- ZED wrapper: `publish_tf: false`, `publish_imu_tf: true` (IMU calibration only).

Multiple publishers to the same TF hop would silently alternate and corrupt
state estimation. The `/**:` YAML override key is a hidden convention — get
it wrong (e.g., write the node name instead) and TF stays on silently.

### Map + localization events
| Topic | Publisher | Subscribers | Type | QoS |
|---|---|---|---|---|
| `/agv/maps/loaded` | `map_manager_node::publish_map_loaded_event`, `agv_ui_backend` boot-time publish | `map_manager_node` (self), `auto_init_orchestrator` | `std_msgs/String` | transient_local(1) reliable |
| `/agv/current_map` | `map_manager_node::on_maps_loaded_event` | `agv_ui_backend` (dashboard header) | `std_msgs/String` | transient_local(1) reliable |
| `/agv/localization/state` | `auto_init_orchestrator` | `agv_ui_backend` (LOC pill) | `std_msgs/String` (JSON) | transient_local(1) reliable |
| `/agv/marker_pose` | `marker_correction_node` | `auto_init_orchestrator` (AprilTag fallback) | `PoseWithCovarianceStamped` | reliable |
| `/agv/marker_raw_detected` | `marker_correction_node` | `agv_ui_backend` | `std_msgs/String` (tag_<id>) | reliable |

### Safety + estop
| Topic | Publisher | Subscribers | Type | QoS |
|---|---|---|---|---|
| `/agv/safety/status` | `safety_supervisor_node` | `cmd_vel_gate`, `agv_ui_backend` | `agv_interfaces/SafetyStatus` | reliable |
| `/agv/collision_monitor_state` | `collision_monitor` | `agv_ui_backend` | `std_msgs/String` | reliable |
| `/agv/e_stop` | `agv_ui_backend`, dashboard | `odrive_can_node`, `cmd_vel_gate` | `std_msgs/Bool` | reliable |
| `/agv/hardware_estop` | (none currently — placeholder for future hardware bridge) | `cmd_vel_gate` | `std_msgs/Bool` | transient_local reliable |
| `/agv/software_estop` | (none — referenced by supervisor) | `safety_supervisor_node` | `std_msgs/Bool` | reliable |

**Gap**: `/agv/hardware_estop` and `/agv/software_estop` have no publisher.
Gate and supervisor watchdog them, so if the watchdog is tight enough they
treat them as stale → unsafe → block motion.

### Mode + UI
| Topic | Publisher | Subscribers | Type | QoS |
|---|---|---|---|---|
| `/agv/mode` | `agv_ui_backend::setMode` ([index.ts:816](src/agv_ui_backend/src/index.ts#L816)) | `auto_init_orchestrator` (reacts to `nav` transition) | `std_msgs/String` | reliable |

---

## Section 5 — Cross-component flows

### cmd_vel flow (the one that's broken)
```
Dashboard joystick (web/agv_dashboard/src/components/Joystick.tsx)
  ↓ 20 Hz {linear, angular}
WebSocket (ws/control.ts)
  ↓ {type: 'cmd_vel'}
teleop_server (index.ts:206) — guards: eStop, currentMode ∈ {teleop, mapping}
  ↓ Twist, limits per mode
/agv/cmd_vel
  │
  ├── has_map=false ─────────────────────────→ agv_odrive_node → CAN → wheels
  │
  └── has_map=true
        ↓
      velocity_smoother (Nav2) — guards: active in lifecycle
        ↓ /agv/cmd_vel_smoothed
      collision_monitor (Nav2) — guards: source timeout 0.5s, stop/slowdown polygons
        ↓ /agv/cmd_vel_collision_safe   [new name post-Fase 2 of old plan]
      cmd_vel_gate (agv_safety) — GUARDS FIRE AT 0.5s post-boot
        ↓ zero output until SafetyStatus arrives   [BUG]
      /agv/cmd_vel_safe
        ↓
      agv_odrive_node → CAN → wheels
```

### TF chain
```
                   map
                    │
         ekf_global publishes → [subscribers: nav2 costmaps, waypoint_manager, auto_init]
                    │
                   odom
                    │
         ekf_local publishes → [subscribers: nav2 local_costmap, fusion_monitor]
                    │
                 base_link
                 │   │   │
   robot_state_publisher (URDF static)
                 │   │   │
     right_wheel left_wheel zed_camera_link → zed_left_camera_optical_frame
```

### Localization cascade (`auto_init_orchestrator`)
```
/agv/maps/loaded (std_msgs/String: map_name)
  ↓
Path A0 — ZED Area Memory
  Check file exists + size > 0 at zed_area_file_path
    ↓ if yes: wait up to 8s for /agv/zed/pose covariance < threshold for N consecutive frames
    ↓ if converged → LOCALIZED
  else fall through
    ↓
Path A — cuVSLAM keyframe DB
  /visual_slam/load_map <map>_cuvslam/
    ↓ wait 10s for /agv/marker_pose (AprilTag hint)
    ↓ if tag seen: /visual_slam/localize_in_map(pose_hint)
    ↓ if successful → LOCALIZED
  else fall through
    ↓
Path B — AprilTag absolute pose (no cuVSLAM)
  Use tag pose as initial SetPose directly
    ↓ if tag seen → DEGRADED
  else fall through
    ↓
Path C — last-known pose from <map>_meta.json
  If file exists: call robot_localization SetPose
    ↓ DEGRADED
  else fall through
    ↓
FAILED — red LOC pill, operator must drive to tag and call
         /agv/localization/reinitialize
```

### Event log flow
```
Node logs (cout / RCLCPP_* macros)
  ↓ via stdout/stderr inside the launch process
systemd journal (captured by agv.service)
  ↓
journalctl -u agv.service
agv_ui_backend EventLog (captures subset of events via WS)
  ↓
~/agv_data/events.jsonl (rolling) + WebSocket broadcast to dashboard
```

---

## Section 6 — Latent bugs

Each bug has severity, file:line, reproduction hint, and impact.

### BUG #1 — Teleop gated by safety chain (FIXED 2026-04-13, root cause updated)
**Severity**: Was Critical — robot could not move when a map was loaded.
**Status**: FIXED in Fase 6, root cause turned out to be different from the
initial hypothesis.

**Initial (wrong) hypothesis**: The watchdog in `cmd_vel_gate` fires at 0.5s
before `safety_supervisor` publishes first, gating cmd_vel forever.

**Actual root cause**: `safety_supervisor` had `/agv/collision_monitor_state`
in its `monitored_topics` list ([src/agv_safety/config/safety_params.yaml](src/agv_safety/config/safety_params.yaml)).
That topic is **event-driven**, not a heartbeat: Nav2's `collision_monitor`
only publishes state when it processes a cmd_vel_smoothed input. In
teleop-at-rest, no cmd_vel flows, the topic stays silent, the supervisor
marks it silent forever (`silent: /agv/collision_monitor_state`),
`safety_ok=false`, and `cmd_vel_gate` forwards zero velocity regardless
of operator input. The gate's watchdog at 0.5s was NOT firing — the code
explicitly skips the watchdog when `last_safety_msg_.nanoseconds() == 0`
([src/agv_safety/src/cmd_vel_gate.cpp:95](src/agv_safety/src/cmd_vel_gate.cpp#L95)).

There was also a silent precondition bug: the spec and the supervisor
config declared `collision_monitor_state` as `std_msgs/msg/String`, but
the real Nav2 publisher uses `nav2_msgs/msg/CollisionMonitorState`. DDS
could not match those, so even in the moments when Nav2 did publish, the
supervisor never received a message. Both the type and the membership in
`monitored_topics` were wrong.

**Fix applied**: Removed `/agv/collision_monitor_state` from
`monitored_topics` entirely. Updated [specs/interfaces.yaml](specs/interfaces.yaml)
to declare the correct type `nav2_msgs/msg/CollisionMonitorState`.
Documented in [src/agv_safety/CLAUDE.md](src/agv_safety/CLAUDE.md) which
kinds of topics are valid inputs for a freshness-based watchdog.
collision_monitor liveness is now verified by:
 1. `agv_healthcheck.sh` at boot (node presence)
 2. `agv_ui_backend` goal-dispatch watchdog

**Verification**: after fix, `ros2 topic echo /agv/safety/status` shows
`safety_ok: true, reason: ''`, `/agv/cmd_vel_safe` has 1 publisher and
1 subscriber. Teleop end-to-end still needs operator hands-on
confirmation but the safety chain no longer blocks it.

### BUG #2 — `*_meta.json` written only on shutdown, not on Save Map (HIGH)
**Severity**: High — Path C (last-known pose fallback) never triggers for
maps saved in the same session they were created.
**Location**: [src/agv_localization_init/src/auto_init_orchestrator_node.cpp](src/agv_localization_init/src/auto_init_orchestrator_node.cpp)
(shutdown hook) vs [src/agv_map_manager/src/map_manager_node.cpp::on_save_map](src/agv_map_manager/src/map_manager_node.cpp)
**Mechanism**: Operator clicks Save Map → `map_manager_node` writes
`<X>.yaml`, `<X>.pgm`, `<X>_cuvslam/`, `<X>.area`. The `<X>_meta.json` is
only written by the orchestrator's periodic save or at clean shutdown. If
the stack crashes before the next periodic save, the new map has no
meta.json. Next Load Map → Path C finds no file → falls to FAILED.
**Fix strategy**: Fase 6, bug #2. Add a ROS service or IPC that the
orchestrator exposes, `write_meta_now(map_name)`, and map_manager calls it
as part of the Save Map chain.

### BUG #3 — Mode transitions have no feedback loop (MEDIUM)
**Severity**: Medium — UI claims `nav` mode while Nav2 is actually down.
**Location**: [src/agv_ui_backend/src/index.ts:810-821](src/agv_ui_backend/src/index.ts#L810-L821)
**Mechanism**: `setMode('nav')` publishes `/agv/mode` and updates
`state.currentMode`, but never checks whether Nav2's lifecycle nodes are
actually active. If Nav2 crashed or is still in `configure`, goals sent to
the action client time out silently.
**Fix strategy**: Fase 6, bug #3. Check `lifecycle_manager_navigation`
state via service before transitioning, or subscribe to Nav2 heartbeat.

### BUG #4 — `ros2 daemon` sees external dev PC over USB bridge (LOW, mitigated)
**Severity**: Low — mitigated by `ROS_DOMAIN_ID=42`.
**Location**: [src/agv_slam/config/cyclonedds.xml](src/agv_slam/config/cyclonedds.xml) peers list.
**Mechanism**: CycloneDDS config has `<Peer address="192.168.55.100"/>` for
a dev PC attached via `l4tbr0` USB bridge. When the dev PC runs any ROS 2
node without setting a domain ID, its nodes appear in the robot's graph.
Caused the "ghost `sim_*` and `/teleop` nodes" incident.
**Mitigation in place**: `ROS_DOMAIN_ID=42` in all agv systemd units.
**Fix strategy**: Fase 5, item 7. Remove the peer from cyclonedds.xml or
document it explicitly as "dev-only peer, never used in production".

### BUG #5 — default_empty.yaml is not documented (LOW)
**Severity**: Low — changes system behavior invisibly to readers of CLAUDE.md.
**Location**: [src/agv_navigation/maps/default_empty.yaml](src/agv_navigation/maps/default_empty.yaml), [src/agv_bringup/scripts/agv_start.sh:89](src/agv_bringup/scripts/agv_start.sh#L89)
**Mechanism**: First boot of a new robot, no saved map → agv_start.sh sets
`MAP=default_empty.yaml` so Nav2 still comes up. This means "mapping-first"
with Nav2 + safety chain running against an empty grid — behavior not
described in any CLAUDE.md.
**Fix strategy**: Fase 2, `specs/state_machine.yaml`; Fase 4, updated
`agv_bringup/CLAUDE.md`.

### BUG #6 — `save_area_memory` path mismatch history (FIXED in prev session)
**Severity**: Was Critical, now resolved.
**Fix**: ZED wrapper patched to re-read `pos_tracking.area_memory_db_path`
live on each `reset_pos_tracking` call, with size-check defensive skip and
retry-without-file fallback. Documented in [src/zed-ros2-wrapper/zed_components/src/zed_camera/src/zed_camera_component.cpp:4898-4986](src/zed-ros2-wrapper/zed_components/src/zed_camera/src/zed_camera_component.cpp#L4898-L4986).
**Follow-up**: specs need to capture the per-map `.area` file flow so
future changes don't silently undo it.

---

## Section 7 — Drift evidence

Concrete examples of documentation/code/comment disagreement.

1. **`safety.launch.py` header** says `NOT yet wired into agv_full.launch.py`
   but it IS wired in [agv_full.launch.py:317-335](src/agv_bringup/launch/agv_full.launch.py#L317-L335) via `IfCondition(has_map)`.
2. **`agv_bringup/CLAUDE.md`** launch-file table lists `agv_modular.launch.py`,
   `nav.launch.py`, etc. as real launch files. None of them are invoked from
   `agv_start.sh`. They are dead code.
3. **`agv_navigation/CLAUDE.md`** says `stop_zone` is "footprint + 5cm" in
   one table and "footprint + 20cm" in another. The actual YAML at
   [src/agv_navigation/config/collision_monitor.yaml:52](src/agv_navigation/config/collision_monitor.yaml#L52)
   specifies coordinates `[0.70, 0.42, ..., -0.35, 0.42]` which is footprint
   (front 0.50) + 20cm = 0.70. The "5cm" claim is stale from a prior audit.
4. **`/specs/interfaces.yaml`** declares 8 topics + 5 services + 1 action.
   Grep of `create_publisher` / `create_subscription` in the workspace finds
   dozens. Spec is incomplete, not wrong — but its incompleteness allows drift
   because readers assume the spec is authoritative.
5. **`CLAUDE.md` root** claims "No Python ROS2 nodes in the robot runtime
   stack" (Rule 2). [src/agv_slam/scripts/session_recorder.py](src/agv_slam/scripts/session_recorder.py)
   is a `rclpy.node.Node` launched by `agv_slam.launch.py` at production
   timing. TASK.yaml lacks `dev_only: true`.
6. **`cmd_vel_gate` remappings in comments** talk about `cmd_vel_safe` as
   input. Current code remaps `cmd_vel_in → cmd_vel_collision_safe` and
   `cmd_vel_out → cmd_vel_safe`. The previous `cmd_vel_safe → cmd_vel_safe`
   self-loop has been fixed but comments not updated everywhere.

---

## Section 8 — Rule violations

From [policies/engineering_rules.md](policies/engineering_rules.md).

| Rule | Violator | Evidence |
|---|---|---|
| 0 — C++17 only robot nodes | `session_recorder.py` | `rclpy.node.Node` at [src/agv_slam/scripts/session_recorder.py](src/agv_slam/scripts/session_recorder.py), launched in production `agv_slam.launch.py`, no `dev_only` in `src/agv_slam/TASK.yaml` |
| 0 — C++17 only robot nodes | `slam_web_gui_node.py` | Same pattern as above. 94% CPU sustained in prior diagnostics. |
| 0 — C++17 only robot nodes | `coverage_monitor.py` | Launched via `playback.launch.py` — needs decision on dev vs prod. |
| 1 — No hardcoded physical params, IPs, paths | [src/agv_map_manager/src/map_manager_node.cpp:45](src/agv_map_manager/src/map_manager_node.cpp#L45) | `"/home/orza/agv_data/maps/.current.area"` (username in path) |
| 1 — No hardcoded physical params, IPs, paths | [src/agv_slam/config/zed2i_override.yaml:38](src/agv_slam/config/zed2i_override.yaml#L38) | Same path hardcoded |
| 1 — No hardcoded physical params, IPs, paths | [src/agv_localization_init/src/auto_init_orchestrator_node.cpp:131](src/agv_localization_init/src/auto_init_orchestrator_node.cpp#L131) | Same path hardcoded |
| 1 — No hardcoded physical params, IPs, paths | [src/agv_slam/config/recording.yaml:11](src/agv_slam/config/recording.yaml#L11) | `/mnt/ssd/sessions/` hardcoded (with the fake-mount issue) |
| 5 — Warnings as errors | [src/agv_odrive/CMakeLists.txt:5](src/agv_odrive/CMakeLists.txt#L5) | `-Wall -Wextra -Wpedantic` but no `-Werror` |
| 5 — Warnings as errors | [src/agv_slam/CMakeLists.txt:5](src/agv_slam/CMakeLists.txt#L5) | Same |
| 9 — Interface governance | `/specs/interfaces.yaml` | Incomplete — 8 topics declared but code uses many more. No version bump process. |

**None of these are caught by automation today** — no pre-commit hook, no CI,
no linter in the workspace.

---

## Section 9 — Dead code

- **10+ launch files** in [src/agv_bringup/launch/](src/agv_bringup/launch/) unreachable from `agv_start.sh`.
- **`enable_behaviors`** argument at [agv_full.launch.py:68](src/agv_bringup/launch/agv_full.launch.py#L68) defaults to `false` and is never overridden. `behavior_executor_node` never runs.
- **`/agv/software_estop`** and **`/agv/hardware_estop`** topics have no publisher. They are expected by `cmd_vel_gate` and `safety_supervisor` watchdogs.
- **Packages without CLAUDE.md or TASK.yaml**: `agv_factor_graph`, `agv_rail_approach`. These packages have code in production but no documentation, creating ambiguity about ownership and contract.
- **Launch-included but structurally disconnected**: `agv_safety/launch/safety.launch.py` is included by `agv_full`, but its own header declares it integration-pending — it launches the nodes but the nodes don't function correctly (see bug #1).

---

## Appendix — Files read during the audit

Direct reads this session:
- Full plan file `/home/orza/.claude/plans/vivid-plotting-sifakis.md`
- `src/agv_bringup/launch/agv_full.launch.py`
- `src/agv_bringup/scripts/agv_start.sh`
- `src/agv_map_manager/src/map_manager_node.cpp`
- `src/agv_localization_init/src/auto_init_orchestrator_node.cpp`
- `src/agv_slam/config/zed2i_override.yaml`
- `src/agv_safety/launch/safety.launch.py`
- `src/agv_navigation/config/collision_monitor.yaml`
- `src/agv_navigation/behavior_trees/navigate_to_pose_forward_only.xml`
- `src/agv_ui_backend/src/index.ts`, `src/app_deps.ts`, `src/ws/control.ts`, `src/routes/maps.ts`
- `web/agv_dashboard/src/components/TopBar.tsx`, `src/api/types.ts`
- `src/zed-ros2-wrapper/zed_components/src/zed_camera/src/zed_camera_component.cpp` (patched regions)

Via subagents:
- Every `src/*/CLAUDE.md` and `src/*/TASK.yaml`
- `/specs/*.yaml`, `/policies/*.md`, `/agents/*.yaml`, `/CLAUDE.md`
- Topic publisher/subscriber pairings across 14 critical topics

Live ROS 2 state:
- `ros2 node list` output (57 nodes)
- `ros2 service list | grep -E 'save_area|load_map|save_map|reset_pos_tracking|localization'`
- `systemctl status agv.service agv-watchdog.service agv-healthcheck.service`

---

## What this document enables

- **Fase 2** (specs as SSOT): each row of the matrix in Section 1, each entry
  in Section 3, and each topic in Section 4 becomes a YAML entry in
  `specs/state_machine.yaml`, `specs/persistence.yaml`, and `specs/interfaces.yaml`.
- **Fase 3** (enforcement): each violation in Section 8 becomes a check
  in `tools/verify_specs/`.
- **Fase 4** (restructure docs): each drift example in Section 7 marks a
  CLAUDE.md that needs refresh.
- **Fase 5** (fix violators): Section 8 is the action list.
- **Fase 6** (fix latent bugs): Section 6 is the action list.

No code has been modified in the production of this document. The robot is
still in its freeze state.
