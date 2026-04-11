# agv_localization_init

C++17 ROS2 orchestrator that automatically localizes the robot against a
pre-built cuVSLAM keyframe database whenever a map is loaded. Replaces the
manual "click on the map to set initial pose" UX with a fully automatic
sequence using AprilTag detections as pose hints.

## Why this package exists

The dual-EKF (ekf_local + ekf_global) architecture requires that `map→odom`
is owned by `ekf_global`. That rules out AMCL (which also wants to publish
`map→odom`). In a greenhouse with repetitive crop rows, 2D scan matching is
ambiguous — five identical rows produce five equiprobable localization
hypotheses.

cuVSLAM solves both problems:
1. It does not publish TF (config has `publish_map_to_odom_tf: false`), so it
   cooperates with `ekf_global` by feeding `/visual_slam/tracking/odometry`.
2. Its relocalization uses 256-D visual descriptors (SIFT/ORB/neural) which
   break the 2D scan ambiguity — same-looking rows have distinctive foliage
   textures that disambiguate.

The service `/visual_slam/localize_in_map` takes a pose hint and runs a grid
search (3m × 10° by default per `cuvslam_greenhouse.yaml`). The best hint
comes from an AprilTag detection, which provides an **absolute** pose in the
map frame from `marker_correction_node::on_marker_pose`.

## Nodes

- **auto_init_orchestrator_node** (C++17): Listens for `maps/loaded` events
  from `map_manager_node`, loads the matching cuVSLAM keyframe DB, waits for
  an AprilTag or falls back to the last-known pose, then calls
  `/visual_slam/localize_in_map` and publishes the localization state.

## Topics

**Subscribed:**
- `/agv/maps/loaded` (std_msgs/String) — map name without extension, emitted
  by `map_manager_node` after a successful load
- `/agv/marker_pose` (geometry_msgs/PoseWithCovarianceStamped) — AprilTag
  absolute pose, from `marker_correction_node`

**Published:**
- `/agv/localization/state` (std_msgs/String, transient_local QoS) — JSON
  payload `{"action":"INITIALIZING|LOCALIZED|DEGRADED|FAILED","detail":"...","map":"..."}`
  The backend mirrors this for the dashboard LOC pill (informational only).

## Services called

- `/visual_slam/load_map` — load keyframe database from folder
- `/visual_slam/localize_in_map` — grid search relocalization with pose hint

## Services exposed

- `localization/reinitialize` (std_srvs/Trigger) — manual operator trigger to
  re-run the init sequence against the currently loaded map (dashboard button)

## Parameters

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `map_dir` | (required) | Directory where map artifacts live |
| `marker_wait_timeout_s` | 10.0 | How long to wait for AprilTag before falling back |
| `localize_retries` | 3 | Retries for `localize_in_map` on transient failure |
| `localize_retry_backoff_s` | 2.0 | Seconds between retries |
| `last_known_pose_filename_suffix` | `_meta.json` | Per-map fallback pose file suffix |
| `kidnapping_drift_m` | 5.0 | (Reserved — future feature) |

## Localization state machine

```
  (boot)
    ↓
  FAILED (no map)
    ↓ maps/loaded event received
  INITIALIZING (load cuVSLAM DB)
    ↓ load success
  INITIALIZING (waiting for AprilTag, up to marker_wait_timeout_s)
    ↓
  ┌── AprilTag received ──────► INITIALIZING (localize_in_map)
  │                                    ↓
  │                              LOCALIZED (AprilTag-anchored) ✓
  │
  ├── Timeout, last-known-pose on disk ──► INITIALIZING (localize_in_map)
  │                                               ↓
  │                                        DEGRADED (no AprilTag yet, visual only)
  │
  └── Timeout, nothing on disk ──► FAILED
                                         ↓
                            Red LOC pill on dashboard.
                            Recovery: drive to AprilTag via teleop,
                            then call /agv/localization/reinitialize.
```

## Dependencies

- `isaac_ros_visual_slam_interfaces` — for FilePath + LocalizeInMap srv types
- `std_msgs`, `std_srvs`, `geometry_msgs`, `nav_msgs`, `rclcpp`

## Integration with the rest of the stack

- Launched at ~t=7s in `agv_full.launch.py`, after cuVSLAM + marker_correction
  are up and before the dashboard connects.
- `map_manager_node` calls `/visual_slam/save_map` on save, so a fresh
  mapping session produces a keyframe DB in `{map_dir}/{name}_cuvslam/`.
- Backend subscribes to `/agv/localization/state` purely for the dashboard
  LOC pill. Nav goals are NOT gated on localization state — the orchestrator
  owns localization end-to-end.
- There is no manual `InitialPoseModal` in the dashboard. Recovery from
  `FAILED` is an explicit operator action: drive to an AprilTag via teleop
  and call `/agv/localization/reinitialize`.

## Improvement opportunities

- Add kidnapping detection: compare `marker_pose` vs `odometry/global`; if
  drift > `kidnapping_drift_m` without a matching wheel odometry delta,
  auto-trigger a re-init.
- Save last-known-pose on clean shutdown (subscribe SIGINT, dump current
  `odometry/global` to `{map_name}_meta.json`).
- Add a secondary AprilTag listener with a recent-history window to avoid
  stale pose hints from tags seen minutes ago.
- Publish diagnostic_msgs so the dashboard's system health panel sees the
  orchestrator node as part of the overall safety chain.
