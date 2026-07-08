# agv_map_manager

C++17 ROS2 node that orchestrates the **full four-sidecar map save/load chain**
and persists keepout/speed zones. It is the single entry point for map
operations from the dashboard and the coordinating layer between Nav2's
map_server, Isaac ROS Visual SLAM, the ZED SDK Area Memory, and the
auto_init_orchestrator's last-known-pose metadata.

## Nodes

- **map_manager_node** (C++17): Coordinates the save/load chain, swaps Area
  Memory files per loaded map, and handles zone CRUD.

## Services

- `map_manager/save_map` (SaveMap) — Runs the full save chain (see below)
- `map_manager/load_map` (LoadMap) — Loads a map by name + swaps Area Memory
- `map_manager/update_zone` (UpdateZone) — Persist/remove keepout or speed zones as JSON

## The four-sidecar save chain (`on_save_map`)

A single `/agv/map_manager/save_map {name: 'corridor_v1'}` call produces
four coordinated artifacts. Only step 1 blocks the service response; steps
2–4 fire asynchronously and land within a few hundred ms.

| # | Step | Implementation | Produces |
|---|------|---------------|----------|
| 1 | Occupancy grid | `popen("ros2 run nav2_map_server map_saver_cli -f {name} -t {map_topic}")` at `map_manager_node.cpp:180-201` | `{name}.pgm` + `{name}.yaml` |
| 2 | cuVSLAM keyframe DB | `save_cuvslam_map()` async call to `/visual_slam/save_map` (`FilePath.srv`) at `map_manager_node.cpp:442-474` | `{name}_cuvslam/` directory of keyframes |
| 3 | ZED Area Memory | `save_zed_area_memory()` async call to `/agv/zed/save_area_memory` (`Trigger.srv`), then `copy_landing_pad_to_per_map()` at `map_manager_node.cpp:481-510, 544-`. Two-step because the ZED SDK writes to an in-RAM landing pad; the copy persists it per map. | `{name}.area` |
| 4 | Last-known pose | `trigger_orchestrator_pose_save()` async call to `/agv/localization/save_last_known_pose` at `map_manager_node.cpp:516-542` | `{name}_meta.json` |

If any of the async steps fail, the map is still saved (step 1 succeeded)
but the orchestrator cold-start cascade for that map will fall through to
the next available path. See [docs/mapping_commissioning.md](../../docs/mapping_commissioning.md)
for the full operator procedure and validation checklist.

## The load chain (`on_load_map`)

Reverse of save. Three coordinated steps in `map_manager_node.cpp:221-260`:

1. `swap_area_memory_for_map(name)` — copies `{name}.area` onto the ZED
   wrapper's landing pad path (or clears the landing pad if no per-map
   file exists) BEFORE triggering nav2 load. This is how each map gets
   its own landmark DB without restarting the wrapper.
2. `load_map_internal(yaml_path)` — calls nav2 `map_server/load_map`
   service with the path.
3. On success: `reset_zed_pos_tracking()` asks the wrapper to re-read
   the (now swapped) Area Memory file, and `publish_map_loaded_event(name)`
   fires `/agv/maps/loaded` so `auto_init_orchestrator` starts its
   Path A0/A/B cascade.

The landing-pad pattern (`swap_area_memory_for_map()` at `map_manager_node.cpp:268-299`)
exists because the ZED SDK does not expose a "load this .area file now"
API — it only reads the path param on `startPosTracking`. The trick is:
copy the right file onto the path the wrapper already knows about, then
trigger a reset to force a re-read.

## Parameters

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `map_dir` | (required, passed via launch) | Directory for all sidecars |
| `default_map` | `""` | Map to load on startup (optional) |
| `map_topic` | `"/agv/map"` | Topic for map_saver subscription |
| `cuvslam_enabled` | `true` | Gates step 2 of the save chain |
| `cuvslam_save_service` | `/visual_slam/save_map` | Service name for keyframe DB save |
| `zed_area_save_enabled` | `true` | Gates step 3 of the save chain |
| `zed_area_save_service` | `/agv/zed/save_area_memory` | Service name for ZED Area Memory save |
| `zed_area_landing_path` | `/home/orza/agv_data/maps/.current.area` | Transient path the ZED wrapper reads on reload |
| `area_memory_autosave_period_s` | `180` | Background auto-save tick for the Area Memory sidecar |

## Configuration

- `config/map_manager_params.yaml` — Map directory and defaults
- `launch/map_manager.launch.py` — Node launch with namespace support

## Key implementation details

- Map names and zone ids whitelist-validated (1-64 chars of `[A-Za-z0-9_-]`,
  see `include/agv_map_manager/name_validation.hpp`) — closes path traversal
  AND shell injection through the `map_saver_cli` popen invocation
- Zones stored as line-delimited JSON in `{map_dir}/zones.json`, rewritten
  atomically (tmp + rename) on every update
- 2-second startup delay to allow nav2 initialization
- Async steps 2–4 never block the service response; failures are logged
  and the chain continues
- `{name}.area` uses a tmp+rename atomic write so partial files never
  corrupt the landing pad (`copy_landing_pad_to_per_map()`)

## Dependencies

- `nav2_map_server` (map_saver_cli, load_map service)
- `isaac_ros_visual_slam_interfaces` (FilePath.srv for save/load)
- `std_srvs` (Trigger.srv for ZED Area Memory and orchestrator save)
- `agv_interfaces`, `rclcpp`

## Improvement opportunities

- Add JSON schema validation for zone data (currently naive string parsing)
- Add zone conflict detection (overlapping keepout zones)
- Integrate zones with Nav2 costmap filters for runtime enforcement
- Replace line-delimited JSON with proper JSON array format
- Add a "verify-all-sidecars" diagnostic service for operator-triggered
  integrity checks on a stored map
