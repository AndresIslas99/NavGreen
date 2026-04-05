# agv_map_manager

C++17 ROS2 node for map persistence and zone management. Saves and loads
occupancy grids via Nav2's map_saver/map_server, and persists keepout/speed
zones as line-delimited JSON.

## Nodes

- **map_manager_node** (C++17): Manages map save/load and zone CRUD operations

## Services

- `map_manager/save_map` (SaveMap) — Saves current occupancy grid to PGM+YAML via nav2 map_saver_cli
- `map_manager/load_map` (LoadMap) — Loads map from disk via nav2 map_server service
- `map_manager/update_zone` (UpdateZone) — Persist/remove keepout or speed zones as JSON

## Parameters

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `map_dir` | (required) | Directory for map and zone persistence |
| `default_map` | `""` | Map to load on startup (optional) |
| `map_topic` | `"/agv/map"` | Topic for map_saver subscription |

## Configuration

- `config/map_manager_params.yaml` — Map directory and defaults
- `launch/map_manager.launch.py` — Node launch with namespace support

## Key Implementation Details

- Map names validated against path traversal (`/` and `..` rejected)
- Shell-escaped map_saver_cli invocation to prevent command injection
- Zones stored as line-delimited JSON in `{map_dir}/zones.json`
- 2-second startup delay to allow nav2 initialization
- Uses nav2 `map_server/load_map` service client for map loading

## Dependencies

- nav2_map_server (map_saver_cli, load_map service), agv_interfaces, rclcpp

## Improvement Opportunities

- Add JSON schema validation for zone data (currently naive string parsing)
- Add zone conflict detection (overlapping keepout zones)
- Integrate zones with Nav2 costmap filters for runtime enforcement
- Add unit test for path traversal validation
- Replace line-delimited JSON with proper JSON array format
