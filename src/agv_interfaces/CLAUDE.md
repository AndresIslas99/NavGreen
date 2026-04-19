# agv_interfaces

Custom ROS2 message and service definitions for the AGV system.
Defines the data contracts between packages for missions, waypoints, maps, and zones.

## Messages

- **Waypoint.msg**: `x`, `y`, `theta` (float64), `action` (string: "none"|"pause"|"signal"), `pause_sec` (float64)
- **Mission.msg**: `id`, `name` (string), `waypoints[]` (Waypoint), `created` (float64 unix timestamp)

## Services

- **SaveWaypoint.srv**: Save mission with optional auto-generated ID -> success + assigned ID
- **ListMissions.srv**: (no request) -> missions[] array
- **ExecuteMission.srv**: mission_id -> success + message
- **SaveMap.srv**: name -> success + message (name validated against path traversal)
- **LoadMap.srv**: name -> success + message
- **UpdateZone.srv**: zone_id, zone_type ("keepout"|"speed"), polygon_x[], polygon_y[], max_speed, remove -> success + message

## Used By

- **agv_map_manager**: SaveMap, LoadMap, UpdateZone
- **agv_waypoint_manager**: SaveWaypoint, ListMissions, ExecuteMission, Waypoint, Mission
- **agv_behaviors**: ExecuteMission
- **agv_ui_backend**: All services (via rclnodejs)

## Configuration

- Built with `rosidl_default_generators` via CMakeLists.txt
- No runtime configuration needed

## Dependencies

- rosidl_default_generators, geometry_msgs (build), rosidl_default_runtime (exec)

## Improvement Opportunities

- Add field range documentation in .msg/.srv comments (valid ranges for x, y, theta, etc.)
- Consider adding a DiagnosticStatus.msg for standardized health reporting
- Add message versioning strategy for backward compatibility
