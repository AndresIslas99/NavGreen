# agv_scan_mapper

C++17 ROS2 node that builds a 2D occupancy grid from LaserScan data using
Bayesian log-odds probability. Used during commissioning to create live maps
before saving via map_manager.

## Nodes

- **scan_grid_mapper_node** (C++17): Subscribes to LaserScan, raycasts via
  Bresenham algorithm, publishes occupancy grid at configurable rate.

## Topics

**Published:**
- `live_map` (OccupancyGrid, configurable rate) — Live occupancy grid for commissioning

**Subscribed:**
- `/scan` (LaserScan) — Input laser scan data
- `clear_map` (Bool) — Clear grid via topic

## Services

- `clear_map` (std_srvs/Empty) — Clear grid via service call

## Parameters

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `resolution` | `0.05` | Meters per cell |
| `width` / `height` | `400` | Grid size in cells |
| `origin_x` / `origin_y` | `-10.0` | Grid origin (m) |
| `publish_rate_hz` | `1.0` | Map publish rate |
| `map_frame` | `"map"` | Frame ID for output |
| `l_occupied` | `0.85` | Log-odds increment for occupied |
| `l_free` | `-0.4` | Log-odds decrement for free |
| `l_min` / `l_max` | `-5.0` / `5.0` | Log-odds clamp range |
| `occupied_threshold` | `0.65` | Probability to mark occupied |
| `free_threshold` | `0.35` | Probability to mark free |
| `max_range` / `min_range` | `8.0` / `0.3` | Valid scan range (m) |

## Key Algorithms

- **Log-odds Bayesian update**: `P(occupied) = 1 / (1 + exp(-log_odds))`
- **Bresenham raycast**: Marks free cells along ray, occupied at endpoint
- **TF handling**: Uses `TimePointZero` instead of exact scan timestamp to avoid sim_time jitter

## Configuration

- `config/scan_mapper_params.yaml` — All parameters above

## Dependencies

- rclcpp, sensor_msgs, nav_msgs, std_srvs, tf2

## Improvement Opportunities

- Add unit tests (currently zero test coverage)
- Add map quality metrics (coverage percentage, noise ratio)
- Add dynamic grid resizing for large environments
- Consider publishing map metadata (resolution, origin) for downstream consumers
