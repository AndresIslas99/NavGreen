# agv_scan_mapper

C++17 ROS2 node that builds a 2D occupancy grid from LaserScan data using
Bayesian log-odds probability. Used during commissioning to create live maps
before saving via map_manager. Features warm-up gating, dynamic grid expansion,
and robot-centered grid initialization.

## Nodes

- **scan_grid_mapper_node** (C++17): Subscribes to LaserScan, raycasts via
  Bresenham algorithm, publishes occupancy grid at configurable rate.

## Topics

**Published:**
- `live_map` (OccupancyGrid, 2 Hz, transient_local QoS) — Live occupancy grid

**Subscribed:**
- `/scan` (LaserScan) — Input laser scan data
- `clear_map` (Bool) — Clear grid and re-center on robot position

## Services

- `clear_map_srv` (std_srvs/Empty) — Clear grid and re-center on robot position

## Parameters

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `resolution` | `0.025` | Meters per cell (2.5cm for fine detail) |
| `initial_width` / `initial_height` | `400` | Starting grid size in cells (10m × 10m) |
| `expand_margin_cells` | `80` | Cells of margin per grid expansion (2m) |
| `max_width_cells` / `max_height_cells` | `6000` | Auto-expansion cap per axis (150m at 0.025 res); scans implying a larger extent are dropped |
| `publish_rate_hz` | `2.0` | Map publish rate (lower for large grids) |
| `map_frame` | `"map"` | Frame ID for output |
| `l_occupied` | `1.2` | Log-odds increment for occupied (stronger per-hit) |
| `l_free` | `-0.4` | Log-odds decrement for free (protects thin objects) |
| `l_min` / `l_max` | `-5.0` / `5.0` | Log-odds clamp range |
| `occupied_threshold` | `0.65` | Probability to mark occupied |
| `free_threshold` | `0.35` | Probability to mark free |
| `max_range` / `min_range` | `8.0` / `0.3` | Valid scan range (m) |
| `ray_subsample` | `1` | Process every ray (was 2 — full resolution) |
| `min_travel_distance` | `0.03` | Min robot movement to process scan (m) |
| `warmup_seconds` | `5.0` | Seconds to skip after first TF (cuVSLAM stabilization) |
| `warmup_min_scans` | `30` | Minimum scans to skip during warm-up |

## Key Algorithms

- **Log-odds Bayesian update**: `P(occupied) = 1 / (1 + exp(-log_odds))`
- **Bresenham raycast**: Marks free cells along ray, occupied at endpoint
- **TF handling**: Uses `TimePointZero` to avoid sim_time jitter
- **Warm-up gate**: Skips first N seconds of scans after cuVSLAM starts to avoid noisy initial poses contaminating the map. After warm-up, resets grid centered on robot position.
- **Grid centering**: On warm-up completion and clear_map, grid origin is centered on current robot position (via TF lookup), not (0,0). Prevents offset between grid and robot.
- **Force publish on clear**: Immediately publishes empty grid after clear to replace transient_local cache, ensuring dashboard shows cleared map.
- **Dynamic grid expansion**: Auto-expands grid when scan endpoints fall outside bounds, copying existing data to new grid with correct offset. Expansion is capped at `max_width_cells`/`max_height_cells`; a scan whose endpoints imply a larger extent (upstream EKF/cuVSLAM pose jump) is dropped with a throttled error instead of allocating an unbounded grid.

## Configuration

- `config/scan_mapper_params.yaml` — All parameters above

## Dependencies

- rclcpp, sensor_msgs, nav_msgs, std_srvs, std_msgs, tf2

## Improvement Opportunities

- Add unit tests (currently zero test coverage)
- Add map quality metrics (coverage percentage, noise ratio)
- Consider publishing map metadata for downstream consumers
