# Mapping Commissioning Procedure

## Overview

This document describes how to create an occupancy grid map of the greenhouse
for Nav2 autonomous navigation. The process uses `scan_grid_mapper` to build
a live 2D map from LaserScan data while the operator drives the robot.

## Pipeline

```
ZED 2i → point cloud → pointcloud_to_laserscan → /agv/scan
                                                      ↓
                                            scan_grid_mapper_node
                                                      ↓
                                            /agv/live_map (OccupancyGrid)
                                                      ↓
                                            map_saver_cli → .pgm + .yaml
```

## Steps

### 1. Launch mapping mode

```bash
# HIL (simulation)
ros2 launch agv_bringup agv_hil_full.launch.py map:=/path/to/initial_map.yaml

# Real hardware
ros2 launch agv_bringup agv_mapping.launch.py
```

The `scan_grid_mapper` node starts automatically and publishes `/agv/live_map`.

### 2. Drive through all corridors

- Open the dashboard at `http://agv.local:8090/`
- Switch to **Map** mode
- Use the joystick to drive at **0.3–0.5 m/s**
- Cover every operational corridor at least once
- Perform **bi-directional passes** in representative aisles
- Keep dynamic activity minimal (no people walking nearby)

The dashboard shows the live scan accumulation. The `scan_grid_mapper` simultaneously
builds a proper OccupancyGrid with Bayesian log-odds update.

### 3. Verify coverage

Check the live map in the dashboard. Dark areas = occupied (walls, shelves),
light areas = free space, gray = unexplored. Ensure all corridors are covered.

You can also check via CLI:
```bash
ros2 topic echo /agv/live_map --once --no-arr | head -10
```

### 4. Save the map

```bash
# Save to a named file
ros2 run nav2_map_server map_saver_cli \
  -f ~/ros2_ws/maps/greenhouse_$(date +%Y%m%d) \
  -t /agv/live_map \
  --ros-args -p save_map_timeout:=10.0
```

This creates two files:
- `greenhouse_YYYYMMDD.pgm` — grayscale occupancy image
- `greenhouse_YYYYMMDD.yaml` — metadata (resolution, origin)

### 5. Load for navigation

Update the systemd service to use the new map:
```bash
sudo systemctl edit agv.service
# Add:
# [Service]
# Environment=AGV_MAP=/home/orza/ros2_ws/maps/greenhouse_YYYYMMDD.yaml
sudo systemctl restart agv.service
```

Or launch directly:
```bash
ros2 launch agv_bringup agv_full.launch.py \
  map:=/home/orza/ros2_ws/maps/greenhouse_YYYYMMDD.yaml
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| resolution | 0.05m | Grid cell size (matches Nav2 costmap) |
| width × height | 400 × 400 | Grid size in cells (20m × 20m) |
| origin_x, origin_y | -10.0 | Grid origin in meters |
| max_range | 8.0m | Ignore scan rays beyond this |
| l_occupied | 0.85 | Log-odds increase for occupied evidence |
| l_free | -0.4 | Log-odds decrease for free evidence |

## Troubleshooting

- **Map is all gray**: Robot isn't moving or `/agv/scan` has no valid rays.
  Check `ros2 topic hz /agv/scan`.
- **Map is all black**: Ground plane being detected as obstacles.
  Check `pointcloud_to_laserscan` height filter (min_height: 0.05, max_height: 1.20).
- **Map drifts over time**: EKF or cuVSLAM losing tracking.
  Check `ros2 topic echo /slam/quality` for SLAM confidence.
- **Map not saved**: Ensure `/agv/live_map` is publishing.
  Check `ros2 topic info /agv/live_map`.
