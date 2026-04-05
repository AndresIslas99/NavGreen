# Nav2 Field Tuning Guide

Quick reference for commissioning the AGV navigation stack on-site.

## Tuning Order

Follow this order to avoid cascading issues:

1. **Velocity** — `desired_linear_vel` (start low: 0.15 m/s, increase to 0.3)
2. **Lookahead** — `lookahead_dist` (0.3 for tight rows, 0.5+ for open corridors)
3. **Goal tolerance** — `xy_goal_tolerance` / `yaw_goal_tolerance` (tighten if stopping accuracy matters)
4. **Costmap** — `inflation_radius` (increase if robot passes too close to shelves)
5. **Obstacle heights** — `min_obstacle_height` (raise if ground noise causes false stops)

## Tunable Parameters Quick Reference

| Parameter | Default | Safe Range | Effect |
|-----------|---------|-----------|--------|
| `desired_linear_vel` | 0.3 m/s | 0.1–0.5 | Max cruise speed |
| `lookahead_dist` | 0.4 m | 0.2–0.7 | Path tracking aggressiveness |
| `xy_goal_tolerance` | 0.15 m | 0.10–0.25 | How close to stop at goal |
| `yaw_goal_tolerance` | 0.25 rad | 0.15–0.40 | Heading accuracy at goal |
| `rotate_to_heading_angular_vel` | 0.3 rad/s | 0.2–0.5 | In-place rotation speed |
| `min_approach_linear_velocity` | 0.05 m/s | 0.03–0.10 | Final approach creep speed |
| `inflation_radius` | 1.0 m | 0.5–1.5 | Obstacle influence on cost |
| `cost_scaling_factor` | 2.0 | 1.0–5.0 | Cost falloff rate (higher = tighter) |
| `min_obstacle_height` | 0.10 m | 0.05–0.20 | Filter ground noise |
| `max_angular_accel` | 0.8 rad/s^2 | 0.5–1.5 | Angular jerk limit |

## Common Symptoms and Fixes

| Symptom | Likely Parameter | Action |
|---------|-----------------|--------|
| Robot oscillates on path | `lookahead_dist` | Increase to 0.5–0.6 |
| Robot too close to shelves | `inflation_radius` | Increase to 1.2–1.5 |
| Robot won't reach goal | `xy_goal_tolerance` | Loosen to 0.20–0.25 |
| Robot stops for phantom obstacles | `min_obstacle_height` | Raise to 0.15–0.20 |
| Robot too slow in open areas | `desired_linear_vel` | Increase to 0.4–0.5 |
| Robot slams on brakes near goal | `approach_velocity_scaling_dist` | Increase to 0.8–1.0 |
| Planner fails often | `max_planning_time` | Increase to 3.0–5.0 |
| Robot doesn't recover from stuck | `movement_time_allowance` | Increase to 15–20 |

## Collision Monitor Zones

The collision monitor has two safety zones around the robot footprint:

```
Robot footprint: front=0.50m, rear=0.30m, left=0.37m, right=0.37m

Stop zone:     footprint + 5cm  →  [[0.55, 0.42], ...] → full stop
Slowdown zone: footprint + 25cm →  [[0.80, 0.50], ...] → 30% speed
```

If the robot footprint changes, recalculate zones by adding margins to each vertex.

## Pre-deployment Checklist

- [ ] Map loaded and visible in RViz
- [ ] EKF running (check `/diagnostics`)
- [ ] cmd_vel_safe topic active (collision monitor online)
- [ ] Send test goal via CLI: `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0}}}}"``
- [ ] Verify robot stops for obstacles (hand in front of scan)
- [ ] Verify slowdown zone triggers (obstacle at ~50cm)
- [ ] Run 3 round-trip missions without intervention
