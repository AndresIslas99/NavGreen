# agv_behaviors

Behavior tree-based mission execution using BehaviorTree.CPP v3. Provides a service
interface for executing missions via Nav2's `/navigate_to_pose` action. Currently MVP
skeleton — full BT execution is post-MVP.

## Nodes

- **behavior_executor_node** (C++17): Loads behavior tree XML files and dispatches
  navigation goals. Currently simplified to single-goal dispatch; full BT execution
  with recovery strategies is deferred.

## Services

- `behavior_executor/execute` (ExecuteMission) — Accepts mission_id, dispatches to Nav2

## Action Clients

- `/navigate_to_pose` (NavigateToPose) — Nav2 goal dispatch

## Behavior Trees (XML)

Three trees provided in `trees/`:

1. **single_waypoint.xml** (MVP): Direct Nav2 call for single goal
2. **navigate_with_recovery.xml**: Full Nav2 recovery pattern (6 retries, costmap clear, spin, backup, wait)
3. **waypoint_patrol.xml**: Multi-goal patrol with per-waypoint recovery

## Parameters

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `trees_dir` | `""` (set via launch) | Directory containing BT XML files |
| `default_tree` | `"single_waypoint.xml"` | Default behavior tree to load |

## Configuration

- `config/behavior_params.yaml` — Tree directory and default tree selection
- `trees/` — Behavior tree XML definitions

## Dependencies

- behaviortree_cpp_v3, nav2_msgs, agv_interfaces, rclcpp, rclcpp_action

## Improvement Opportunities

- Integrate Nav2 BT plugins for full behavior tree execution (currently skeleton only)
- Register custom BT nodes for mission-specific actions (pause, signal, etc.)
- Add tests for BT tree loading with Nav2 action mocking
- Implement mission preemption and pause/resume
