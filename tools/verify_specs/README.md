# tools/verify_specs

Repository-side verifier suite. Runs as a pre-commit hook (see
`install_git_hook.sh`) and on CI. `all.sh` orchestrates every script.

## Quick reference

```bash
bash tools/verify_specs/all.sh
```

Output ends in `all.sh: OK` (zero blocking failures) or
`all.sh: FAILED` (one or more blocking scripts returned non-zero).

## Scripts

### BLOCKING

| Script | What it checks |
|---|---|
| `verify_canonical_sources.sh` | CLAUDE.md / SPEC / AGENT_INSTRUCTIONS files exist and reference each other |
| `verify_no_hardcoded_paths.sh` | No literal `/home/orza/...` in source — paths go through `AGV_DATA_DIR` |
| `verify_werror.sh` | Every `add_executable(...)` in an AGV package is compiled with `-Werror` |
| `verify_dev_only.py` | Python ROS2 nodes are marked `dev_only: true` in TASK.yaml (no Python in production runtime) |
| `verify_interfaces.py` | Topics declared in code appear in `specs/interfaces.yaml` (presence only — NOT type matching, see `verify_topic_types.py` for that) |
| `verify_geometry_ssot.py` | `wheel_radius`, `track_width`, `gear_ratio` declared only in `robot_geometry.yaml` (single source of truth) |
| `verify_no_waypoint_manager.sh` | No live `Node(package='agv_waypoint_manager', ...)` in production launches (HIGH-11-B-01, Section 0 G_F7) |
| **`verify_topic_types.py`** | **Topic message types in code match `specs/interfaces.yaml`. See § "Bug pattern history" below.** |

### WARNING-ONLY (informational)

| Script | What it surfaces |
|---|---|
| `verify_claude_md_coverage.sh` | Packages missing a `CLAUDE.md` |
| `verify_persistence.py` | Persistence artefacts declared in `specs/persistence.yaml` but missing in code |
| `verify_state_machine.py` | State-machine YAML vs code drift |
| `verify_launch_sequence.py` | Launch DAG order vs spec |

## Bug pattern history — why `verify_topic_types.py` exists

This script exists because the same bug class hit the AGV **three times**
in eight months:

| Date | Finding | Package | Symptom |
|---|---|---|---|
| 2026-04-13 | Audit bug #1 | `agv_safety/safety_supervisor` | safety_supervisor marked `/agv/collision_monitor_state` silent and blocked cmd_vel — the subscriber was typed `std_msgs/String` but Nav2 publishes `nav2_msgs/CollisionMonitorState`. DDS dropped every message. |
| 2026-05-13 | CRITICAL-11-A-01 | `agv_mode_arbiter` | Same defect: subscriber `std_msgs/String` on the same topic. `BLOCKED_HANDOFF` unreachable on real STOP. Fixed in Sprint A.5 commit `8d81517`. |
| 2026-05-13 | G4 / Section 0 | `agv_rail_driver` | Same defect, discovered on hardware during Section-0 Day-2 verification via `ros2 topic info -v`. The Sprint A.5 hardening fix never propagated. Fixed inline in commit `08ac348`. |

Three occurrences of the same root cause in one repository is not
coincidence — it's a verification-toolchain gap. C++ template-typed
subscriptions in `rclcpp` accept any well-formed type; the DDS layer
silently drops the wrong one without a compilation or runtime error.
The compiler is happy. The build is happy. The unit tests are happy
(they're statically typed). Only an end-to-end runtime check via
`ros2 topic info -v` reveals the mismatch — and only if someone runs
it on the specific topic.

`verify_topic_types.py` closes the gap at the commit boundary.

### Regression-test evidence

Running the verifier against the historical pre-fix file content
empirically demonstrates it would have caught both prior occurrences:

```
# Against rail_driver pre-fix (commit 08ac348~1):
FAIL: src/agv_rail_driver/src/rail_driver_node.cpp:82
      subscription<std_msgs/msg/String>(/agv/collision_monitor_state)
      spec says: nav2_msgs/msg/CollisionMonitorState

# Against mode_arbiter pre-fix (commit 8d81517~1):
FAIL: src/agv_mode_arbiter/src/mode_arbiter_node.cpp:171
      subscription<std_msgs/msg/String>(/agv/collision_monitor_state)
      spec says: nav2_msgs/msg/CollisionMonitorState
```

A fourth occurrence cannot ship past this gate.

### What it covers

Scans every `.cpp`/`.hpp` under `src/agv_*` (workspace's own packages —
external pkgs like `isaac_ros_*`, `zed-ros2-*`, `nvblox*` are out of
scope). For each `create_subscription<TYPE>(X, ...)` and
`create_publisher<TYPE>(X, ...)` call:

1. Resolves the topic name. Two patterns supported:
   - Literal-string: `create_subscription<T>("/agv/foo", ...)`.
   - Parameter-default chain: `declare_parameter<std::string>("foo_topic", "/agv/foo")` then
     `auto v = get_parameter("foo_topic").as_string()` then
     `create_subscription<T>(v, ...)`.
2. Cross-checks the C++ type (e.g., `nav2_msgs::msg::CollisionMonitorState`)
   against the canonical type in `specs/interfaces.yaml` (after
   normalising `::` → `/`).
3. Mismatch → BLOCKING failure.
4. Topic not in spec → informational only (suppressed unless `VERBOSE=1`).
   This is a spec-coverage gap to address separately, not a type-match
   error.

### Limitations (intentional v1)

- TypeScript / `rclnodejs` not parsed. `rclnodejs` uses runtime IDL
  strings (`createSubscription('std_msgs/msg/String', ...)`) so the
  catastrophic silent-drop mode of C++ template-typed DDS pubs/subs
  doesn't apply the same way.
- Macro-generated subscriptions and topics built via shared header
  constants are not resolved. Add cases as they appear; the script
  enumerates exactly which patterns work today.
- The script trusts the spec. If a topic's canonical type in
  `specs/interfaces.yaml` is itself wrong, the verifier will pass
  code that matches the wrong spec. The 2026-04-13 audit calls this
  out at `interfaces.yaml:404` for `/agv/collision_monitor_state` —
  the spec was updated alongside the code.

## Adding a new verifier

1. Drop `verify_<name>.{sh,py}` in this directory.
2. Make it print a final line `verify_<name>: OK` or
   `verify_<name>: FAIL ...` and exit accordingly.
3. Add the path to either the `BLOCKING` or `WARNING` array in
   `all.sh`.
4. Document it in this README.
5. Provide a regression test (introduce a violation, confirm the
   script catches it, restore).
