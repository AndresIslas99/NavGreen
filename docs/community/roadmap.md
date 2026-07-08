# Roadmap

NavGreen's public roadmap grew out of the
[2026-07-06 community-readiness review](../reviews/2026-07-06-community-readiness-review.md)
— a full-repository review (109 verified findings) done before opening the
project. Everything that was safe to fix was fixed; what remained is the work
below, tracked in the
**[roadmap tracking issue #18](https://github.com/AndresIslas99/agv-greenhouse/issues/18)**.

!!! note "GitHub is the live source"
    This page is a snapshot (July 2026). Issue states, discussion, and new
    items live on the
    [issue tracker](https://github.com/AndresIslas99/agv-greenhouse/issues).

## A simulation contributors can actually run

The single biggest blocker to non-documentation contributions: the full
autonomy stack currently needs the maintainer's private HIL simulation host,
vendor SDKs, and a physical Jetson.

- [#6 — Publish a runnable simulation story](https://github.com/AndresIslas99/agv-greenhouse/issues/6)
  — the goal is that you can clone the repo and see the robot navigate with
  no proprietary hardware. The first step already landed: `agv_sim` runs the
  AGV in Gazebo Classic with real physics and the production controller
  gains — but it is **drivetrain-only** (no cameras or lidar yet). Next:
  simulated sensors, then Nav2 in sim.
- [#20 — Make the headless Gazebo controller smoke test blocking in CI](https://github.com/AndresIslas99/agv-greenhouse/issues/20)
  — `gazebo_ros2_control` (Humble) can't start the controller_manager
  headless, so the CI Gazebo bringup is best-effort today. The identical
  controller stack is already proven headless via `ros2_control` mock
  components; fixing this (possibly by migrating off EOL Gazebo Classic to
  modern Gazebo / `gz_ros2_control`) makes the sim gate blocking.

Try what works today: [Drive the robot in simulation](../tutorials/drive-in-simulation.md).

## Architecture consolidation

Deliberately not "fixed" in a quick PR — each needs a decision or a larger
change, discussed in its issue first:

- [#7 — Consolidate the two mission-execution subsystems](https://github.com/AndresIslas99/agv-greenhouse/issues/7)
  — the TypeScript executor in `agv_ui_backend` and the C++
  `agv_behaviors`/`agv_waypoint_manager` stack both exist; one owner should
  win.
- [#8 — Honor `robot_namespace` instead of hardcoding `agv`](https://github.com/AndresIslas99/agv-greenhouse/issues/8)
  — the canonical namespace parameter is currently decorative, which blocks
  multi-robot deployments.
- [#9 — Extract the duplicated CAN/ODrive protocol layer](https://github.com/AndresIslas99/agv-greenhouse/issues/9)
  — `agv_odrive` and `agv_hw_interface` copy-paste the SocketCAN/ODrive
  code; it should be one shared library package.
- [#10 — Make `agv_factor_graph` an actually-independent estimator](https://github.com/AndresIslas99/agv-greenhouse/issues/10)
  — it is pitched as independent validation of `ekf_global` but consumes
  `ekf_global`'s own output.

## Test expansion

- [#11 — TypeScript tests for the UI backend and VDA 5050 adapter](https://github.com/AndresIslas99/agv-greenhouse/issues/11)
  — vitest suites now cover the dashboard and fleet manager; the two
  rclnodejs-dependent packages (auth, nav-goal validation, VDA 5050
  conformance) still have none.
- [#12 — Unit tests for `agv_factor_graph` and `agv_localization_init`](https://github.com/AndresIslas99/agv-greenhouse/issues/12)
  — the two substantive C++ packages with zero tests, following the ROS-free
  gtest pattern used elsewhere in the tree.

## CI coverage for the vendor-SDK packages

- [#13 — Optional CI job that builds the vendor-SDK packages](https://github.com/AndresIslas99/agv-greenhouse/issues/13)
  — `agv_map_manager` (Isaac ROS), `agv_localization_init` (ZED SDK),
  `agv_factor_graph` (GTSAM), and `agv_bringup` are excluded from CI today.
  GTSAM is the cheapest to unblock first.

## Correctness & security hygiene

Smaller, well-scoped items from the review ledger:

- [#15 — Atomic map/zone/mission writes + dashboard sidecar parity](https://github.com/AndresIslas99/agv-greenhouse/issues/15)
- [#16 — Strongly-typed collision-monitor subscription in `agv_rail_driver`](https://github.com/AndresIslas99/agv-greenhouse/issues/16)
- [#17 — Move the JWT out of the WebSocket URL query parameter](https://github.com/AndresIslas99/agv-greenhouse/issues/17)

## Polish & first impressions

- [#14 — Dashboard/HIL demo GIF in the README](https://github.com/AndresIslas99/agv-greenhouse/issues/14)
- [#19 — Clear the dashboard's pre-existing eslint errors](https://github.com/AndresIslas99/agv-greenhouse/issues/19)

## Owner-only items

A few items from the review can only be done by the repository owner and are
listed in [#18](https://github.com/AndresIslas99/agv-greenhouse/issues/18)
and
[`docs/going_public_checklist.md`](https://github.com/AndresIslas99/agv-greenhouse/blob/main/docs/going_public_checklist.md):
rewriting git history to remove a leaked credential, scrubbing the sibling
`agv-greenhouse-sim` repository, and branch protection on `main`.

## Want to help?

Pick an issue labeled
[`help wanted`](https://github.com/AndresIslas99/agv-greenhouse/labels/help%20wanted)
or
[`good first issue`](https://github.com/AndresIslas99/agv-greenhouse/labels/good%20first%20issue),
comment on it, and read the [contributing guide](contributing.md) — the
spec-first workflow applies to roadmap work like everything else.
