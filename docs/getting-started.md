# Getting started

This page takes you from zero to a built workspace and a running simulation.
You do not need a robot, a Jetson, a GPU, or any vendor SDK for anything on
this page.

There are three paths. Pick one:

| Path | Best for | You need |
|------|----------|----------|
| [A. Dev container](#path-a-dev-container) | Fastest start, zero host setup | Docker + VS Code (or any devcontainer tool) |
| [B. Native install](#path-b-native-ros-2-humble) | Daily development on your own machine | Ubuntu 22.04 + ROS 2 Humble |
| [C. Zero hardware, zero ROS build](#path-c-what-runs-with-zero-hardware) | Seeing what works before committing | Varies — some items need only Python 3 or Node 20 |

First, clone the repository:

```bash
git clone https://github.com/AndresIslas99/NavGreen.git
cd agv-greenhouse
```

## Path A — Dev container

The repo ships a dev container in
[`.devcontainer/`](https://github.com/AndresIslas99/NavGreen/tree/main/.devcontainer)
that mirrors CI exactly: the `ros:humble` base image plus colcon, rosdep, git,
and Node 20, with ROS sourced in every shell.

1. Open the repo in VS Code and choose **Reopen in Container** (or use any
   devcontainer-compatible tool, e.g. `devcontainer up`).
2. On first launch, `post_create.sh` runs automatically. It resolves the
   whole workspace with rosdep, skipping only the vendor-SDK keys
   (`isaac_ros_visual_slam isaac_ros_visual_slam_interfaces isaac_ros_nvblox
   isaac_ros_apriltag_interfaces zed_msgs gtsam`), and creates the robot data
   directory at `.agv_data/` inside the workspace (exported as
   `AGV_DATA_DIR`).
3. When it finishes, build and test:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_CXX_FLAGS="-Werror"
colcon test && colcon test-result --verbose
bash tools/verify_specs/all.sh        # SSOT verifier suite
cd web/agv_dashboard && npm ci && npm run build
```

!!! warning "GUI apps inside the container"
    The container has no display by default. Headless work — building,
    testing, `ros2 launch agv_sim sim.launch.py` (headless Gazebo), the spec
    verifiers, the TypeScript builds — all works out of the box. To see the
    Gazebo GUI or RViz you need X forwarding into the container, or use
    Path B on a desktop.

!!! note "Vendor-SDK packages are skipped here too"
    The dev container matches CI, so `agv_map_manager`,
    `agv_localization_init`, and `agv_factor_graph` will not build in it
    either, and `agv_bringup`'s production launch entry points cannot run —
    see [the vendor-SDK caveat](#the-vendor-sdk-caveat) below. Skip the three
    with the same `--packages-skip` flags as Path B.

## Path B — Native ROS 2 Humble

### Prerequisites

- Ubuntu 22.04 with [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
  installed (`ros-humble-desktop` recommended — it includes RViz).
- `python3-colcon-common-extensions` and `python3-rosdep`.
- Node 20, only if you want the TypeScript packages (dashboard, backend,
  fleet).

### 1. Resolve dependencies

```bash
source /opt/ros/humble/setup.bash

sudo rosdep init 2>/dev/null; rosdep update
rosdep install --from-paths src --ignore-src -y \
  --skip-keys="isaac_ros_visual_slam isaac_ros_visual_slam_interfaces isaac_ros_nvblox isaac_ros_apriltag_interfaces zed_msgs gtsam OpenCV"
```

The `--skip-keys` are dependencies that are **not on the public apt index**
(NVIDIA Isaac ROS, the ZED ROS 2 wrapper, GTSAM). Everything else — Nav2,
`robot_localization`, BehaviorTree.CPP, `ros2_control`, Gazebo Classic,
`teleop_twist_keyboard` — resolves from public repositories, so this one
command also pulls in everything the simulation needs.

!!! note "CI skips a few more keys"
    [`.github/workflows/ci.yaml`](https://github.com/AndresIslas99/NavGreen/blob/main/.github/workflows/ci.yaml)
    additionally skips `gazebo_ros gazebo_ros2_control gazebo_ros_pkgs
    teleop_twist_keyboard` in its build job, because CI installs the Gazebo
    stack via apt only in the dedicated simulation job. On a native machine
    you want those installed, so do **not** add them to your skip list.

### 2. Build

```bash
colcon build --symlink-install \
  --packages-skip agv_map_manager agv_localization_init agv_factor_graph \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-Werror"
```

Warnings are errors in every AGV C++ package — a clean build really is clean.

### The vendor-SDK caveat

Three packages have compile-time dependencies on vendor stacks that are not
on public apt, which is why they are skipped above (and not built in CI):

| Package | Needs | Source |
|---------|-------|--------|
| `agv_map_manager` | `isaac_ros_visual_slam_interfaces` | [NVIDIA Isaac ROS](https://nvidia-isaac-ros.github.io/) |
| `agv_localization_init` | `zed_msgs` | [ZED ROS 2 wrapper](https://github.com/stereolabs/zed-ros2-wrapper) (ZED SDK) |
| `agv_factor_graph` | GTSAM | [borglab/gtsam](https://github.com/borglab/gtsam) |

`agv_bringup` declares all three as runtime dependencies, so the production
launch entry points (`agv_full.launch.py`, `agv_mapping.launch.py`,
`agv_hil_full.launch.py`) only work on a machine with the full vendor
install — in practice, the Jetson. With the vendor stacks installed, all four
packages build normally with the same colcon command, without the
`--packages-skip` flags.

### 3. Test

```bash
colcon test --packages-skip agv_map_manager agv_localization_init agv_factor_graph
colcon test-result --verbose
```

### 4. TypeScript packages (optional)

`agv_ui_backend` uses [rclnodejs](https://github.com/RobotWebTools/rclnodejs),
which generates its ROS bindings at `npm ci` time from whatever interfaces are
in the sourced environment. So the backend needs `nav2_msgs` and a built
`agv_interfaces` **before** `npm ci`:

```bash
sudo apt install ros-humble-nav2-msgs
source /opt/ros/humble/setup.bash
colcon build --packages-select agv_interfaces
source install/setup.bash

cd src/agv_ui_backend && npm ci && npm run build    # operator backend (:8090)
```

The dashboard is plain Vite/React and needs only Node 20:

```bash
cd web/agv_dashboard && npm ci && npm run build
npm test        # vitest
```

The optional fleet layer builds the same way (with ROS sourced, matching CI):

```bash
cd fleet/agv_vda5050_adapter && npm ci && npm run build
cd ../agv_fleet_manager && npm ci && npm run build && npm test
```

## Path C — What runs with zero hardware

Everything in this section works on a laptop with no robot, no Jetson, no
camera, and no vendor SDK.

### Drive the AGV in Gazebo (simulation)

`agv_sim` spawns the real robot geometry in Gazebo Classic with physics, using
the same `diff_drive_controller` gains as the physical drivetrain:

```bash
ros2 launch agv_sim teleop_sim.launch.py            # GUI + keyboard teleop (needs xterm)
ros2 launch agv_sim sim.launch.py gui:=true rviz:=true
ros2 launch agv_sim sim.launch.py                   # headless — what CI runs
```

Drive it by publishing `geometry_msgs/Twist` on `/cmd_vel` and watch `/odom`.

!!! warning "The simulation is drivetrain-only"
    `agv_sim` simulates physics and the two-wheel differential drive —
    **no cameras, no lidar, no IMU**. That is deliberate: it keeps headless
    `gzserver` GPU-free so it runs in CI, and it respects the project rule
    that production compute (SLAM, EKF, Nav2) belongs on the Jetson, never in
    the sim. You cannot run the full autonomy stack in simulation — the
    production launch files require the external `agv_slam` (cuVSLAM) overlay,
    which is not published. Adding sensors and Nav2 to the sim is roadmap
    work (issue [#18](https://github.com/AndresIslas99/NavGreen/issues/18)).

Full walkthrough: [Drive the robot in simulation](tutorials/drive-in-simulation.md).

### Mock drivetrain (even lighter — no Gazebo)

Runs the production `ros2_control` stack against `mock_components` — the same
controller configuration as the robot, no physics engine at all:

```bash
ros2 launch agv_hw_interface agv_ros2control_mock.launch.py
```

Verify it in a second terminal:

```bash
ros2 control list_controllers
ros2 topic pub -r 10 /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.0}}'
ros2 topic echo /joint_states
```

(The controller runs with `use_stamped_vel: false`, so it consumes plain
`Twist` on `~/cmd_vel_unstamped`; its `cmd_vel_timeout` is 0.2 s, hence the
`-r 10`.)

### Unit tests

`colcon test` (see Path B step 3) exercises the C++ packages — safety chain,
mode arbiter, rail stack, sensor fusion, and more — entirely on your machine.

### Spec verifiers

The SSOT verifier suite is stdlib-only Python and bash — it needs no ROS at
all:

```bash
bash tools/verify_specs/all.sh                 # 9 checks; the BLOCKING set gates commits
bash tools/verify_specs/install_git_hook.sh    # once, installs the pre-commit hook
```

### TypeScript builds

The dashboard builds and tests with nothing but Node 20 (Path B step 4); the
backend and fleet packages need a sourced ROS environment but no hardware.

### What you cannot run without hardware

!!! warning "Honest limits"
    - **The full autonomy stack** (`ros2 launch agv_bringup
      agv_full.launch.py`): needs the vendor SDKs above plus the external
      `agv_slam` overlay (the cuVSLAM pipeline), which is not published.
    - **The HIL validation loop**: needs the maintainer's unpublished
      simulation workspace plus a physical Jetson — see the
      [HIL runbook](validation/RUNBOOK_lan_hil.md).
    - **Motor bring-up**: needs the real CAN bus and ODrive S1 controllers —
      see [Jetson & CAN setup](hardware_setup.md), and take its warning
      seriously: getting `can0` up on a Jetson is not trivial.

## Where to go next

- [Drive the robot in simulation](tutorials/drive-in-simulation.md) — the
  first tutorial; teleop, odometry, and what the sim does and does not model.
- [Run the operator dashboard](tutorials/operator-dashboard.md) — the browser
  HMI and its backend.
- [Architecture overview](architecture/overview.md) — the command chain,
  localization, and mode arbitration in one picture.
- [The spec system](architecture/spec-system.md) — read this before your
  first PR; specs and code must change in the same commit.
- [Contributing](community/contributing.md) — ground rules and workflow
  (also: [`CONTRIBUTING.md`](https://github.com/AndresIslas99/NavGreen/blob/main/CONTRIBUTING.md)).
