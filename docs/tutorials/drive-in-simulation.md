# Drive the robot in simulation

The `agv_sim` package runs the NavGreen AGV in **Gazebo Classic** with real
physics — no robot, no CAN bus, no Jetson, no vendor SDK. You spawn the same
geometry the field robot uses (from `agv_description`), drive it with the
keyboard or by publishing `geometry_msgs/Twist` on `/cmd_vel`, and watch
odometry come back on `/odom` and `/joint_states`.

The sim reuses the production `diff_drive_controller` gains verbatim
(`wheel_separation: 0.960`, `wheel_radius: 0.0781`, the same velocity and
acceleration limits), so it drives kinematically like the real robot.

!!! warning "What this sim is NOT"
    `agv_sim` is a **drivetrain-only** simulation. There are **no cameras, no
    lidar, no IMU** — Gazebo supplies physics and a two-wheel differential
    drive, nothing else. It runs **no production compute**: no SLAM, no EKF,
    no Nav2. You cannot run the full autonomy stack here — the production
    launch files need the external `agv_slam` overlay (NVIDIA cuVSLAM) and
    other vendor SDKs that are not on public package indexes. See
    ["Running without the robot" in the README](https://github.com/AndresIslas99/agv-greenhouse/blob/main/README.md#running-without-the-robot)
    for the full picture of what does and does not work without hardware.

## Prerequisites

- **ROS 2 Humble** on Ubuntu 22.04 (the sim targets Gazebo Classic via
  `gazebo_ros` / `gazebo_ros2_control`).
- A cloned workspace — see [Getting started](../getting-started.md).
- The Gazebo and controller packages that
  [`src/agv_sim/package.xml`](https://github.com/AndresIslas99/agv-greenhouse/blob/main/src/agv_sim/package.xml)
  depends on — the same Gazebo/controller list the
  [CI `simulation` job](https://github.com/AndresIslas99/agv-greenhouse/blob/main/.github/workflows/ci.yaml)
  installs, plus `teleop-twist-keyboard` and `xterm` for the keyboard teleop
  (CI runs headless and skips those two):

```bash
sudo apt update
sudo apt install \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-xacro \
  ros-humble-teleop-twist-keyboard \
  xterm
```

`xterm` is only needed for `teleop_sim.launch.py`, which opens the keyboard
teleop in its own terminal window so it has a real TTY.

## Build

Only two packages are needed — `agv_sim` itself and `agv_description`, which
owns the robot geometry the sim composes:

```bash
cd ~/agv-greenhouse   # your workspace root
source /opt/ros/humble/setup.bash
colcon build --packages-select agv_description agv_sim
source install/setup.bash
```

## Quick start: keyboard teleop

```bash
ros2 launch agv_sim teleop_sim.launch.py
```

This starts Gazebo with its GUI (`gui:=true` is the default here), spawns the
AGV in a small greenhouse-corridor world (inline sun, ground plane, and four
box "crop rows" — no network model fetches), and opens
`teleop_twist_keyboard` in an xterm window. Click into the xterm and drive
with the keys it prints (`i` forward, `,` back, `j`/`l` to turn, `k` to stop).

If you are headless, or prefer your own terminal, launch the sim and teleop
separately:

```bash
ros2 launch agv_sim sim.launch.py gui:=true

# in a second terminal:
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/cmd_vel
```

!!! tip "Deliberately un-namespaced"
    The sim publishes flat topics — `/cmd_vel`, `/odom`, `/joint_states` —
    **not** the production `/agv/...` namespace. That is intentional: a
    newcomer should be able to drive with a single command. Production
    interfaces are specified in
    [`specs/interfaces.yaml`](https://github.com/AndresIslas99/agv-greenhouse/blob/main/specs/interfaces.yaml).

## `sim.launch.py` reference

`teleop_sim.launch.py` is a thin wrapper: it includes `sim.launch.py` and adds
the keyboard node. All the knobs live on `sim.launch.py`
([source](https://github.com/AndresIslas99/agv-greenhouse/blob/main/src/agv_sim/launch/sim.launch.py)):

| Argument | Default | Description |
|----------|---------|-------------|
| `gui` | `false` | Start the `gzclient` GUI. `false` = headless `gzserver` only (CI-friendly). |
| `world` | `agv_sim/worlds/greenhouse.world` | Gazebo world file to load. |
| `rviz` | `false` | Start RViz2 (with `use_sim_time: true`). |
| `x` | `0.0` | Robot spawn x position [m]. |
| `y` | `0.0` | Robot spawn y position [m]. |
| `yaw` | `0.0` | Robot spawn yaw [rad]. |

Examples:

```bash
ros2 launch agv_sim sim.launch.py                        # headless (what CI runs)
ros2 launch agv_sim sim.launch.py gui:=true rviz:=true   # interactive + RViz
ros2 launch agv_sim sim.launch.py gui:=true x:=1.5 yaw:=1.57
```

What it brings up: `gzserver` (plus `gzclient` when `gui:=true`),
`robot_state_publisher` fed by the xacro-expanded
`urdf/agv_sim.urdf.xacro`, `spawn_entity.py`, then — chained after the spawn —
the `joint_state_broadcaster` and `diff_drive_controller` spawners. The
controller is remapped so it consumes `/cmd_vel` and publishes `/odom`.

There is no pre-made RViz config: with `rviz:=true`, add **TF** and
**RobotModel** displays yourself and set the fixed frame to `odom`.

## Drive it from the command line

The controller accepts plain (unstamped) `geometry_msgs/Twist` on `/cmd_vel`.
It also has a `cmd_vel_timeout` of **0.5 s** — a single `--once` publish moves
the robot for half a second and stops. Publish at a steady rate instead:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.0}}"
```

Press `Ctrl-C` to stop publishing; the robot halts on its own 0.5 s later.
To turn while moving, set `angular.z` (e.g. `0.5`).

The controller enforces the production limits from
[`config/sim_controllers.yaml`](https://github.com/AndresIslas99/agv-greenhouse/blob/main/src/agv_sim/config/sim_controllers.yaml):
±0.4 m/s linear, ±1.5 rad/s angular. Commanding more gets clamped.

## Watch the odometry

```bash
ros2 topic hz /odom            # ~50 Hz (matches the real drivetrain rate)
ros2 topic echo /odom --once   # frame_id: odom, child_frame_id: base_link
ros2 topic echo /joint_states --once   # left_wheel_joint / right_wheel_joint
ros2 run tf2_ros tf2_echo odom base_link
```

Everything runs on simulation time: `gzserver` publishes `/clock` and every
node is started with `use_sim_time: true`.

## `enable_odom_tf`: the one intentional difference from production

In this sim, `diff_drive_controller` publishes the `odom -> base_link`
transform itself (`enable_odom_tf: true` in
[`sim_controllers.yaml`](https://github.com/AndresIslas99/agv-greenhouse/blob/main/src/agv_sim/config/sim_controllers.yaml)).

On the real robot this is **false**: the dual-EKF localization stack owns the
TF tree — `ekf_local` publishes `odom -> base_link` and `ekf_global` publishes
`map -> odom` (see
[Localization architecture](../architecture/localization.md)). The standalone
sim has no EKF, so the controller must publish its own transform or the TF
tree would be incomplete. Keep this in mind if you ever port code between the
sim and the robot: never enable the controller's odom TF alongside the EKFs.

## Troubleshooting

### Headless `gzserver`: controllers never activate (known limitation)

With `gui:=false` and **no display**, the world loads and the robot spawns,
but the `controller_manager` embedded in `gazebo_ros2_control` fails to start
on Humble:

```
[ERROR] [gazebo_ros2_control]: parser error Couldn't parse parameter
override rule: '--param robot_description:= ...
```

Humble's apt `gazebo_ros2_control` forwards the whole URDF to its embedded
controller manager as a `--param robot_description:=` command-line rule, which
`rcl` rejects (multi-line value). Result: no `/odom`, no `/joint_states`
headless. A full physics drive **works locally with a display**. This is
tracked in [issue #20](https://github.com/AndresIslas99/agv-greenhouse/issues/20);
the [CI `simulation` job](https://github.com/AndresIslas99/agv-greenhouse/blob/main/.github/workflows/ci.yaml)
therefore runs the headless Gazebo bringup as a *best-effort* step and gets
its blocking "it drives with no hardware" guarantee from the identical
controller stack running on `ros2_control` mock components instead
(`ros2 launch agv_hw_interface agv_ros2control_mock.launch.py`).

### No `/odom` or `/joint_states`, spawners fail

`ros-humble-gazebo-ros2-control` is probably not installed. The model appears
in Gazebo but the controller manager never starts inside `gzserver`. Install
it and relaunch.

### Teleop window doesn't respond to keys

`teleop_twist_keyboard` needs an interactive TTY. Make sure the xterm window
has focus, or run it manually in your own terminal (see Quick start above).
If `xterm` is not installed, `teleop_sim.launch.py` cannot open the window at
all — install `xterm` or use the two-terminal variant.

### Gazebo hangs at startup

The bundled world is self-contained (no `model://` includes), so Gazebo needs
no network. If a customized world stalls fetching models, disable the online
model database:

```bash
export GAZEBO_MODEL_DATABASE_URI=""
```

## Where to go next

- [Run the operator dashboard](operator-dashboard.md) — bring up the browser
  HMI (note: it listens on the production `/agv/...` namespace, so it does not
  display this un-namespaced sim).
- [Map a greenhouse](build-a-map.md) — the commissioning workflow on the real
  robot.
- [`src/agv_sim/CLAUDE.md`](https://github.com/AndresIslas99/agv-greenhouse/blob/main/src/agv_sim/CLAUDE.md)
  — the package contract: interfaces, invariants, and failure modes.
