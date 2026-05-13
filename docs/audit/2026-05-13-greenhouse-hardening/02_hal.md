# Phase 2 — Hardware Abstraction Layer (HAL)

> Phase 2 goal (per prompt): "ros2_control gobierna ODrive con watchdog y
> semánticas claras. ZED entrega data sincronizada y a frecuencia objetivo."

This file is the deep dive into the HAL: ODrive driver architecture, URDF
fidelity, the **opt-in** `agv_hw_interface` ros2_control alternative, and the
ZED 2i integration that lives outside this repo. CR-00-02 from Phase 0 is
expanded here with the runtime parameter resolution discovery and a
concrete refactor plan.

---

## A. Architectural choice today: plain Node, not ros2_control

The production path uses **`agv_odrive_node`** (a plain `rclcpp::Node`)
that reads its parameters from `src/agv_odrive/config/odrive_params.yaml`
and drives CAN directly. The workspace **also contains** an alternative,
opt-in package:

- `src/agv_hw_interface/` — implements `hardware_interface::SystemInterface`
  with a `pluginlib` description (`agv_hw_interface_plugin.xml`).
- `src/agv_hw_interface/launch/agv_ros2control.launch.py` — real hardware path.
- `src/agv_hw_interface/launch/agv_ros2control_mock.launch.py` — mock components.

Neither launch is invoked by `agv_full.launch.py`, `agv_mapping.launch.py`,
or `agv_hil_full.launch.py`. The package's own CLAUDE.md describes it as
"Validation-only until full cutover plan."

### Why this matters

The prompt §2.5 and field practice both rank `ros2_control + diff_drive_controller`
as the SOTA reference for differential-drive HAL. The benefits over a
plain Node:

- **Hot-pluggable controllers.** Operator can switch from velocity to
  effort to admittance control without re-launching.
- **Standard watchdog semantics.** `controller_manager` provides
  documented behavior on subscriber timeout, controller crash, hardware
  fault — instead of the bespoke `cmd_vel_timeout_ms` + `slip_cooldown` +
  `caster_settling_tau` state machine inside `agv_odrive_node`.
- **Mock components for CI.** `mock_components/GenericSystem` lets CI run
  the full stack against a fake HAL — exactly what `agv_hw_interface`
  was scaffolded for.
- **Public diff_drive_controller** consumes `wheel_separation` and
  `wheel_radius` from its YAML — same numbers that `xacro` uses — closing
  the loop on CR-00-02 (single source of truth).

### Why NOT to migrate today

- The plain-Node path has months of in-the-field tuning baked into it
  (caster compensation, dual-rate accel limiter, EMA velocity filter,
  zero-velocity bypass, stiction torque feedforward). The 2026-04-08
  calibration and the `wheel_slip_detector_node` interlock would all
  need to be redone in the ros2_control controller plugin or upstream
  of the controller (as a velocity-shaping controller chain).
- Production hardware needs `ros-humble-ros2-control*` apt-installed,
  per `agv_hw_interface/CLAUDE.md` — a deployment dependency this audit
  does not propose adding before MVP.
- A migration mid-MVP would re-introduce the class of bugs that fixing
  CR-00-02 closes.

### Recommendation

**`HIGH-02-01` — Plan the ros2_control cutover as a post-MVP milestone, not
this audit cycle.** Keep `agv_hw_interface` building in CI (already happens —
no `COLCON_IGNORE` present). Use the mock path in CI to validate Nav2 +
EKF against a simulated HAL — this is also the most honest way to give
the CI a "hardware-in-the-loop without hardware" signal. The cutover
plan should be tracked as an ADR (`docs/adr/0001-ros2-control-cutover.md`,
to be written) listing: tuning to port, validation steps, cutover
acceptance criteria, rollback plan.

---

## B. CR-00-02 expanded — Geometry SSOT, with runtime resolution evidence

### B.1 Code-level finding (verified)

`src/agv_odrive/src/odrive_can_node.cpp`:
```cpp
this->declare_parameter("wheel_radius", 0.0625);   // line 13 — matches URDF
this->declare_parameter("track_width",  0.735);    // line 14 — matches URDF
this->declare_parameter("gear_ratio",   1.0);      // line 31 — DOES NOT MATCH YAML
```

`src/agv_odrive/config/odrive_params.yaml`:
```yaml
wheel_radius: 0.0781        # overrides C++ default
track_width:  0.960         # overrides C++ default
gear_ratio:  10.0           # overrides C++ default
```

**Runtime resolution wins**: the YAML is loaded by `odrive.launch.py` and
becomes the live `ros2 param` value. So:

| Source | Wheel radius (m) | Track width (m) | Gear ratio |
|---|---|---|---|
| `agv_full.urdf.xacro:20-25` | **0.0625** | **0.735** (derived) | n/a |
| `agv_description/config/robot_params.yaml` | **0.0625** | **0.735** | n/a |
| `agv_odrive/src/odrive_can_node.cpp:13,14,31` (C++ defaults) | **0.0625** | **0.735** | **1.0** |
| `agv_odrive/config/odrive_params.yaml` (loaded YAML) | **0.0781** | **0.960** | **10.0** |
| `agv_navigation/config/nav2_params.yaml:250,322` (footprint width) | n/a | implied **0.74** | n/a |
| `agv_description/CLAUDE.md` (docs) | **0.0625** | **0.735** | n/a |
| `agv_odrive/CLAUDE.md` (table at lines 30,31) | **0.0781** | **0.960** | **10.0** |

**Six sources, two conflicting families.** The C++ author's defaults are
the URDF family (0.0625 / 0.735 / 1.0). The deployed YAML is the calibrated
family (0.0781 / 0.960 / 10.0). The CLAUDE.md files split the two — the
ODrive CLAUDE describes the YAML as authoritative; the description CLAUDE
describes its own YAML as "Single Source of Truth".

### B.2 Why the YAML values look like more than UMBmark calibration

A UMBmark calibration extracts a per-axis correction (typically <5 %) from
known forward distance and known rotation tests. A 25 % wheel radius
correction (0.0625 → 0.0781) and a 23 % track width correction (0.735 →
0.960) are **far outside the UMBmark working range**. Two physically
plausible scenarios fit the magnitudes:

1. **The robot has a 10:1 gearbox that was added between the chassis and
   the wheels at some point.** `gear_ratio: 10.0` in the YAML (vs 1.0 in
   the C++ default) is consistent with this. The 0.0781 / 0.960 numbers
   would then be the *effective* wheel radius and track width after the
   gearbox change — physically the wheels are still 0.125 m in diameter,
   but the relationship between motor turns and chassis motion changed.

2. **Larger wheels were swapped in.** A real wheel diameter of ~0.156 m
   (radius 0.078) with 0.0625 still in the URDF means the URDF is stale.
   This is consistent with `odrive_params.yaml:7` annotation "calibrated
   2026-04-08 (ratio 2.66/2.13 from 3-trial test)" — a measurement
   campaign that produced new values, but the URDF was not updated.

The CLAUDE.md for `agv_description` says "Measured 2026-03-18" and the
ODrive YAML says "calibrated 2026-04-08". The 3-week gap and the 25 %
delta strongly suggest **scenario 2**: a physical wheel swap happened
between those dates and only the ODrive YAML was updated. **This is
verifiable in 5 minutes with a caliper.**

### B.3 The Nav2 footprint puzzle

`nav2_params.yaml:250` defines the global costmap footprint:
```
[[0.50, 0.37], [0.50, -0.37], [-0.30, -0.37], [-0.30, 0.37]]
```

- Width = 0.74 m. Matches URDF track_width = 0.735 m almost exactly.
- Length = 0.80 m. URDF chassis is 1.0 m (chassis at `agv_base.xacro:11`
  origin `0.2 0 0` with `chassis_length=1.0` → spans x ∈ [-0.3, +0.7]).
  Footprint is 0.20 m **shorter than the chassis in front**.

Two interpretations:
- The 0.20 m gap at the front is **intentional** — that's the camera
  housing zone (ZED is mounted at x = 0.700, the very front edge of the
  chassis). Nav2 footprint ends at x = 0.50 so the camera doesn't trigger
  costmap "inside footprint" warnings on its own depth observations.
- The footprint **is wrong** — Nav2 plans paths that scrape the front of
  the chassis (and the camera mount) against any 200 mm-tall obstacle.

In **both** interpretations, the footprint **width (0.74 m) is wrong if
track_width is actually 0.960 m**. Wheels would extend 0.11 m beyond the
footprint on each side. In 1.0–1.5 m greenhouse corridors that's
catastrophic.

### B.4 Findings

---

### CRITICAL-02-02 — Geometry SSOT must be re-established before any navigation accuracy claim
**File(s)**:
- Same as CR-00-02 in Phase 0 (paths listed there).
- Plus `src/agv_odrive/src/odrive_can_node.cpp:13-14,31` (C++ defaults match URDF, YAML overrides them).

**Category**: calibration / bug.
**Symptom**: As in CR-00-02, with the additional discovery that the C++
defaults in `odrive_can_node.cpp` **agree with the URDF**. Only the YAML
diverges — and the YAML is the runtime value. So a programmer reading
the C++ code sees "0.0625 / 0.735 / 1.0" — consistent with URDF — and
cannot tell from the code that production is running 0.0781 / 0.960 /
10.0.
**Analysis**: see CR-00-02 + B.1 above.
**Greenhouse impact** (Phase 2-specific additions to CR-00-02):
- **MPPI dynamics model wrong.** `nav2_mppi_controller` (configured at
  `nav2_params.yaml:108-155`) uses the controller's internal kinematic
  model — at `controller_frequency: 20 Hz`, `model_dt = 0.05`. The model
  assumes the kinematic relationship `v_left/right = (linear_x ∓ angular_z * track_width/2) / wheel_radius`.
  Wrong track_width → MPPI predicts the wrong rotation rate from a
  given cmd_vel and over-corrects on every iteration. Observable as the
  "PathAlignCritic weight 20.0 doesn't stick — robot oscillates around
  the path".
- **HIL ↔ real divergence.** `agv_hil_bridges` is documented to mirror
  `odrive_params.yaml` (`agv_hil_bridges/CLAUDE.md:22-24`). HIL is
  consistent with ODrive but inconsistent with URDF, so HIL is **not
  testing the dynamics Nav2 sees in production**. HIL acceptance gates
  in `specs/acceptance.yaml` and the precision test in
  `agv_integration_tests/test/test_waypoint_precision.py` may be passing
  for the wrong reason.
**Benchmark**:
- ros2_control's `diff_drive_controller` reads `wheel_separation` and
  `wheel_radius` directly from its YAML — and the corresponding
  `<xacro:property name="..."/>` in URDF reads the same numbers. No two
  files can disagree because they share the YAML.
- Husarion ROSbot 2R ships a single `husarion_robot/config/rosbot_params.yaml`
  loaded by URDF and controller alike.
**Recommendation** (concrete steps):
1. **Measure physical wheels and track with a caliper / tape**. Photo
   the wheel beside a metric reference; document in
   `docs/calibration/physical_geometry_2026-05-XX.md` with the operator
   name and date.
2. **Update `robot_params.yaml`** to the physical values. Add a new
   block `robot.kinematics.calibration` that documents *whether* a
   UMBmark calibration delta is applied separately.
3. **Refactor `agv_odrive_node` to read from `robot_params.yaml`**:
   - The default values in `odrive_can_node.cpp:13-14` are removed —
     the parameters become *required* (`declare_parameter<double>("wheel_radius")`
     with no default, which throws if not provided).
   - `odrive_params.yaml` is renamed `odrive_dynamics_params.yaml` and
     no longer declares `wheel_radius` / `track_width`.
   - `odrive.launch.py` loads BOTH `robot_params.yaml` (for kinematics)
     AND `odrive_dynamics_params.yaml` (for motor tuning).
   - The URDF xacro args also read from `robot_params.yaml` via
     `description.launch.py`.
4. **Update `nav2_params.yaml` footprint** to derive from the same
   numbers (a `xacro --inorder` pass produces the polygon, or the launch
   script computes it). Add a `verify_geometry_ssot.py` script that
   asserts URDF, ODrive runtime params, Nav2 footprint, and HIL bridge
   all derive from the same `robot_params.yaml`.
5. **If a UMBmark calibration is needed**, put its result in
   `docs/calibration/wheel_odom_umbmark_<date>.yaml` and apply it as a
   per-side scale factor (`left_scale`, `right_scale` — these already
   exist in `odrive_params.yaml:19-20` as 1.0 / 1.0). The kinematic
   `wheel_radius` and `track_width` stay at the physical value.
**Acceptance criterion**:
- `grep -rh 'wheel_radius:\|track_width:' src/ | sort -u` returns one
  authoritative value, propagated to all consumers.
- A new `tools/verify_specs/verify_geometry_ssot.py` script blocks
  commits that introduce a divergent value.
- Closed-loop UMBmark test on hardware: forward 5 m × bidirectional →
  systematic error < 1 % distance; rotation test 4 × π rad → yaw error
  < 5 % per cycle.
**Effort**: M (measurement: half-day; refactor + verifier: 1 day; field
re-validation: half-day).
**Prerequisites**: hardware access for the measurement.

---

### HIGH-02-03 — URDF chassis inertia is the literal 1.0 placeholder
**File(s)**: `src/agv_description/urdf/agv_base.xacro:25-29`:
```xml
<inertial>
  <mass value="30.0"/>
  <origin xyz="0.2 0 0"/>
  <inertia ixx="1.0" ixy="0" ixz="0" iyy="1.0" iyz="0" izz="1.0"/>
</inertial>
```
**Category**: bug / sim-real-gap.
**Symptom**: The 30 kg chassis inertia is declared as the identity matrix
multiplied by 1.0. This is exactly the anti-pattern the prompt §3.4 calls
out: "no usar 1.0 placeholders".
**Analysis**: For a 30 kg box of dimensions 1.0 × 0.6 × 0.15 m the
*correct* solid-box inertia (CoM at the geometric center) is:
- ixx = m/12 × (h² + d²) = 30/12 × (0.15² + 0.6²) = 2.5 × (0.0225 + 0.36) = **0.956 kg·m²**
- iyy = m/12 × (h² + l²) = 30/12 × (0.15² + 1.0²) = 2.5 × (0.0225 + 1.0) = **2.556 kg·m²**
- izz = m/12 × (l² + d²) = 30/12 × (1.0² + 0.6²) = 2.5 × (1.0 + 0.36) = **3.4 kg·m²**

The hardcoded `1.0` value is **wrong by up to 3.4×** along the yaw axis.
This matters for:
- **Gazebo / Isaac Sim dynamics**. Yaw acceleration in sim is currently
  over-predicted by ~3.4× compared to the real robot. The HIL test
  `test_waypoint_precision.py` will give different answers than the
  field. Sim-to-real gap.
- **MPPI's internal dynamics model**, if it ever consumes URDF mass
  properties (not by default in 2D mode, but a 3-D extension would).
- **External tooling** that computes the robot's max angular acceleration
  for safety analysis.
**Greenhouse impact**: Indirect via HIL parity. A controller tuned in
sim will need re-tuning in the real greenhouse because the chassis
responds slower to yaw commands than sim predicts.
**Benchmark**: Husky URDF computes inertia from the box dimensions
explicitly. TurtleBot4 URDFs include a `<xacro:macro name="box_inertia">`
that does the m/12 × (h² + d²) computation in xacro.
**Recommendation**:
1. Add a xacro macro `<xacro:macro name="box_inertia" params="m h w l">` that emits the correct 3×3 inertia.
2. Apply to chassis with the actual dimensions and the actual measured mass (the 30 kg value should also be verified against a scale).
3. Same treatment for wheels (`wheel.xacro:19-23` currently uses `0.002 / 0.002 / 0.002` for a 2 kg, 0.0625 m radius, 0.05 m wide cylinder — solid cylinder inertia is m/4 × r² + m/12 × h² for ixx/iyy and m/2 × r² for izz, giving roughly **0.00208 / 0.00208 / 0.00391** — close enough that the existing 0.002 is acceptable but the rotation-axis term should be 0.004 not 0.002).
**Acceptance criterion**: `check_urdf` passes and the inertia values match a derivation comment in xacro showing the formula and inputs.
**Effort**: S.
**Prerequisites**: weighing the actual robot.

---

### HIGH-02-04 — ZED depth and IMU pipeline lives in an external package, no config visibility from this repo
**File(s)**:
- `agv_full.launch.py:172-185` — `IncludeLaunchDescription(... agv_slam.launch.py ...)`
- `find /home/user/agv-greenhouse -name "zed*.yaml"` returns nothing.

**Category**: architecture / docs.
**Symptom**: All ZED 2i configuration — depth mode (NEURAL vs ULTRA vs
PERFORMANCE), FPS, resolution, IMU sample rate, positional tracking on/off,
Area Memory path — lives in `agv_slam` (external). From this workspace
alone, there is no way to answer:
- What depth mode does production use?
- What resolution / FPS is the ZED running at?
- Is the ZED SDK's built-in positional tracking enabled (it publishes
  `pose1` in `ekf_global` per `ekf_global.yaml:77`, so something is
  publishing `/agv/zed/pose_with_covariance`)?
- Is `publish_imu_tf: true` actually set? (CLAUDE.md says it must be.)
- What does the IMU sample rate look like? `imu_filter.yaml` assumes 200 Hz.
**Analysis**: The prompt §2.5 explicitly asks: NEURAL depth mode for
robustness to vegetation but expensive on Orin NX 16GB; consider the
trade-off. **This decision cannot be reviewed from this workspace.** The
external `agv_slam` repo holds the answer.
**Greenhouse impact**: depth quality is the single biggest perception
parameter for greenhouse navigation. PERFORMANCE mode is well-known to
miss thin obstacles (manguera Ø2cm at 1.5m); NEURAL_PLUS handles them but
costs 50 % more CPU.
**Recommendation**:
1. **Pull ZED params and the camera-extrinsic static_transform_publisher
   into this repo.** Either vendor `agv_slam` or move the ZED-config-only
   pieces into a new `src/agv_perception/` package in this workspace.
2. **Document the depth mode decision** with a tested trade-off table:
   FPS, CPU%, missed-obstacle rate for NEURAL vs ULTRA vs PERFORMANCE
   on a static test scene. Decision recorded in `docs/adr/0002-zed-depth-mode.md`.
**Acceptance criterion**: From a fresh clone of this repo, an engineer
can answer "what depth mode is the ZED running in production?" by
reading a YAML in `src/`. No external repo needed.
**Effort**: M (pull-and-document) → L (full vendor).
**Prerequisites**: CR-00-01 (repo reproducibility).

---

### MEDIUM-02-05 — CAN bitrate documented as 250 kbps but example boot command says 500 kbps
**File(s)**:
- `specs/project.yaml:69` — `bitrate_kbps: 250`.
- `src/agv_bringup/scripts/agv_start.sh:241` — `echo "  sudo ip link set can0 up type can bitrate 500000"`.

**Category**: docs / bug.
**Symptom**: The spec says 250 kbps; the boot script error message
suggests 500 kbps. One of them is wrong. The actual bitrate used in
production is set by `can-setup.service` (referenced at
`agv_full.launch.py:114` precondition) which lives in `/etc/systemd/`
— **not in this repo**.
**Analysis**: At 50 Hz publish rate × 4 frames per axis × 2 axes = 400
frames/s. At 250 kbps with 8-byte CAN 2.0 frames (~140 bits including
inter-frame), bus utilization ≈ 400 × 140 / 250 000 = 22 %. At 500 kbps
the utilization halves. Both are operational; the discrepancy is purely
between docs/script comments and the real systemd unit.
**Greenhouse impact**: A field technician who follows the error message
to bring up CAN manually will use the wrong bitrate; nodes will not
communicate; the symptom (timeouts on every command) will not point at
the bitrate mismatch.
**Benchmark**: ODrive S1 supports up to 1 Mbps. Many AGVs choose 500 kbps
for headroom; greenhouses with a single AGV don't need it. **Pick one and
align.**
**Recommendation**:
1. **Decide the canonical bitrate** (probably 500 kbps to match the
   working script, but verify with the actual `can-setup.service`).
2. **Update `specs/project.yaml`** to the decided value.
3. **Move `can-setup.service`** under `src/agv_bringup/systemd/` (it
   currently lives at `/etc/systemd/...` per `agv_full.launch.py:114`
   commentary). Bundle into the repo with an install script.
4. **Document in `docs/hardware_setup.md`** with the canonical value
   and a `verify_can_bitrate.sh` script in `scripts/health/`.
**Acceptance criterion**: `ip -details link show can0 | grep bitrate`
output matches the value in `specs/project.yaml`. `agv_start.sh:241`
error message references the same value.
**Effort**: S.
**Prerequisites**: none.

---

### MEDIUM-02-06 — Nav2 params header documents the wrong controller
**File(s)**: `src/agv_navigation/config/nav2_params.yaml:11-12`:
```yaml
# Planner: SmacPlanner2D (flat greenhouse floor)
# Controller: RegulatedPurePursuit (smooth, predictable)
```
But the controller_server config below uses `controller_plugins: ["FollowPath"]` with the MPPI plugin (`nav2_params.yaml:42,108-155`). `specs/project.yaml:92` correctly lists `controller: "MPPI (nav2_mppi_controller::MPPIController)"` and notes "Updated 2026-04-13 audit ... Previous entries (Smac Hybrid-A* / Regulated Pure Pursuit) were stale".
**Category**: docs / drift.
**Symptom**: The Nav2 YAML header lies about which controller is actually configured.
**Analysis**: The 2026-04-13 audit updated `project.yaml` but the YAML
header in `nav2_params.yaml` was not touched. A reader who only opens
`nav2_params.yaml` will believe the controller is RegulatedPurePursuit
and will tune the wrong thing.
**Greenhouse impact**: A field engineer trying to debug MPPI behaviour
will look at the wrong section first.
**Recommendation**: One-line edit — replace `# Controller: RegulatedPurePursuit (smooth, predictable)` with `# Controller: MPPI (nav2_mppi_controller::MPPIController)` and update the planning section comment if appropriate.
**Acceptance criterion**: The YAML header is true.
**Effort**: S (literal one-line change).
**Prerequisites**: none.

---

## C. ODrive driver — what's good, what to watch

### C.1 Strengths (worth preserving across any cutover)

Strengths visible from `agv_odrive/src/odrive_can_node.cpp` and
`odrive_params.yaml` and `agv_odrive/CLAUDE.md`:

- **Asymmetric accel/decel** (`max_wheel_accel 0.5` vs `max_wheel_decel
  1.5` turns/s²) — gentler acceleration to reduce caster slip, faster
  decel for safety. Documented and rationalized.
- **Pure rotation gate** — when wheels spin opposite at similar
  magnitude, encodes a hard zero translation. Eliminates phantom
  translation in odometry. Standard greenhouse-floor robustness move.
- **EMA velocity filter (α=0.3)** at ~8 Hz cutoff — smooths encoder
  noise before twist and rotation detection.
- **Slip detection** at the encoder level (`slip_velocity_threshold 0.5`),
  reducing cmd_vel by 0.7× during slip with a 200 ms cooldown.
- **Caster compensation** that inflates odometry covariance during
  direction changes and sustained rotation — signaling the dual-EKF to
  shift weight to IMU. This is a *correct* design (the
  `wheel_slip_detector_node` is the second layer, working at the topic
  boundary as the doc explains).
- **Parameter validation in C++** (`if (wheel_radius_ <= 0.0) { FATAL }`)
  — the node refuses to start with bad values.
- **`cmd_vel_timeout_ms: 200`** — node-local watchdog stops motors if
  no command for 200 ms.

These are the engineering investments that any ros2_control migration
must preserve.

### C.2 Watch items

| Item | Notes |
|---|---|
| **No CAN retry / backoff** | `agv_odrive/CLAUDE.md` improvement note. If CAN fails (e.g., transient noise from a humid plug), the node spams errors. Recommend exponential backoff + a `/diagnostics` WARN at first failure, ERROR after N retries. |
| **No motor temperature shutdown** | Temps published but not monitored. ODrive S1 motors are rated to 90 °C continuous. Recommend a software thermal shutdown at 85 °C with warning at 75 °C. Surface to dashboard. |
| **No DC bus voltage publish** | The `agv_battery` topic in `interfaces.yaml` is `planned` — the ODrive can read VBUS via CAN `GET_VBUS_VOLTAGE` (cmd 0x017). Add it. Surface to dashboard as battery indicator. |
| **`zero_vel_epsilon: 0.03 turns/s`** | Below this, all shaping is bypassed and exact zero is sent. At wheel_radius 0.0781 and gear_ratio 10.0 this is 0.03 / 10 × 2π × 0.0781 ≈ 0.0015 m/s. Reasonable. |
| **`gear_ratio: 10.0` in YAML, 1.0 in C++ default** | If the ODrive firmware is *already* configured with gear_ratio 10.0 in its NVRAM (per the CLAUDE.md comment "Set to 1.0 if ODrive firmware already has gear_ratio configured"), then `10.0` in the YAML is **double-counting** and motors are commanded at 10× the intended velocity. **This is testable and should be verified before next field run.** |

Item #5 is the most suspicious of the four. Worth tagging separately:

---

### HIGH-02-07 — `gear_ratio: 10.0` may be double-counted if ODrive firmware also has it set
**File(s)**:
- `src/agv_odrive/config/odrive_params.yaml:9` — `gear_ratio: 10.0`.
- `src/agv_odrive/src/odrive_can_node.cpp:28-31` comment — "Set to 1.0 if ODrive firmware already has gear_ratio configured."

**Category**: bug.
**Symptom**: The C++ comment explicitly warns about this case but there
is no verifier or runtime check that catches the wrong combination.
**Analysis**: The ODrive S1 has a configurable gear_ratio in its NVRAM
(set via the ODrive Tool / CLI). If that NVRAM value is 10.0, then
ODrive itself converts motor turns to wheel turns. The ROS driver
multiplying by another 10.0 results in commanded motor velocity 10× the
intended wheel velocity. Either:
- Motors saturate at velocity_limit and the robot moves at velocity_limit
  / 10 (might *look* normal, masking the bug entirely),
- OR motors spin 10× faster and the robot crashes immediately at first
  command.
The 2026-04-08 calibration "ratio 2.66/2.13 from 3-trial test" produced
the wheel_radius 0.0781 value — this is a *measured* commanded-distance /
expected-distance ratio. If the measurement showed the robot moving the
"right" distance at the current configuration, then either the gear
ratio in firmware is 1.0 (and 10.0 in ROS is correct) OR the gear ratio
in firmware is 10.0 (and 10.0 in ROS is correct only because the
calibration absorbed a 10× error into the radius — which would explain
why the calibrated radius is 25 % higher than the geometric measurement).
**Greenhouse impact**: If the second scenario is real, the "calibrated"
values are masking a deeper configuration error. Any change to the
ODrive firmware (gear_ratio reset, motor swap, firmware update) will
re-expose the bug catastrophically.
**Benchmark**: ODrive's own documentation: keep gear_ratio in firmware
unless the upstream client explicitly needs it. Most production ROS
drivers set gear_ratio in firmware and use 1.0 in ROS.
**Recommendation**:
1. **Read the live ODrive firmware config** at first boot:
   `odrivetool` or CAN `GET_*` reads. Log to `events.jsonl`.
2. **Add a startup invariant** in `odrive_can_node`: if firmware reports
   `motor.gear_ratio != 1.0` AND ROS `gear_ratio != 1.0`, refuse to
   start. Document.
3. **Re-run the calibration** with the canonical configuration (firmware
   = 10.0 + ROS = 1.0 OR firmware = 1.0 + ROS = 10.0, decided
   explicitly). Update `wheel_radius` and `track_width` to the values
   the calibration produces with the canonical configuration.
**Acceptance criterion**: A documented ADR (`docs/adr/0003-odrive-gear-ratio-source.md`)
states whether firmware or ROS owns the gear ratio. A boot-time check
asserts the other side is set to 1.0.
**Effort**: S (boot check + ADR; calibration is M but separate).
**Prerequisites**: none.

---

## D. Hardware-dependent gates deferred

| Acceptance criterion (per prompt §4 Phase 2) | Hardware needed | Proposed harness |
|---|---|---|
| `ros2 control list_hardware_interfaces` shows joints ACTIVE | Production ros2_control cutover | Post-MVP |
| Command → motion latency < 50 ms p99 | Jetson + ODrive + caliper-instrumented run | `tests/hal/latency.py` (to write) |
| Watchdog test: kill controller_manager → motors stop < 100 ms | Hardware | `tests/hal/watchdog.py` (to write) |
| ZED `/agv/zed/depth/depth_registered` ≥ 15 Hz sustained 30 min, no drop > 100 ms | Hardware | `tests/hal/zed_depth_endurance.py` (to write) |
| ZED USB recovery < 5 s | Hardware (with USB unplug capability) | Manual test, document in RUNBOOK.md |

---

## E. Status

| Item | Status |
|---|---|
| ros2_control alternative path analysis | ✅ filed `HIGH-02-01` (recommendation: defer cutover, keep CI active) |
| Geometry SSOT deep dive | ✅ filed `CRITICAL-02-02` |
| URDF inertia | ✅ filed `HIGH-02-03` |
| ZED config externalization | ✅ filed `HIGH-02-04` |
| CAN bitrate inconsistency | ✅ filed `MEDIUM-02-05` |
| Nav2 controller header drift | ✅ filed `MEDIUM-02-06` |
| Gear ratio double-counting risk | ✅ filed `HIGH-02-07` |
| Hardware-dependent gates | ⏸ deferred (table D) |

End of Phase 2.
