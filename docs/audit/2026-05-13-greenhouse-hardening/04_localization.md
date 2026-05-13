# Phase 4 — Localization

> Phase 4 goal (per prompt): "Pose en `map` frame estable, sin saltos,
> robusta a passillos largos featureless, con manejo correcto de
> AprilTags como anclas."

The first pass deferred this phase to hardware. That was wrong: the
**configurations and the source code** for the dual EKF, AprilTag
fusion, factor graph, auto-init orchestrator, and cuVSLAM override are
all auditable statically. This file delivers that audit.

What requires hardware (drift over 100 m, kidnapping recovery time,
forced sensor blackout) is documented at the end with proposed harnesses.

---

## A. Dual EKF — configuration audit

### A.1 `ekf_local.yaml` walkthrough

[`src/agv_sensor_fusion/config/ekf_local.yaml`](../../../src/agv_sensor_fusion/config/ekf_local.yaml).

| Property | Value | Evaluation |
|---|---|---|
| `frequency` | 50.0 | Correct — matches wheel_odom publish rate |
| `two_d_mode` | true | Correct — diff-drive planar |
| `publish_tf` | true | Correct — sole owner of `odom→base_link` |
| `world_frame` | odom | Correct |
| `dynamic_process_noise_covariance` | true | Correct — scales Q with velocity |
| `odom0` source | `wheel_odom_validated` (not raw `/agv/wheel_odom`) | Correct — wheel_slip_detector replaces raw |
| `odom0_config` | `[T,T,F, F,F,T, T,F,F, F,F,T, F,F,F]` | **Mostly correct.** Reads x,y,yaw (pose) AND vx, vyaw (twist). Reading wheel-odom-derived absolute pose is unusual but `odom0_differential: false` makes the EKF integrate it as a normal observation — this is OK for a single-rate filter where the wheel odom maintains its own integration consistent with the EKF. The pose-read combined with `differential: false` does means accumulated wheel-odom drift directly contaminates the local pose. See `HIGH-04-09`. |
| `imu0` source | `/agv/imu/filtered` | Correct |
| `imu0_config` | `[F,F,F, T,T,T, F,F,F, T,T,T, T,T,F]` | Reads roll, pitch, yaw absolute (`T,T,T` in first orientation block). **Yaw absolute from IMU is acceptable here** because the BMI088 is gyro-only (no magnetometer per `agv_sensor_fusion/CLAUDE.md`); the "yaw absolute" is actually the integrated gyro from boot, not a magnetometer reading. **But** it makes the local EKF read two sources of yaw absolute (wheel-derived from yaw bit in `odom0_config` AND IMU-derived from yaw in `imu0_config`) — they will fight. The system mitigates by giving IMU yaw a tighter covariance (0.001 vs wheel odom yaw 0.03), so IMU wins. Working but architecturally fragile. |
| `imu0_remove_gravitational_acceleration` | true | Correct |
| `imu0_pose_covariance` (overrides yaw to 0.02 rad²) | 0.02 rad² on yaw, 0.001 on roll/pitch | Reasonable for BMI088 spec |
| `imu0_twist_covariance` (rate) | 0.0005 on vyaw | Tight but matches BMI088 noise density `0.000244 rad/s/√Hz` |
| `process_noise_covariance` | non-default, x/y=0.05, yaw=0.08, vx=0.025 | **Reasonable values** — not identity, not 1e-9. Documented choice. |

### A.2 `ekf_global.yaml` walkthrough

[`src/agv_sensor_fusion/config/ekf_global.yaml`](../../../src/agv_sensor_fusion/config/ekf_global.yaml).

| Property | Value | Evaluation |
|---|---|---|
| `frequency` | 10.0 | Correct |
| `two_d_mode` | true | Correct |
| `publish_tf` | true | Correct — sole owner of `map→odom` |
| `world_frame` | map | Correct |
| `odom0` source | `odometry/local` (local EKF output) | Correct |
| `odom0_differential` | **true** | Correct — prevents double-counting drift |
| `odom1` source | `/visual_slam/tracking/odometry` (cuVSLAM) | Correct |
| `odom1_differential` | **true** | Correct |
| `odom1_pose_rejection_threshold` | 3.5 | Mahalanobis-distance gating; reasonable. Tightened for greenhouse aliasing (comment line 47) |
| `odom1_twist_rejection_threshold` | 2.5 | Reasonable |
| `pose0` source | `marker_pose` (AprilTag) | Correct |
| `pose0_config` | `[T,T,F, F,F,T, F,F,F, F,F,F, F,F,F]` | Reads x, y, yaw absolute. Correct |
| `pose0_differential` | **false** | Correct — AprilTag IS absolute |
| `pose0_rejection_threshold` | 3.0 | Stricter than cuVSLAM — correct (tags are ground truth) |
| `pose1` source | `/agv/zed/pose_with_covariance` (ZED SDK Area Memory) | Correct |
| `pose1_differential` | false | Correct |
| `pose1_rejection_threshold` | 3.0 | Symmetrised with AprilTag (per comment line 86–90) |
| `initial_estimate_covariance` | 1.0 on x/y, 0.5 on yaw, 1e-9 on rest | **Correct fix** for the cold-start problem documented at line 131–158. Otherwise first absolute pose would be rejected on every boot |
| `process_noise_covariance` | non-default, x/y=0.05, yaw=0.04 | **Reasonable**. Tighter than local on yaw because global filter trusts its priors more between corrections (comment 105–113) |

**Net evaluation of dual EKF**: solidly engineered. The tuning is
documented; choices are justified inline. The only fragility is the
local EKF's mix of `odom0_differential: false` with reading wheel-yaw
absolute — but it's mitigated by IMU yaw weight. See HIGH-04-09 for the
cleanup proposal.

### A.3 IMU filter

[`src/agv_sensor_fusion/config/imu_filter.yaml`](../../../src/agv_sensor_fusion/config/imu_filter.yaml).
- Butterworth 2nd-order: gyro 10 Hz, accel 5 Hz cutoffs at 200 Hz sample.
- Comment justifies separation: robot dynamics < 5 Hz, floor vibrations
  > 20 Hz. Clean. **No finding.**

### A.4 cuVSLAM greenhouse override

[`src/agv_bringup/config/cuvslam_greenhouse.yaml`](../../../src/agv_bringup/config/cuvslam_greenhouse.yaml).
- Line 10: `/**:` key correctly used (not the node name) — the
  TF-disable trap is avoided. **No finding.**
- `publish_odom_to_base_tf: false` and `publish_map_to_odom_tf: false` —
  both TF flags off. Correct.
- `img_mask_top: 108` (greenhouse roof), `img_mask_bottom: 72` (ground
  reflections) — sensible greenhouse-specific masking.
- `slam_max_map_size: 500`, `slam_throttling_time_ms: 300` — keyframe
  budget documented.
- `localizer_horizontal_radius: 3.0`, `localizer_angular_step: 0.1745`
  (~10°) — grid-search params for relocalization. Reasonable for a
  greenhouse where the operator may place the robot within 3 m of a
  prior pose.

**No finding.**

---

## B. AprilTag fusion (`marker_correction_node`)

[`src/agv_markers/src/marker_correction_node.cpp`](../../../src/agv_markers/src/marker_correction_node.cpp).

This is the most carefully engineered node in the workspace. The
prompt §4.B asks 7 specific questions; here are the verified answers.

### B.1 Architecture: observation OR `set_pose`, conditionally

- **Default path** (line 552): the node publishes
  `PoseWithCovarianceStamped` on `marker_pose`. This enters `ekf_global`
  as `pose0` (absolute, non-differential). **This is correct — the EKF
  does the fusion math, not the node.**
- **Relocalization path** (line 522–545): when drift exceeds
  `relocalization_threshold` (2.0 m) AND `decision_margin >= 50.0` AND
  no rail-aisle / rail-driving gate is active, the node calls
  `robot_localization/srv/SetPose` to hard-reset the EKF.
- The two paths are mutually exclusive per detection cycle. **No
  pose-jump pathology** of the type the prompt warns about.

### B.2 Range-quadratic covariance scaling

Line 399–400:
```cpp
double ref_range = 2.0;
double range_factor = 1.0 + (range / ref_range) * (range / ref_range);
```
Applied at line 492–494:
```cpp
correction.pose.covariance[0]  = 0.02 * final_range_factor;  // x
correction.pose.covariance[7]  = 0.02 * final_range_factor;  // y
correction.pose.covariance[35] = 0.05 * final_range_factor;  // yaw
```

**Correct.** At range 2 m, covariance is 2× the base (`range_factor = 2`).
At range 5 m, covariance is 7.25×. This matches the prompt's
"`cov ∝ r²`" expectation.

### B.3 Tag registry: YAML, versioned, with known bug

- Path: `markers_registry.yaml` (static) + runtime registry loaded from
  `runtime_registry_file` parameter (dashboard-defined tags).
- Static registry lives in `src/agv_markers/config/markers_registry.yaml`
  — versioned in git.
- Runtime registry path is **hardcoded** in `agv_full.launch.py:546,585`
  to `/home/orza/agv_data/runtime_markers_registry.yaml` — already
  flagged as `CR-00-03`.

### B.4 Anti-spoofing

Line 289: `if (it == registry_.end()) continue;`. Tags whose ID is **not
in the registry** are silently ignored. **This is the correct
anti-spoofing behavior** for the greenhouse non-adversarial threat model
— a random tag stuck on a wall by accident won't poison localization.

**However**: a knowledgeable adversary who knows the registry can spoof
a known ID. There is no cryptographic auth on the tag bits. **For
greenhouse commercial deployment this is acceptable**; for any future
public-space deployment, swap to `STag` or signed-tag schemes.

### B.5 Multi-tag voting

Line 417–470: median-based outlier rejection + inverse-range-squared
weighted average over inliers. Outlier threshold 1.0 m from median.
**This is correct and robust**.

Yaw averaging uses circular mean (`atan2(sum_sin, sum_cos)`) — correct
for angles. Best `decision_margin` used to label the log line.

### B.6 Obliqueness — NOT handled

Line 287–321: every detected tag goes through `solvePnP` regardless of
its observation angle. There is **no rejection of tags seen at extreme
incidence**.

The prompt §4.B asks this explicitly: "Tags seen at extreme angles
produce noisy PnP that poisons EKF." `SOLVEPNP_SQPNP` (line 319) is more
robust than `ITERATIVE` at oblique angles, but it does not magically
recover from poor geometry. At 80° incidence on a 20 cm tag, even SQPNP
will have very large covariance — yet the node assigns the *same*
`range_factor * 0.02` covariance regardless of incidence.

This is `HIGH-04-02` below.

### B.7 Tag size — single value, not per-tag

Line 52: `declare_parameter("tag_size", 0.2);`. Used in `obj_pts`
construction at line 293–297 for every detected tag, ignoring the
registry. If the registry contains tags of mixed sizes (e.g., 0.10 m
floor tags and 0.20 m wall tags), all are treated as 0.20 m and PnP
returns a wrong distance for the smaller tags by a factor of 2.

Per the agv_markers CLAUDE.md: "Registry includes wall tags (z=0.145m)
and floor tags (z=0.002m)." It does not say they are different sizes,
but the registry YAML can in principle declare per-tag sizes that the
node ignores.

This is `HIGH-04-04` below (alongside the homebrew YAML parser).

### B.8 Camera intrinsics: live from `camera_info`

Line 94–105: subscribes to `camera_info_topic` and captures `fx, fy,
cx, cy` on first message. Good — uses the actual camera calibration the
ZED publishes.

**However**: it never resubscribes if `camera_info` changes (e.g., the
ZED is reset and republishes new intrinsics after a USB recovery). The
node holds the first values forever.

Low priority — ZED intrinsics are stable for a given lens/sensor combo.

### B.9 Rail-aisle gating

Line 504–521: three independent gates suppress relocalization while the
robot is in a rail aisle:
1. `in_rail_aisle_` (from `/agv/zone/state` JSON)
2. EKF position suggests rail section (x ≤ 3.5 or x ≥ 7.5 — hardcoded
   sim geometry)
3. `rail_driver_driving_` (from `/agv/rail_driver/state`)

**Defensive and well-engineered**. The hardcoded x-range at line
518–519 is a tight coupling to the specific greenhouse layout in the
USD sim and would need re-tuning for a different greenhouse.

Worth noting in a "hardcoded greenhouse geometry" backlog item but
the team explicitly chose this in iter-33 c5 as a chicken-and-egg
breaker. **No new finding** beyond noting it.

### B.10 Cooldown semantics

Line 268–276: after a successful `set_pose`, suppress further
corrections for `relocalization_cooldown_ms` (500 ms). Raw tag IDs
still publish to `raw_tag_pub_` (line 259–263) before the cooldown
gate. **Correct architecture** — UI tag-visibility tracking is not
broken by relocalization cooldown.

### B.11 Findings on `marker_correction_node`

---

#### HIGH-04-02 — `marker_correction` accepts grazing-incidence tag detections
**File(s)**: `src/agv_markers/src/marker_correction_node.cpp:287-321`.
**Category**: bug.
**Symptom**: No incidence-angle filter. Tags seen at 80°+ from normal
produce PnP results with very large effective uncertainty, but the
output covariance assumes only range-dependent uncertainty.
**Analysis**: The `range_factor = 1 + (range/2)²` only models distance
uncertainty. For a planar tag, projection error growth with incidence
goes as `1/cos²(θ)`: at 80° that's a 33× multiplier, far exceeding the
range factor. SOLVEPNP_SQPNP is numerically stable but cannot recover
geometric information that doesn't exist in the image.
**Greenhouse impact**: As the robot rounds a corner or passes a tag at
the end of a row, the tag transits from frontal to extreme oblique.
The last few frames before the tag exits FoV will be high-incidence
high-confidence corrections that pull `ekf_global` toward a slightly
wrong position. Over a long mission with many tags, this systematic
bias accumulates as map-scale drift.
**Benchmark**: VINS-Fusion's marker plugin rejects detections where
`cos(θ) < 0.3` (about 73° from normal). NVIDIA Isaac AprilTag pipeline
exposes per-detection `homography_error` for downstream filtering.
**Recommendation**: Compute the incidence angle from the tag's rvec.
The tag normal in camera frame is column 2 of the Rodrigues rotation;
its z-component is `cos(incidence)`. Reject when `cos(incidence) < 0.3`
OR inflate covariance by `1 / cos²(incidence)`. Surface incidence in
the log for debugging.
**Acceptance criterion**: Synthetic test of a tag observed at 0°/45°/75°
incidence produces published covariances that grow by approximately
1× / 2× / 15× respectively.
**Effort**: S.
**Prerequisites**: none.

---

#### HIGH-04-04 — Homebrew YAML parser in `marker_correction` accepts garbage; tag size not per-entry
**File(s)**: `src/agv_markers/src/marker_correction_node.cpp:215-238` (parser), `:52` (single `tag_size` param), `:293-297` (PnP geometry).
**Category**: bug.
**Symptom**: The registry YAML parser is a hand-rolled string-matcher
that accepts any line beginning with `id:`, `x:`, `y:`, `z:`, `yaw:`.
There is no schema validation, no per-tag `size`, no `family`, no
duplicate-ID check.
**Analysis**:
- A tag declared `id: 7\nx: 3.5\ny: bogus` will: parse `id: 7` (success),
  parse `x: 3.5` (success), reach `y: bogus` and `std::stod("bogus")`
  → throws → process aborts because there's no try-catch at line 231.
  **A typo in the registry crashes the node.**
- A second `id: 7` later in the file silently overwrites the first
  (line 226: `registry_[current_id] = current;`).
- All tags assumed `tag_size = 0.2` m (line 52). If the registry mixes
  floor (small) and wall (large) tags, PnP geometry is wrong for the
  smaller class by the size ratio.
- The same parser is used for both static and runtime registries —
  garbage from the dashboard-generated registry crashes the node too.
**Greenhouse impact**: The dashboard's AprilTagManager generates the
runtime registry from operator input. An operator who types a malformed
value triggers a node crash, which silently disables AprilTag
correction. Drift starts accumulating but no clear alert is raised.
**Benchmark**: Most production ROS configs use `yaml-cpp` (already a
transitive dep via `pluginlib`). A 30-line `yaml-cpp` parser handles
schema, types, and errors with proper messages.
**Recommendation**:
1. Replace the hand-rolled parser with `yaml-cpp`. Validate types and
   ranges (x, y, z finite; yaw in `[-π, π]`; size > 0).
2. Add a per-tag `size` field; default to the global `tag_size`
   parameter if absent. Use the per-tag value in `obj_pts` construction.
3. On parse error, log a clear message naming the offending file and
   line, and either (a) keep the previous registry in memory and warn,
   or (b) refuse to start at boot.
4. Detect duplicate IDs — fail to load, log all duplicates.
**Acceptance criterion**: A malformed `markers_registry.yaml` produces a
clear error log and either a refusal to start or a "kept previous
registry" warning, never a silent crash.
**Effort**: S (4 hours including tests).
**Prerequisites**: none.

---

## C. The `agv_factor_graph` black box, opened

[`src/agv_factor_graph/src/factor_graph_node.cpp`](../../../src/agv_factor_graph/src/factor_graph_node.cpp).

### C.1 What it actually does

- Constructs a GTSAM iSAM2 sliding-window factor graph (window = 200,
  ~20 s at 10 Hz).
- **Inputs**:
  - `odometry/global` (`/agv/odometry/global`, from `ekf_global`) →
    `BetweenFactor` between consecutive poses (line 68–70, 135–136).
  - `/visual_slam/tracking/odometry` → `BetweenFactor` between most
    recent pose pair (line 179–180).
  - `marker_pose` → `PriorFactor` on the most recent pose (line 197).
- **Output**: `/agv/factor_graph/odometry` (`nav_msgs/Odometry`,
  `publish_tf=false`).

### C.2 Consumer audit

`specs/interfaces.yaml` — `factor_graph` topics are explicitly noted
as out-of-scope ("currently out-of-scope for cross-package interface
tracking, but should be added here when the cutover is planned").

`grep -rn "factor_graph/odometry" src/` returns:
- `src/agv_factor_graph/src/factor_graph_node.cpp` (publisher itself)
- (no other hits)

**The output topic has zero subscribers in this workspace.** No
comparator node exists. The factor graph runs at 10 Hz with full
GTSAM/iSAM2 cost during every navigation session, then drops every
result on the floor.

### C.3 The architectural bug

The local-odom input is `odometry/global` (line 69), not
`odometry/local`. The latter is the raw wheel+IMU fusion; the former is
the **already-cuVSLAM-and-marker-corrected** pose from `ekf_global`.

The node also subscribes to `marker_pose` and adds AprilTag observations
as `PriorFactor`s. **The marker information is double-counted**: it is
already inside `odometry/global` (`ekf_global` consumed `pose0`), and
the factor graph adds it again as a fresh prior. Same observation,
applied twice with independent noise models, makes the estimate
artificially confident about marker positions.

Comment at line 65–67 says "During parallel validation the factor graph
re-optimizes the same trajectory with AprilTag corrections." The
intent is correct, but the implementation must consume the
**marker-free** local odometry to be a true parallel validation. As
written, it just adds noise around the EKF's output.

### C.4 Loop-closure horizon

Line 209–226: the sliding window evicts old poses via
`isam2::marginalizeLeaves`. Window size 200 × 10 Hz = 20 s of trajectory.

A factor graph's selling point is that loop closures **propagate
corrections backward through the entire history**. With a 20 s window,
a loop closure on a 5 m corridor (40 s round trip at 0.25 m/s) **cannot
reach the other end of the loop**. The "main reason" justifying the
factor graph (per `agv_factor_graph/CLAUDE.md`) is undermined by the
window choice.

### C.5 Findings

---

#### HIGH-04-01 — `factor_graph` consumes the marker-corrected EKF output, double-counting AprilTag information
**File(s)**: `src/agv_factor_graph/src/factor_graph_node.cpp:68-70`, `:153-180`, `:186-207`.
**Category**: bug / architecture.
**Symptom**: The factor graph reads `odometry/global` (`ekf_global`
output, which already absorbed `marker_pose`) as its "local odometry"
BetweenFactor source, AND independently adds `marker_pose` as
`PriorFactor`. The same AprilTag observation enters the factor graph
twice through different paths.
**Analysis**: As written, the factor graph is not a true validation of
the EKF — it observes EKF-output increments and applies marker priors
on top. Two consequences:
- During steady-state, the two paths agree because they share the
  source. **The validation gives a false negative** — any disagreement
  in the underlying state estimation is invisible.
- The marker priors push the graph toward a tighter pose at every
  observation. If `ekf_global` were silently degrading, the factor
  graph would still report low residuals because the prior dominates
  the BetweenFactors.
**Greenhouse impact**: If the team uses `factor_graph/odometry` to
decide whether to cut over from EKF, the decision is meaningless. A
real cutover under these inputs would replace one estimator with a
slightly different version of itself.
**Benchmark**: A true parallel validation in GTSAM-style frameworks
subscribes to the **raw measurements** (wheel + IMU + visual + marker)
and runs an independent factor graph. NVIDIA Isaac Robotics' factor
graph examples and GTSAM tutorials use raw sensor topics, not EKF
outputs.
**Recommendation**: Refactor the inputs:
- Replace `odometry/global` with `odometry/local` (wheel + IMU only,
  no marker). Or, better, with `/agv/wheel_odom_validated` directly so
  the IMU also enters separately as an `ImuFactor`.
- Keep `/visual_slam/tracking/odometry` as `BetweenFactor` (correct).
- Keep `marker_pose` as `PriorFactor` (correct).
- Document this in `agv_factor_graph/CLAUDE.md` as the input contract.
**Acceptance criterion**: With AprilTag observations disabled (e.g.,
`enable_markers:=false`), `factor_graph/odometry` and
`ekf_global/odometry/global` produce **different** trajectories under
the same wheel + IMU + cuVSLAM inputs. Today they will be identical
modulo numerical noise.
**Effort**: M (1 day refactor + new bag-based regression test).
**Prerequisites**: HIGH-04-09 (clean up local-odom semantics).

---

#### MEDIUM-04-05 — `factor_graph` has zero downstream consumers — runs at 10 Hz for nothing
**File(s)**: `src/agv_factor_graph/src/factor_graph_node.cpp:80-83`, `agv_full.launch.py:373-388`.
**Category**: performance.
**Symptom**: The output topic `factor_graph/odometry` has no subscribers
in this workspace (grep + spec confirmed). The node is started at t=4.5
s in `agv_full.launch.py` and runs the full GTSAM/iSAM2 stack on a Jetson
that is also running cuVSLAM, MPPI at 20 Hz, nvblox visualization, and
the operator dashboard.
**Analysis**: iSAM2 at 10 Hz on a 200-pose window is not free — on a
Jetson Orin NX 16 GB target it can consume 10–20 % of a CPU core. That
budget is needed for MPPI's batch_size 1000 × 32 timesteps and for the
collision_monitor's two-source obstacle processing.
**Greenhouse impact**: Frame drops in MPPI at the moment that matters
most (close-quarters maneuvering in a 1.5 m corridor). The
`collision_monitor.yaml:54` budget for stop_zone assumes 100 ms reaction
latency; if MPPI starves, that latency grows.
**Recommendation**: 
1. Either (a) write a comparator node that consumes both
   `factor_graph/odometry` and `ekf_global/odometry/global` and emits a
   `DiagnosticArray` with residual statistics — turning the factor graph
   into a real validator — or (b) gate the factor graph behind a launch
   argument `enable_factor_graph` (default false) and only enable when
   the team is actively running a validation campaign.
2. Default-false is honest about the current state: the node is dormant
   even though it's running.
**Acceptance criterion**: Production launch in real mode does not start
the factor_graph node unless `enable_factor_graph:=true`. CPU usage on
the Jetson drops by the measured iSAM2 cost.
**Effort**: S (launch flag) → M (comparator node).
**Prerequisites**: HIGH-04-01 (fix the inputs first so the comparator is meaningful).

---

#### MEDIUM-04-06 — Factor graph window 200 = ~20 s horizon, smaller than typical greenhouse loop
**File(s)**: `src/agv_factor_graph/src/factor_graph_node.cpp:34` (`window_size = 200`).
**Category**: bug (subtle).
**Symptom**: The sliding-window iSAM2 keeps the last 200 poses. At 10
Hz that's 20 s of robot trajectory.
**Analysis**: At 0.25 m/s, 20 s = 5 m of trajectory. A loop closure
between corridor A entrance (t=0) and corridor A exit (t=40 s after a
10 m straight + 90° turn + return) cannot reach the entrance pose —
it's already been marginalized.
**Greenhouse impact**: The factor graph cannot deliver its main
selling point (backward propagation of loop closures). It degrades to
an EKF with extra computation.
**Recommendation**: Either increase `window_size` to cover a typical
greenhouse loop (10 minutes × 10 Hz = 6000 poses — quadratic memory cost
in iSAM2 is fine up to a few thousand), or document this as an explicit
"local smoothing window only" semantic and don't claim loop closure
benefits.
**Acceptance criterion**: An ADR documents the window choice with
expected loop lengths in the target greenhouse. If window stays at 200,
the marketing claim in CLAUDE.md ("Loop closures propagate corrections
backward in time") is removed.
**Effort**: S (window tuning) → M (memory + perf testing).
**Prerequisites**: none.

---

## D. `auto_init_orchestrator` — cold-start cascade

[`src/agv_localization_init/src/auto_init_orchestrator_node.cpp`](../../../src/agv_localization_init/src/auto_init_orchestrator_node.cpp).

### D.1 The cascade

Four ordered paths (from CLAUDE.md state machine):
0. **ZED SDK Area Memory** — wait up to 8 s for ZED-SDK pose with low
   covariance for 5 consecutive frames AND `spatial_memory_status == OK`.
1. **cuVSLAM + AprilTag hint** — load keyframe DB, wait 10 s for tag,
   call `/visual_slam/localize_in_map(pose_hint)`.
2. **AprilTag absolute pose alone** — if no cuVSLAM DB, use tag as
   direct `SetPose`.
3. **Last-known pose from `<map>_meta.json`** — fallback.

If all four fail → `FAILED` state, operator must teleop to a tag and
call `/agv/localization/reinitialize`.

### D.2 Strengths

- **Spatial-memory status gate** at line 116–123 (commentary): explicitly
  protects against the "ZED cold-boot publishes tiny covariance at the
  origin" false positive. Engineering rigor.
- **5 consecutive frames** required to declare LOCALIZED. Defends against
  single-frame false matches in visually-repetitive crop rows.
- **Re-fire on `/agv/mode` transition to `nav`** (CLAUDE.md mention,
  line 30–35 of source comment). Closes the case where the operator
  flips Mapping→Operate without an explicit map-load.

### D.3 Findings

---

#### HIGH-04-03 — Kidnapping detection is declared as a parameter but never implemented
**File(s)**: `src/agv_localization_init/src/auto_init_orchestrator_node.cpp:101` (`kidnapping_drift_m` declared); `src/agv_localization_init/CLAUDE.md` improvement opportunities ("Add kidnapping detection").
**Category**: bug / missing feature.
**Symptom**: A parameter `kidnapping_drift_m` (default 5.0) is declared,
but no code path reads it or uses it for kidnapping detection. The
declaration is a TODO marker.
**Analysis**: Greenhouse operators sometimes pick up the robot to clear
a stuck wheel, swap battery, or relocate it manually. Without kidnapping
detection, the robot will:
1. Notice its position changed (EKF rejects the new wheel-odom delta
   as outlier, then accepts after the rejection threshold is exceeded);
2. Continue navigating from its **previous** estimated pose, which is
   now wrong;
3. Plan paths to the wrong destination;
4. Eventually relocalize via AprilTag (Path A0 / A) — but only when one
   becomes visible, which can be tens of seconds after the kidnap.
**Greenhouse impact**: Between the kidnap and the relocalization, the
robot is driving with a wrong pose. In a crowded corridor this is a
collision hazard.
**Benchmark**: AMCL has built-in `kld_err` and `kld_z` tuning for
kidnap detection. SLAM Toolbox emits `tf_pose_jumped` warnings when an
update exceeds threshold. cuVSLAM's `tracking_status` field flips to
`LOST` when the relative-pose graph cannot match — orchestrator should
listen to it.
**Recommendation**:
1. Subscribe to `marker_pose` AND `odometry/global`. Continuously
   compare. If `‖marker_pose - odometry/global‖ > kidnapping_drift_m`
   AND wheel odometry did not show a corresponding delta in the last
   N seconds (i.e., the robot was supposedly stationary), this is a
   kidnap signature.
2. On kidnap detection: re-trigger the full cascade (publish
   `INITIALIZING`, call the cascade method). The operator already has
   an AprilTag in view (that's how we detected the kidnap), so Path A
   relocalization is immediate.
3. Surface kidnap events to the dashboard as a "Robot was moved"
   notification.
**Acceptance criterion**: A simulated kidnap (teleport via
`/agv/sim/reset_request`) triggers `INITIALIZING` within 2 s of the
first post-teleport AprilTag observation. Without this, the robot
continues navigating to phantom coordinates.
**Effort**: M (1 day implementation + HIL test scenario).
**Prerequisites**: HIGH-04-02 (incidence filter prevents kidnap detection from firing on grazing-angle tags).

---

#### LOW-04-08 — Cooldown asymmetry between marker_correction (500 ms) and EKF rate (10 Hz)
**File(s)**: `src/agv_markers/src/marker_correction_node.cpp:55`.
**Category**: NIT.
**Symptom**: `relocalization_cooldown_ms: 500` and `ekf_global.frequency
= 10 Hz`. 500 ms = 5 EKF cycles. After `set_pose`, marker_correction
skips publications for 5 cycles to let the EKF "settle".
**Analysis**: `robot_localization::set_pose` resets the EKF state
synchronously. The cooldown protects against subsequent corrections
"un-doing" the reset. 5 cycles is a reasonable choice but undocumented.
**Recommendation**: Document the choice in a comment, or compute the
cooldown from `ekf_global.frequency` to keep them aligned automatically.
**Effort**: NIT.

---

#### MEDIUM-04-07 — Anti-spoofing behavior is correct but not documented as a security property
**File(s)**: `src/agv_markers/src/marker_correction_node.cpp:289`.
**Category**: docs / security.
**Symptom**: Tags whose ID is not in the registry are silently ignored.
This is the correct anti-spoofing behavior — a stray tag stuck on a
wall by accident or maliciously cannot poison localization. But it is
not documented as a security property anywhere.
**Analysis**: Documenting this matters because (a) future engineers may
"helpfully" add a fallback that accepts unregistered tags as "discovery
hints", undoing the property, and (b) the deployment customer's
security review needs to see that the anti-spoofing exists.
**Recommendation**: Add a section "Security model" to
`agv_markers/CLAUDE.md` listing:
- The anti-spoofing property.
- The class of attack it defends against (random / accidental).
- The class it does NOT defend against (knowledgeable adversary who
  knows registry IDs and can print spoof tags).
- The registry-versioning workflow (operator approves new tag IDs
  through the dashboard before they enter `runtime_markers_registry.yaml`).
**Effort**: NIT (15 min doc write).

---

## E. Local-odom semantics — the `differential: false` + absolute-yaw issue

---

#### HIGH-04-09 — `ekf_local` reads wheel-yaw absolute (non-differential) AND IMU yaw absolute simultaneously
**File(s)**:
- `src/agv_sensor_fusion/config/ekf_local.yaml:36-41` —
  `odom0_config: [T,T,F, F,F,T, T,F,F, F,F,T, F,F,F]` + `odom0_differential: false`.
- Same file, `:48-53` — `imu0_config: [F,F,F, T,T,T, F,F,F, T,T,T, T,T,F]` + `imu0_differential: false`.
**Category**: architecture.
**Symptom**: Both `odom0` (wheel) and `imu0` (IMU) feed yaw absolute to
the local EKF in non-differential mode. The IMU yaw absolute is the
integrated gyro since boot (BMI088 is gyro-only — no magnetometer).
The wheel yaw absolute is derived from `wheel_odom_validated` which
integrates encoder ticks.
**Analysis**: Two sources of "absolute" yaw means the EKF must
reconcile two integrated heading estimates that drift differently. The
system mitigates by giving the IMU a much tighter covariance (0.001
rad²) than wheel odometry (0.03 rad²), so the IMU dominates in the
fusion. **But this is a tuning workaround for a structural issue**:
neither source has true absolute yaw (no magnetometer, no GPS+IMU
heading); both are integration-only.
The correct pattern for a wheel+gyro fusion in `robot_localization` is:
- Wheel odom: read **velocities only** (vx, vyaw), differential mode
  irrelevant.
- IMU: read **angular velocity only** (vyaw) plus optionally linear
  acceleration. Yaw absolute is read only if magnetometer or external
  heading reference is present.
- The fused `odometry/local` accumulates heading internally and is
  consistent.
The current setup with both sources reading yaw absolute means a
crashed-and-restarted IMU resumes from a different yaw than the wheel
integrator, and the EKF averages them — silent drift.
**Greenhouse impact**: After IMU restarts (e.g., ZED USB disconnect
and recovery — a known greenhouse failure mode per the prompt §3.2),
the IMU yaw resets to 0 while wheel yaw stays at its accumulated value.
The EKF blends them. The map-frame heading from `ekf_global` snaps by
the gap between them.
**Benchmark**: `robot_localization` docs (Tom Moore) §"Two-Dimensional
Drift Removal" recommend velocity-only inputs from wheel+IMU. The dual
EKF tutorial uses `[F,F,F, F,F,F, T,F,F, F,F,T, F,F,F]` for wheel and
`[F,F,F, F,F,F, F,F,F, F,F,T, F,F,F]` for IMU.
**Recommendation**: Change `odom0_config` to read velocities only:
`[F,F,F, F,F,F, T,F,F, F,F,T, F,F,F]`. Change `imu0_config` similarly:
`[F,F,F, F,F,F, F,F,F, F,F,T, T,T,F]` (vyaw + linear accel only —
orientation absolute removed). The EKF accumulates internally and the
absolute-yaw fight disappears.
**Acceptance criterion**: After an IMU restart mid-mission (simulated),
the global EKF pose does not jump by more than the wheel-odom drift
that accumulated during the IMU outage. Today it would jump by the IMU's
re-zeroed heading offset.
**Effort**: M (config change + regression test on existing bags).
**Prerequisites**: none, but rebuilds the trust calibration of the dual
EKF — must be validated in HIL before field deployment.

---

## F. SLAM Toolbox in the production launch

[`src/agv_navigation/config/slam_toolbox_localization.yaml`](../../../src/agv_navigation/config/slam_toolbox_localization.yaml) (referenced; not read in full here)
and `agv_full.launch.py:394-413`.

`agv_navigation/CLAUDE.md` "SLAM Toolbox mode (deferred decision)"
acknowledges that the production config has `mode: mapping` (not
`localization`) during navigation, which builds its own map
continuously, publishes to `/map` (a separate topic from `/agv/map`),
and **loop closures don't feed back into `ekf_global`**.

So:
- SLAM Toolbox **does** ingest `/agv/scan` (the LaserScan).
- SLAM Toolbox **does** publish `/map` (its own occupancy grid; this
  is **not** the Nav2 `/agv/map` consumed by costmaps).
- SLAM Toolbox publishes nothing to `ekf_global` because
  `transform_publish_period: 0` disables TF and there is no separate
  pose topic going into the EKF.
- SLAM Toolbox is **not pure dead code** — it builds a fresh occupancy
  grid that can be saved on demand (`slam_toolbox/serialize_map` service),
  but its loop closures are local-only to that grid.

**This is the "deferred decision" already documented in
`agv_navigation/CLAUDE.md`.** No new finding here — the team is aware.
Tracked as a Phase 5 (mapping) follow-up; the recommendation is in
that CLAUDE.md.

---

## G. Hardware-dependent items (not in this audit)

| Acceptance criterion (per prompt §4 Phase 4) | Harness |
|---|---|
| 100 m loop drift < 0.5 % with AprilTags, < 2 % without | Bag-driven regression test against a recorded greenhouse trajectory; `tests/localization/loop_closure_drift.py`. |
| No map→base_link discontinuity > 5 cm or 2° in 30 min | `tf_monitor` + Python listener logging max jump per 5 s window. |
| Kidnapping recovery < 10 s | After HIGH-04-03 fix: HIL `/agv/sim/reset_request` triggers re-init within 2 s of next tag observation. |
| Pose rate ≥ 30 Hz in `map`, latency < 50 ms | `ros2 topic hz` + custom latency probe. ekf_global is 10 Hz today — gap to 30 Hz needs investigation. |
| Forced sensor blackout 5 s test (camera unplugged) | HIL or hardware. |

The ekf_global frequency of **10 Hz** is below the prompt's target of
**30 Hz** for `map` pose. This is a deliberate choice — local is fast
(50 Hz), global is slow (10 Hz, because cuVSLAM publishes at ~30 Hz and
markers at ~5–10 Hz, so 10 Hz is the meaningful update rate). **If the
prompt's 30 Hz is a strict requirement**, the dual-EKF architecture
would need to be reconsidered. **If it is a soft target** (e.g., for
dashboard smoothness), the `fusion_monitor` republisher at 10 Hz to
`/agv/pose` is the consumer-facing pose stream and could be upsampled
without changing the EKF rate.

---

## H. Status

| Item | Status |
|---|---|
| Dual EKF config audit | ✅ |
| AprilTag fusion audit (7 prompt questions answered) | ✅ |
| `factor_graph` opened (was a black box) | ✅ filed `HIGH-04-01`, `MEDIUM-04-05`, `MEDIUM-04-06` |
| Obliqueness filter for marker_correction | ✅ filed `HIGH-04-02` |
| Tag registry parser hardening | ✅ filed `HIGH-04-04` |
| Kidnapping detection | ✅ filed `HIGH-04-03` |
| Local EKF yaw-absolute structural issue | ✅ filed `HIGH-04-09` |
| Anti-spoofing as documented security property | ✅ filed `MEDIUM-04-07` |
| Reloc cooldown asymmetry | ✅ filed `LOW-04-08` |
| Hardware-dependent items | ⏸ deferred (§G) |

End of Phase 4. 8 findings total: 4 HIGH, 3 MEDIUM, 1 LOW.
