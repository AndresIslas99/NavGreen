# Phase 1 — Foundations: TF tree, time, build, deps

> Phase 1 goal (per prompt): "El robot arranca de forma reproducible y todas
> las bases temporales y espaciales son sanas."

The build / dependency / portability angle of Phase 1 is already filed in
Phase 0 (CR-00-01, CR-00-03, CR-00-04). This file covers what is left:
**spatial conventions (TF tree)** and **temporal conventions (time, clocks,
`use_sim_time`)** — both static-analysis only. Hardware measurements are
proposed but not executed.

---

## A. TF tree as a contract

The spec's claimed TF tree ([`specs/interfaces.yaml#ros.tf_tree`](../../../specs/interfaces.yaml)):

```
map → odom → base_link → {wheels, base_footprint, [zed frames]}
```

The verified tree from code:

```
map
 │  publisher: ekf_global (robot_localization)   @ 10 Hz
 │  source:    src/agv_sensor_fusion/config/ekf_global.yaml#publish_tf: true
odom
 │  publisher: ekf_local (robot_localization)    @ 50 Hz
 │  source:    src/agv_sensor_fusion/config/ekf_local.yaml#publish_tf: true
base_link
 ├── left_wheel  (continuous joint)    URDF urdf/wheel.xacro
 ├── right_wheel (continuous joint)    URDF urdf/wheel.xacro
 ├── base_footprint (fixed, z=-0.200)  URDF urdf/agv_base.xacro:33
 │   ← base_link is PARENT, base_footprint is CHILD (non-standard, see HIGH-01-01)
 └── [zed_camera_link ... ]            NOT IN URDF
     publisher: static_transform_publisher
     source:    src/agv_slam/launch/agv_slam.launch.py
     issue:     agv_slam not in this workspace (see CR-00-01)
```

### Findings

---

### HIGH-01-01 — `base_footprint` is a child of `base_link`, contrary to REP-105 spirit
**File(s)**: `src/agv_description/urdf/agv_base.xacro:33-39`.
**Category**: architecture / docs.
**Symptom**: The URDF declares `base_footprint` as a fixed child of `base_link` at `z=-0.200`. REP-105 lists `… → base_link → base_footprint` in its example diagram but states "the `base_footprint` is the representation of the robot position on the floor", which the broader community implements with `base_footprint` as the **root of the robot's kinematic chain** and `base_link` as its child. ROS-Industrial and Nav2 reference robots (Husky, Jackal, TurtleBot4) all use `base_footprint → base_link` ordering.
**Analysis**: Nav2's costmap2d uses `robot_base_frame: base_link` (`nav2_params.yaml:17,245,319`), so the current ordering works functionally. It breaks however for:
- **External tooling** that assumes `base_footprint` is the floor-contact origin (a single-axis kinematic offset of zero relative to ground): people-tracking modules, social-navigation Behavior Trees, and multi-robot fleet schedulers.
- **Diff-drive controllers** in ros2_control (`diff_drive_controller`) expect the parent to be ground-level; the `agv_hw_interface` package which sits opt-in alongside (`src/agv_hw_interface/`) will see the chain inverted from its conventional assumption.
- **MPPI and SmacPlanner2D** with `consider_footprint: true` (`nav2_params.yaml:159`) project the footprint along `base_link`, which is 200 mm above ground. The footprint should be evaluated at the floor; the 200 mm offset is masked here because Nav2 uses the polygon directly (a 2-D shape, ignoring z), but any future 3-D costmap or elevation-aware planner will compute incorrectly.
**Greenhouse impact**: Indirect today, but every future addition (elevation_mapping_cupy for puddle/curb detection, social-navigation BTs, fleet integration) will hit this inverted assumption.
**Benchmark**: REP-105 example diagram + Nav2 Jackal Sim — `base_footprint` is the root, `base_link` is `base_footprint → base_link` with z = 0.200. Husky's URDF is identical pattern. Clearpath's robot_state_publisher conventions documented in their reference materials.
**Recommendation**: Invert the joint so `base_footprint` is the root link:
```xml
<link name="base_footprint"/>
<joint name="base_link_joint" type="fixed">
  <parent link="base_footprint"/>
  <child link="base_link"/>
  <origin xyz="0 0 0.200" rpy="0 0 0"/>
</joint>
<link name="base_link"> ... </link>
```
Update Nav2 `robot_base_frame: base_link` is unchanged. The dual-EKF `base_link_frame: base_link` is unchanged. No code consumers need to change.
**Acceptance criterion**: `ros2 run tf2_tools view_frames` shows `base_footprint` as the root of the chassis subtree. All downstream consumers (Nav2, dual-EKF, mode_arbiter, marker_correction) work unchanged. URDF passes `check_urdf`.
**Effort**: S.
**Prerequisites**: none. **Note**: small risk — any test fixture that hardcodes the chain ordering will need a one-line update.

---

### HIGH-01-02 — Camera frame transform lives outside the workspace
**File(s)**:
- `src/agv_description/urdf/agv_full.urdf.xacro:14-15` — "Camera TF (base_link → zed_camera_link) is NOT included here. It is published by agv_slam's static_transform_publisher."
- `agv_slam` package — **not in this workspace** (CR-00-01).

**Category**: architecture / safety.
**Symptom**: The camera-to-base extrinsic transform — the most critical extrinsic in the perception pipeline — is published by `static_transform_publisher` defined in a launch file inside the external `agv_slam` package. The transform is therefore **not version-controlled in this repo** and **not part of any `verify_specs` check**.
**Analysis**: This camera extrinsic enters every layer of the localization stack:
- AprilTag detection in `marker_correction` projects observed tag poses from `zed_left_camera_optical_frame` to `base_link` to update `map→odom` via `pose0` factor in `ekf_global`.
- cuVSLAM tracking output is in the camera frame and must transform to `base_link` for `ekf_global`'s `odom1` input.
- The ZED SDK Area Memory pose (`pose1` in `ekf_global`) is in the ZED's internal map frame, with the wrapper applying the base_link offset.
A wrong number here propagates *quadratically* with distance — a 1° camera-mount yaw error at 5 m of AprilTag range = 9 cm of lateral pose error.
**Greenhouse impact**: The camera mount on a robot operating in 70–95 % humidity vibrates. Greenhouse floors are not always level. Workers occasionally brush against the camera housing. The mount-to-base extrinsic will drift in the field. If the calibration lives in an external repo, field re-calibration requires the operator to know which file in which repo holds it. **A field technician cannot recalibrate the camera mount without a developer present.**
**Benchmark**:
- NVIDIA Isaac Robotics: camera extrinsics live in the robot's URDF as a calibrated `xacro` macro, with calibration result YAML loaded via `xacro` args. See Isaac ROS Tutorials.
- ETH RSL ANYmal: every sensor mount is a URDF link with a YAML-loaded calibration block.
- Clearpath Husky: all sensors with `mount.yaml` per-robot calibration.
**Recommendation**:
1. **Move the camera extrinsic into the URDF** as a xacro macro. Read x, y, z, roll, pitch, yaw from `src/agv_description/config/sensor_extrinsics.yaml` (new file) at xacro processing time.
2. **Document the calibration procedure** in `docs/calibration/camera_mount.md` — operator drives to a known AprilTag-on-floor, the GUI captures N detections, runs `solvePnP` to extract camera→base, writes back to `sensor_extrinsics.yaml`, commits.
3. **Add to `verify_persistence.yaml`** as `${AGV_CONFIG_DIR}/sensor_extrinsics.yaml` with declared writer (commissioning wizard) and readers (URDF xacro, marker_correction).
4. **Online monitoring**: cross-validate cuVSLAM trajectory against wheel+IMU after a turn-in-place — if they diverge by > X cm/m, the camera mount likely drifted. Surface as `DiagnosticArray` warning.
**Acceptance criterion**:
- URDF `view_frames` output shows `base_link → zed_camera_center` directly from this repo, no external static publisher needed.
- The GUI commissioning wizard can recalibrate camera→base in < 5 min without a terminal.
- `solvePnP` reprojection RMSE < 1 px on a validation set captured in the field.
**Effort**: M.
**Prerequisites**: CR-00-01 (need camera transform pulled out of the external `agv_slam` repo).

---

### MEDIUM-01-03 — `static_transform_publisher` for camera obscures vibration-induced drift
**File(s)**: same as HIGH-01-02.
**Category**: bug / failure mode.
**Symptom**: A `static_transform_publisher` declares a fixed transform that never changes at runtime. The greenhouse environment guarantees that the camera mount will physically deflect from time to time. Static publishers cannot model this.
**Analysis**: If the mount deflects by 2° during operation (humid wood pallet shifted under a wheel, technician's hip brushed the housing), every subsequent AprilTag observation contributes a yaw bias of 2° to `ekf_global`. The Mahalanobis rejection thresholds (`pose0_rejection_threshold: 3.0`) will reject extreme observations, but moderate biases will integrate into the global pose estimate. No mechanism today flags this.
**Greenhouse impact**: Slow degradation of localization quality over days, surfacing as "the robot's path is slightly off but Nav2 reports success". Hard to diagnose without on-board monitoring.
**Benchmark**: Online camera-IMU temporal calibration (Furgale, Rehder et al. — `kalibr_calibrate_imu_camera`) detects drift online. NVIDIA Isaac Robotics' Mission Dispatch can re-trigger extrinsic calibration on a schedule. SLAM Toolbox emits "loop closure rejected" diagnostics when graph residuals exceed thresholds — a similar signal could be derived for camera-to-base drift.
**Recommendation**:
1. **Online consistency check**: compare two independent estimates of `base_link → camera_optical` — derived from (a) ZED SDK internal positional tracking and (b) the URDF static transform — and flag persistent disagreement.
2. **In-the-field re-calibration trigger**: the GUI shows a "camera extrinsic drift suspected" badge when the residual exceeds threshold; operator drives to a calibration AprilTag and re-runs the wizard.
**Acceptance criterion**: a 2° forced rotation of the camera (e.g., during testing) produces a dashboard warning within 30 s of operation, not silent degradation over a day.
**Effort**: M.
**Prerequisites**: HIGH-01-02 (extrinsic must be a parameter, not baked into a static publisher).

---

## B. Time, clocks, sim_time

### B.1 What the code does today

- All Nodes in `agv_full.launch.py` accept `use_sim_time` derived from `hil_mode` (`agv_full.launch.py:96`): `'true' if hil_mode else 'false'`.
- The HIL clock is `IsaacReadSimulationTime` at 72 Hz, with the exception of `/agv/motor_state` and `/agv/drive_debug` which are wall-clock (sim_motor_gate, commit 3a4467b on the sim side — documented at `agv_full.launch.py:91-95`).
- `agv_start.sh` does **no NTP / chrony setup**. There is no documented requirement that the Jetson's clock be synced to a master.
- `cmd_vel_timeout_ms: 200` in `odrive_params.yaml:13` is a node-local watchdog; it compares against the node's own steady clock (verified at `agv_odrive/src/odrive_can_node.cpp` — not read in this audit, but the relevant timeout pattern is standard).

### B.2 Findings

---

### MEDIUM-01-04 — No master clock discipline at boot
**File(s)**: `src/agv_bringup/scripts/agv_start.sh` — no `chronyc waitsync` or `timedatectl` invocation.
**Category**: bug / failure mode.
**Symptom**: When the Jetson boots cold in the field with no internet (likely scenario in a remote greenhouse), `journalctl -u systemd-timesyncd` shows "Time has been changed" entries asynchronously *during* the ROS startup. Every TF lookup that bridges the boot moment sees a time jump.
**Analysis**: The dual EKF, scan_grid_mapper, image_server, and any TF-based subscriber buffer messages by timestamp. A clock jump of even 100 ms at t=5 s of boot makes the EKF reject every message that was buffered before the jump (`TF_OLD_DATA`) and the first 30 s of operation is "blind". The orchestrator's `localization.action` flips to FAILED. The operator sees "robot offline" with no clear cause.
The local router (UniFi documented in CLAUDE.md) is the natural NTP master in the greenhouse — but agv_start.sh does not wait for sync, and there is no documented `chrony.conf` pointing to the router.
**Greenhouse impact**: Every cold boot in the field carries this risk. The 90 s "wait for IPv4" loop at `agv_start.sh:67-78` waits for the network interface but not for the clock.
**Benchmark**:
- Tier IV Autoware boots with `chronyc waitsync 10 0.01` — block until clock skew is below 10 ms.
- Boston Dynamics Spot SDK: `bd-sysmgr` enforces PTP between the controller and the chassis processor.
- Robotec.ai launches verify wall-clock skew against `ros2 bag info` of a recorded mission before allowing playback.
**Recommendation**:
1. **Add `/etc/chrony/chrony.conf`** template under `src/agv_bringup/config/` pointing to the local router as NTP server with `iburst` and a fallback `pool` for when the router is unavailable.
2. **Add to `agv_start.sh`**, after network is ready and before sourcing ROS:
   ```bash
   if ! chronyc waitsync 5 0.05 2>/dev/null; then
       echo "WARN: NTP not synced after 5 attempts (50 ms tolerance); continuing with potential clock skew."
   fi
   ```
3. **Document** in `docs/hardware_setup.md` that the local router must be configured as an NTP server.
**Acceptance criterion**: Cold boot in a clock-isolated environment produces a clear journal log either confirming sync or declaring "not synced — degraded mode". Dashboard surfaces clock-skew status.
**Effort**: S.
**Prerequisites**: none.

---

### MEDIUM-01-05 — Wall-clock and sim-clock coexist in HIL with implicit subscriber assumptions
**File(s)**: `src/agv_bringup/launch/agv_full.launch.py:86-95` (commentary), implicit in HIL subscribers.
**Category**: docs / failure mode.
**Symptom**: In HIL, most topics carry `IsaacReadSimulationTime` stamps but `/agv/motor_state` and `/agv/drive_debug` are wall-clock. The launch file comment notes this and states "subscribers must NOT age-validate those two topics against `use_sim_time=true` — verified that `agv_ui_backend` does not."
**Analysis**: This is an architecturally awkward state. The "verified that agv_ui_backend does not" claim is a one-line audit comment, not an enforceable contract. A future engineer adding a freshness watchdog to `motor_state` (a reasonable thing to want) will silently break HIL.
**Greenhouse impact**: HIL-only — no direct greenhouse impact. But it weakens the sim-to-real claim because HIL is supposed to mirror production.
**Recommendation**:
1. **Declare the wall-clock topics in `specs/interfaces.yaml`** with an explicit `clock_source: "wall"` annotation and a `do_not_age_validate: true` flag.
2. **Add to `verify_interfaces.py`** a check that any subscriber to one of these topics does not call `now() - msg.header.stamp` against sim time.
**Acceptance criterion**: A PR that adds a `motor_state` freshness check (or any age-based validation) fails the verifier with a clear message pointing at this spec annotation.
**Effort**: S.
**Prerequisites**: none.

---

## C. Spec-level invariants worth elevating

Items in the existing specs that read as "we know this is a footgun" — calling out so they can become explicit checks rather than tribal knowledge.

| Spec ref | Invariant | Currently enforced by |
|---|---|---|
| `state_machine.yaml#tf_map_odom_single_owner` | Exactly one node publishes `map→odom`. | YAML override (`/**:` key) — no runtime check |
| `state_machine.yaml#odrive_cmd_vel_source` | `agv_odrive_node` subscribes to one cmd_vel topic depending on `has_map`. | Launch file `IfCondition` / `UnlessCondition` |
| `state_machine.yaml#safety_chain_never_silent_on_idle` | `monitored_topics` only contains continuous-rate topics. | Code review only |
| `interfaces.yaml#tf_ownership` | Single publishers for each TF hop. | Verbose comment, not a check |

**Recommendation (`LOW-01-06`)**: write `tools/verify_specs/verify_tf_single_owner.py` that:
- parses every YAML in `src/*/config/` looking for `publish_tf: true` and `transform_publish_period > 0`,
- cross-references against `specs/interfaces.yaml#tf_ownership`,
- fails if more than one publisher is enabled for the same TF hop.
This converts the `/**:` YAML key trap (silent breakage when written incorrectly) into a hard verifier failure.

---

## D. Hardware-dependent gates deferred

| Acceptance criterion (per prompt §4 Phase 1) | Hardware needed | Proposed harness |
|---|---|---|
| Build clean in <5 min on Orin (ccache hot) | Jetson | CI runner with industrial_ci on `agv_greenhouse.repos` (CR-00-01) |
| 0 warnings in -Wall -Wextra -Wpedantic | Already enforced (-Werror); compile must pass | `colcon build --cmake-args -DCMAKE_CXX_FLAGS="-Werror"` |
| TF tree connected, no extrapolation warnings 10 min | Running stack | `ros2 run tf2_tools view_frames` + `tf_monitor` script for 10 min |
| `tf_monitor` delay < 50 ms, rate ≥ 20 Hz | Running stack | Same |
| Clock synced to ±10 ms | Field deployment | `chronyc tracking` script logged to `events.jsonl` |

Each gate becomes a row in `tests/foundations/checklist.md` (to be written) and a script in `scripts/health/` (also to be written) when hardware is available.

---

## E. Status

| Item | Status |
|---|---|
| TF tree contract analysis | ✅ |
| REP-105 conformance | ✅ flagged `HIGH-01-01` |
| Camera frame ownership | ✅ flagged `HIGH-01-02` + `MEDIUM-01-03` |
| Time / clock | ✅ flagged `MEDIUM-01-04` + `MEDIUM-01-05` |
| Spec invariant hardening | ✅ proposed `LOW-01-06` |
| Hardware-dependent gates | ⏸ deferred (table D) |

End of Phase 1.
