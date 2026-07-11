# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow
[SemVer](https://semver.org/) with a `0.x` series while interfaces are still
settling.

## [Unreleased]

### Added
- Immersive operator HMI (May design sprint, now integrated): greenhouse
  place layer on the map (rail tracks, crop rows, charging dock), state-aware
  animated robot icon, full-bleed map with camera follow, AprilTag detection
  ripples, compass/scale, WCAG AA accessibility pass, Spanish operator copy,
  and a greenhouse-geometry SSOT route (`GET/PUT /api/greenhouse/geometry`).
  Rebranded to NavGreen.
- May 2026 field-readiness audit integrated (sprints A–E + Section-0,
  hardware-validated): kinematic geometry SSOT
  (`agv_description/config/robot_geometry.yaml`, geometric truth
  0.0625 m wheel radius / 0.735 m track width, vernier + manual-rotation
  verified — closes CRITICAL-02-02), two new BLOCKING verifiers
  (`verify_topic_types.py` catches pub/sub type drift against the spec;
  `verify_geometry_ssot.py` blocks geometry re-declaration), per-source
  staleness timeout in the mode arbiter, WebSocket heartbeat + mission
  pause on operator disconnect, chrony wait at boot, `enable_factor_graph`
  and `enable_event_recording` launch gates, per-tag AprilTag sizes,
  per-topic QoS in the safety supervisor, `agv_greenhouse.repos` manifest
  for fresh-Jetson recovery, and ~6,000 lines of audit evidence under
  `docs/audit/` (HAZOP skeleton, E-stop wiring procedure, ODrive NVRAM
  dump + G5 calibration records, ADRs 0001/0002).

### Fixed
- `rail_driver` subscribed to `/agv/collision_monitor_state` with the
  wrong message type, so its own collision hold (`BLOCKED_WAIT`) never
  engaged during rail drive — the arbiter's `BLOCKED_HANDOFF` was the
  only effective stop. Same bug class as the arbiter fix in 0.1.0; the
  new `verify_topic_types.py` blocks a fourth occurrence.
- `/agv/mode/set` was volatile on both sides: when the arbiter booted
  after the dashboard backend, the operator-mode seed was lost and the
  joystick was clobbered by a 20 Hz zero-Twist stream. Both sides are
  now reliable + transient_local.
- `ekf_local` fused IMU orientation as absolute heading; any IMU re-zero
  (USB reboot) snapped the local filter (+2.4° measured on a stationary
  robot). It now consumes gyro rates only (HIGH-04-09).
- `rail_approach` missed the latched localization state on late join
  (volatile vs transient_local durability mismatch).
- Collision stop_zone rear lobe trimmed to the footprint — the robot is
  forward-only, so the 5 cm rear margin only produced false stops.
- Hardcoded `/home/orza` defaults removed from `auto_init_orchestrator`
  and `map_manager` (derive from `AGV_DATA_DIR`).

## [0.1.0] — 2026-07-08

### Added
- **NavGreen**: the project's public name and visual identity (logo, README
  hero). ROS package names keep the `agv_` prefix.
- **Documentation site** (MkDocs Material): getting started, tutorials
  (simulation, dashboard, mapping), architecture deep-dives (dual EKF, mode
  arbitration, the SSOT spec system, safety model), package/interface
  reference, and community pages — built strict on every PR and deployed to
  GitHub Pages from `main` (`.github/workflows/docs.yaml`).
- First green CI pipeline in the project's history: 20 of 24 packages build
  with `-Werror` plus tests, all four Node packages build, and the SSOT
  verifier suite runs as a blocking job.
- Open-source skeleton: MIT `LICENSE`, `CONTRIBUTING.md`, `CODE_OF_CONDUCT.md`,
  `SECURITY.md`, issue/PR templates, `CODEOWNERS`, dev container.
- Dashboard auth hardening: salted scrypt password hashing (transparent
  upgrade from legacy hashes), no default credentials, enforced role checks;
  stop-type endpoints intentionally remain anonymous.
- Fleet manager token auth (`FLEET_API_TOKEN`) and configurable bind address
  (`FLEET_BIND_ADDR`); fleet manager moved to port 8092 (image server keeps
  8091).
- CAN feedback-loss watchdog and frame validation in the ODrive driver;
  `feedback_ok` field on `/agv/motor_state`.
- `camera_frame_is_optical` parameter on the marker correction node.
- First TypeScript unit tests: 30 vitest cases across the fleet manager
  (traffic mutual-exclusion, FIFO grants, deadlock detection) and the
  dashboard API client, wired into CI.
- `agv_sim` package: a hardware-free Gazebo Classic simulation. Contributors
  with no robot can `ros2 launch agv_sim teleop_sim.launch.py` and drive the
  AGV with physics, reusing the real geometry and `diff_drive_controller`
  gains. CI builds it, validates the URDF, and smoke-tests the identical
  controller stack headless via `ros2_control` mock components; the Gazebo
  world-load + spawn run as a best-effort check (`simulation` job).

### Fixed
- AprilTag corrections mixed optical- and body-frame conventions, biasing
  every position correction.
- Shell-injection path through operator-supplied map names; map/zone names
  now restricted to `[A-Za-z0-9_-]{1,64}`.
- Two executor deadlocks (`spin_until_future_complete` inside callbacks) in
  map load and mission execution.
- `/agv/collision_monitor_state` subscribed with the wrong message type, so
  safety stops never registered in the mode arbiter.
- Both EKFs advertised `set_pose`; only `ekf_global` does now.
- Superseded 1341-line Python operator backend removed (TypeScript backend
  is the single implementation).
- Spec verifiers now fail loudly on drift; specs resynchronized to the code
  (15 missing interfaces registered, QoS and subscriber claims corrected,
  runtime-arbiter FSM layer added).

### Security
- Committed SSH password and home-lab topology scrubbed from the docs tree
  (history rewrite + credential rotation required before going public — see
  `docs/going_public_checklist.md`).
- Customer site configuration replaced with a neutral example site.

