# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow
[SemVer](https://semver.org/) with a `0.x` series while interfaces are still
settling.

## [Unreleased]

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

## [0.1.0] — unreleased

Tag `v0.1.0` on the merge commit of the community-readiness PR (#5) once it
lands on `main`; move the entries above under it.
