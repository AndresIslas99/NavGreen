# Community-Readiness Review — 2026-07-06

Full-repository review performed ahead of opening this repository to the
open-source community. Fourteen specialized reviewers (architecture; C++ in
four package groups; Python; TypeScript; documentation accuracy; spec-vs-code
drift; security and secrets; build system; tests; house-rule consistency;
community readiness; and a fresh-eyes newcomer pass) produced 187 raw
findings, deduplicated to **109 unique findings** (11 critical / 45 high / 45
medium / 8 low). Every finding was re-verified against the code before any
fix was applied; a spot-check pass of 26 adversarial verifications confirmed
all 26. Fixes were applied by scoped agents, each owning a disjoint set of
packages, and validated by the spec verifiers, local TypeScript/Python
builds, and the (newly repaired) CI pipeline.

This document is the durable record. The living state of the repo is the
code, the specs, and CI — if they disagree with this file, they win.

## Headline outcomes

- **CI was red on every recorded run and is now green**, building 21 of 24
  packages with `-Werror` plus tests, all four Node packages, and the SSOT
  verification suite. The three CI-excluded packages need vendor SDKs
  (Isaac ROS, ZED, GTSAM).
- **A committed plaintext SSH password and the maintainer's home-lab
  topology were scrubbed** from the docs tree (see the mandatory owner
  actions below — the git history still contains them).
- **Robot-control surfaces hardened**: dashboard auth is now genuinely
  enforced when enabled, ships no default credentials, and uses salted
  scrypt hashing; the fleet manager gained token auth and a configurable
  bind address; stop-type endpoints deliberately remain unauthenticated so
  the robot can always be stopped.
- **Real field bugs fixed**, including: the AprilTag optical-vs-body frame
  error in marker corrections, a shell-injection path through operator map
  names, two executor deadlocks (`spin_until_future_complete` inside
  callbacks), an undetected mid-motion CAN feedback loss, a message-type
  mismatch on `/agv/collision_monitor_state`, and both EKFs answering the
  same `set_pose` service.
- **The repo now has a complete open-source skeleton**: MIT LICENSE (all 24
  packages now consistently MIT), CONTRIBUTING, CODE_OF_CONDUCT, SECURITY,
  issue/PR templates, and a rewritten README front door.

## Mandatory owner actions before making the repository public

1. **Rotate the exposed SSH password and treat the sim host as
   compromised.** `docs/validation/RUNBOOK_lan_hil.md` published a real
   password; it is removed from the tree but **still lives in git
   history**. Rewrite history (`git-filter-repo` or BFG) before flipping
   the repo public, or accept that the credential is permanently public.
2. **Scrub the sibling `agv-greenhouse-sim` repository** — the runbook
   cross-references it and it very likely carries the same credentials,
   usernames, and LAN topology.
3. **Commit the robot-only launch files** for `agv_behaviors`,
   `agv_markers`, `agv_rail_approach`, and `agv_waypoint_manager` (their
   CMake installs are `OPTIONAL` today so fresh clones build, but the
   files themselves exist only on commissioned robots).
4. **Decide the `/home/orza` code defaults**: docs now use `$AGV_DATA_DIR`,
   but some parameter defaults in code and the spec still fall back to the
   personal home directory. Changing runtime defaults was deliberately left
   to you (it alters robot behavior).
5. **Enable authentication for any real deployment** — dashboard auth and
   broker auth ship disabled by default for the isolated-LAN workflow; the
   enable procedures are documented in SECURITY.md and
   `fleet/mosquitto/config/mosquitto.conf`.

## What reviewers called out as genuinely strong

- The machine-readable spec system (`specs/*.yaml` + nine pre-commit
  verifiers) — rare in robotics repos and the project's signature feature.
- The dual-EKF architecture with single-owner TF invariants, explained well
  enough to teach from.
- The cmd_vel arbitration seam: per-controller topics muxed by a
  single-owner 8-state FSM.
- Fail-safe defaults in the safety chain (`cmd_vel_gate` starts blocked
  until the supervisor proves liveness).
- Pure-logic, ROS-free control cores with real gtest coverage and
  literature citations (wheel-slip detector, caster dwell advisor, rail
  controllers).
- An honest engineering culture: audit trail, `known_gap`/`current_bug`
  annotations inside the specs, and comments that record which field
  iteration motivated each guard.

## Architectural roadmap (deliberately not "fixed" in a PR)

- **Two mission-execution subsystems** (TypeScript executor in
  `agv_ui_backend` vs C++ `agv_behaviors`/`agv_waypoint_manager`) — decide
  on one owner for mission dispatch; retire or gate the other (finding 3).
- **Fleet layer as a third dispatcher**: the VDA5050 adapter can drive the
  robot outside the mode arbiter's knowledge; it is now spec-registered and
  gated, but the arbitration story should include it first-class (finding 2).
- **`robot_namespace` is declared canonical but decorative** — nodes
  hardcode the `agv` namespace rather than reading the parameter (finding 6).
- **Duplicated CAN/ODrive protocol layer** between `agv_odrive` and
  `agv_hw_interface` — extract a shared library package (finding 7).
- **`agv_factor_graph` validates against its own input**: it consumes
  `ekf_global` output while being pitched as an independent check
  (finding 25).
- **TypeScript test coverage is zero** across the operator backend,
  dashboard, and fleet layer (finding 98); `agv_factor_graph` and
  `agv_localization_init` have no tests either (finding 100).
- **A contributor-runnable simulation story** — today HIL needs the
  owner's private sim host (finding 106).

## Finding ledger

Status meanings: **Fixed** (change applied and verified), **Partially
fixed** (tree fixed; some part needs owner action or hardware validation),
**Fixed in spec sync** (applied in the specs/verifier sync),
**Deferred** (needs a decision or out-of-scope change),
**Open (owner/roadmap)** (documented here, intentionally not auto-fixed).

| # | Sev | Area | Location | Finding | Status |
|---|-----|------|----------|---------|--------|
| 8 | critical | cpp-drivetrain | `src/agv_odrive/src/odrive_can_node.cpp` | A mid-motion CAN/ODrive feedback loss is completely undetected: heartbeat freshness is never checked, send() failures are ignored, and the node keeps publishing stale ... | Fixed |
| 20 | critical | cpp-localization | `src/agv_markers/src/marker_correction_node.cpp` | AprilTag position correction transforms the optical-frame solvePnP tvec with the body-convention camera TF, so the optical->body rotation is never applied and every pu... | Fixed |
| 30 | critical | cpp-navigation | `src/agv_mode_arbiter/src/mode_arbiter_node.cpp` | mode_arbiter and rail_driver subscribe /agv/collision_monitor_state as std_msgs/String while the only production publisher (Nav2 collision_monitor) and specs/interface... | Partially fixed |
| 31 | critical | cpp-navigation | `src/agv_map_manager/src/map_manager_node.cpp` | save_map builds a shell command from the operator-supplied map name with only '/' and '..' rejected, so a name containing a single quote achieves arbitrary command exe... | Fixed |
| 32 | critical | cpp-navigation | `src/agv_waypoint_manager/src/waypoint_manager_node.cpp` | Mission execution calls rclcpp::spin_until_future_complete on a node that is already spinning in main(), which throws 'Node has already been added to an executor' insi... | Fixed |
| 50 | critical | typescript | `src/agv_ui_backend/src/auth.ts` | Every REST control endpoint that drives the physical robot is completely unauthenticated — requireAuth is wired only to /api/auth/* routes, so even with auth enabled a... | Fixed |
| 51 | critical | typescript | `fleet/agv_fleet_manager/src/index.ts` | The fleet manager exposes fleet-wide control — fleet e-stop, resume, and per-robot navigate — on 0.0.0.0 with zero authentication on both REST and WebSocket, connects ... | Partially fixed |
| 59 | critical | docs | `docs/validation/RUNBOOK_lan_hil.md` | A committed, git-tracked runbook (docs/validation/RUNBOOK_lan_hil.md) publishes a real plaintext SSH password for a named user on a reachable dev machine, plus usernam... | Partially fixed |
| 64 | critical | community | `src/agv_rail_approach/package.xml` | src/agv_rail_approach declares <license>Proprietary</license> (and a different maintainer) while the repository root LICENSE is MIT — a legal contradiction that makes ... | Fixed |
| 70 | critical | ssot-drift | `src/agv_bringup/launch/agv_full.launch.py` | All three launch entry points hard-depend on an 'agv_slam' package that does not exist anywhere in the repository (never in git history, gitignored under '# Third-part... | Partially fixed |
| 86 | critical | build | `README.md` | A fresh clone cannot build by following the README: there is no rosdep step, several first-party packages need vendor SDKs that are not on apt, and the nine gitignored... | Partially fixed |
| 0 | high | arch | `src/agv_zone_detector/include/agv_zone_detector/zone_classifier_impl.hpp` | Site-specific greenhouse geometry, AprilTag marker IDs, and zone names are compiled into C++ across at least four packages — zone_classifier_impl.hpp (rail x-ranges, a... | Deferred |
| 1 | high | arch | `src/agv_ui_backend/src/index.ts` | The UI backend subscribes to /agv/rail_approach/status, which nothing publishes (the node publishes rail_approach/state), so railApproachState is frozen at 'idle' and ... | Fixed |
| 2 | high | arch | `fleet/agv_vda5050_adapter/src/index.ts` | The fleet/ layer is a shadow control plane invisible to the SSOT: the VDA5050 adapter is a third, ungated dispatcher of /navigate_to_pose that also publishes the safet... | Partially fixed |
| 3 | high | arch | `src/agv_bringup/launch/agv_full.launch.py` | Two complete, divergent mission-execution subsystems coexist: the TypeScript executor inside agv_ui_backend (used by the dashboard) and the C++ agv_waypoint_manager no... | Open (owner/roadmap) |
| 4 | high | arch | `README.md` | The repository's front door is stale: the README Launch Modes table instructs users to run launch files deleted in the 2026-04-13 audit (agv_robot_core.launch.py, agv_... | Partially fixed |
| 9 | high | cpp-drivetrain | `src/agv_odrive/src/odrive_can_node.cpp` | Odometry initialization latches prev_position before the first real encoder frame arrives, so restarting the driver while the ODrive stays powered produces a massive o... | Fixed |
| 10 | high | cpp-drivetrain | `src/agv_odrive/src/odrive_can_node.cpp` | Received CAN frames are parsed as encoder/temperature data without checking the RTR flag or DLC, so RTR poll requests from any other local process (the repo's own Pyth... | Fixed |
| 11 | high | cpp-drivetrain | `src/agv_rail_driver/src/rail_driver_node.cpp` | The zone-state JSON parser calls std::stod without a try/catch, so one malformed or truncated /agv/zone/state message throws out of the subscription callback and kills... | Fixed |
| 12 | high | cpp-drivetrain | `specs/interfaces.yaml` | specs/interfaces.yaml makes several claims that are false against the code: it says cmd_vel_gate subscribes /agv/e_stop for 'zero output' but the gate never subscribes... | Fixed in spec sync |
| 13 | high | cpp-drivetrain | `src/agv_odrive/src/odrive_can_node.cpp` | Motor disarm on shutdown relies solely on the node destructor and there is no ODrive firmware watchdog configured or verified anywhere in the repo, so a driver crash m... | Deferred |
| 21 | high | cpp-localization | `src/agv_sensor_fusion/launch/fusion.launch.py` | The production fusion launch leaves both EKFs advertising the same /agv/set_pose service/topic, so marker RELOC and auto-init pose seeding can nondeterministically res... | Partially fixed |
| 22 | high | cpp-localization | `src/agv_sensor_fusion/src/fusion_monitor_node.cpp` | apply_noise_multiplier() calls future.get() on an AsyncParametersClient response from inside a subscription callback of a single-threaded-spun node; the response can n... | Fixed |
| 33 | high | cpp-navigation | `src/agv_rail_approach/src/rail_approach_node.cpp` | nav_goal_handle_ is never assigned (async_send_goal's response is discarded), so aborting during COARSE_APPROACH never cancels the Nav2 goal and the robot keeps drivin... | Fixed |
| 34 | high | cpp-navigation | `src/agv_rail_approach/src/rail_approach_node.cpp` | Tag-loss timeouts are only evaluated inside the detection callback, so if the AprilTag/camera stream stops entirely during FINE_SERVOING the node stays in the 'driving... | Fixed |
| 35 | high | cpp-navigation | `src/agv_behaviors/src/behavior_executor_node.cpp` | The behavior executor never writes goal_x/goal_y/goal_theta to the blackboard and ignores getInput() failures, so executing the default single_waypoint.xml tree silent... | Partially fixed |
| 36 | high | cpp-navigation | `src/agv_map_manager/src/map_manager_node.cpp` | load_map_internal calls rclcpp::spin_until_future_complete on the node from inside a service/timer callback while the node is already spinning in main(), which throws ... | Fixed |
| 41 | high | python | `src/agv_ui_backend/scripts/teleop_server.py` | A superseded 1341-line Python rclpy/FastAPI backend (teleop_server.py, plus live_map_bridge.py and teleop_web.launch.py) still ships inside the runtime agv_ui_backend ... | Fixed |
| 42 | high | python | `src/agv_hil_bridges/launch/hil_bridges.launch.py` | With use_gt_odom:=true, both gt_to_wheel_odom and joint_states_to_wheel_odom publish /agv/wheel_odom simultaneously, feeding the EKF non-deterministic conflicting odom... | Partially fixed |
| 43 | high | python | `src/agv_integration_tests/test/test_e_stop.py` | The three advertised integration tests (e-stop, service availability, topic availability) contain zero assertions and pass unconditionally against a fully broken stack... | Fixed |
| 44 | high | python | `tools/verify_specs/verify_dev_only.py` | The Rule-0 enforcement verifier cannot see most Python nodes: it only flags executables ending in '.py' (entry-point-installed Python nodes have no suffix), its regex ... | Fixed in spec sync |
| 45 | high | python | `tools/verify_specs/verify_state_machine.py` | The state-machine spec verifier exits 0 even when it detects that state_machine.yaml is missing every required top-level section, and all.sh then reports 'RESULT: OK' ... | Fixed in spec sync |
| 52 | high | typescript | `src/agv_ui_backend/src/routes/nav.ts` | Unvalidated numeric input from REST/WebSocket flows into ROS nav goals and cmd_vel as NaN, producing undefined motion commands. | Fixed |
| 53 | high | typescript | `fleet/agv_vda5050_adapter/src/index.ts` | Several VDA5050 conformance defects: pause actions are conflated with emergency stop, unknown actions report FINISHED, header IDs are not monotonic per topic, and orde... | Fixed |
| 54 | high | typescript | `src/agv_ui_backend/src/index.ts` | Navigation state is a single global mutable object shared by all goal sources, so a manual goal issued during mission execution corrupts mission progress and cancel lo... | Partially fixed |
| 60 | high | docs | `README.md` | The README's headline claim that this is a "ROS2 (Humble) workspace" contradicts the SSOT specs and validation docs, which state the Jetson robot runs ROS 2 Jazzy. | Fixed |
| 65 | high | community | `.github/workflows/ci.yaml` | CI never runs the repository's flagship SSOT verification suite (tools/verify_specs/all.sh), so the BLOCKING spec-sync gate the PR template demands exists only as an o... | Fixed |
| 66 | high | community | `specs/persistence.yaml` | The maintainer's personal username 'orza' and home directory /home/orza are baked in repo-wide: C++ parameter defaults in two runtime nodes (auto_init_orchestrator, ma... | Open (owner/roadmap) |
| 67 | high | community | `src/agv_bringup/config/sites/chada/site.yaml` | The paying customer's identity ('Chada Farms', Mexico) is committed as the default launch site config, alongside real development-LAN IPs and ssh targets — publicly ty... | Partially fixed |
| 71 | high | ssot-drift | `tools/verify_specs/verify_interfaces.py` | The SSOT interface enforcement is far weaker than advertised: verify_interfaces.py is a raw substring grep over all source files (a topic mentioned only in a comment c... | Fixed in spec sync |
| 72 | high | ssot-drift | `specs/interfaces.yaml` | At least six real cross-package ROS interfaces are missing from interfaces.yaml, violating its own coverage_target 'Every topic consumed by an agv_* package MUST be li... | Fixed in spec sync |
| 73 | high | ssot-drift | `specs/interfaces.yaml` | interfaces.yaml and launch_sequence.yaml still describe audit bug #1 ('teleop broken, cmd_vel blocked') as a current bug, contradicting both state_machine.yaml (which ... | Fixed in spec sync |
| 74 | high | ssot-drift | `src/agv_bringup/launch/agv_full.launch.py` | agv_full.launch.py resolves the data root three inconsistent ways — env-derived AGV_DATA_DIR for map_manager but hardcoded /home/orza paths in four places (teleop_serv... | Partially fixed |
| 75 | high | ssot-drift | `specs/launch_sequence.yaml` | specs/launch_sequence.yaml has drifted materially from agv_full.launch.py: every cited source line range is stale, the t=3.8s wheel_slip_detector and caster_dwell_advi... | Fixed in spec sync |
| 80 | high | security | `src/agv_ui_backend/src/auth.ts` | The dashboard auth seeds well-known default credentials (engineer/agv2026, operator/agv) hashed with unsalted single-round SHA-256 and persists a plaintext JWT secret ... | Fixed |
| 81 | high | security | `fleet/mosquitto/config/mosquitto.conf` | The fleet MQTT broker ships with 'allow_anonymous true' on network-exposed listeners (1883 and WebSocket 9001), and the ACL file that is supposed to restrict robot top... | Fixed |
| 87 | high | build | `src/agv_rail_approach/package.xml` | `<build_depend>OpenCV</build_depend>` is not a valid rosdep key, so `rosdep install --from-paths src` fails hard; CI papers over the bug with --skip-keys instead of fi... | Fixed |
| 88 | high | build | `.github/workflows/ci.yaml` | CI builds and tests only ~14 of 24 packages, excluding agv_safety (the software safety chain with real gtests and a hardware-free launch smoke test), agv_hw_interface,... | Fixed |
| 89 | high | build | `src/agv_bringup/package.xml` | agv_bringup's launch files launch nodes from at least 15 packages that are not declared as exec_depend, so rosdep will not install slam_toolbox/apriltag_ros/robot_loca... | Fixed |
| 96 | high | tests | `src/agv_odrive/test/test_kinematics.cpp` | 8 of 20 C++ unit test files never exercise production code: agv_odrive's tests reimplement the kinematics inline, test_marker_lookup tests a locally re-declared struct... | Partially fixed |
| 97 | high | tests | `src/agv_integration_tests/test/test_ekf_frames.py` | test_ekf_frames.py can never pass in any environment: tf2_echo runs forever, so subprocess.run(timeout=8) always raises TimeoutExpired before the assertion can be eval... | Fixed |
| 98 | high | tests | `src/agv_ui_backend/package.json` | The entire TypeScript surface — the 3,145-line operator backend with JWT auth, the React dashboard, the fleet manager, and the VDA5050 adapter — has zero real tests: t... | Open (owner/roadmap) |
| 102 | high | consistency | `fleet/agv_fleet_manager/src/index.ts` | The fleet manager and the per-robot C++ image server both default to port 8091 on the same Jetson, and the dashboard treats host:8091 as a single origin serving both c... | Partially fixed |
| 103 | high | consistency | `src/agv_odrive/src/odrive_can_node.cpp` | The robot's physical kinematic constants exist in at least five places with two conflicting value sets and two parameter names — the README states 125mm wheel diameter... | Partially fixed |
| 104 | high | consistency | `specs/project.yaml` | The repo's status metadata is frozen at the 2026-04-13 audit and asserts now-false 'current state' facts: project.yaml claims agv_sensor_fusion 'not yet created', docs... | Fixed in spec sync |
| 106 | high | newcomer | `docs/validation/RUNBOOK_lan_hil.md` | No outside contributor can run or see the system without the owner's hardware: the simulator lives only on a private PC in an unpublished agv-greenhouse-sim repo (ment... | Partially fixed |
| 5 | medium | arch | `specs/state_machine.yaml` | The state-machine SSOT models four mode layers, but the Phase-2 mode_arbiter added a fifth: an 8-state FSM (CORRIDOR_NAV, RAIL_APPROACH_PEND/ACTIVE, RAIL_DRIVE, RAIL_E... | Fixed in spec sync |
| 6 | medium | arch | `src/agv_sensor_fusion/src/fusion_monitor_node.cpp` | The canonical 'robot_namespace parameter, default agv' decision is decorative: no node reads a robot_namespace parameter, many C++ runtime nodes hardcode absolute /agv... | Open (owner/roadmap) |
| 7 | medium | arch | `src/agv_hw_interface/src/can_socket.cpp` | The SocketCAN wrapper and ODrive protocol layer are copy-pasted between agv_odrive and agv_hw_interface (can_socket.{hpp,cpp}, odrive_protocol.hpp, plus separate diff-... | Open (owner/roadmap) |
| 14 | medium | cpp-drivetrain | `src/agv_odrive/src/odrive_can_node.cpp` | The asymmetric accel/decel limiter assumes cmd_vel arrives at exactly publish_rate_hz, so the effective acceleration limit silently scales with whatever rate the upstr... | Fixed |
| 15 | medium | cpp-drivetrain | `src/agv_odrive/src/odrive_can_node.cpp` | Thermal protection cross-wires the motor temperature against the FET limit and lets the healthy axis overwrite a critical verdict, causing premature shutdowns and a fl... | Fixed |
| 16 | medium | cpp-drivetrain | `src/agv_odrive/src/odrive_can_node.cpp` | publish_rate_hz is the one physically critical parameter that is never validated: 0 causes an integer division-by-zero crash (SIGFPE) and values above 1000 create a 0 ... | Fixed |
| 17 | medium | cpp-drivetrain | `src/agv_safety/src/cmd_vel_gate.cpp` | The gate and supervisor subscribe to the (planned) estop topics with transient_local durability, but the spec records no durability requirement — a future publisher us... | Partially fixed |
| 18 | medium | cpp-drivetrain | `src/agv_hw_interface/src/agv_diff_drive_system.cpp` | The ros2_control plugin's write()/on_activate() ignore every CANSocket::send() return value and read() unconditionally returns OK, so controller_manager can never see ... | Fixed |
| 19 | medium | cpp-drivetrain | `src/agv_safety/launch/safety.launch.py` | Safety-chain documentation contradicts itself: the launch file header says the chain is not wired into production while CLAUDE.md and agv_full.launch.py show it is, an... | Partially fixed |
| 23 | medium | cpp-localization | `src/agv_markers/src/marker_correction_node.cpp` | The declared and documented covariance_xy/covariance_yaw parameters are read into members that are never used — the published correction covariance is hardcoded 0.02/0... | Partially fixed |
| 24 | medium | cpp-localization | `src/agv_sensor_fusion/src/wheel_slip_detector.cpp` | The dwell-triggered slip branch contains a self-contradictory condition that is dead on all but the first-ever activation, so slip_started_t_ keeps a stale timestamp a... | Fixed |
| 25 | medium | cpp-localization | `src/agv_factor_graph/src/factor_graph_node.cpp` | The factor graph's 'independent validation' estimator actually consumes ekf_global's output (which already fused cuVSLAM and marker_pose) and then re-adds the same cuV... | Open (owner/roadmap) |
| 26 | medium | cpp-localization | `src/agv_localization_init/src/auto_init_orchestrator_node.cpp` | worker_.join() is called inside executor callbacks (blocking the only executor thread for up to ~100 s of cascade time, during which the running worker's own service r... | Fixed |
| 27 | medium | cpp-localization | `src/agv_scan_mapper/src/scan_grid_mapper_node.cpp` | Grid auto-expansion has no maximum size cap, so a single EKF pose jump makes ensure_bounds() allocate an arbitrarily large grid published at 1-2 Hz over reliable trans... | Fixed |
| 28 | medium | cpp-localization | `src/agv_markers/src/marker_correction_node.cpp` | The hand-rolled registry YAML parser calls std::stoi/std::stod with no exception handling on a file that the ui_backend rewrites at runtime and hot-reloads, so one mal... | Fixed |
| 37 | medium | cpp-navigation | `src/agv_behaviors/trees/waypoint_patrol.xml` | Two of the three shipped behavior trees reference node types that are not registered anywhere — including two that do not exist in any library — so they can never load... | Fixed |
| 38 | medium | cpp-navigation | `src/agv_image_server/src/image_server_node.cpp` | MJPEG stream workers are detached threads holding references to node members, and the destructor only joins the accept thread, so on shutdown live streams dereference ... | Fixed |
| 39 | medium | cpp-navigation | `src/agv_map_manager/src/map_manager_node.cpp` | The default_map startup loader stores its wall timer in a local variable that is destroyed when the constructor returns, so the timer is cancelled before it ever fires... | Fixed |
| 40 | medium | cpp-navigation | `src/agv_map_manager/src/map_manager_node.cpp` | Zone persistence rewrites zones.json in place with no tmp+rename (a crash mid-write loses every zone), and the waypoint missions file is seeded with a stray '[]' line ... | Partially fixed |
| 46 | medium | python | `src/agv_bringup/launch/agv_full.launch.py` | wheel_slip_detector and caster_dwell_advisor are the only nodes in agv_full.launch.py launched with no parameters at all — no use_sim_time and no config YAML — so in h... | Fixed |
| 47 | medium | python | `src/agv_bringup/TASK.yaml` | agv_bringup's TASK.yaml declares launch files that were deleted in the 2026-04-13 audit and omits the real HIL entrypoint, and the hardcoded-paths whitelist justifies ... | Partially fixed |
| 55 | medium | typescript | `web/agv_dashboard/src/hooks/useFleetSocket.ts` | The fleet WebSocket hook hardcodes host and port 8091 and ignores VITE_FLEET_BASE and the auth token, contradicting the project's no-hardcoded-endpoints rule and the c... | Fixed |
| 56 | medium | typescript | `src/agv_ui_backend/src/ws/control.ts` | Pervasive empty catch blocks silently swallow all errors — including programming errors in the WebSocket command handler — making field failures invisible. | Fixed |
| 57 | medium | typescript | `src/agv_ui_backend/src/app_deps.ts` | `any`-typed escape hatches are pervasive and no schema validation is applied to external MQTT/WebSocket payloads before they reach ROS publishers and action clients. | Partially fixed |
| 61 | medium | docs | `docs/dual_ekf_validation.md` | Operational/validation docs the README advertises as current are unrunnable or contradicted by the tree: the dual-EKF validation procedure and docs/hil_validation.md s... | Fixed |
| 62 | medium | docs | `CLAUDE.md` | Documentation language and structure are inconsistent: root CLAUDE.md promises every package CLAUDE.md follows a fixed Spanish-named section template (Responsabilidade... | Open (owner/roadmap) |
| 63 | medium | docs | `specs/acceptance.yaml` | The quality-gate spec that CLAUDE.md's Definition of Done points to (specs/acceptance.yaml) has declared itself non-authoritative and pending rewrite since April, so t... | Fixed in spec sync |
| 68 | medium | community | `README.md` | The README lacks every presentation-card element for a public release: no CI/license badges, no links to CONTRIBUTING.md, LICENSE, or CODE_OF_CONDUCT.md (so the genuin... | Partially fixed |
| 69 | medium | community | `AGENTS.md` | AGENTS.md — the filename most AI tools auto-load — is a stale third entry point that contradicts AGENT_INSTRUCTIONS.md: it mandates a .agent_locks.yaml locking workflo... | Fixed |
| 76 | medium | ssot-drift | `specs/state_machine.yaml` | state_machine.yaml contradicts itself about the mode_coherence invariant: the invariant block says the Nav2-liveness check was FIXED 2026-04-13 (which matches the code... | Fixed in spec sync |
| 77 | medium | ssot-drift | `specs/persistence.yaml` | persistence.yaml's zed_area_landing_pad writer/reader code_refs point to src/zed-ros2-wrapper/..., a tree that is not in the repo, so the shipped verification suite em... | Fixed in spec sync |
| 78 | medium | ssot-drift | `specs/interfaces.yaml` | /agv/wheel_odom QoS in the spec (best_effort) does not match the code, which publishes with the default RELIABLE profile. | Fixed in spec sync |
| 79 | medium | ssot-drift | `specs/state_machine.yaml` | Layer 1 (AGV_MODE) in state_machine.yaml declares only real/mapping/hil, but agv_start.sh implements a fourth mode 'hil_full' that runs the production launch with hil_... | Fixed in spec sync |
| 82 | medium | security | `specs/interfaces.yaml` | The specs give two contradictory, both-wrong descriptions of how to enable dashboard auth, so a deployer following the SSOT cannot actually turn auth on. | Fixed in spec sync |
| 83 | medium | security | `SECURITY.md` | SECURITY.md's posture table says the dashboard has JWT role-based auth without disclosing that auth is disabled by default and that a disabled state grants every WebSo... | Fixed |
| 84 | medium | security | `src/agv_image_server/src/image_server_node.cpp` | The unauthenticated MJPEG camera server sends Access-Control-Allow-Origin: * on streams and snapshots, letting any webpage open in a browser on the greenhouse LAN read... | Partially fixed |
| 90 | medium | build | `src/agv_ui_backend/scripts/teleop_backend.sh` | The installed backend wrapper resolves its entry point by climbing out of the install space into `<ws>/src/agv_ui_backend/dist/index.js`, so the installed package is n... | Partially fixed |
| 91 | medium | build | `src/agv_odrive/TASK.yaml` | agv_odrive — which now ships the production C++17 motor driver launched by agv_full.launch.py — still has a TASK.yaml declaring dev_only: true and 'current_language: P... | Fixed |
| 92 | medium | build | `src/agv_ui_backend/package.json` | Node package metadata contradicts the ROS packaging: agv_ui_backend declares license ISC (repo is MIT), version 1.0.0 (package.xml says 0.1.0), a nonexistent `main: in... | Partially fixed |
| 93 | medium | build | `web/agv_dashboard/package.json` | The flagship operator dashboard still carries its scaffold identity — package name 'agv_dash_init', version 0.0.0, and the unmodified stock Vite starter README that te... | Fixed |
| 94 | medium | build | `src/agv_waypoint_manager/package.xml` | Several packages use interfaces they never declare, compiling only through transitive luck: agv_waypoint_manager includes std_msgs without declaring it, agv_markers/ag... | Partially fixed |
| 95 | medium | build | `src/agv_behaviors/CMakeLists.txt` | Four packages institutionalize untracked, robot-only launch files ('launch/ exists only on commissioned robots (untracked)'), meaning part of the deployed launch layer... | Open (owner/roadmap) |
| 99 | medium | tests | `src/agv_integration_tests/CMakeLists.txt` | The four ROS-free pure-logic test suites never run in CI because they share a package with stack-required tests, and `colcon test --packages-select agv_integration_tes... | Partially fixed |
| 100 | medium | tests | `src/agv_factor_graph/src/factor_graph_node.cpp` | Substantive robot packages have zero tests: agv_factor_graph, agv_localization_init (both C++ runtime nodes), agv_hil_bridges (5 Python bridge scripts), and agv_bringu... | Open (owner/roadmap) |
| 107 | medium | newcomer | `README.md` | The repo's most distinctive feature — machine-readable specs enforced against the code by 9 automated pre-commit checks — is buried as a bare link list under 'Canonica... | Fixed |
| 29 | low | cpp-localization | `src/agv_markers/src/marker_correction_node.cpp` | yaw_from_tag is unconditionally set true inside its own scope, making the entire map->base_link TF heading fallback (with its warning log) unreachable dead code that m... | Fixed |
| 48 | low | python | `src/agv_hil_bridges/scripts/apriltag_sim_shim.py` | The HIL AprilTag shim hardcodes the greenhouse floor-tag ID list in its diagnostics and exits with code 0 when startup fails, so launch never notices the shim died. | Open (owner/roadmap) |
| 49 | low | python | `src/agv_bringup/launch/agv_full.launch.py` | AGV_BOOT_MAP_NAME is computed by splicing the raw map path into an eval'd PythonExpression using __import__('os'), which breaks on any path containing a quote and is n... | Fixed |
| 58 | low | typescript | `web/agv_dashboard/src/api/client.ts` | Any 401 response triggers a full page reload, risking a reload loop and loss of UI state. | Fixed |
| 85 | low | security | `specs/hmi_api.yaml` | The WebSocket auth design places the JWT in a URL query parameter, where it leaks into proxy/server logs and browser history. | Open (owner/roadmap) |
| 101 | low | tests | `src/agv_odrive/test/test_flake8.py` | Dead test files are committed but never wired into the build, misleading readers about coverage. | Partially fixed |
| 105 | low | consistency | `src/agv_odrive/resource/agv_odrive` | Leftover ament_python scaffolding survives in packages that migrated to CMake/TypeScript: empty resource/ index stubs and an orphan Python module directory. | Fixed |
| 108 | low | newcomer | `README.md` | Domain acronyms are used at first contact without expansion or links: HIL, cuVSLAM, ISA-101, ODrive, tag36h11 — a robotics generalist has to leave the repo to decode t... | Fixed |

