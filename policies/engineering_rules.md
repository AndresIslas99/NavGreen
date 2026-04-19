# Engineering Rules

## Rule 0 — Language policy
- Every ROS2 robot node is written in **C++17**
- Python is prohibited in the robot runtime stack
- Python ROS2 packages serving exclusively as development, commissioning, or diagnostic tools are permitted as interim dev tooling. They must be marked `dev_only: true` in their TASK.yaml and replaced with C++17 before production deployment.
- TypeScript is allowed for the UI backend and dashboard
- Python is allowed only for offline tooling and generators

## Rule 1 — No hardcode
Do not hardcode:
- physical robot dimensions
- CAN interface names beyond defaults
- Jetson IP addresses
- marker IDs
- map paths
- namespace values
- greenhouse row numbers or site-specific routes

All runtime values must come from:
- ROS2 parameters
- YAML config
- environment variables
- persisted editable mission or map data

## Rule 2 — Logic over sequences
Do not hardcode missions, greenhouse-specific routes, or row numbers in source code.
Missions and zones must come from persisted data and editable configuration.

## Rule 3 — Greenhouse perception rules
- Treat visual SLAM as environmentally fragile in repetitive crop rows and changing lighting
- Stereo-inertial mode is required for the current MVP
- Wheel odometry must remain fused continuously
- AprilTags are supplemental pose anchors, not a substitute for full localization
- Mapping runs must be executed with low dynamic activity and documented revision metadata

## Rule 4 — Failure protocol
When build or tests fail:
1. read the full error
2. identify root cause
3. apply the minimum valid fix
4. rebuild
5. verify no regressions
6. after 3 failed attempts, document in `BLOCKERS.md`

## Rule 5 — Completion policy
A task is not done until:
- build succeeds with zero warnings
- tests pass
- runtime behavior matches `TASK.yaml`
- relevant acceptance gate is satisfied

## Rule 6 — Safety boundary
- Do not claim software-only collision handling is certified functional safety
- `nav2_collision_monitor` and software E-stop handling are operational safeguards only
- Safety-rated scanners, safety PLC logic, and formal compliance work sit outside this MVP unless explicitly added as hardware-integrated scope

## Rule 7 — Canonical terminology
- Use **AprilTag** and **tag36h11**
- Do not use QR or ArUco terminology for the implemented marker system
- Use **`/navigate_to_pose` action** as the canonical goal interface

## Rule 8 — Production-first development
Develop every node, algorithm, and pipeline as if it will run on the real robot.
- Simulation provides raw sensor data (the same topics the real hardware would publish)
- All processing (filtering, fusion, navigation, perception) runs on the Jetson
- Never implement workarounds on the sim side that wouldn't exist in production
- If a pipeline works in sim but requires sim-side processing, it is architecturally wrong
- HIL mode must exercise the same code paths as production — only the sensor source changes

## Rule 9 — Interface change governance
`specs/interfaces.yaml` is the binding contract for TF frames, topics, services, actions, and message rates across the workspace. Any change to it requires:
- The PR description must list every package that produces or consumes the affected interface and explain why the new shape is necessary.
- Review by every team that owns one of those packages before merge.
- Bumping the `spec_version` field at the top of `interfaces.yaml` on every merge.
- A corresponding update to any package whose runtime behavior no longer matches the contract, in the same PR or a follow-up PR linked from the description.

Adding a new interface follows the same process: list the producing package, list the intended consumers, and bump the version.
