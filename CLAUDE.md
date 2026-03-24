# AGV Greenhouse — CLAUDE.md

Read this file before modifying any code.

## Canonical source order
1. `/specs/project.yaml`
2. `/specs/interfaces.yaml`
3. `/specs/acceptance.yaml`
4. `/agents/registry.yaml`
5. `/policies/engineering_rules.md`
6. relevant package `TASK.yaml`

## Project context
AGV differential-drive robot for greenhouse deployment in Mexico.
Target for current MVP: first field visit with local WiFi operator workflow.
Development compute: Jetson AGX Orin 64GB.
Production compute: Jetson Orin NX 16GB.

## Package-level CLAUDE.md
Some packages have their own CLAUDE.md with package-specific detail (e.g., `src/agv_slam/CLAUDE.md`).
Workspace-level rules here take precedence on any conflict. Package CLAUDE.md files add specificity, not override.

## Absolute rules
- Every ROS2 robot node is C++17 only
- Python ROS2 packages serving exclusively as development, commissioning, or diagnostic tools are permitted as interim dev tooling. They must be marked `dev_only: true` in their TASK.yaml and replaced with C++17 before production.
- No Python ROS2 nodes in the robot runtime stack
- No hardcoded physical parameters, marker IDs, namespace values, or Jetson IPs
- All configuration must come from YAML or environment
- Build warnings are treated as errors
- Do not edit files locked by another agent
- Do not describe software-only collision handling as certified safety

## Canonical decisions
- Marker system: AprilTag family tag36h11
- Goal dispatch: `/navigate_to_pose` action
- Robot namespace source: parameter `robot_namespace`, default `agv`
- Localization architecture: dual EKF
  - local filter owns `odom -> base_link`
  - global filter owns `map -> odom`
- Commissioning mapping protocol:
  - nominal speed 0.3 to 0.5 m/s
  - prefer bi-directional passes in operational corridors
  - perform runs when dynamic activity is minimal
- AprilTags are pose anchors and drift correctors, not the sole localization strategy

## Greenhouse-specific cautions
- Crop rows can be visually repetitive
- Lighting changes across the day can degrade visual SLAM
- Wet or reflective surfaces can disturb visual features
- Wheel odometry must remain fused continuously
- Any map revision must document its occupancy-grid generation pipeline

## Hardware setup
See `docs/hardware_setup.md` for real CAN/pinmux/ODrive setup documentation.
The `can0 available` prerequisite is NOT trivial — that document explains why.

## Safety boundary
This repository can define operational safeguards, but it does not create certified functional safety by itself.
If a requirement involves certified human safety, safety scanners, safety PLC/relay logic, or compliance claims, treat that as external hardware-integrated scope unless explicitly specified otherwise.

## Definition of done
A task is done only when:
- it builds with zero warnings
- tests pass
- runtime interface complies with `/specs/interfaces.yaml`
- relevant acceptance gate passes
