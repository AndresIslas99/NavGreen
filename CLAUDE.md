# AGV Greenhouse — CLAUDE.md

Read this file before modifying any code. If you are an AI coding agent,
read [AGENT_INSTRUCTIONS.md](AGENT_INSTRUCTIONS.md) **first**, then this
file, then the specs.

## Single Source of Truth (SSOT)

The machine-readable contracts in [specs/](specs/) are authoritative.
If a spec and the code disagree, one of them is a bug. Pre-commit hooks
enforce that specs and code stay in sync.

## Canonical source order
1. [specs/README.md](specs/README.md) — meta-doc for the spec directory
2. [specs/project.yaml](specs/project.yaml) — phase, deployment targets
3. [specs/state_machine.yaml](specs/state_machine.yaml) — operation modes and transitions
4. [specs/launch_sequence.yaml](specs/launch_sequence.yaml) — startup DAG, timings, preconditions
5. [specs/persistence.yaml](specs/persistence.yaml) — persistent artifacts, writers, readers
6. [specs/interfaces.yaml](specs/interfaces.yaml) — topic/service/action contracts
7. [specs/acceptance.yaml](specs/acceptance.yaml) — quality gates
8. [agents/registry.yaml](agents/registry.yaml) — agent roles
9. [policies/engineering_rules.md](policies/engineering_rules.md) — Rules 0–9
10. [AGENT_INSTRUCTIONS.md](AGENT_INSTRUCTIONS.md) — rules for AI agents
11. Relevant package `CLAUDE.md` and `TASK.yaml`

## Verification

Before committing:
```bash
bash tools/verify_specs/all.sh
```

This runs 9 checks that enforce the rules below. A pre-commit hook
(installed via `bash tools/verify_specs/install_git_hook.sh`) runs the
same checks automatically. The BLOCKING set must pass before a commit
is accepted.

## Project context
AGV differential-drive robot for greenhouse deployment in Mexico.
Target for current MVP: first field visit with local WiFi operator workflow.
Development compute: Jetson AGX Orin 64GB.
Production compute: Jetson Orin NX 16GB.

## Package-level CLAUDE.md
Every AGV package has its own `CLAUDE.md` with the following sections:
- **Responsabilidades** — what the package does and does NOT do
- **Interfaces propias** — topics/services/actions the package owns (cross-referenced to `specs/interfaces.yaml`)
- **Interfaces consumidas** — what the package consumes from others
- **Invariantes** — rules that must always hold
- **Failure modes** — what happens downstream if this package fails
- **Relación con otros specs** — links into `specs/*.yaml`

Workspace-level rules here take precedence on any conflict. Package CLAUDE.md
files add specificity, not override.

## Absolute rules
- Every ROS2 robot node is C++17 only
- Python ROS2 packages serving exclusively as development, commissioning, or diagnostic tools are permitted as interim dev tooling. They must be marked `dev_only: true` in their TASK.yaml and replaced with C++17 before production.
- No Python ROS2 nodes in the robot runtime stack
- No hardcoded physical parameters, marker IDs, namespace values, or Jetson IPs
- All configuration must come from YAML or environment
- Build warnings are treated as errors
- Do not edit files locked by another agent
- Do not describe software-only collision handling as certified safety
- Develop as production: every algorithm, node, and pipeline must be designed to run on the Jetson in production. Simulation only provides sensor inputs — processing belongs on the Jetson. Never push compute or filtering to the sim side that would run on the robot in the real world.

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

## Audit trail

The workspace underwent a full audit on 2026-04-13 to consolidate implicit
behavior into machine-readable specs. The audit document at
[docs/audit/2026-04-13-full-audit.md](docs/audit/2026-04-13-full-audit.md)
lists known bugs, rule violations, and drift evidence. The specs in
`specs/*.yaml` are the synthesis of that audit. Any future agent
(human or AI) that finds the specs contradicting reality should update
the audit document and the specs together.

## Definition of done

A task is done only when:
- **The specs are up to date.** If you added a topic, it is in `specs/interfaces.yaml`. If you added a persistent artifact, it is in `specs/persistence.yaml`. If you changed a mode, it is in `specs/state_machine.yaml`.
- **`bash tools/verify_specs/all.sh` reports zero BLOCKING failures.**
- **The build succeeds with zero warnings** (`-Werror` is mandatory in every AGV C++ package).
- **Tests pass** (`colcon test` for touched packages).
- **The relevant acceptance gate passes** (see [specs/acceptance.yaml](specs/acceptance.yaml)).
- **The runtime interface matches** [specs/interfaces.yaml](specs/interfaces.yaml) (type, QoS, namespace).
