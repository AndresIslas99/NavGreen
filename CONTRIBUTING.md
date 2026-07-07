# Contributing to AGV Greenhouse

Thanks for your interest! This is a working codebase for a differential-drive
autonomous robot deployed in commercial greenhouses in Mexico. Contributions
of all sizes are welcome — code, tests, documentation, and field reports.

## The one rule that matters most: specs first

This workspace is **spec-driven**. The machine-readable contracts in
[`specs/`](specs/) are the Single Source of Truth (SSOT), and pre-commit
verification keeps code and specs in sync. If your change adds or modifies a
ROS interface, a persistent artifact, an operation mode, or a launch step, the
matching spec file must change **in the same commit**:

| You changed… | Update |
|---|---|
| A topic / service / action / cross-package parameter | [`specs/interfaces.yaml`](specs/interfaces.yaml) |
| A persistent file or folder the robot reads/writes | [`specs/persistence.yaml`](specs/persistence.yaml) |
| An operation mode, transition, or runtime state | [`specs/state_machine.yaml`](specs/state_machine.yaml) |
| Nodes in a launch file | [`specs/launch_sequence.yaml`](specs/launch_sequence.yaml) |

Verify locally before committing:

```bash
bash tools/verify_specs/all.sh          # 9 checks; BLOCKING set must pass
bash tools/verify_specs/install_git_hook.sh   # once, to install the pre-commit hook
```

## Ground rules

These are absolute — see [`CLAUDE.md`](CLAUDE.md) and
[`policies/engineering_rules.md`](policies/engineering_rules.md) for the full
set:

- **Robot runtime nodes are C++17 only.** Python ROS 2 packages are allowed
  only as development/commissioning/diagnostic tooling and must be marked
  `dev_only: true` in their `TASK.yaml`.
- **Build warnings are errors.** Every AGV C++ package compiles with
  `-Wall -Wextra -Werror`. Never silence a warning with `-Wno-*`; fix it.
- **No hardcoded physical parameters, marker IDs, namespaces, or IPs.** All
  configuration comes from YAML files or environment variables.
- **No absolute paths.** Use the `AGV_DATA_DIR` environment variable or a ROS
  parameter.

## Building and testing

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_CXX_FLAGS="-Werror"
colcon test && colcon test-result --verbose
```

Some packages need vendor dependencies that are not on the public apt index
(NVIDIA Isaac ROS for `agv_map_manager`, the ZED SDK for
`agv_localization_init`, GTSAM for `agv_factor_graph`). CI builds the subset
that works with stock ROS 2 Humble — see
[`.github/workflows/ci.yaml`](.github/workflows/ci.yaml). You can develop and
test most of the stack without a robot or GPU.

The TypeScript pieces build independently of ROS hardware:

```bash
cd web/agv_dashboard && npm ci && npm run build     # operator dashboard
cd fleet/agv_fleet_manager && npm ci && npm run build
# src/agv_ui_backend and fleet/agv_vda5050_adapter need a sourced ROS 2
# environment because rclnodejs compiles native bindings at install time.
```

## Pull requests

1. Fork, create a topic branch, keep the change focused (a PR touching 5+
   packages is probably several PRs).
2. Update the relevant `specs/*.yaml` and the package `CLAUDE.md` if behavior
   changed.
3. Make sure `bash tools/verify_specs/all.sh` reports zero BLOCKING failures
   and the touched packages build with zero warnings.
4. Fill in the PR template checklist honestly — "not applicable" is a fine
   answer when it's true.

## Where to start

- **Good first contributions**: enabling more packages in CI, adding unit
  tests to packages that have none, documentation fixes, translating docs.
- **Read first**: [`README.md`](README.md), then
  [`specs/README.md`](specs/README.md), then the `CLAUDE.md` of the package
  you want to touch — every package has one describing its responsibilities,
  interfaces, and invariants.

## A note on AI coding agents

This repository is deliberately agent-friendly: if you use Claude Code,
Copilot, Cursor, or similar, point your agent at
[`AGENT_INSTRUCTIONS.md`](AGENT_INSTRUCTIONS.md) before it writes anything.
The same spec-sync rules apply to humans and machines alike, and the
pre-commit verifiers enforce them for both.

## Maintainership and decisions

The project currently has one maintainer, [@AndresIslas99](https://github.com/AndresIslas99),
who reviews PRs and makes final calls on architecture and scope (the open
architectural questions live in
[`docs/reviews/2026-07-06-community-readiness-review.md`](docs/reviews/2026-07-06-community-readiness-review.md)
and the issue tracker). Practical expectations:

- **Review turnaround**: best effort within a week. This is a working robot
  deployed in a commercial greenhouse — field weeks happen; a quiet spell is
  not abandonment.
- **What merges**: green CI (all three jobs), spec sync honored, and a diff
  the maintainer can reason about. Large changes go smoother if you open an
  issue first and agree on the approach.
- **Becoming a co-maintainer**: a track record of good PRs and reviews in a
  subsystem is the path; ask once you have a few merged. `CODEOWNERS` is
  split by subsystem as co-maintainers join.
- **Hardware-dependent claims**: if your change affects robot behavior and
  you don't have hardware, say so in the PR — the maintainer validates on
  the robot or in HIL before merging. Never claim field validation you
  didn't do.

## Releases

Versions are tagged from `main` (`v0.x.y`, SemVer) with notes in
[`CHANGELOG.md`](CHANGELOG.md). Add a line to the `Unreleased` section of
the changelog when your change is user-visible.

## Code of conduct

Be excellent to each other — see [`CODE_OF_CONDUCT.md`](CODE_OF_CONDUCT.md).
