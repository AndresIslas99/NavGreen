# Contributing

Contributions of all sizes are welcome — code, tests, documentation, and
field reports. This page is the docs-site summary; the canonical, always
up-to-date guide is
[`CONTRIBUTING.md`](https://github.com/AndresIslas99/NavGreen/blob/main/CONTRIBUTING.md)
on GitHub.

## The one rule that matters most: specs first

NavGreen is **spec-driven**. The machine-readable contracts in
[`specs/`](https://github.com/AndresIslas99/NavGreen/tree/main/specs)
are the Single Source of Truth, and automated verifiers keep code and specs in
sync. If your change touches a ROS interface, a persistent artifact, an
operation mode, or a launch step, the matching spec file must change **in the
same commit**:

| You changed… | Update |
|---|---|
| A topic / service / action / cross-package parameter | [`specs/interfaces.yaml`](https://github.com/AndresIslas99/NavGreen/blob/main/specs/interfaces.yaml) |
| A persistent file or folder the robot reads/writes | [`specs/persistence.yaml`](https://github.com/AndresIslas99/NavGreen/blob/main/specs/persistence.yaml) |
| An operation mode, transition, or runtime state | [`specs/state_machine.yaml`](https://github.com/AndresIslas99/NavGreen/blob/main/specs/state_machine.yaml) |
| Nodes in a launch file | [`specs/launch_sequence.yaml`](https://github.com/AndresIslas99/NavGreen/blob/main/specs/launch_sequence.yaml) |

Verify locally before committing:

```bash
bash tools/verify_specs/all.sh                # 9 checks; the BLOCKING set must pass
bash tools/verify_specs/install_git_hook.sh   # once, to install the pre-commit hook
```

The same suite runs in CI on every PR. See
[Interfaces & specs](../reference/interfaces.md) for how to read and update
the interface contract.

## Ground rules

These are absolute — the full set lives in
[`CLAUDE.md`](https://github.com/AndresIslas99/NavGreen/blob/main/CLAUDE.md)
and
[`policies/engineering_rules.md`](https://github.com/AndresIslas99/NavGreen/blob/main/policies/engineering_rules.md):

- **Robot runtime nodes are C++17 only.** Python ROS 2 packages are allowed
  only as development/commissioning/diagnostic tooling, marked
  `dev_only: true` in their `TASK.yaml`.
- **Build warnings are errors.** Every AGV C++ package compiles with
  `-Wall -Wextra -Werror`. Fix warnings; never silence them with `-Wno-*`.
- **No hardcoded physical parameters, marker IDs, namespaces, or IPs.**
  Configuration comes from YAML files or environment variables.
- **No absolute paths.** Use `AGV_DATA_DIR` or a ROS parameter.
- **Never describe the software safeguards as certified safety.** See the
  [security & safety page](security.md).

## Getting a working environment

The fastest path is the dev container in
[`.devcontainer/`](https://github.com/AndresIslas99/NavGreen/tree/main/.devcontainer):
open the repo in VS Code and "Reopen in Container" (any
devcontainer-compatible tool works). It matches CI — ROS 2 Humble, colcon,
Node 20 — resolves workspace dependencies on first launch, and sets
`AGV_DATA_DIR` for you, so `colcon build` and the TypeScript builds work with
no host setup.

Without the container, follow [Getting started](../getting-started.md). Most
of the stack builds and tests without a robot or GPU; three packages need
vendor SDKs and are skipped in CI (see the
[package reference](../reference/packages.md)).

## Where to start

Issues labeled
[`good first issue`](https://github.com/AndresIslas99/NavGreen/labels/good%20first%20issue)
are curated to be self-contained and hardware-free — at the time of writing,
things like clearing the dashboard's pre-existing eslint errors
([#19](https://github.com/AndresIslas99/NavGreen/issues/19)), an
optional CI job for the vendor-SDK packages
([#13](https://github.com/AndresIslas99/NavGreen/issues/13)), and a
dashboard demo GIF for the README
([#14](https://github.com/AndresIslas99/NavGreen/issues/14)). The
larger open workstreams are on the [roadmap](roadmap.md).

Good first contributions beyond the labeled issues: enabling more packages in
CI, adding unit tests to packages that have none, documentation fixes, and
translating docs.

Read first: the README, then
[`specs/README.md`](https://github.com/AndresIslas99/NavGreen/blob/main/specs/README.md),
then the `CLAUDE.md` of the package you want to touch — every package has one
describing its responsibilities, interfaces, and invariants.

## Pull requests and review

1. Fork, create a topic branch, keep the change focused (a PR touching 5+
   packages is probably several PRs).
2. Update the relevant `specs/*.yaml` and the package `CLAUDE.md` if behavior
   changed.
3. `bash tools/verify_specs/all.sh` must report zero BLOCKING failures, and
   touched packages must build with zero warnings.
4. Fill in the PR template checklist honestly — "not applicable" is a fine
   answer when it's true.

**Maintainership.** The project currently has one maintainer,
[@AndresIslas99](https://github.com/AndresIslas99), who reviews PRs and makes
final calls on architecture and scope. What to expect:

- **Review turnaround**: best effort within a week. This is a working robot
  deployed in a commercial greenhouse — field weeks happen; a quiet spell is
  not abandonment.
- **What merges**: green CI, spec sync honored, and a diff the maintainer can
  reason about. For large changes, open an issue first and agree on the
  approach.
- **Becoming a co-maintainer**: a track record of good PRs and reviews in a
  subsystem is the path; ask once you have a few merged.
- **Hardware-dependent claims**: if your change affects robot behavior and
  you don't have hardware, say so in the PR — the maintainer validates on the
  robot or in HIL before merging. Never claim field validation you didn't do.

!!! tip "Using an AI coding agent?"
    The repository is deliberately agent-friendly. Point your agent at
    [`AGENT_INSTRUCTIONS.md`](https://github.com/AndresIslas99/NavGreen/blob/main/AGENT_INSTRUCTIONS.md)
    before it writes anything — the same spec-sync rules apply to humans and
    machines, and the pre-commit verifiers enforce them for both.

Be excellent to each other — see the
[Code of Conduct](https://github.com/AndresIslas99/NavGreen/blob/main/CODE_OF_CONDUCT.md).
