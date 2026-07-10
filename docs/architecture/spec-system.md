# The spec system (SSOT)

The most distinctive thing in this repository is not a node. It is the
contract system around the nodes: machine-readable specs that describe every
interface, mode, launch step, and persistent file — plus nine verifiers that
block any commit where code and spec disagree. If you study one thing in
NavGreen, study this.

## Why it exists

By early 2026 the workspace had accumulated drift faster than documentation
could keep up: launch files nobody invoked, topics whose declared QoS did not
match the code, a safety watchdog monitoring an event-driven topic that
silently blocked teleop for minutes (the "teleop-broken" incident — audit
bug #1). A full audit on 2026-04-13 consolidated all implicit behavior into
machine-readable specs and made them the **Single Source of Truth (SSOT)**:

> If a spec and the code disagree, one of them is a bug.

The audit document
([`docs/audit/2026-04-13-full-audit.md`](https://github.com/AndresIslas99/NavGreen/blob/main/docs/audit/2026-04-13-full-audit.md))
records the bugs and drift evidence that justified each contract; the specs
are its synthesis. A second reconciliation on 2026-07-06 registered the rail
stack, the fleet layer, and hardened the verifiers.

The rule that keeps it true: **any change that adds, removes, or renames a
topic, service, action, mode, launch node, or persistent artifact must update
the corresponding spec in the same commit.** The pre-commit hook rejects
commits that don't.

## The contracts

Seven YAML files under
[`specs/`](https://github.com/AndresIslas99/NavGreen/blob/main/specs/README.md),
each opening with a `spec_version` header whose presence is itself verified:

| Spec | Contract | Answers questions like |
|---|---|---|
| [`project.yaml`](https://github.com/AndresIslas99/NavGreen/blob/main/specs/project.yaml) | Phase, success definition, deployment targets, canonical deployment constants (ROS domain, ports, data dir), hard constraints | "What is the ROS_DOMAIN_ID and why 42?" |
| [`state_machine.yaml`](https://github.com/AndresIslas99/NavGreen/blob/main/specs/state_machine.yaml) | The five-layer mode matrix, valid combinations, invariants, authorized transitions | "Who publishes `map -> odom` in mapping mode?" |
| [`launch_sequence.yaml`](https://github.com/AndresIslas99/NavGreen/blob/main/specs/launch_sequence.yaml) | The startup DAG of `agv_full.launch.py`: per-node timing, preconditions, failure impact, HIL conditions | "What breaks downstream if `ekf_local` fails to start at t=4 s?" |
| [`persistence.yaml`](https://github.com/AndresIslas99/NavGreen/blob/main/specs/persistence.yaml) | Every persistent artifact: writer, readers, format, lifecycle, atomicity rules | "Who writes `<map>_meta.json` and who reads it back?" |
| [`interfaces.yaml`](https://github.com/AndresIslas99/NavGreen/blob/main/specs/interfaces.yaml) | Every cross-package ROS interface — 60 topics, 15 services, 1 action at last count — with owner, type, QoS, rate, and subscribers | "What QoS does `/agv/live_map` use and who consumes it?" |
| [`hmi_api.yaml`](https://github.com/AndresIslas99/NavGreen/blob/main/specs/hmi_api.yaml) | The dashboard ↔ backend REST + WebSocket contract, roles, and the action gates protecting dispatch | "Which endpoints stay unauthenticated so the robot can always be stopped?" |
| [`acceptance.yaml`](https://github.com/AndresIslas99/NavGreen/blob/main/specs/acceptance.yaml) | Quality gates per phase, from static (spec sync, `-Werror` build) to HIL closed-loop precision numbers | "What does 'done' mean for a waypoint-precision change?" |

The four structural contracts (interfaces, state machine, launch sequence,
persistence) also record their provenance and the script that enforces them —
here the header of `interfaces.yaml`:

```yaml
spec_version: "2.1"
last_updated: "2026-07-06"
generated_from: "docs/audit/2026-04-13-full-audit.md#section-4 (...)"
verified_by: "tools/verify_specs/verify_interfaces.py"
```

The specs deliberately do **not** contain code-level documentation (that
lives in per-package `CLAUDE.md` files) or every internal parameter — only
contracts that cross package boundaries.

## The nine verifiers

[`tools/verify_specs/all.sh`](https://github.com/AndresIslas99/NavGreen/blob/main/tools/verify_specs/all.sh)
runs the suite: five BLOCKING scripts and four WARNING scripts. All are
stdlib-only bash/Python, so they run anywhere in seconds.

```bash
bash tools/verify_specs/all.sh
```

**BLOCKING tier** — a non-zero exit fails the suite:

| Verifier | What it catches |
|---|---|
| `verify_canonical_sources.sh` | A canonical source file (specs, engineering rules, agent registry, root CLAUDE.md) missing, empty, or lacking its `spec_version` header |
| `verify_no_hardcoded_paths.sh` | Hardcoded deployment paths (`/home/orza/`, `/mnt/ssd/`) or IP addresses (`192.168.`) in `src/`, against an explicit, rationale-documented whitelist |
| `verify_werror.sh` | Any AGV C++ package whose `CMakeLists.txt` does not compile with `-Werror` |
| `verify_dev_only.py` | A Python ROS 2 node reachable from a production launch file whose package is not marked `dev_only: true` — enforcing the C++17-only robot-runtime rule |
| `verify_interfaces.py` | Four passes over `interfaces.yaml`: (1) the declared owner package really references each interface name as a quoted string literal, (2) every declared consumer does too, (3) the declared message/service type appears in the owner package, and (4) a **reverse pass** — every absolute `/agv/...` name passed to a publisher/subscription/service/action creation call in the code must be declared in the spec. Undeclared interfaces and name/type drift cannot land. |

**WARNING tier** — prints `WARN:` without blocking, **but** any structural
`FAIL:` line from these scripts still blocks the suite (a gutted spec must
never pass green):

| Verifier | What it catches |
|---|---|
| `verify_claude_md_coverage.sh` | An AGV package missing its `CLAUDE.md` or `TASK.yaml` |
| `verify_persistence.py` | A `persistence.yaml` writer `code_ref` pointing at a file that no longer exists |
| `verify_state_machine.py` | Structural: required keys present, and the layer-5 arbiter states in the spec **must match `enum class Mode` in `mode_fsm.hpp`, in enum order** — the FSM and its spec cannot drift (FAIL, blocking) |
| `verify_launch_sequence.py` | A `source:` path referencing a deleted launch file (FAIL, blocking); a vanished greppable anchor inside a file that still exists (WARN) |

Two more escalation rules in `all.sh` are worth copying into your own
projects: a **missing BLOCKING script is itself a blocking failure** (you
cannot disable a gate by deleting it), and FAIL lines block **from any
tier**.

## Enforcement: pre-commit hook and CI

Install once per clone:

```bash
bash tools/verify_specs/install_git_hook.sh
```

The hook runs the identical suite before every commit and rejects the commit
on any blocking failure. Bypassing with `git commit --no-verify` is reserved
for active incident response, noted in the commit message.

CI runs the same suite as a dedicated `spec-verification` job in
[`.github/workflows/ci.yaml`](https://github.com/AndresIslas99/NavGreen/blob/main/.github/workflows/ci.yaml),
alongside the `-Werror` build/test job, the TypeScript builds, and the
simulation job — so a contributor who skipped the hook still cannot merge
drift.

## Built for agents (and better for humans)

The workspace assumes AI coding agents will modify it, and codifies how:

- [`AGENT_INSTRUCTIONS.md`](https://github.com/AndresIslas99/NavGreen/blob/main/AGENT_INSTRUCTIONS.md)
  defines a specs-first workflow: a required reading order (root
  `CLAUDE.md` → `specs/README.md` → the relevant specs → engineering rules →
  package `CLAUDE.md` → only then code), a six-step change procedure in
  which **specs are updated before code**, and a "no invention" rule — if
  the spec does not say it, do not assume it, with real examples of past bad
  assumptions.
- **Every package carries a `CLAUDE.md` contract** with the same sections:
  responsibilities (what it does *and does not* do), owned interfaces,
  consumed interfaces, invariants, failure modes, and cross-references into
  the specs. These double as excellent package documentation for humans —
  e.g.
  [`agv_mode_arbiter/CLAUDE.md`](https://github.com/AndresIslas99/NavGreen/blob/main/src/agv_mode_arbiter/CLAUDE.md)
  or
  [`agv_safety/CLAUDE.md`](https://github.com/AndresIslas99/NavGreen/blob/main/src/agv_safety/CLAUDE.md).
- [`policies/engineering_rules.md`](https://github.com/AndresIslas99/NavGreen/blob/main/policies/engineering_rules.md)
  states the enforceable Rules 0–9 (language policy, no hardcode, safety
  boundary, canonical terminology, interface-change governance, ...), and
  the verifiers are their teeth.

## A worked example: adding a topic

Suppose you add `/agv/battery_estimate` published by `agv_odrive`:

1. Declare it in `specs/interfaces.yaml` — owner, type, QoS, rate,
   subscribers — **first**.
2. Write the C++ (in `agv_odrive`, with `-Werror` already enforced).
3. Run `bash tools/verify_specs/all.sh`.

Skip step 1 and `verify_interfaces.py`'s reverse pass finds an undeclared
absolute `/agv/...` literal next to a `create_publisher` call and blocks the
commit. Declare it but typo the owner package, and the owner-presence pass
blocks it too. Add a subscriber entry for a package that never subscribes,
and the consumer pass catches that. The failure message names the script and
the fix.

## Definition of done

A task is complete only when
([`specs/acceptance.yaml`](https://github.com/AndresIslas99/NavGreen/blob/main/specs/acceptance.yaml)
is authoritative):

- specs are up to date and `bash tools/verify_specs/all.sh` reports zero
  blocking failures,
- the build succeeds with zero warnings (`-Werror` everywhere),
- `colcon test` passes for touched packages,
- the runtime interface matches `interfaces.yaml` (type, QoS, namespace),
- the acceptance gate relevant to the change passes.

## Honest limits

The verifiers are static and lexical. They catch name, type, ownership, and
existence drift — the failure modes actually observed in the audits — but
they do not verify QoS depth at runtime, launch-time remapping semantics, or
behavioral invariants (those belong to runtime health checks and the HIL
gates). `verify_persistence.py` checks that writer files exist, not that
they write. `hmi_api.yaml` does not have its verifier yet (`verified_by:
TODO` in its header). The system's value is not perfection; it is that the
*documented* contract can no longer silently rot.
