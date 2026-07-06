# specs/ — Single Source of Truth for the AGV workspace

This directory holds the **machine-readable contracts** that describe the
runtime behavior of the AGV greenhouse robot. They exist so that humans and
AI coding agents can answer questions like "which node owns the
`map→odom` transform?" or "what happens in the system when the operator
clicks Load Map?" without reading C++ code.

These files are the **Single Source of Truth (SSOT)**. If a spec and the
code disagree, one of them is a bug. The `tools/verify_specs/` scripts
detect those disagreements and block commits via a pre-commit hook.

## Reading order

When approaching the codebase (as a new contributor or as an AI agent),
read these files in order, top to bottom:

1. **[/CLAUDE.md](../CLAUDE.md)** — project overview, absolute rules, canonical source pointers
2. **[project.yaml](project.yaml)** — phase, deployment targets, commissioning protocol
3. **[state_machine.yaml](state_machine.yaml)** — the matrix of valid runtime modes and their invariants
4. **[launch_sequence.yaml](launch_sequence.yaml)** — DAG of startup, node dependencies, timings
5. **[persistence.yaml](persistence.yaml)** — inventory of persistent artifacts (who writes, who reads)
6. **[interfaces.yaml](interfaces.yaml)** — topic/service/action contracts (type, QoS, namespace, rate)
7. **[acceptance.yaml](acceptance.yaml)** — quality gates and task completion criteria. Rewritten 2026-07-06 (`status: active`) — authoritative; CLAUDE.md's Definition of Done points here.
8. **[hmi_api.yaml](hmi_api.yaml)** — operator HMI ↔ `agv_ui_backend` contract: REST endpoints, WebSocket protocol (`/ws/control`, `/ws/teleop`), action gates that protect dispatch (nav goals, mode transitions, motor enable). Authoritative once a frontend depends on it.
9. **[/policies/engineering_rules.md](../policies/engineering_rules.md)** — enforceable rules (Rules 0–9)
10. **[/agents/registry.yaml](../agents/registry.yaml)** — agent roles and coordination rules
11. **[/AGENT_INSTRUCTIONS.md](../AGENT_INSTRUCTIONS.md)** — what an AI agent must do before proposing changes
12. **Relevant package `CLAUDE.md`** — package-specific invariants and ownership
13. **Code**

## Update rule

**Any change to the code that adds, removes, or renames a topic/service/
action/parameter/mode/persistent artifact MUST update the corresponding
spec in the same commit.** The pre-commit hook rejects commits that
introduce new ROS interfaces without a matching spec update.

If you are unsure whether a change needs a spec update, read
[/AGENT_INSTRUCTIONS.md](../AGENT_INSTRUCTIONS.md) first.

## Spec file headers

Every spec YAML has a standard header:

```yaml
spec_version: 2.0
last_updated: "YYYY-MM-DD"
generated_from: "docs/audit/YYYY-MM-DD-full-audit.md"
verified_by: "tools/verify_specs/verify_<name>.py"
```

- `spec_version` — major.minor; bump major on breaking schema changes
- `last_updated` — ISO date
- `generated_from` — the audit document that justifies the current content
- `verified_by` — the check script that enforces this spec against the code

## What lives where

| Spec | Scope | Maintainer |
|---|---|---|
| `project.yaml` | Phase of the project, success definition, deployment targets, commissioning protocol, hard constraints | Product lead + engineering lead |
| `state_machine.yaml` | Operation modes (systemd, launch, runtime, state) and valid transitions | Architect |
| `launch_sequence.yaml` | Bringup DAG: which node starts when, preconditions, failure modes | Launch owner (`agv_bringup`) |
| `persistence.yaml` | Filesystem artifacts (maps, state, logs, configs), writers, readers, lifecycle | Platform lead |
| `interfaces.yaml` | ROS topics, services, actions, QoS, namespace, rates | Each package owner for its own interfaces |
| `acceptance.yaml` | Quality gates, acceptance tests, task completion criteria | QA lead |
| `hmi_api.yaml` | REST + WebSocket contract between operator HMI and agv_ui_backend, with action gates | UI/HMI owner (`agv_ui_backend`) |

## Do NOT

- Do not write code-level documentation in these specs. Code-level docs live
  in `src/*/CLAUDE.md` (per-package) and the code itself.
- Do not list every internal ROS parameter in `interfaces.yaml`. Only the
  contracts that cross package boundaries or are consumed by external
  clients (the dashboard backend, other robots, operators).
- Do not duplicate content between specs. If two specs would benefit from
  the same information, one references the other with a file:line link.

## How to add a new spec

Only add a new spec if all three are true:
1. The content does not fit in any existing spec.
2. It has a clearly defined owner (a package, a role, or a lifecycle).
3. It has a corresponding `verify_<name>.py` in `tools/verify_specs/`.

Otherwise, extend an existing spec.

## Version history

- **2.1 (2026-07-06)** — SSOT reconciliation after the community-readiness
  fix wave: registered the Phase-2 rail/arbiter stack (state_machine
  `layer_5_runtime_arbiter`, cross-checked against `mode_fsm.hpp`),
  the fleet/ layer (VDA 5050 adapter + fleet manager :8092), motor/e-stop
  telemetry topics, and the real auth mechanism (users.json `enabled` +
  scrypt). Rewrote `acceptance.yaml` (now `status: active`). Regenerated
  `launch_sequence.yaml` from `agv_full.launch.py` with greppable anchors
  instead of line numbers. Hardened `tools/verify_specs/` (quoted-literal
  + consumer-side + reverse-pass interface checks; structural FAILs now
  block from any tier; missing gate scripts fail the suite).
- **2.0 (2026-04-13)** — Full schema overhaul as part of the audit and
  restructure (see [docs/audit/2026-04-13-full-audit.md](../docs/audit/2026-04-13-full-audit.md)).
  Added `state_machine.yaml`, `launch_sequence.yaml`, `persistence.yaml`.
  Rewrote `interfaces.yaml` with per-interface owner, QoS, subscribers.
- **1.x** — Initial minimal specs (topics, services, actions only).
