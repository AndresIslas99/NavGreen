# AGENT_INSTRUCTIONS.md — Rules for AI coding agents

You are an AI coding agent (Claude Code, Copilot, Cursor, or similar) that has
been invited to modify the AGV greenhouse robot workspace. **Read this file
before doing anything else.** Humans working on the codebase should also read
it.

## Why this file exists

The workspace has accumulated drift from ad-hoc fixes faster than the
documentation could keep up. An audit on 2026-04-13 produced machine-readable
specs that are now the Single Source of Truth (SSOT). This file tells you
how to work with them so that your changes do not add more drift.

The audit report is at [docs/audit/2026-04-13-full-audit.md](docs/audit/2026-04-13-full-audit.md).
Read it if you want to understand the shape of the problems that led to
these rules.

## Absolute rules — do not break

1. **Never add a new ROS topic, service, action, or cross-package
   parameter without updating [specs/interfaces.yaml](specs/interfaces.yaml)
   in the same commit.** The pre-commit hook will reject the commit.
2. **Never add a new persistent artifact (file/folder) without updating
   [specs/persistence.yaml](specs/persistence.yaml).** Same enforcement.
3. **Never introduce a new operation mode, launch condition, or runtime
   state without updating [specs/state_machine.yaml](specs/state_machine.yaml).**
4. **Never add a new node to a launch file without updating
   [specs/launch_sequence.yaml](specs/launch_sequence.yaml).**
5. **Never hardcode an absolute path.** Use `${AGV_DATA_DIR}` env var or a
   ROS parameter. Whitelist lives in
   [tools/verify_specs/verify_no_hardcoded_paths.sh](tools/verify_specs/verify_no_hardcoded_paths.sh).
6. **Never add a Python ROS 2 node to a production launch file without
   setting `dev_only: true` in the package's TASK.yaml** — or better, port
   it to C++17.
7. **Never add `-Wno-*` or remove `-Werror`** to silence warnings. Fix the
   underlying warning instead.
8. **Never commit with `--no-verify`** unless you are actively resolving an
   incident and you note it in the commit message.

## Required reading order before you write code

Follow this order every time, without skipping:

1. [CLAUDE.md](CLAUDE.md) — project root: absolute rules, canonical sources,
   hardware setup pointers.
2. [specs/README.md](specs/README.md) — the meta-doc for the spec
   directory, including the reading order inside it.
3. The relevant specs from `specs/*.yaml`. For almost any change, read at
   minimum `interfaces.yaml`, `state_machine.yaml`, and `persistence.yaml`.
4. [policies/engineering_rules.md](policies/engineering_rules.md) — Rules
   0–9 plus enforcement notes.
5. The CLAUDE.md of the package you are touching (e.g.
   `src/agv_map_manager/CLAUDE.md`).
6. Only then: the code.

If you find yourself writing code without having read 1–5, **stop** and go
back. This is not optional.

## The six-step workflow for any code change

1. **Understand the contract.** Find the interface in `specs/interfaces.yaml`
   (if applicable) or the state in `specs/state_machine.yaml`. If the
   contract does not exist, STOP and ask a human whether you should add it
   or whether the change is out of scope.
2. **Plan the change.** Write what files you will touch and why. Reference
   the specs. If your plan touches 5+ files you are probably doing too
   much at once — split it.
3. **Update the specs FIRST.** Modify `specs/*.yaml` to reflect the new
   contract. Commit the spec updates as a separate step or as the first
   hunks of the change.
4. **Write the code.** Match the specs exactly. If you discover the specs
   were wrong, update them again — do not diverge silently.
5. **Run the verifiers.** Execute `bash tools/verify_specs/all.sh` locally
   before committing. If any BLOCKING script fails, fix and retry.
6. **Run the build.** `cd /home/orza/ros2_ws && colcon build
   --packages-select <touched>`. Zero warnings, zero errors.

## The "no invention" rule

If the spec does not say something, **do not assume**. Examples of bad
assumptions that agents have made in the past:

- Assuming a topic exists because a comment mentions it.
- Assuming a ROS parameter is read-only when the spec does not say so.
- Assuming a path like `/mnt/ssd/` is a real mount point (it is not — it
  is a bare directory on the root FS; see [specs/persistence.yaml](specs/persistence.yaml)
  for the canonical directory tree).
- Assuming `safety_supervisor_node` publishes `SafetyStatus` at boot (it
  does, but only after ~N seconds — see the bug in
  [docs/audit/2026-04-13-full-audit.md](docs/audit/2026-04-13-full-audit.md) section 6).

When in doubt: grep the code, read the spec, ask the user. Do not guess.

## Common change patterns and what to update

| Change | Specs to update | Other files |
|---|---|---|
| Add a new topic | `interfaces.yaml` → `topics` | Usually also `state_machine.yaml` if it triggers transitions |
| Add a new service | `interfaces.yaml` → `services` | — |
| Add a new launch file node | `launch_sequence.yaml` | `specs/state_machine.yaml` if it affects a mode |
| Add a new persistent file/folder | `persistence.yaml` | — |
| Add a new operation mode | `state_machine.yaml` — new layer value | Almost certainly others |
| Add a new package | root `CLAUDE.md`, new `src/<pkg>/CLAUDE.md`, new `src/<pkg>/TASK.yaml` | — |
| Fix a bug in existing behavior | Usually no spec change, but VERIFY the bug's description in `docs/audit/*.md` matches what you are doing |
| Refactor without behavior change | No spec change needed, but run verify |

## What happens if you skip the specs

The pre-commit hook runs
[tools/verify_specs/all.sh](tools/verify_specs/all.sh) and blocks the
commit. The error message tells you which script failed and how to fix
it. Do NOT bypass the hook with `--no-verify` unless the user has
explicitly told you to do so.

If you run into an enforcement that seems wrong (e.g., a warning that is
a false positive), tell the user. The verify scripts are themselves
maintained in version control and can be improved — but only with review.

## Reporting uncertainty

When you are unsure:
- Tell the user explicitly: "I'm not sure about X — can you confirm Y?"
- Cite the specific spec or rule that creates the uncertainty.
- Offer 2-3 concrete options.

When you are certain but the user might disagree:
- Cite your source (spec, file:line, or audit document).
- Explain the tradeoff.

## What NOT to do

- Do not create new documentation files unless a spec says they should
  exist.
- Do not create new launch files unless `launch_sequence.yaml` lists them.
- Do not create new ROS parameters unless `interfaces.yaml` lists them
  (cross-package) or they are internal to a single node and documented in
  that node's CLAUDE.md.
- Do not invent new state-machine modes.
- Do not add a new `dev_only: true` file if the same functionality can be
  done in C++17. Python is a temporary refuge, not a destination.
- Do not delete a spec entry just to make a check pass. The check is
  there to protect against drift.

## When the specs are wrong

Specs are the SSOT but they are not infallible. If you find a spec that
contradicts working code:

1. Verify which one is correct by running the code and observing.
2. If the code is correct and the spec is wrong, update the spec to match
   the code, and commit that as a "spec correction" with a clear message.
3. If the spec is correct and the code is wrong, file the bug in the
   audit document (`docs/audit/*.md`) or as a GitHub issue.

Never update the code to match a wrong spec. Always verify reality first.

## One last thing

This file, and the specs it references, are the product of many people
(and agents) making mistakes. Every rule here exists because something
broke in production. Treat them seriously — they are cheap to follow and
expensive to violate.

When you finish a change, leave the workspace better than you found it:
add a missing spec entry, fix a stale comment, tighten a whitelist.
Small improvements compound.
