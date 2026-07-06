# AGENTS.md

The single source of truth for AI-agent rules in this workspace is
[AGENT_INSTRUCTIONS.md](AGENT_INSTRUCTIONS.md). This file exists only so
tools that auto-load `AGENTS.md` land in the right place — it defines no
rules of its own.

Reading order (do not skip):

1. [AGENT_INSTRUCTIONS.md](AGENT_INSTRUCTIONS.md) — absolute rules, the
   six-step specs-first workflow, and the required reading order.
2. [CLAUDE.md](CLAUDE.md) — project rules and the full canonical source
   order (`specs/*.yaml`, `policies/engineering_rules.md`, per-package docs).
3. The `CLAUDE.md` and `TASK.yaml` of any package you touch.

Before committing, `bash tools/verify_specs/all.sh` must pass — the
pre-commit hook runs the same checks.

Agent roles are registered in [agents/registry.yaml](agents/registry.yaml).
