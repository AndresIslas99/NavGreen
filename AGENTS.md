# AGENTS.md

## Agent roles
- `architect_agent`: multi-file and cross-package work
- `refactor_agent`: single-file cleanup and commits
- `generator_agent`: structured generation and scaffolding

## Canonical sources
1. `/specs/project.yaml`
2. `/specs/interfaces.yaml`
3. `/specs/acceptance.yaml`
4. `/policies/engineering_rules.md`
5. relevant package `TASK.yaml`

## Coordination
- one agent, one domain, one lock at a time
- no simultaneous edits to the same file
- claim ownership in `.agent_locks.yaml` before editing

## Required workflow
1. read root specs
2. read relevant `TASK.yaml`
3. lock files
4. implement
5. build
6. test
7. release lock
