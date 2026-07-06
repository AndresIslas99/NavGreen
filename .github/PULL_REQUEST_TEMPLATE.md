## What

<!-- One or two sentences: what does this PR change and why? -->

## Spec sync (SSOT)

<!-- The specs in specs/ are authoritative. Mark what applies. -->

- [ ] No interface / mode / persistence / launch changes — specs untouched
- [ ] `specs/interfaces.yaml` updated (new/changed topic, service, action, or cross-package parameter)
- [ ] `specs/persistence.yaml` updated (new/changed persistent artifact)
- [ ] `specs/state_machine.yaml` updated (new/changed mode or transition)
- [ ] `specs/launch_sequence.yaml` updated (new/changed launch node)
- [ ] Package `CLAUDE.md` updated if its responsibilities or interfaces changed

## Verification

- [ ] `bash tools/verify_specs/all.sh` — zero BLOCKING failures
- [ ] Touched packages build with zero warnings (`-Werror`)
- [ ] `colcon test` passes for touched packages (or: no testable change)

## How was this validated?

<!-- Simulation / HIL / real hardware / unit tests only — be specific. -->
