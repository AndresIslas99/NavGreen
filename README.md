# AGV Agentic Spec Pack v1.1

This pack converts the greenhouse AGV MVP into an agent-ready specification set.

## What changed in v1.1
- Added greenhouse-specific visual SLAM cautions and commissioning protocol
- Made dual-EKF frame ownership explicit
- Added map revision metadata requirements
- Clarified that software collision handling is not certified functional safety
- Tightened current-MVP scope and kept Open-RMF, docking, and certified safety out of scope

## Canonical files
- `specs/project.yaml`
- `specs/interfaces.yaml`
- `specs/acceptance.yaml`
- `agents/registry.yaml`
- `policies/engineering_rules.md`
- package-level `TASK.yaml`
