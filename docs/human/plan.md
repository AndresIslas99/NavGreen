> Canonical note: if this human-readable plan conflicts with `/specs/project.yaml`, `/specs/interfaces.yaml`, or `/specs/acceptance.yaml`, the spec files win.

# AGV Greenhouse MVP — Human Overview

This folder is the human-facing rendering layer.

The machine-facing truth lives in:
- `specs/`
- `agents/`
- `policies/`
- package `TASK.yaml`

## Summary
The MVP targets a first greenhouse site visit where the operator can:
- connect a tablet over local WiFi
- teleoperate the robot
- build and save a map
- load a map
- create waypoints and missions
- run autonomous navigation
- monitor robot state live
- trigger E-stop from the dashboard
