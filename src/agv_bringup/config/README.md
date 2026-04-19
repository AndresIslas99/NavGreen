# agv_bringup/config — centralized deployment configuration

This directory is the **single place** to look when you need to know what
parameters a deployment of the AGV will run with. It is the analogue of the
Clearpath `clearpath_config` layout.

## Layout

```
config/
  README.md            (this file)
  common/              # parameters shared across all deployments
  robot/               # this physical AGV's calibrated identity
  sites/               # per-site overlays (one subdir per greenhouse)
    _template/         # blank starter — copy this when adding a new site
    chada/             # Chada Farms greenhouse
  simulation/          # simulation-only overrides
  hil/                 # hardware-in-the-loop overrides

  # legacy (to be migrated, see "Migration plan" below)
  cuvslam_greenhouse.yaml
  cuvslam_no_tf.yaml
  cyclonedds_hil.xml
  cyclonedds_production.xml
```

## How to choose parameters

Launch files should compose parameters in this order, **last writer wins**:

1. `common/*.yaml`              — never per-deployment
2. `robot/*.yaml`               — this AGV's calibrated values
3. `sites/<site>/*.yaml`        — site-specific overrides
4. one of `simulation/` or `hil/` — only when running in those modes

The `site` launch argument selects step 3. Default is `chada`. To deploy at a
new site, copy `sites/_template/` to `sites/<your_site>/`, edit, and pass
`site:=<your_site>` on the command line.

## Adding a new site

```bash
cp -r src/agv_bringup/config/sites/_template src/agv_bringup/config/sites/casa_verde
# edit src/agv_bringup/config/sites/casa_verde/site.yaml
ros2 launch agv_bringup agv_full.launch.py site:=casa_verde map:=/path/to/map.yaml
```

## What goes where

| Parameter type                            | Lives in                       |
|-------------------------------------------|--------------------------------|
| Wheel radius, track width, gear ratio     | `robot/agv_geometry.yaml`      |
| ODrive CAN node IDs, axis directions      | `robot/agv_motor_id.yaml`      |
| QoS profiles for critical topics          | `common/qos_profiles.yaml`     |
| MPPI velocity limits per site             | `sites/<site>/nav_overlay.yaml`|
| Marker registry path per site             | `sites/<site>/site.yaml`       |
| Simulation-only ZED topic remappings      | `simulation/sim_overlay.yaml`  |
| HIL Cyclone DDS XML                       | `hil/cyclonedds_hil.xml`       |

## Migration plan

The package-internal YAMLs that currently live under each package's `config/`
should migrate here over time. The migration is **incremental** to avoid
breaking the production launch chain. Suggested order:

1. `agv_odrive/config/odrive_params.yaml`        → split into `robot/` + `common/`
2. `agv_navigation/config/nav2_params.yaml`      → keep package-internal, add `sites/<site>/nav_overlay.yaml`
3. `agv_navigation/config/collision_monitor.yaml` → keep, add `sites/<site>/collision_overlay.yaml`
4. `agv_sensor_fusion/config/imu_filter.yaml`    → keep package-internal
5. The four legacy files at this directory's top level → move into `common/` and `hil/`
   on the next field-test cycle (cuvslam tuning is greenhouse-specific so it
   actually belongs under `sites/<site>/perception_overlay.yaml`).

Each migration step is its own PR with verification on real hardware.

## Rule 1 compliance

Per [policies/engineering_rules.md](../../../../policies/engineering_rules.md)
Rule 1, none of the following may be hardcoded in source: physical robot
dimensions, CAN interface names, Jetson IPs, marker IDs, map paths, namespace
values, or greenhouse row numbers. This directory is the binding-time location
for all of those.

## See also

- [docs/architectural_gaps.md](../../../../docs/architectural_gaps.md) — Gap 3 of the roadmap
- [policies/engineering_rules.md](../../../../policies/engineering_rules.md)
- [specs/interfaces.yaml](../../../../specs/interfaces.yaml)
