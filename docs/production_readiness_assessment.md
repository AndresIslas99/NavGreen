# Production Readiness Assessment — AGV Greenhouse
**Date**: 2026-03-31
**Git**: 398a73a (main)

## Executive Verdict

**Status: ENGINEERING PROTOTYPE — HIL VALIDATED, FIELD PRE-ALPHA**

The robotics stack (drive, perception, localization, navigation) is 80% complete with HIL validation. The operator dashboard exists and is functional. Three C++17 ROS2 packages listed in TASK.yaml as required (agv_map_manager, agv_waypoint_manager, agv_behaviors) have no implementation — however, the backend Python server already handles map save/load and mission CRUD as interim REST endpoints, bypassing the need for those packages for the MVP demo.

---

## What's Actually Working Today

| Component | Real Status | Notes |
|---|---|---|
| CAN bus + ODrive | **Validated on hardware** | Boot-persistent via systemd, 250kbps, both axes |
| C++17 ODrive node | **Built, 14/14 tests pass** | wheel_odom 50Hz, cmd_vel, e-stop, watchdog, drive_debug |
| URDF / TF tree | **Validated** | All frames correct, measured dimensions |
| cuVSLAM | **Validated** | GPU stereo-inertial, depth filter, nvblox |
| Dual EKF (HIL) | **Validated** | local 40Hz + global 10Hz, clean TF chain |
| Nav2 (HIL) | **5/5 routes passed** | SmacPlanner2D + RegulatedPurePursuit |
| Operator dashboard | **Working** | React/TS, ISA-101 colors, state machine, health, events |
| Map save/load | **Working** | Via REST + nav2_map_saver (not C++17 node) |
| Mission CRUD + execute | **Working** | Via REST + sequential navigate_to_pose (not C++17 node) |
| Scan accumulation map | **Working** | Probabilistic grid, live visualization |
| Ground plane filter | **Working** | pointcloud_to_laserscan with min_height/max_height |
| Boot persistence | **Configured** | agv.service + can-setup.service |
| Teleop from tablet | **Working** | Joystick, e-stop, motor arm, all modes |
| Event log | **Working** | Persistent to disk, 500 entries, severity levels |

## What's NOT Working / Missing

### Critical (blocks field demo)

| Gap | Impact | Effort |
|---|---|---|
| **Dual EKF on real hardware** | Can't navigate in real greenhouse | 1-2 weeks (code ready, needs physical testing) |
| **Real greenhouse map** | Nav2 needs an actual map with walls | 1 day (drive mapping mode in greenhouse) |
| **30-min stress test** | No proof of stability for demo | 1 day (run and monitor) |

### Important (degrades quality)

| Gap | Impact | Effort |
|---|---|---|
| Right wheel asymmetry | Robot drifts, needs right_scale tuning | 1 hour with robot |
| vel_integrator_gain fine-tuning | Low-speed smoothness not validated post-0.167 | 1 hour with robot |
| No battery topic | Dashboard can't show battery state | Small (no BMS hardware) |
| No pose topic at 10Hz | Spec requires /agv/pose, currently only /agv/odometry/global | Small |

### Spec Compliance Gaps (not blocking MVP)

| Spec Requirement | Status | Notes |
|---|---|---|
| agv_map_manager C++17 node | **Not implemented** | REST endpoints in Python backend handle this for now |
| agv_waypoint_manager C++17 node | **Not implemented** | REST endpoints in Python backend handle this for now |
| agv_behaviors BT | **Not implemented** | Sequential navigate_to_pose dispatch works for MVP |
| agv_ui_backend in TypeScript | **Python interim** | Works, marked dev_only, migration post-MVP |
| agv_markers C++17 node | **Not implemented** | Post-MVP stretch, not needed for first visit |
| Keepout/speed zones | **Not implemented** | No costmap filter integration |

## Assessment vs TASK.yaml Specs

The TASK.yaml files define C++17 nodes for map_manager, waypoint_manager, and behaviors. These don't exist as code. However, **the functionality they specify IS implemented** in the Python backend:

| TASK.yaml Spec | Spec Says | Reality |
|---|---|---|
| `agv_map_manager/save_map` service | C++17 ROS2 service | Python REST endpoint calls nav2_map_saver |
| `agv_map_manager/load_map` service | C++17 ROS2 service | Python REST endpoint calls nav2 LoadMap service |
| `agv_waypoint_manager/save` service | C++17 ROS2 service | Python REST endpoint writes missions.json |
| `agv_waypoint_manager/execute` service | C++17 ROS2 service | Python backend dispatches navigate_to_pose sequentially |

**This is architecturally acceptable for MVP** per CLAUDE.md Rule 0: "Python ROS2 packages serving exclusively as development, commissioning, or diagnostic tools are permitted as interim dev tooling."

## Phase Status (Corrected)

| Phase | Spec Gate | Actual Status | Corrected % |
|---|---|---|---|
| 1: Foundation | wheel_odom@50Hz, robot moves | **Done** (C++17 node, validated) | 95% |
| 2: Perception | 2D map, relocalize | **HIL validated**, needs real hardware | 85% |
| 3: Navigation | A→B autonomous | **HIL 5/5**, needs real dual EKF | 70% |
| 4: Markers | pose error <5cm | Not started (post-MVP) | 5% |
| 5: Dashboard | Operator workflow from tablet | **Working** (React + Python backend) | 75% |
| 6: Integration | 30-min demo | Not tested | 20% |

## Path to Field Visit

### Week 1-2: Real Hardware Validation
1. Kill HIL → launch `agv_fusion.launch.py` on real robot
2. Validate dual EKF with real cuVSLAM + wheel odom
3. Drive greenhouse at 0.3-0.5 m/s, save map
4. Tune right_scale, vel_integrator_gain
5. Run Nav2 with real map, validate 5 routes

### Week 3: Integration + Stress Test
1. Launch `agv_full.launch.py` with real map
2. Dashboard end-to-end: map → save → load → goal → mission → e-stop
3. 30-minute continuous operation test
4. Document all failure modes

### Go/No-Go Checklist

- [ ] Dual EKF runs clean on real hardware for 10+ minutes
- [ ] Real greenhouse map saved and loadable
- [ ] 5 Nav2 routes complete with ≤0.15m error on real hardware
- [ ] Dashboard operator workflow works without terminal
- [ ] 30-minute continuous operation passes
- [ ] E-stop latency ≤0.2s verified
- [ ] Right wheel asymmetry corrected

## What "Full Production AMR" Requires Beyond MVP

| Category | Gap | Effort |
|---|---|---|
| **Safety** | Certified functional safety (SIL/PL), safety scanners, safety PLC | Large (hardware + certification) |
| **Language compliance** | Migrate agv_ui_backend Python → TypeScript | 2-3 weeks |
| **C++17 ROS2 nodes** | agv_map_manager, agv_waypoint_manager as proper services | 3-4 weeks |
| **Behavior trees** | agv_behaviors for complex missions with recovery | 2-3 weeks |
| **AprilTag markers** | agv_markers for drift correction in repetitive rows | 2-3 weeks |
| **Fleet readiness** | VDA 5050 protocol, MQTT transport, multi-robot | Large |
| **Orin NX optimization** | Port from AGX Orin 64GB to NX 16GB | 2-4 weeks |
| **Test coverage** | Integration tests, hardware-in-loop CI | 3-4 weeks |
| **Monitoring** | Grafana/InfluxDB for long-term metrics | 1-2 weeks |
| **Auto-docking/charging** | Battery management, dock detection | Large |
| **Zone management** | Costmap filters for keepout/speed zones | 1-2 weeks |

## Conclusion

The system is **significantly further along than the TASK.yaml specs suggest**. The specs define C++17 nodes that don't exist, but the Python backend implements equivalent functionality. For the MVP field visit, the bottleneck is no longer code — it's **physical validation on the real robot** (dual EKF + real map + Nav2 routes). The dashboard and operator workflow already work end-to-end in simulation.

**Estimated time to field-ready: 2-3 weeks** (if robot hardware is available for daily testing).
