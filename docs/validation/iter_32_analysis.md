# iter-32 análisis — 18/20, c3 unlocked, c5 mystery rotation

**Resultado**: 18/20 SUCCEEDED (igual que iter-31 numérico). Cambió lo que
importa: c3 REAR y=+2.2 ahora **deterministic** (4/4), durations bajaron
dramáticamente (approach 63s vs 90s, drive_in 25s vs 85s). Failures
restantes: c5_drive_in ABORT + cascade reverse_exit. Collisions 0.

## Trajectoria de iteraciones

| iter | resultado | fix clave |
|---|---|---|
| 26f | 12/20 | base |
| 27 | 15/20 | tolerance + shallow REAR + no-post |
| 28 | 16/20 | P1 visual lat gate |
| 29 | 17/20 | P2 pose lat gate |
| 30 | 17/20 | P3 goal-based rail_operation |
| 31 | 18/20 | marker_correction RELOC suppressed in rail_aisle |
| **32** | **18/20** | **launch YAML fix: params finally loaded** |

iter-32 es la primera iteración donde **todos los fixes acumulados
realmente corren** — iter-26d..31 los YAML params (yaw_abort=0.35,
lateral_abort=0.30, speed_max=1.0) nunca se aplicaron al runtime
porque el hil_full launcher no pasaba el YAML y el YAML key era
`rail_driver:` en lugar de `/**:` (node corre bajo namespace `/agv`).

## Hallazgo crítico iter-32: **dos bugs silenciosos de config**

### Bug A: launch file no cargaba YAML

`src/agv_bringup/launch/agv_hil_full.launch.py` L372-379 tenía:
```python
Node(package='agv_rail_driver', executable='rail_driver_node',
     name='rail_driver', namespace=ns,
     parameters=[{'use_sim_time': True}],  # ← ONLY this, no YAML
     output='log')
```

Durante 5 iteraciones subí yaw_abort_rad 0.26 → 0.35, lateral_abort_m,
min_mode_dwell_s, etc. NINGUNO se aplicó porque rail_driver_node no
recibía el YAML. El node corría con los defaults del header.

### Bug B: YAML key no matchea node namespaced

El YAML usaba `rail_driver:` como root. Con namespace `/agv`, el node
se registra como `/agv/rail_driver`. Sólo matchea keys `/agv.rail_driver:`
o el wildcard `/**:`. Per bringup CLAUDE.md: "YAML override files MUST
use `/**:`". Ambos YAMLs (rail_driver + mode_arbiter) lo tenían mal.

### Confirmación runtime

Antes del fix: `ros2 param get /agv/rail_driver yaw_abort_rad` → 0.26  
Después: 0.35 ✅

iter-32 es primer run con params reales. c3_drive_in cayó de aborto
consistente a éxito inmediato.

## Desglose por ciclo iter-32

| ciclo | prealign | approach | drive_in | reverse_exit | status |
|---|---|---|---|---|---|
| c1 REAR y=0 | ✅ 0.008 (sub-1cm) | ✅ 0.150/63s | ✅ 0.107/26s | ✅ 0.238/21s | **4/4 🎯** |
| c2 FRONT y=0 | ✅ 0.0 teleport | ✅ 0.139/57s | ✅ 0.049/25s | ✅ 0.049/29s | **4/4 🎯** |
| c3 REAR y=+2.2 | ✅ 0.049 | ✅ 0.145/58s | **✅ 0.153/25s** | ✅ 0.142/25s | **4/4 🎯 NEW** |
| c4 FRONT y=-2.2 | ✅ 0.0 | ✅ 0.140/57s | ✅ 0.054/27s | ✅ 0.047/29s | **4/4 🎯** |
| c5 REAR y=-2.2 | ✅ 0.047 | ✅ 0.146/61s | **❌ ABORT 1.94/71s** | ❌ cascade | 2/4 |

**Durations improvement over iter-31**: approach 90s→60s, drive_in
85s→25s. ~30% faster throughout.

## c5_drive_in ABORT — misterio rotación 64°

Diag log exacto:
```
BLOCKED blocked_misaligned (reason=pose_yaw)
  pose=(4.431, -1.879, yaw=-2.020)
  goal=(2.500, -2.200)
  visual={lat=0.000 yaw=0.000 conf=0.00 age=infs}
  pose_lat_err=0.321  rail_yaw_err=1.121  in_rail_zone=0
```

**Smoking gun**: robot's yaw changed 64° (from π = 3.14 to -2.02) during
drive_in. rail_driver hardcodea `angular.z = 0`. **Alguien más está
publicando wz != 0 al `/agv/cmd_vel`**.

### Candidatos

1. **mode_arbiter relaying stale `cmd_vel_approach`**: fine_servo
   publica después de SETTLE si no se detiene limpio. mode_arbiter
   source=RAIL debería ignorar cmd_vel_approach pero podría tener
   lag en la transición.

2. **collision_monitor's velocity_smoother injecting angular velocity**:
   al entrar en slowdown_zone, smoother podría distorsionar cmd.

3. **Nav2 controller_server still active**: bt_navigator no cancelado
   entre waypoints. rotation_shim_controller publica a cmd_vel_nav.
   mode_arbiter source=RAIL NO debería relay nav, pero verificar.

4. **physical contact/wheel slip**: el robot topa con un obstáculo
   físico en sim (rail tube) y gira por reacción. Improbable porque
   odometry registra la rotación (si fuera slip, odom no lo vería).

### Diferencia clave c3 vs c5

- c3 REAR y=+2.2 tag 36: rail_detector IS active (visual conf>0).
  visual yaw check usa data fresca. Even if rotation happens, visual_yaw
  fires first and aborts at a lower yaw angle. But c3 PASSED iter-32.
- c5 REAR y=-2.2 tag 34: `visual={conf=0 age=inf}` — rail_detector
  silent. Falls back to `rail_yaw_error` from zone_detector (pose-based).
  That value is derived from the poisoned ekf_global yaw.

Why is rail_detector silent in c5 and not c3? Possible:
- Tag 34 obscured by something in sim
- rail_detector's BEV can't find rails at y=-2.2 REAR but finds them
  at y=+2.2 REAR (asymmetry in sim assets)
- rail_detector node state latching problem

## Plan iter-33

1. **[15 min] Instrument mode_arbiter**: add RCLCPP_WARN when publishing
   non-zero cmd_vel.angular.z. Tell us which upstream source
   (cmd_vel_nav/approach/rail) carries the angular command at the c5
   abort moment. cmd_vel_rail should always be 0.
   - file: `src/agv_mode_arbiter/src/mode_arbiter_node.cpp` in the
     cmd_vel relay loop.

2. **[diag observation] Check if rail_detector is active at c5**.
   If not, figure out WHY tag 34 doesn't yield rail-pair detections
   while tag 36 does. Possible sim asset issue.

3. **[diferible] Investigate cmd_vel pipeline**: `cmd_vel` → 
   `velocity_smoother` → `cmd_vel_smoothed` → `collision_monitor` →
   `cmd_vel_safe` → ODrive. Any stage could inject angular.

Target iter-33: **19-20/20**. c5 is the last consistent failure.

## Qué preservar (iter-32 config locked in)

- ✅ rail_driver yaw_abort_rad=0.35, lateral_abort_m=0.30, speed_max=1.0
- ✅ marker_correction: RELOC suppressed en rail_aisle
- ✅ rail_driver: visual+pose lat_abort off en rail_operation (P3)
- ✅ harness: continuous-flow, clear_state pre-dispatch, cancel_goal
- ✅ waypoints v5: 5 cycles × 4 wp, shallow REAR, no 90° posts
- ✅ tolerance_xy=0.15, tolerance_yaw=0.25, settle_frames=5

## Observaciones meta

**El descubrimiento iter-32 es el más importante desde iter-23**:
- iter-26d..iter-31 gastaron 5 iteraciones probando fixes cuya
  configuración nunca llegó al runtime.
- Diagnostic WARN log (iter-31) + param verify post-launch salvaron
  esta investigación. Sin el WARN habríamos seguido adivinando.
- Misma lección aplicable al futuro: **cada iteración debería
  `ros2 param get` los valores críticos al arrancar el brain** para
  confirmar que la config efectivamente cargó.
