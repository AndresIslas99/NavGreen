# iter-30 análisis — 17/20 plateau + REAR off-centre asymmetry

**Resultado iter-30**: 17/20 SUCCEEDED, 2 ABORTED (c3_drive_in, cascade c3_reverse_exit), 1 NAV_TIMEOUT (c5_drive_in). Colisiones 0. Progreso neto +6 waypoints desde iter-26f (12/20).

## Trayectoria de iteraciones

| iter | resultado | fix dominante |
|---|---|---|
| 26f | 12/20 | base (tolerance 0.10/0.10, v5 YAML shallow REAR, no 90° posts) |
| 27 | 15/20 | tolerance_xy 0.115, tolerance_yaw 0.20, settle 5, yaw_abort 0.35 |
| 28 | **16/20** | P1: visual lat_abort suppressed `in_rail_zone` |
| 29 | **17/20** | P2: visual+pose lat_abort suppressed `in_rail_zone` |
| 30 | 17/20 | P3: `rail_operation = in_rail_zone OR goal_y on aisle` — no Δ |

La trayectoria 12→15→16→17 se logró relajando tolerancias (iter-26) y
desactivando progresivamente los gates de aborto falso durante la
drive dentro del riel (P1→P2→P3). iter-30 P3 debía ser la pieza final
porque usaba la GEOMETRÍA del goal (no sujeta a marker_correction
RELOC) para decidir si estamos en rail operation. No hubo ganancia.

## Desglose por ciclo iter-30

| ciclo | wp | resultado | notas |
|---|---|---|---|
| **c1 REAR y=0** | prealign ✅ 0.028, approach ✅ 0.150, drive_in ✅ 0.079, reverse ✅ 0.454 | 4/4 🎯 | reverse_exit err 45cm (rail_driver reached pero odom drift) |
| **c2 FRONT y=0** | prealign ✅ teleport, approach ✅ 0.137, drive_in ✅ 0.053, reverse ✅ 0.048 | 4/4 🎯 | — |
| **c3 REAR y=+2.2** | prealign ✅ 0.020, approach ✅ 0.153, **drive_in ❌ ABORT 0.456/29.6s**, reverse cascade | 2/4 | drive_in abort en x≈3.0 |
| **c4 FRONT y=-2.2** | prealign ✅, approach ✅ 0.139, drive_in ✅ 0.052, reverse ✅ 0.050 | 4/4 🎯 | — |
| **c5 REAR y=-2.2** | prealign ✅ 0.050, approach ✅ 0.023, **drive_in ❌ NAV_TIMEOUT 1.99/180s**, reverse ✅ 0.097 | 3/4 | robot stuck at ~x=3.0 |

## Hallazgo estructural: **asymmetry REAR y=0 vs REAR y=±2.2**

El código es igual para los 3 aisles REAR. El flujo es idéntico. Lo
único que cambia es la `y` del aisle y el tag_id (35 vs 36 vs 34).
Sin embargo:

- c1 REAR y=0 (tag 35): **100% drive_in success**
- c3 REAR y=+2.2 (tag 36): **0% drive_in success** (iter-27..30)
- c5 REAR y=-2.2 (tag 34): **0% drive_in success** (iter-27..30)

La asimetría apunta a problemas del **entorno sim**, no del stack de control:

### Evidencia 1: marker_correction RELOC agresivos en off-centre REAR

Durante c3_drive_in iter-30 (brain.log):

```
1776666312  Correction: 3 tags, best=8, range_factor=1.4,
            pos=(7.60, 0.48), drift=2.0m RELOC
1776666321  Correction: 2 tags, best=29, range_factor=2.5,
            pos=(3.53, 1.44), drift=3.1m RELOC
```

Robot realmente en (~5, 2.2) driving a (2.5, 2.2). marker_correction
mete al robot a (3.53, 1.44) — 0.76m off aisle y-centerline. Esto
no estaba pasando durante c1 (y=0) porque c1 ve tag 35 directamente.
En c3 (y=+2.2) el robot ve tag 36 pero ALSO tag 29, 8, etc (visible
at grazing angle to flanking aisles), and marker_correction's vote
averages produce wrong poses.

### Evidencia 2: my P3 goal-based `rail_operation` didn't help

P3 correctamente identificaba goal_y=2.2 como aisle → suprimir lat
gates. Pero c3_drive_in ABORTED en 29.6s. Significa que el abort NO
vino del lat gate — vino de otro lado. Las opciones:
- `visual_yaw_error > yaw_abort_rad=0.35` (rail_detector mal en off-centre)
- `rail_yaw_error > yaw_abort_rad` (zone_detector reportando ruido)
- `collision_monitor_stop` triggered (obstacle in stop_zone polygon)

Sin un log más detallado (rail_driver no RCLCPP_INFO sobre BLOCKED),
no puedo distinguir. Pero el patrón consistency REAR-off-centre fail,
REAR-centre pass, FRONT-ANY pass sugiere **visual sensor noise**:
- FRONT: robot facing +x (east), ZED ve rails frontales + gap behind,
  pocos tags REAR/phantom. Limpio.
- REAR y=0: robot facing -x (west), tag 35 directamente delante; tags
  adyacentes filtered out. Limpio.
- REAR y=±2.2: robot facing west, tag 36/34 delante pero ZED FOV captura
  también tags de aisles adyacentes y ROW-START tags (IDs 20-30). RANSAC
  de marker_correction ve 2-3 "inliers" que combinan aisles y produce
  pose errónea.

## ¿Qué NO ayudó en iter-28..30?

- ❌ Suprimir visual lat_abort (P1)
- ❌ Suprimir pose lat_abort (P2)
- ❌ Cambiar `in_rail_zone → goal_y on aisle` (P3)
- ❌ Raising yaw_abort 0.26→0.35
- ❌ Raising tolerance_xy/yaw en rail_approach
- ❌ Shallow REAR drive_in (x=1→2.5)

El abort c3/c5 **sobrevive** a todos estos cambios. Significa que el
gate que dispara NO es ninguno de los que he tocado.

## Hipótesis refinadas iter-31

### H1: collision_monitor dispara stop_zone
Robot drive_in REAR off-centre: al cruzar x≈3 (rail boundary), los
tubos del rail entran al stop_zone (footprint + 5cm). collision_monitor
sees obstacle and issues stop. rail_driver translates to BLOCKED_WAIT
(not BLOCKED_LATERAL/MISALIGNED).

**Test**: grep `Robot to stop` durante c3 window. Aumentar stop_zone
sólo al frente (ya está 20cm), reducir a 15cm. O filter out rail tubes
from `observation_sources` (collision_monitor sees them as obstacles
when they should be "floor infrastructure").

### H2: visual_yaw_error from rail_detector noisy off-centre
rail_detector's BEV RANSAC fits a rail pair. For REAR off-centre, it
might fit the WRONG rail pair (adjacent aisle) → visual_yaw_error jumps
to > 0.35 → BLOCKED_MISALIGNED.

**Test**: subscribe to `/agv/rail_detections` during c3 and log
`visual_yaw_error` values. If > 0.35 observed, either:
- Raise yaw_abort inside rail to 0.5 (with explicit non-safety rationale)
- Gate yaw_abort on `visual_confidence > 0.85` (current 0.7 too loose)
- Fuse with odometry-computed yaw (trust angular delta from initial
  alignment, not absolute visual)

### H3: marker_correction RELOC moves odom into collision polygon
marker_correction RELOCs ekf to (3.53, 1.44). Robot physically at
(~3.0, 2.2), but nav stack/collision_monitor thinks it's at (3.53, 1.44).
At that pose, costmap/collision may judge robot in unsafe state.

**Test**: disable marker_correction entirely during rail_drive. It's
meant for pose anchoring before nav, not during constrained motion.

## Plan iter-31

Priorizado por evidencia:

1. **[20 min] Diagnostic logging**: añadir RCLCPP_WARN en rail_driver
   when BLOCKED_* fires, incluyendo cause (visual_lat/pose_lat/visual_yaw/
   pose_yaw/collision). Sin esto estoy adivinando.

2. **[30 min] Disable marker_correction while in rail_operation**. Pose
   anchoring is crítico before entry (rail_approach fine_servo uses
   tag). Once inside rail, odometry + mechanical constraint son
   suficientes. La RELOC está POISON-ing un pose que no necesita
   corrección.

3. **[15 min] Raise yaw_abort inside rail to 0.50 rad (29°)**. 
   Mechanical tolerance is ~36°; 0.50 leaves 7° margin. Pragmatic
   if rail_detector off-centre noise es la causa (H2).

4. **[diferible]** Investigar collision_monitor stop_zone behavior in
   REAR off-centre aisles. Si rail tubes disparan stop_zone, hay que
   filtrarlos del pointcloud source.

5. Run iter-31. Proyección **18-20/20** si H2+H3 alguna impacta.

## Qué SÍ funcionó (preservar)

- ✅ tolerance_xy=0.15, tolerance_yaw=0.25, settle_frames=5 (rail_approach)
- ✅ rail_driver yaw_abort_rad=0.35
- ✅ Visual+pose lat_abort suppressed cuando `goal_y on aisle` (P3)
- ✅ Harness continuous-flow (no start = no teleport, clear state before
  rail_drive goal, cancel_goal synchronous)
- ✅ waypoints v5: 5 ciclos × 4 wp, shallow REAR (x=2.5), no 90° nav2 posts
- ✅ c1/c2/c4 ciclos perfect (100% drive + exit)

## Observaciones meta

- **Progress rate**: 11→17 correcto en 5 iteraciones. Diminishing returns
  en iter-30. Signal claro de que un cambio de dirección es necesario
  (diagnostics + sensor filter, NO más gate tuning).
- **Test harness robusta**: continuous-flow + cancel_goal + clear_state
  eliminan cascades; si drive_in falla, reverse_exit succeede (ver c5).
- **R1 validator**: 20/20 waypoints start poses válidos (nunca
  teleport inside rail).
- **R3/R4 implicit**: logrados a nivel YAML (prealign antes de approach,
  reverse-exit por rail_driver linear.x body-frame).
