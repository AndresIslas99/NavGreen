# iter-23 análisis — 13/16, root causes confirmados con evidencia

**Resultado**: 13/16 SUCCEEDED, 2 NAV_TIMEOUT (wp10, wp12), 1 ABORTED (wp15).
Collision count = 0. p95_err_xy = 0.101 m (rail_approach gate).

Primera iteración sobre el plateau de 11/16 (iter-20/21/22). El fix de
`compute_rail_exit` gap-branch (iter-23) desbloqueó **wp13 y wp14** por
primera vez (ambos 0.050 m / ~65 s). Los 3 fallos restantes son bugs
independientes con evidencia directa en `brain.log`.

## Comparativa iter-22 → iter-23

| wp | iter-22 | iter-23 | Δ | Causa |
|---|---|---|---|---|
| wp01–09 | 8/9 | 9/9 | **+WIN** | — |
| wp10 | ❌ 0.85 NAV_TIMEOUT | ❌ 1.78 NAV_TIMEOUT | worse | **planner fail** (nuevo hallazgo) |
| wp11 | ✅ 0.101 | ✅ 0.101 | = | — |
| wp12 | ❌ 0.50 NAV_TIMEOUT | ❌ 0.50 NAV_TIMEOUT | = | **FSM RAIL_DRIVE stuck** |
| wp13 | ❌ 1.54 | ✅ 0.050 | **+WIN** | iter-23 geometry fix |
| wp14 | ❌ 1.54 | ✅ 0.049 | **+WIN** | iter-23 geometry fix |
| wp15 | ❌ 3.52 NAV_TIMEOUT | ❌ 2.16 ABORTED | diff err | **costmap lethal** (nuevo hallazgo) |
| wp16 | ✅ 0.090 | ✅ 0.090 | = | — |

Net: +3 wins (wp13, wp14, +indirecto wp01 que ya pasaba). Ningún regresión.

## Root causes con evidencia directa

### wp10 — Nav2 planner fails "no valid path found"

**Evidencia** (`brain.log`):
```
1124  [bt_navigator-16] [1776645869.700] Begin navigating from current location (7.00, 0.00) to (5.50, 0.00)
1125  [planner_server-13] [1776645869.886] GridBased: failed to create plan, no valid path found.
1126  [planner_server-13] [1776645869.886] Planning algorithm GridBased failed to generate a valid path to (5.50, 0.00)
1127  [planner_server-13] [1776645869.886] [compute_path_to_pose] [ActionServer] Aborting handle.
1133+ [controller_server-12] [1776645870.9+] Extrapolation Error looking up target frame: Lookup would require extrapolation into the future. ... from frame [map] to frame [odom]
1135+ [controller_server-12] Rotation Shim Controller was unable to find a goal point, a rotational collision was detected, or TF failed to transform into base frame!
```

- 1091 rail_driver reached wp08 goal (err=0.049), 1117 wp09 (err=0.050)
- 1122 FSM rail_exit → corridor_nav (OK, release gate funcionó)
- 1124 wp10 arranca dispatch nav2 con start=(7.00, 0.00) y goal=(5.50, 0.00)
- 1125 **global planner FALLA**: "no valid path found" — la ruta nav2 no
  existe entre esos dos puntos a pesar de ser 1.5 m en línea recta
- 1130+ rotation_shim retries con TF extrapolation errors durante 180 s
- Usuario observó: "robot rotated and hit bushes" — consistente con
  rotation_shim girando sin plan válido

**Causa raíz**: El **global_costmap** acumula celdas lethal de ZED
pointcloud durante el rail_exit push (robot pasa junto a tubos del
riel a x≈7.0 con ZED montada a 10 mm del piso). Después del push, la
inflación del costmap marca el corredor entre (7.0, 0) y (5.5, 0)
como bloqueado. El planner no puede encontrar ruta.

Además hay evidencia de **marker_correction RELOC agresivos durante
wp09→wp10**:
```
1102  [1776645841.997] Correction: 1 tags, best=28, range_factor=7.2, pos=(3.52, -0.76), drift=5.7m RELOC
1108  [1776645850.295] Correction: 5 tags, best=27, range_factor=5.6, pos=(3.53, -0.76), drift=4.5m RELOC
1113  [1776645859.427] Correction: 4 tags, best=27, range_factor=4.3, pos=(3.53, -0.75), drift=3.7m RELOC
1120  [1776645868.411] Correction: 4 tags, best=27, range_factor=4.1, pos=(3.53, -0.75), drift=3.6m RELOC
```
El robot está realmente en (7, 0) pero marker_correction mete poses
de (3.53, -0.76) — drift de 3.6-5.7 m. Si EKF acepta estos, ekf_global
mueve map→odom y el costmap global cree que el robot está en el lado
oeste. Planner fail puede ser consecuencia: el robot "virtual" pensado
por nav2 está dentro de una zona lethal.

**Fix formal** (2 piezas):
1. **Limpiar global_costmap antes de dispatch nav2** post-rail_exit.
   Harness debería llamar `/agv/local_costmap/clear_entirely_local_costmap`
   y `/agv/global_costmap/clear_entirely_global_costmap` justo antes
   de `navigate_to_pose`. Nav2 ya expone esos services.
   - Archivo: `src/agv_integration_tests/test/test_waypoint_precision.py`
   - Helper nuevo `_clear_costmaps(harness, timeout=2.0)` llamar antes
     de `_wait_for_fresh_tf` cuando `dispatch == "nav2"`.

2. **Reducir agresividad de marker_correction RELOC**.
   - Archivo: `src/agv_marker_correction/config/marker_correction.yaml`
     (si existe) o el node source.
   - Aumentar `reloc_max_drift_m` para rechazar correcciones > 2 m
     (actualmente acepta drift=5.7 m).
   - Investigar por qué best=27, best=28 generan pos=(3.53, -0.76)
     cuando robot está en (7, 0). Posible bug en tag registry o en
     apriltag_sim_shim.

### wp12 — FSM stuck en RAIL_DRIVE, no observa rail_approach_state="driving"

**Evidencia** (`brain.log`):
```
5445  [rail_approach_node-28] [1776646135.043] Settled! error: x=0.0981 y=0.0238 yaw=-0.0732     (wp11 final)
5446  [rail_approach_node-28] [1776646135.043] Approach complete: Precision approach complete
5447  [mode_arbiter_node-22]  [1776646135.115] WARN: request_rail_drive_goal without pose or aisle_y_center; holding.
5448  [mode_arbiter_node-22]  [1776646135.115] mode rail_approach_active → rail_drive (source=rail)
5451  [rail_driver_node-23]   [1776646135.993] cancel_goal: have_goal_=false, emitting CANCELED tick      (entre wp11/wp12)
5454  [rail_approach_node-28] [1776646137.528] Starting rail approach to tag 3 (offset: 0.300, 0.000)     (wp12 dispatch)
5455  [rail_approach_node-28] [1776646137.528] Skipping Nav2 coarse_approach: robot at (5.50, -2.20) is 1.50 m from tag 3
5456  [rail_approach_node-28] [1776646137.643] Tag 3 acquired, starting fine servoing
... (270 s after — robot no se mueve) ...
5684  [mode_arbiter_node-22]  [1776646447.199] mode rail_drive → rail_exit (source=rail)                   (wp13 dispatch)
```

**Cadena de fallo**:
1. wp11 settles correctamente (5445). FSM transita RAIL_APPROACH_ACTIVE
   → RAIL_DRIVE (5448) porque `rail_approach_state == "settled"`.
2. Al publicar `request_rail_drive_goal`, arbiter encuentra
   `current_x` o `aisle_y_center` en NaN (5447 WARN). Early-return,
   rail_driver nunca recibe goal. `have_goal_=false`.
3. Harness reset entre wp11/wp12 llama cancel_goal (5451 — idempotente,
   ya era false).
4. Harness dispatch wp12 llama `/agv/rail_approach/execute` tag 3 (5454).
5. rail_approach acquires tag, inicia fine_servoing (5456). Publica
   `rail_approach_state = "driving"` y emite cmd_vel_approach.
6. **Pero FSM está en RAIL_DRIVE** (sticky desde step 1). El case
   `RAIL_DRIVE` del FSM **no observa `rail_approach_state == "driving"`** —
   solo transita en `rail_driver_state == "reached"` o
   `blocked_*`. rail_driver está idle sin goal, nunca emitirá
   "reached". FSM stuck por 270 s.
7. `active_source = RAIL` → arbiter relay cmd_vel_rail (silencio —
   rail_driver idle). cmd_vel_approach se publica pero no se relay-ea.
   Robot estático.

Usuario observó: "lo vi solo estático sin moverse" — consistente.
err_xy = 0.50 m = exactamente la distancia (5.5 → 6.0), robot nunca
se movió.

**Fix formal**: Agregar observación de `rail_approach_state == "driving"`
en `case Mode::RAIL_DRIVE` y en `case Mode::RAIL_EXIT` del FSM.
Archivo: `src/agv_mode_arbiter/include/agv_mode_arbiter/mode_fsm.hpp`
líneas 238–260 (RAIL_DRIVE) y 262–305 (RAIL_EXIT).

```cpp
case Mode::RAIL_DRIVE:
  // Iter-24: harness can dispatch rail_approach mid-stream (e.g.
  // chained rail waypoints). If rail_approach transitions to
  // "driving" while we're in RAIL_DRIVE, swap source so
  // cmd_vel_approach is relayed. Otherwise the harness dispatch
  // orphans and we burn 270 s stuck.
  if (in.rail_approach_state == "driving") {
    out.next_mode = Mode::RAIL_APPROACH_ACTIVE;
    out.active_source = Source::APPROACH;
    return out;
  }
  if (in.rail_driver_state == "reached") { ... }  // existing
  ...

case Mode::RAIL_EXIT:
  // Same rationale — harness may re-dispatch rail_approach while
  // we're still in RAIL_EXIT from the previous wp.
  if (in.rail_approach_state == "driving") {
    out.next_mode = Mode::RAIL_APPROACH_ACTIVE;
    out.active_source = Source::APPROACH;
    return out;
  }
  // existing release gates...
```

Test adicional en `test_mode_fsm.cpp`:
```cpp
TEST(ModeFsm, RailDriveTransitionsToApproachOnExternalDispatch) {
  // Simulates iter-23 wp12 regression: wp11 settles → RAIL_DRIVE
  // without a goal (NaN pose). Harness dispatches rail_approach for
  // wp12. FSM should swap to APPROACH, not stay in RAIL_DRIVE.
  FsmInputs in;
  in.rail_approach_state = "driving";
  in.rail_driver_state = "idle";
  auto out = step(Mode::RAIL_DRIVE, in);
  EXPECT_EQ(out.next_mode, Mode::RAIL_APPROACH_ACTIVE);
  EXPECT_EQ(out.active_source, Source::APPROACH);
}
```

**Issue adicional no-bloqueante**: El WARN "request_rail_drive_goal
without pose or aisle_y_center" (5447) indica que en el momento de
settle, el arbiter no tenía `current_x` válido. Esto ocurre porque la
subscripción a `/agv/odometry/global` puede tener delay durante un
tick crítico (EKF re-sync post-reset). No es la causa de wp12 — incluso
con pose válida, el goal se habría publicado y rail_driver habría
ido hacia (6, -2.2), pero habría iniciado un rail_drive que compite con
wp12's rail_approach. El fix FSM elimina esa posibilidad.

### wp15 — Global planner rejects: "Starting point in lethal space"

**Evidencia** (`brain.log`):
```
5774  [bt_navigator-16] [1776646547.817] Begin navigating from current location (5.50, 0.00) to (5.50, -2.20)
6657  [planner_server-13] [1776646586.523] GridBased: failed to create plan, invalid use: Starting point in lethal space! Cannot create feasible plan..
6658  [planner_server-13] [1776646586.523] Planning algorithm GridBased failed to generate a valid path to (5.50, -2.20)
6669  [planner_server-13] [1776646586.754] GridBased: failed to create plan, invalid use: Starting point in lethal space! ...  (retry #2)
6727  [planner_server-13] [1776646658.263] GridBased: failed to create plan, invalid use: Starting point in lethal space! ...  (retry #3, 72 s later)
6731  [planner_server-13] [1776646658.754] GridBased: failed to create plan, invalid use: Starting point in lethal space! ...  (retry #4)
...
```

**Cadena de fallo**:
1. 5774 wp15 dispatch nav2 start=(5.50, 0.00) → goal=(5.50, -2.20).
   Esto es un lane-change 90° (misma x, Δy=-2.2 m).
2. 6657 global planner rechaza: **"Starting point in lethal space"**.
   El pose estimado del robot (5.50, 0.00) está marcado como obstáculo
   en el global_costmap.
3. Retries fallan por 139 s hasta que BT llama ABORTED.
4. Rotation_shim controller intenta rotar en sitio durante ese tiempo
   (el BT tick sigue corriendo aún cuando planner falla) →
   err_yaw=2.93 rad ≈ 168°. Robot rota ~180° sin plan válido.

**Causa raíz**: El **global_costmap tiene celdas lethal acumuladas en la
posición actual del robot**. Esto puede deberse a:

(a) **ZED pointcloud detecta tubos de riel cercanos** al robot justo
    después del rail_exit (robot llega a (5.5, 0) desde el exit tag
    a x=7 via EXIT_PUSH que publica rail_driver goal (5.5, 0)). La ZED
    montada 10 mm del piso ve los tubos frontales de los rieles
    norte (y=0.2+) y sur (y=-0.2-) y los inyecta al costmap como
    obstacle_layer.

(b) **Static obstacle layer del mapa contiene tubos de riel** en zona
    aisle. El mapa greenhouse puede estar marcando x∈[4.0, 7.0], y=0
    como lethal por los tubos de entrada/salida.

(c) **Marker correction RELOC mete al robot en posición lethal**. Sin
    embargo el report.json dice `gt_y=0.000` al inicio — robot está
    físicamente en el aisle central. Si EKF cree que robot está en
    otra y-aisle con obstáculos, planner falla.

**Evidencia de causa (a) y (b)**: En wp10 también vimos "no valid path"
entre (7, 0) y (5.5, 0) — la zona entre x=5.5 y x=7 en y=0 está
percibida como bloqueada. Coherente con tubos de riel sur/norte cerca
del path.

**Fix formal** (2 piezas complementarias):

1. **Global costmap clear antes de dispatch nav2**. Harness ya tiene
   `_wait_for_fresh_tf`; agregar antes `_clear_global_costmap` via
   service `/agv/global_costmap/clear_entirely_global_costmap`.
   - Archivo: `src/agv_integration_tests/test/test_waypoint_precision.py`
   - Agregar helper `_clear_costmaps(harness)` que llama ambos
     `clear_entirely_*_costmap` services.
   - Llamar en dispatch nav2 después de `_motor_prepare` y antes de
     `navigate_to`.

2. **BT `<Spin>` recovery más permisivo**. Actualmente el
   `navigate_to_pose_forward_only.xml` tiene `<Spin spin_dist="1.57"/>`
   (90°). Si el robot rotó 90° mal, no recupera. Aumentar a 3.14
   (180°) como fallback final.
   - Archivo: `src/agv_navigation/behavior_trees/navigate_to_pose_forward_only.xml`.
   - Agregar además un `<Spin spin_dist="-1.57"/>` para rotar en el
     otro sentido.

3. **Pre-rotate harness cuando |Δyaw_to_goal_direction| > 45°**.
   Para wp15 específicamente: start yaw=0, goal dir=atan2(-2.2, 0)=
   -π/2. Diff=90°. Harness podría ejecutar una rotación in-place
   via teleport SetPose o un service de rotate_to_yaw antes del
   dispatch nav2. Pero esto es un workaround, no fix raíz.

Mi recomendación es **solo los fixes 1 y 2**. El fix 3 es hack —
usuario quiere soluciones formales.

## Cross-cutting observations

### CC1: marker_correction RELOC drift descontrolado

En **múltiples ventanas** (wp10 window: lines 1102-1120; wp12 window:
lines 5442-5519) marker_correction acepta drift 2-5 m. Ejemplos:
- Line 1102: `drift=5.7m RELOC` — robot en (7, 0), corrección a (3.52, -0.76)
- Line 5443: `drift=1.7m RELOC` — robot en (5.5, -2.2), corrección a (3.46, 1.86)
- Line 5459: `drift=2.1m RELOC` — robot en (5.5, -2.2), corrección a (7.59, -1.73)

Esto contamina `ekf_global`'s `map→odom` transform y hace que costmaps,
FSM, rail_approach vean el robot en lugares equivocados.

**Fix sugerido**: Investigar por qué tags con `best=27, 28` (tags de
corredor oeste x≈3.5) están siendo "vistos" cuando el robot está en
x≈7. Probable causa: `apriltag_sim_shim` con `max_incidence=89.5°`
proyecta tags que en el mundo real no se verían. Reducir a 85° o 80°
eliminaría las observaciones casi-rasantes que provocan solvePnP
inestable.

- Archivo: `src/agv_apriltag_sim_shim/config/apriltag_sim_shim.yaml`
- Cambio: `max_incidence_deg: 85.0` (era 89.5).
- Verificar que rail_approach (que sí depende de tags rasantes a
  89.4°) siga funcionando — posiblemente necesita un
  `max_incidence_approach` aparte para los floor tags del aisle.

### CC2: min_mode_dwell_s=0.0 no es el problema

iter-22b bajó `min_mode_dwell_s` a 0.0. Los fallos wp10/wp12/wp15 no
son oscilaciones de modo — son FSM transitions legítimas que quedan
atrapadas en estados sin salida. Mantener 0.0.

### CC3: push_published_this_exit_ sticky flag funciona

Las 4 entradas a RAIL_EXIT en iter-23 (wp06, wp09, wp13, wp14)
muestran una sola publicación de EXIT_PUSH cada una. No hay
double-publish observado.

### CC4: Harness cancel_goal funciona correctamente

Lines 5451, 1095, 1118 muestran `cancel_goal: have_goal_=false,
emitting CANCELED tick`. El contrato iter-22 brain 1.1 funciona.

## Plan iter-24

Priorizado por ROI:

1. **[30 min] Fix FSM RAIL_DRIVE/RAIL_EXIT observe rail_approach_state**.
   - `src/agv_mode_arbiter/include/agv_mode_arbiter/mode_fsm.hpp`.
   - 2 transiciones agregadas + 2 tests.
   - **Unlockea wp12**.

2. **[30 min] Harness clear_costmaps antes de dispatch nav2**.
   - `src/agv_integration_tests/test/test_waypoint_precision.py`.
   - Helper nuevo + call en bloque dispatch=="nav2".
   - **Probablemente unlockea wp10 y wp15**.

3. **[15 min] BT Spin recovery con 180°**.
   - `src/agv_navigation/behavior_trees/navigate_to_pose_forward_only.xml`.
   - Última línea de defensa para lane-change grandes.

4. **Run iter-24**. Proyección **14-16/16**.

5. Si marker_correction sigue ruidosa: **[1 h] reducir
   max_incidence_deg y segregar tag types**. Pero no bloquea iter-24.

6. Si 15-16/16: **Phase 3.1 rail_approach híbrido geométrico** para
   cerrar el gate 2 cm y completar Phase 2.

## Qué NO cambia (preservar)

- iter-22 brain 1.1 cancel_goal service ✅
- iter-22 brain 1.3 TF gate ✅
- iter-22 brain 1.4 harness consume sim endpoints ✅
- iter-22b min_mode_dwell_s=0.0 ✅
- iter-23 compute_rail_exit gap-branch fix ✅
- Sticky push flag ✅

## Qué necesito del lado del sim

✅ Nada nuevo — los 4 endpoints implementados siguen funcionando.

Cambio potencial opcional (no bloqueante iter-24): reducir
`max_incidence_deg` en apriltag_sim_shim para eliminar observaciones
rasantes problemáticas. Eso es brain-side (el shim corre en el sim
pero su config es parte de este repo).
