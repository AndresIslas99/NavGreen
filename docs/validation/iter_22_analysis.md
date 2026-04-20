# iter-22 análisis — qué hacemos con la data acumulada

**Resultado iter-22**: 11/16 SUCCEEDED (mismo que iter-20/21 numéricamente).
Pero **la mezcla cambió**, y eso nos da información.

## Comparativa iter-21 → iter-22 por waypoint

| wp | iter-21 | iter-22 | Δ | Causa del cambio |
|---|---|---|---|---|
| wp01 | ✅ 0.064 | ✅ 0.085 | = | ambas pasan |
| wp02 | ❌ 1.00 TF | ✅ 0.065 | **+WIN** | TF gate (brain 1.3) funcionó |
| wp03 | ✅ 0.057 | ✅ 0.082 | = | — |
| wp04 | ✅ 0.101 | ✅ 0.101 | = | rail_approach robusto |
| wp05 | ✅ 0.050 | ✅ 0.050 | = | — |
| wp06 | ✅ 0.050 | ✅ 0.047 | = | — |
| wp07 | ✅ 0.090 | ✅ 0.090 | = | — |
| wp08 | ✅ 0.049 | ✅ 0.048 | = | — |
| wp09 | ✅ 0.049 | ✅ 0.048 | = | — |
| wp10 | ❌ 1.82 ABORT | ❌ 0.85 NAV_TIMEOUT | ~= | modestly better |
| wp11 | ✅ 0.101 | ✅ 0.101 | = | rail_approach robusto |
| wp12 | ✅ 0.090 | ❌ 0.50 NAV_TIMEOUT | **-REGRESSION** | FSM dwell → tag lost |
| wp13 | ❌ 1.54 | ❌ 1.54 | = | misma causa |
| wp14 | ❌ 1.54 | ❌ 1.54 | = | misma causa |
| wp15 | ❌ 2.16 ABORT | ❌ 3.52 NAV_TIMEOUT | **-REGRESSION** | FSM dwell peor |
| wp16 | ✅ 0.090 | ✅ 0.090 | = | — |

**Net**: +1 (wp02) −1 (wp12) = 0. La dwell regresó dos, la TF gate ganó uno.

## Lo que aprendimos (win)

1. **`_wait_for_fresh_tf` gate funciona** (wp02 pasa consistentemente).
   Solución formal para el problema `rotation_shim_controller: Failed to
   transform pose to base frame!` en post-teleport.

2. **`/motor/prepare` + `/reset` validado + `cancel_goal` service
   eliminaron la carrera post-teleport** — ya no vemos el drift a
   `current_x=9.95 clearance_now=-2.95` que plagaba iter-21.

3. **rail_approach robusto**: 5/5 en iter-21, 5/5 con matiz en iter-22.
   La regresión de wp12 NO es rail_approach-intrínseca — es la dwell
   del FSM interrumpiendo la relay de cmd_vel_approach.

## Lo que aprendimos (fallos persistentes)

### wp13/wp14 rail_exit — **root cause identificado**

Evidencia iter-22 (timestamp 1776643870):
```
goal reached (err=0.050 m)
rail_exit_push skipped: clearance=-0.05 m (current_x=3.950)
mode rail_drive → rail_exit (source=rail)
```

wp13 robot arranca en (1, 0, 0), rail_driver lo lleva a (4, 0) (goal).
Al reach (err=0.050m), el arbiter entra a RAIL_EXIT y llama
`publish_rail_exit_push_goal()`. Esa función llama
`compute_rail_exit(current_x=3.95)`:

```cpp
} else {
  // Already in gap — pick whichever tag is closer so the clearance
  // metric still makes sense (distance past the nearer tag, outward
  // from it). This is the case where the robot entered RAIL_EXIT with
  // a very short rail stretch; no push needed.                   ← COMMENT
  ...
  out.skip_push = true;                                            ← BUG
}
```

El código asume que "si robot está en gap (3.5 < x < 7.5) → ya salió
del riel, no necesita push". Pero wp13 llega a x=3.95 **desde el
corredor oeste**, no desde dentro del riel. El tag está en x=4.0 y el
robot NO ha pasado aún. Sin embargo el código marca `skip_push=true` +
`clearance=-0.05` (negativo) y jamás publica el push goal.

FSM release primario necesita `clearance ≥ 1.0` — **imposible sin push**.
Deadlock.

**Solución formal**: eliminar el early-skip en la rama "in gap". Si
`clearance < 1.0`, publicar push goal siempre — independientemente de
si el robot está en gap o en rail section. La tolerancia `clearance >=
1.0 → skip_push = true` (línea 68-70) ya maneja el caso "ya pasado el
tag, no empujes más".

### wp10/wp15 — nav2 post-rail — **causa diferente al resto**

wp10: (7, 0, π) → (5.5, 0, π). Robot ended err_xy=0.85 err_yaw=0.30
en 180s. Se movió ~0.65m pero NO llegó. MPPI no converge.

wp15: (5.5, 0, 0) → (5.5, -2.2, 0). Robot ended err=3.52 err_yaw=3.08
(~176°). Rotó mucho sin avanzar.

Causa común: **MPPI forward-only + rotation_shim mal coordinado**.
- wp10: robot yaw=π, path tangent = -X (yaw=π). Debería poder
  avanzar sin rotar. Pero MPPI interpreta path tangent en marco
  mundo y el `PreferForwardCritic` prefiere "moverse en dirección
  del tangent" (yaw=π world = -X world). Eso es igual al robot
  yaw=π. Debería funcionar... pero no.
- wp15: rotación 90° necesaria. MPPI con vx_min=0 no puede generar
  una trayectoria de lane-change porque rotaría 90° primero y luego
  avanzaría — pero MPPI `wz` y `vx` no se muestran independientes en
  muestreo forward-only.

**Solución formal para wp15**: BT custom con `<Spin>` antes del
`<FollowPath>` cuando `diff_yaw > threshold`. Nav2 ya tiene
`nav2_behaviors::Spin` cargado.

**Solución para wp10**: más sutil. Posibles opciones:
- A) Agregar también `<Spin>` antes con threshold menor (10° por
  ejemplo) para alinearse antes de perseguir el path. Robusto pero
  puede agregar tiempo innecesario.
- B) Dar a rail_driver el control inicial — publicar un rail_drive
  hasta que el robot esté "fuera" de la influencia del tag, luego
  Nav2 toma control. Sí pero es arquitectónicamente raro (Nav2
  no se usa en rail_drive).
- C) **Simplemente esperar ≥ 2s entre el fin del rail_drive/rail_exit
  y el dispatch de nav2**. La `rotation_shim` falla durante la ventana
  post-teleport. Con `_wait_for_fresh_tf` ya esperamos, pero tal vez
  el window para wp10 necesita ser más largo. No es elegante pero es
  de bajo costo.

Mi lectura: **wp10 SÍ funciona cuando no hay carrera de modos/TF, pero
el encadenado wp09→wp10 crea más interferencia que wp01→wp02 (ambos
nav2 iniciando limpios)**. Los fixes de iter-22 ayudaron (de 1.82m a
0.85m) pero no eliminaron el problema.

### wp12 regresión — **FSM dwell interferir con rail_approach**

Evidencia iter-22: "Tag lost during fine servoing" en rail_approach
mientras la dwell aún bloqueaba mode transitions. El fine_servo
estaba emitiendo cmd_vel_approach pero el arbiter no lo relay-eaba
porque la dwell impidió la transición a `RAIL_APPROACH_ACTIVE`.

**Solución formal**: **eliminada** — `min_mode_dwell_s` se puso default
en 0.0 en iter-22b (ya committeado). La sticky flag se mantiene porque
previene double EXIT_PUSH sin bloquear nada.

## Plan de ataque iter-23

Propuesta priorizada:

1. **Fix `compute_rail_exit` gap-branch bug** (15 min + 1 test case).
   Unlockea wp13 y wp14.
   Archivo: `src/agv_mode_arbiter/include/agv_mode_arbiter/rail_exit_geometry.hpp`.

2. **BT `<Spin>` pre-`<FollowPath>` con threshold ~10°** (1 h).
   Unlockea wp15, probablemente mejora wp10.
   Archivo:
   `src/agv_navigation/behavior_trees/navigate_to_pose_forward_only.xml`.

3. **Run iter-23** con estos dos fixes + iter-22b dwell=0 aplicado.
   Proyección: **14-15/16**.

4. Si wp10 sigue fallando: investigar si es carrera residual entre
   EKF sync y TF o algo más profundo. Probablemente necesite aumentar
   `_wait_for_fresh_tf` timeout o agregar un settle post-sync.

5. Si 14-15/16: **Phase 3.1 rail_approach híbrido geométrico** para
   cerrar el gate 2cm de Phase-2.

## Qué NO cambia (preservar)

- iter-22 brain 1.1 (cancel_goal service) — elimina el hack.
- iter-22 brain 1.3 (TF gate) — fixea wp02.
- iter-22 brain 1.4 (harness consume /motor/prepare + /reset validated).
- iter-22b (dwell default 0.0).
- Sticky push flag — prevent double-emission sin bloquear transiciones.

## Qué necesito del lado del sim

✅ Nada nuevo — los 4 endpoints ya están implementados y usados.
