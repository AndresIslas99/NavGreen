# ADR 0001 — Diagnostics & telemetry distribution strategy

**Status**: Accepted — Sprint D, 2026-05-13.
**Closes**: `MEDIUM-10-01` in `docs/audit/2026-05-13-greenhouse-hardening/10_comms.md`.
**Audience**: anyone wiring a new node's health signal into the operator dashboard or external observability tooling.

## Context

The Phase 10 audit found:
- Only **one** node in the workspace publishes the ROS-standard
  `/diagnostics` topic: `fusion_monitor_node` in `agv_sensor_fusion`.
- Several other subsystems (`agv_odrive`, `agv_safety`,
  `agv_localization_init`, `agv_mode_arbiter`, `agv_zone_detector`,
  etc.) publish their own per-subsystem topics with custom payloads,
  typically `std_msgs/String` JSON or a dedicated `agv_interfaces/*`
  message. Examples: `/agv/motor_state`,
  `/agv/safety/status`, `/agv/localization/state`, `/agv/mode/state`,
  `/agv/zone/state`, `/agv/rail_driver/state`, `/agv/rail_approach/state`.
- The operator dashboard (`agv_ui_backend`) subscribes to each of those
  individually and merges them into the WebSocket status frame at 5 Hz.
- There is no `diagnostic_aggregator` configured in the production
  launch. Off-the-shelf tools (`rqt_robot_monitor`,
  `runtime_monitor`) therefore do not show a useful picture.

## Decision

**Keep the custom-topic pattern as the primary distribution mechanism**.
**Do not** retrofit every node to also publish `/diagnostics`. Continue
to use `agv_ui_backend` as the single aggregator for operator-facing
display.

Rationale:

1. **Typed messages over `KeyValue` blobs.** Every subsystem's state is
   already modeled as a typed payload (`SafetyStatus` is a custom
   message; the JSON-on-String topics carry well-defined fields). The
   standard `diagnostic_msgs::DiagnosticArray` flattens everything to
   `name, message, hardware_id, level, [KeyValue]` arrays. We would
   lose the type-checked schema on every transition.
2. **The HMI is the only operator-facing consumer today.** The
   dashboard already knows how to render each subsystem's payload
   correctly (the React panels are wired to specific JSON keys). No
   third-party diagnostic UI is in the deployment plan. Foxglove
   Studio (engineer-side) consumes raw topics directly via the
   foxglove_bridge — also no `/diagnostics` dependency.
3. **Aggregation logic stays in one place.** `agv_ui_backend`
   already merges per-subsystem state into a single status frame and
   applies UI-visibility rules. Splitting this between a
   `diagnostic_aggregator` analyzer YAML and the backend's TypeScript
   would create two sources of truth for "what should the operator
   see".
4. **Custom topics survive cross-distro and namespace changes more
   gracefully** than the brittle DiagnosticArray name/hardware_id
   convention.

## Consequences

### Required by this decision

- Every new subsystem that wants operator visibility MUST:
  1. Publish a typed payload on a dedicated topic under `/agv/*/`.
  2. Add a subscription in `src/agv_ui_backend/src/index.ts` that maps
     the payload into the WebSocket status frame.
  3. Document the topic in `specs/interfaces.yaml` with its owner
     package and intended consumers.
- The dashboard remains the single authoritative aggregator. Engineer-
  side debugging via Foxglove Studio reads the same raw topics directly.
- `fusion_monitor_node` continues to publish `/diagnostics` because it
  already does and the cost of removing it is non-trivial. This is the
  one exception. New `/diagnostics` publishers SHOULD NOT be added.

### Accepted limitations

- `rqt_robot_monitor` and similar off-the-shelf ROS tools do not work
  out of the box for this stack. Engineers debug via Foxglove
  (raw-topic view) or via the operator dashboard. The team accepts
  this in exchange for keeping the typed-message API.
- A future deployment that requires `/diagnostics` integration (e.g.,
  a customer's fleet observability layer expects standard diagnostic
  topics) will need to add a one-time bridge node that reads the
  custom topics and republishes a DiagnosticArray. The bridge stays
  separate from the producers.

## Alternatives considered

### Alternative A — retrofit every node to publish `/diagnostics`

Pros: standard. Off-the-shelf tools work immediately.
Cons: every typed payload flattens to `KeyValue[]`; backend would
still need to UNFLATTEN to render typed UI panels; the cost of
churning every producer is real and risk-prone.
**Rejected.**

### Alternative B — dual-publish (typed + `/diagnostics`)

Every node publishes its typed topic AND a `/diagnostics` summary.
Pros: best of both worlds.
Cons: doubled API surface, two-place updates per change, drift
inevitable.
**Rejected** for the same reasons CR-00-02 geometry-SSOT was a
problem: any value that lives in two places will eventually disagree.

### Alternative C — central bridge node

A new `agv_diagnostics_bridge` subscribes to all custom topics and
republishes a DiagnosticArray. Producers stay unchanged.
Pros: producers unchanged; one place owns the translation; external
tools can subscribe to `/diagnostics` if needed.
Cons: extra node, extra CPU, an additional point of failure for the
operator-visible state.
**Deferred** — viable if/when an external customer requires
DiagnosticArray. Not built today because no consumer requires it.

## Implementation status

| Subsystem | Custom topic | In dashboard? | Notes |
|---|---|---|---|
| agv_odrive | `/agv/motor_state` (JSON) | yes | Phase 11.C audit, line 51-67 of backend status |
| agv_safety | `/agv/safety/status` (`SafetyStatus`) | yes | typed |
| agv_localization_init | `/agv/localization/state` (JSON) | yes | LOC pill |
| agv_mode_arbiter | `/agv/mode/state` (JSON) | yes | mode pill |
| agv_zone_detector | `/agv/zone/state` (JSON) | yes | rail status |
| agv_rail_driver | `/agv/rail_driver/state` (JSON) | yes | rail status |
| agv_rail_approach | `/agv/rail_approach/state` (JSON) | yes | rail status |
| agv_sensor_fusion | `/diagnostics` (DiagnosticArray) | partial | **exception — historical, not removed** |
| Nav2 (lifecycle, costmaps, controller) | `/diagnostics_lifecycle`, etc. | no | upstream Nav2 internal |

A new "add a node" checklist in the workspace README or contribution
guide should reference this ADR and the steps above.

## When to revisit

Revisit this decision if:
- An external customer requires DiagnosticArray for fleet integration
  → adopt Alternative C as a bridge.
- The number of custom-topic subsystems grows past ~15 and the
  dashboard's per-topic subscription glue becomes a maintenance
  burden → consider a JSON-schema-aware aggregator pattern.
- The audit's Phase 13 (CI/CD) ships an automated state-machine
  regression test → consider standardizing on `/diagnostics_agg` for
  CI consumption.
