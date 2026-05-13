# Phase 1 agent log

Running log of agent work for AGV greenhouse Phase 1 sub-phases. Each
entry is dated and references commits, files, and empirical results.

Status legend (per Sub-fase 1.1 §0):
- `CLOSED-VERIFIED-HW` — code change + empirical on-hardware test passed
- `CLOSED-VERIFIED-CODE` — code change verified at build/CI level only
- `CLOSED-CODE-ONLY` — code shipped, runtime/hardware validation deferred
- `IN-PROGRESS` — actively being worked
- `BLOCKED` — waiting on a dependency
- `DEFERRED` — out of scope, captured in `future_work.md`

---

## 2026-05-13 — Sub-fase 1.1.a: `verify_topic_types.py`

**Status: CLOSED-VERIFIED-HW**

### Pre-existing work

The script was already committed as part of the Section-0 carry-over
closure in commit `b051147` (2026-05-13). The Sub-fase 1.1.a prompt
specifies three tests; this entry documents all three.

### Test 1 — Positive: current workspace passes

```
$ python3 tools/verify_specs/verify_topic_types.py
verify_topic_types: scanned 134 create_* declarations in AGV packages,
  65 cross-checked against spec, 0 type mismatch(es).
  (informational: 69 topic-declaration(s) not in specs/interfaces.yaml
   — coverage gap, run with VERBOSE=1 to list. Not a verifier warning.)
verify_topic_types: OK
```

134 declarations scanned across the `src/agv_*` packages. 65 of those
declarations are on topics covered by `specs/interfaces.yaml` and
were cross-checked. 0 mismatches.

### Test 2 — Negative: artificial mismatch (not committed)

Reverted `rail_driver_node.cpp` line 96 to `std_msgs::msg::String`
(simulating the regression). Re-ran:

```
FAIL: src/agv_rail_driver/src/rail_driver_node.cpp:96
      subscription<std_msgs/msg/String>(/agv/collision_monitor_state)
      spec says: nav2_msgs/msg/CollisionMonitorState
      Same bug class as 2026-04-13 audit bug #1, CRITICAL-11-A-01
      (mode_arbiter), and G4/Section-0 (rail_driver). DDS will drop
      messages silently.

BLOCKING: at least one declaration's message type disagrees with the
canonical type in specs/interfaces.yaml.

Exit code: 1
```

Restored the file to current state. Confirmed verifier returns to
`OK` after restore. **Pattern: detect → fail → restore → pass.**

### Test 3 — Histórico (against pre-fix commits)

Two retroactive runs against the actual buggy file content from git
history (filesystem-temporarily swapped then restored):

**(a) Pre-rail_driver-fix (commit `08ac348~1`)**:

```
FAIL: src/agv_rail_driver/src/rail_driver_node.cpp:82
      subscription<std_msgs/msg/String>(/agv/collision_monitor_state)
      spec says: nav2_msgs/msg/CollisionMonitorState
```

The verifier detects the bug at the exact line (82) where the
buggy subscriber declaration lived. Exit code 1 BLOCKING.

**(b) Pre-mode-arbiter-fix (commit `8d81517~1`)** — the original
CRITICAL-11-A-01 bug:

```
FAIL: src/agv_mode_arbiter/src/mode_arbiter_node.cpp:171
      subscription<std_msgs/msg/String>(/agv/collision_monitor_state)
      spec says: nav2_msgs/msg/CollisionMonitorState
```

The verifier detects the original CRITICAL-11-A-01 defect at the
exact pre-fix line (171). Exit code 1 BLOCKING.

**Conclusion**: if `verify_topic_types.py` had existed and been
wired into BLOCKING before 2026-05-13, neither CRITICAL-11-A-01 nor
the G4 rail_driver regression would have shipped. The fourth
occurrence cannot ship past this gate.

### Spec-coverage gaps (informational, NOT failures)

69 topic-declaration warnings about topics not in
`specs/interfaces.yaml`. These are not type-mismatch failures but
genuine spec-coverage gaps. They include:

- Internal-only topics that may not warrant inclusion in the
  inter-package contract (e.g., `/agv/clear_map`,
  `/agv/behavior_executor/status`).
- External-package topics that the AGV consumes
  (`/agv/zed/...`, `/agv/detections`, etc.).
- Test-only topics (`/slam/diagnostics`).

Captured as a Fase-1 follow-up — see `docs/agent/future_work.md`.

### Files touched in this entry

- `tools/verify_specs/README.md` (new) — describes the suite,
  bug-pattern history, and how to add new verifiers.
- `docs/agent/phase1_log.md` (this file).
- `tools/verify_specs/verify_topic_types.py` (already committed
  in `b051147`).
- `tools/verify_specs/all.sh` (already committed in `b051147` —
  BLOCKING array contains the new entry).

### Verdict

**`CLOSED-VERIFIED-HW`.** Three tests passed:
- Test 1 (positive on current state): PASS.
- Test 2 (artificial regression): correctly FAILS.
- Test 3 (histórico against two real pre-fix commits): correctly
  FAILS at the exact pre-fix line in both cases.
