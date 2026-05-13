# Phase 1 lessons learned — running record

Permanent record of process failures and their corrections. Each
sub-phase adds its own entry. The goal: future sub-phases (and
future agents) read this BEFORE starting their own work so the same
failure doesn't repeat.

---

## 2026-05-13 — Sub-fase 1.1: the optimistic CLOSED verdict trap

### What happened

After implementing the "server-first" refactor (commit `b79c148`), the
agent marked Sub-fase 1.1.b.full as `CLOSED-VERIFIED-HW`. The
evidence: a clean-room test (`AGV_PORT=8091 node dist/index.js`)
that brought the backend up before `rclnodejs.init()` completed.

The operator then ran the operator's actual scenario —
`sudo systemctl stop agv.service` followed by browser refresh — and
the dashboard did not load. **The trauma scenario that motivated
the entire refactor was still open.**

### Root cause

The agent conflated two distinct claims:

  (a) "The in-process refactor is correct." — true, demonstrated by
      the clean-room test.
  (b) "The operator's stated problem is closed." — FALSE.

The trauma scenario lives in a multi-layer stack:

```
operator's browser
  ↓ HTTP
network
  ↓
systemd unit boundary       ←──── the agent's test didn't touch this layer
  ↓
process (node.js)
  ↓
in-process bootstrap        ←──── the agent's test only validated here
  ↓
rclnodejs.init()
```

The clean-room test demonstrated the in-process layer was correct.
But the failure mode was at the systemd layer: `systemctl stop
agv.service` killed the entire process, regardless of how
beautifully the in-process bootstrap was structured.

The fix that genuinely closed the scenario (commit `c24606c`) was
at the systemd layer: split the backend into its own
`agv-dashboard.service` independent of `agv.service`. The
in-process refactor (b79c148) was a useful prerequisite but not
sufficient.

### Why the agent got it wrong

1. **The agent's tests stopped at the layer the agent could easily
   probe** (process). The agent ran `node dist/index.js` directly
   because that was the path of least resistance from a shell.

2. **The agent didn't reproduce the operator's exact failure scenario
   before claiming closure.** The operator's scenario uses
   `systemctl stop`, not `kill -9` on a manually-started process.
   Those two operations look similar but exercise completely
   different code paths (systemd unit teardown vs raw signal).

3. **The agent reported the test that PASSED rather than the test
   the operator would run.** A test that demonstrates the desired
   behaviour in a clean-room is necessary but not sufficient.

### Rules going forward

These are NOT suggestions. Future sub-phases follow them:

1. **`CLOSED-VERIFIED-HW` requires the operator's confirmation from
   their environment.** Not the agent's confirmation from a clean-
   room. If the operator's scenario is "I refresh my browser", the
   verdict requires the operator pressing F5 in their browser and
   reporting empirically.

2. **Reproduce the operator's exact failure mode BEFORE claiming
   closure.** If the operator says `sudo systemctl stop X` breaks
   the dashboard, the agent's verification run MUST start with
   `sudo systemctl stop X` from the agent's shell. Not a
   simulation of it. The exact same command.

3. **Multi-layer scenarios get multi-layer verdicts.**

   A scenario that crosses process/systemd/network/browser is closed
   at each layer separately and the OVERALL closure is the AND of all
   layer closures. A pass at one layer is named `CLOSED-VERIFIED-<layer>`,
   e.g. `CLOSED-VERIFIED-CODE` for code change, `CLOSED-VERIFIED-CODE
   (agent-side)` for agent-shell-only verification.

4. **The operator runs the tests BEFORE the checkpoint report,
   not after.**

   Old flow: agent ships → agent writes report claiming closure →
   operator runs tests → discovers failures → agent has to revise.

   New flow: agent ships → agent notifies operator with specific
   test instructions → operator runs tests → operator reports
   results → agent iterates if needed → ONLY when operator confirms
   all tests pass, agent writes the checkpoint report.

5. **A test that passes in the agent's environment but not the
   operator's is a process bug, not an operator bug.** The agent
   re-investigates rather than dismissing the operator's report.

### Reference commits

  - The wrong verdict: `b79c148` (server-first in-process refactor;
    correctly described in the code but mis-classified as
    `CLOSED-VERIFIED-HW`).
  - The closure: `c24606c` (systemd separation; the actual fix for
    the operator's scenario).
  - The operator-verified verdict: `2abe5c0` (docs upgraded to
    `CLOSED-VERIFIED-HW` after the operator confirmed "pagina carga").

The diagnostic log is in `docs/agent/phase1_1_followup_log.md`.
