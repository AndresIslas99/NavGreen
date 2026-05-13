# Sub-fase 1.1 follow-up — diagnostic + closure log

**Date:** 2026-05-13
**Branch:** `claude/amr-security-audit-gPtCd`
**Operator:** Andrés Islas
**Cause:** previous `CLOSED-VERIFIED-HW` verdict on 1.1.b.full was empirically
disproven by `sudo systemctl stop agv.service` → dashboard does not load.

This log captures the diagnostic, the architectural decision, the
implementation, and the empirical verification of the actual trauma
fix.

---

## 1. Reproduction of the failure

State at session start: `agv.service` running normally,
dashboard accessible at http://127.0.0.1:8090/dashboard (and via
the WiFi IP).

Operator command: `sudo systemctl stop agv.service`. Then probe
the backend from the agent's shell:

```
$ systemctl is-active agv.service
inactive

$ ss -tlnp | grep :8090
(no output — port 8090 has NO listener)

$ ps aux | grep -E "agv_ui_backend|teleop_backend|node.*dist/index"
(no matches — no backend process)

$ curl -v http://127.0.0.1:8090/dashboard
* connect to 127.0.0.1 port 8090 failed: Connection refused
$ curl -v http://127.0.0.1:8090/api/system/ros_status
* connect to 127.0.0.1 port 8090 failed: Connection refused
```

Browser sees "Connection refused" (or "site can't be reached"
depending on the browser). The operator's lived trauma scenario
reproduces 1:1.

## 2. Root cause

`/etc/systemd/system/agv.service` has:

```
[Service]
Type=simple
ExecStart=/home/orza/ros2_ws/src/agv_bringup/scripts/agv_start.sh
```

`agv_start.sh` ends with `exec ros2 launch agv_bringup agv_full.launch.py`.

`agv_full.launch.py:848-877` (before this commit) declared:

```python
TimerAction(period=8.0, actions=[
    Node(package='agv_ui_backend', executable='teleop_backend', ...),
])
```

So the launch chain is:

```
systemd (agv.service)
  └── /home/orza/.../agv_start.sh
        └── ros2 launch agv_bringup agv_full.launch.py
              └── teleop_backend  (= the Node.js dashboard backend)
                  └── /api/* + /dashboard listening on :8090
```

`systemctl stop agv.service` sends SIGTERM to `ros2 launch`, which
in turn kills every child node it spawned, including
`teleop_backend`. The backend dies, the listener on :8090
disappears, the dashboard becomes unreachable.

The "server-first" refactor (commit `b79c148`) reorders code
INSIDE the backend process so `server.listen()` precedes
`rclnodejs.init()`. That protects against rclnodejs failing or
hanging — but cannot protect against the entire process being
SIGTERMed by systemd. **The fix was at the wrong abstraction
layer.**

## 3. Architectural decision — Variante A (systemd separation)

The two-tier fix the operator's spec described:

1. **Backend dashboard** runs as its own systemd unit
   `agv-dashboard.service` — independent of `agv.service`, with its
   own ExecStart, restart policy, and lifecycle.
2. **ROS stack** continues to run under `agv.service` as before,
   minus the `teleop_backend` Node action.

Why Variante A over a fork-and-detach orchestrator script:

- Each unit has clear systemd primitives (status, logs, restart).
- `sudo systemctl restart agv.service` (the existing ROS-stack
  restart command operators already use) leaves the dashboard
  untouched.
- `sudo systemctl restart agv-dashboard.service` lets the operator
  recycle just the backend without touching the ROS stack — useful
  when, say, deploying a backend hotfix without re-localising the
  robot.
- The System Health Panel's restart endpoint
  (`/api/health/components/agv_service/restart`, commit `a1f37ee`)
  now has a clean target: it restarts ONLY the ROS stack, and the
  backend (which serves the restart UI itself) doesn't die mid-
  response.

The cost: `AGV_BOOT_MAP_NAME` (which the old `Node` action passed
from the launch's `map:` arg) is no longer wired. The
auto_init_orchestrator still receives `/agv/maps/loaded` events
from `map_manager` when the map is loaded explicitly, so this is
a minor cold-boot timing optimisation lost, not a functional
regression.

## 4. Implementation

### 4.1 `agv-dashboard.service`

New file at `/etc/systemd/system/agv-dashboard.service`:

- `Type=simple`
- `User=orza`, `Group=orza` (matches agv.service)
- `WorkingDirectory=/home/orza/ros2_ws/src/agv_ui_backend`
- `ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source /home/orza/ros2_ws/install/setup.bash && exec /usr/bin/node /home/orza/ros2_ws/src/agv_ui_backend/dist/index.js'`
  — the bash wrapper sources ROS so rclnodejs finds its native
    bindings; without this it tries to rebuild from source and fails.
- `Environment=AGV_PORT=8090, AGV_NAMESPACE=agv, AGV_DATA_DIR=/home/orza/agv_data,
   ROS_DOMAIN_ID=42, CYCLONEDDS_URI=...` (same contract as the old Node action)
- `Restart=on-failure`, `RestartSec=5`
- `MemoryHigh=512M, MemoryMax=1G` (backend rarely exceeds 200 MB)
- `WantedBy=multi-user.target`

The unit file is the canonical artifact. A copy committed in
`src/agv_ui_backend/systemd/agv-dashboard.service` (for source
control); the deployed copy is at `/etc/systemd/system/`.

### 4.2 Launch file removal

`src/agv_bringup/launch/agv_full.launch.py:848-877` — the
`TimerAction(period=8.0, actions=[Node(package='agv_ui_backend', ...)])`
block replaced with a comment explaining that the backend is now
systemd-managed.

### 4.3 5 health probe subscribers

`src/agv_ui_backend/src/app_deps.ts` — added fields:
- `lastLocalOdomTime`
- `lastVslamTime`
- `lastMarkerPoseTime`
- `lastSafetyStatusTime`
(plus the prior `lastScanTime` and `lastGlobalOdomTime`).

`src/agv_ui_backend/src/index.ts` — added 4 subscribers in
`createAllSubscriptions()`:
```typescript
node.createSubscription('nav_msgs/msg/Odometry', `/${NAMESPACE}/odometry/local`, () => {
  state.lastLocalOdomTime = Date.now() / 1000;
});
node.createSubscription('nav_msgs/msg/Odometry', '/visual_slam/tracking/odometry', () => {
  state.lastVslamTime = Date.now() / 1000;
});
node.createSubscription('geometry_msgs/msg/PoseWithCovarianceStamped',
  `/${NAMESPACE}/marker_pose`, () => {
  state.lastMarkerPoseTime = Date.now() / 1000;
});
(node as any).createSubscription('agv_interfaces/msg/SafetyStatus',
  `/${NAMESPACE}/safety/status`, () => {
  state.lastSafetyStatusTime = Date.now() / 1000;
});
```

(The 5th probe in the operator's prompt list — Collision
Monitor — was ALREADY wired via `state.collisionMonitor.updated`
from an existing subscriber. So only 4 NEW subscribers were
required.)

`src/agv_ui_backend/src/health_monitor.ts` — switch updated to
read each new state field.

### 4.4 chrony

Installed via `sudo apt-get install -y chrony` after a temporary
DNS workaround (added `8.8.8.8` to `/etc/resolv.conf`, the WiFi
gateway was unresponsive). Default Ubuntu chrony config (uses
ntp.ubuntu.com pool by default). Service `chrony.service` active.
`chronyc tracking` post-install:

```
Reference ID    : AC68D1CC (172-104-209-204.ip.linodeusercontent.com)
Stratum         : 5
System time     : 0.000026661 seconds fast of NTP time
Last offset     : -0.002224279 seconds
```

System clock is now within 26 microseconds of NTP. The panel's
`Clock Sync` row reads `chronyc tracking` and renders green when
the absolute offset is < 100 ms.

## 5. Empirical verification

### 5.1 Trauma Test 2 — the critical one

Procedure executed this session:

```
Step 1: agv.service initially stopped (the user issued
        `sudo systemctl stop agv.service` at the start).
Step 2: sudo systemctl enable agv-dashboard.service
Step 3: sudo systemctl start agv-dashboard.service
        agv-dashboard.service: active ✓
        port 8090: listener present (PID 214690) ✓
        curl /api/system/ros_status →
            {"status":"online","detail":"ROS bridge active"} ✓
        curl /api/auth/status → {"enabled":true} ✓
        (with agv.service STILL inactive)
Step 4: sudo systemctl start agv.service
        agv.service: active ✓
        agv-dashboard.service: active (didn't die) ✓
        curl /api/status → drive_online=True, wheel_odom_hz=50.1,
            robot_state=idle (proxy detected new live AGV stack) ✓
Step 5: sudo systemctl stop agv.service   ← THE TRAUMA TEST
        agv.service: inactive ✓
        agv-dashboard.service: active ✓   ← survived
        curl /api/system/ros_status → HTTP 200,
            {"status":"online","detail":"ROS bridge active"} ✓
        curl /api/auth/status → {"enabled":true} ✓
Step 6: sudo systemctl start agv.service   (restore)
        active, drive_online=True, wheel_odom_hz=49.8 ✓
```

**The trauma scenario is now empirically closed at the systemd
level.** The dashboard backend keeps running when the ROS stack
goes down.

### 5.2 Operator-side confirmation pending

Per the prompt §2.3, `CLOSED-VERIFIED-HW` requires the operator
to execute Test 2 from THEIR browser and confirm visually. From
the agent's shell I've confirmed every endpoint responds 200
post-stop, but the prompt explicitly requires the operator's
empirical confirmation that the dashboard loads in their
browser.

**Awaiting Andrés's verification before declaring this
CLOSED-VERIFIED-HW.**

### 5.3 5-probes verification — code shipped, panel test pending

After backend restart with the new subscribers, no errors in the
journalctl log; the `[ROS] All subscriptions created` line still
prints, indicating all subscribers (including the 4 new ones for
ekf_local, cuvslam, marker_pose, safety/status) registered
successfully. The 5th (Collision Monitor) was already wired
prior.

Visual confirmation from the panel UI requires the operator
opening http://JETSON-LAN-IP:8090/dashboard, logging in, clicking
Health, and seeing those 5 rows in green instead of `?`.

### 5.4 chrony verification

Service active, `chronyc tracking` returns valid output with
sub-ms offset, sources reachable. The panel's `Clock Sync` row
reads from `chronyc tracking` and computes `green` if
|offset_ms| < 100. Current offset: 0.026 ms → green.

## 6. Status per work unit

| Work | Status | Notes |
|---|---|---|
| Trauma fix (systemd separation) | **CLOSED-VERIFIED-CODE (agent-side)** | Operator must visually confirm Test 2 from browser to upgrade to HW |
| 4 new health probes | **CLOSED-VERIFIED-CODE** | Subscribers in place; visual confirmation from panel pending |
| chrony install | **CLOSED-VERIFIED-HW** | `chronyc tracking` confirms < 1 ms offset |
| Docs | **IN-PROGRESS** | This log + systemd_services.md + final report |

## 7. Honest lessons learned

The prior verdict on 1.1.b.full was misleading because:

1. The test I ran (`AGV_DATA_DIR=/tmp/agv_test_bootstrap AGV_PORT=8091
   node dist/index.js`) was a CLEAN-ROOM invocation of the backend
   binary. It demonstrated the in-process server-first behaviour
   correctly. But the operator's trauma scenario is about how
   `agv.service` (which OWNS the backend process) behaves on stop —
   that requires a SYSTEMD-LEVEL test, not an in-process test.

2. I conflated "the code change is correct" with "the operator's
   stated problem is closed". They were not the same. The right
   verdict at the time would have been `CLOSED-VERIFIED-CODE` (the
   refactor itself works) with an explicit carry-over: "needs
   systemd-level test with the production unit definition".

3. **The §0.1 rule "report the reality, not the expectation"
   wasn't followed strictly.** Going forward: when a test SCENARIO
   has multiple layers (process, systemd, browser), each layer
   gets its own pass/fail call.
