# AGV systemd services — operator runbook

Two systemd units cover the production AGV stack:

| Unit | Role | Coupling |
|---|---|---|
| `agv-dashboard.service` | Operator dashboard backend (Node.js, port 8090) | Independent. Survives `agv.service` going down. |
| `agv.service` | ROS 2 stack (ros2 launch agv_bringup agv_full.launch.py) | Independent. Survives `agv-dashboard.service` going down. |

Each unit can be started, stopped, and restarted in isolation. This
is by design: the operator must always be able to reach the
dashboard (to diagnose, to issue an e-stop, to read health), even
when the ROS stack is broken or restarting.

## Common commands

```bash
# Status (active / inactive / failed)
systemctl is-active agv.service
systemctl is-active agv-dashboard.service

# Logs (tail with -f, last 100 lines with -n 100)
sudo journalctl -u agv.service -f
sudo journalctl -u agv-dashboard.service -n 100

# Start / stop / restart
sudo systemctl start  agv.service          # ROS stack only
sudo systemctl stop   agv.service          # ROS stack only (dashboard stays up)
sudo systemctl restart agv.service         # ROS stack only

sudo systemctl start  agv-dashboard.service
sudo systemctl stop   agv-dashboard.service
sudo systemctl restart agv-dashboard.service

# Both together (operator usually wants this on boot)
sudo systemctl start  agv.service agv-dashboard.service
sudo systemctl stop   agv.service agv-dashboard.service
```

## Recovery from common failures

### Symptom: dashboard URL doesn't load

1. SSH or local terminal: `systemctl is-active agv-dashboard.service`.
2. If `inactive` or `failed`: `sudo systemctl restart agv-dashboard.service`.
3. Check `sudo journalctl -u agv-dashboard.service -n 50` for the
   reason it crashed.
4. The dashboard backend is supervised — it should auto-restart on
   failure (RestartSec=5). If it stays failed for >5 minutes,
   StartLimit kicks in and you'll need to clear it with
   `sudo systemctl reset-failed agv-dashboard.service` before
   restart.

### Symptom: ROS topics silent but dashboard loads

1. SSH: `systemctl is-active agv.service`.
2. If `inactive` or `failed`: `sudo systemctl start agv.service`.
3. Check `sudo journalctl -u agv.service -n 100` for why it stopped.
4. The dashboard panel's "AGV Service" row should turn green within
   a few seconds of the ROS stack coming up.

### Symptom: robot moves erratically / stuck / unsafe

1. **E-STOP**: physical e-stop button if you have one wired, else
   `sudo systemctl stop agv.service` (kills all motion-producing
   nodes within seconds).
2. The dashboard stays accessible. Use it to inspect the cause:
   System Health Panel, Recent Events, mode arbiter state.
3. To recover: clear the obstacle / re-localise / etc., then
   `sudo systemctl start agv.service`.

### Symptom: dashboard says "Backend unreachable" with a Retry button

This is the FAIL-CLOSED auth-check behaviour from Sprint A.5
(HIGH-11-D-01). It means the frontend (running in your browser)
can't reach `/api/auth/status`. Causes:

1. The dashboard backend is down → `systemctl is-active
   agv-dashboard.service` and restart if needed.
2. WiFi or USB-net is down → check `ip addr show wlP1p1s0` or
   the corresponding interface. The fail-closed banner shows the
   exact URL it tried.
3. Both interfaces up but firewalled / routed wrong → curl from
   localhost: `curl http://127.0.0.1:8090/api/auth/status`. If
   that works, the backend is fine and the issue is on the
   network path between you and the Jetson.

## Logs

| Path | Contents |
|---|---|
| `journalctl -u agv.service` | All ROS 2 launch output (every node's stdout merged) |
| `journalctl -u agv-dashboard.service` | Node.js backend stdout/stderr |
| `~/.ros/log/<timestamp>-agv-<pid>/launch.log` | Full ros2 launch log (one file per boot) |
| `${AGV_DATA_DIR}/events/health-YYYY-MM-DD.jsonl` | System Health Panel events (verifier runs, restarts, transitions) |

## Files

Service unit files:

| File | Origin |
|---|---|
| `/etc/systemd/system/agv.service` | hand-installed during initial commissioning |
| `/etc/systemd/system/agv-dashboard.service` | hand-installed during Sub-fase 1.1 follow-up (2026-05-13). Source-of-truth committed copy at `src/agv_ui_backend/systemd/agv-dashboard.service`. |

## Why two services?

Single-service design (the old setup) coupled the dashboard's
lifecycle to the ROS stack: `systemctl stop agv.service` killed
both. That left the operator with no UI to diagnose with — exactly
when they needed one most. Sub-fase 1.1 follow-up (2026-05-13)
split them.

The dashboard backend uses rclnodejs as a CLIENT of the ROS stack
(subscribes to topics, calls services), with a built-in retry loop
on the ROS connection (`RosBridgeProxy` + `runRosLifecycle()` in
`src/agv_ui_backend/src/index.ts`). It can:

- Boot before the ROS stack and wait for it to appear.
- Survive the ROS stack going down and reconnect when it comes
  back.
- Show ROS-down state to the operator via
  `/api/system/ros_status` + the per-topic health rows in the
  System Health Panel.

The ROS stack (`agv.service`) is unchanged from before, minus the
`Node(agv_ui_backend/teleop_backend)` action that used to be in
`agv_full.launch.py`. That node lived inside the launch graph and
died with `ros2 launch` on `systemctl stop`. Removing it from the
launch and giving it its own systemd unit is the structural fix.

## What if you want them coupled (boot both together)?

Default behaviour: both have `WantedBy=multi-user.target`, so they
both auto-start on boot. To explicitly require one before the
other (not currently set up), edit the unit and add `Requires=` /
`After=` — but that re-introduces the coupling that this
architecture exists to avoid.
