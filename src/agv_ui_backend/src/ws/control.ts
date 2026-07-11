/**
 * WebSocket /ws/control handler — 5Hz status broadcast + message handlers.
 *
 * Auth: When auth is enabled, clients must pass ?token=<jwt> in the WS URL.
 * Viewers receive status only; operators/engineers can send commands.
 */

import { WebSocketServer, WebSocket } from 'ws';
import * as http from 'http';
import * as rclnodejs from 'rclnodejs';
import type { AppDeps } from '../app_deps';
import { allowedActions } from '../state_machine';
import type { Role } from '../auth';

function calcHz(times: number[]): number {
  if (times.length < 2) return 0;
  const dt = times[times.length - 1] - times[0];
  return dt > 0 ? Math.round(((times.length - 1) / dt) * 10) / 10 : 0;
}

function getStatus(deps: AppDeps) {
  const s = deps.state;
  return {
    robot_state: s.robotState,
    allowed_actions: allowedActions(s.robotState),
    wheel_odom_hz: calcHz(s.odomTimes),
    velocity: s.latestVelocity,
    e_stop: s.eStopActive,
    motors_armed: s.motorState.armed,
    left_state: s.motorState.left_state,
    right_state: s.motorState.right_state,
    motor_errors: s.motorState.left_errors !== 0 || s.motorState.right_errors !== 0,
    drive_online: calcHz(s.odomTimes) > 1.0,
    slam_tracking: s.slamTracking,
    recording: s.recordingActive,
    clients: s.activeClients,
    mode: s.currentMode,
    pose: s.robotPose,
    nav_state: s.navState,
    health: s.health,
    battery_pct: s.batteryPct,
    battery_time_to_empty_s: s.batteryTteS,
    mission_progress: s.missionProgress,
    home_point: s.homePoint,
    mapping_coverage: 0,
    collision_monitor: {
      action: s.collisionMonitor.action,
      polygon: s.collisionMonitor.polygon,
      age_s: s.collisionMonitor.updated > 0
        ? Math.round((Date.now() / 1000 - s.collisionMonitor.updated) * 10) / 10
        : null,
    },
    localization: {
      action: s.localization.action,
      detail: s.localization.detail,
      map: s.localization.map,
    },
    current_map_name: s.currentMapName,
    // Iter-37 Phase 2 — mode arbiter FSM / zone / rail driver state,
    // piggybacked on the 5 Hz status broadcast so every connected client
    // receives the whole rail pipeline snapshot without an extra frame.
    // Consumers render RailStatus.tsx + map overlays from this block.
    rail_state: {
      mode_arbiter: s.modeArbiterState,
      zone: s.zoneDetectorState,
      rail_driver: s.railDriverState,
      rail_approach_state: s.railApproachState,
    },
  };
}

/**
 * Verify WebSocket auth token from query string.
 * Returns role if valid (or 'operator' if auth disabled), null if rejected.
 */
function verifyWsAuth(req: http.IncomingMessage, deps: AppDeps): Role | null {
  if (!deps.authManager.enabled) return 'operator';
  try {
    const url = new URL(req.url || '', `http://${req.headers.host || 'localhost'}`);
    const token = url.searchParams.get('token');
    if (!token) return null;
    const user = deps.authManager.verify(token);
    return user ? user.role : null;
  } catch {
    return null;
  }
}

/**
 * Sprint B / MEDIUM-11-C-06 (2026-05-13 audit) heartbeat. Server pings every
 * 2 s; if the client misses two consecutive pongs (~5 s) the connection is
 * terminated so the close handler runs. Browsers answer ping frames with
 * pong automatically at the WebSocket wire level — no frontend change.
 * Without this, a TCP half-open connection (WiFi drops on the dashboard
 * side) goes undetected until the next write attempt, by which time the
 * 5 Hz status loop may have queued seconds of pending messages. Returns the
 * interval handle; the caller must clearInterval it on close.
 */
function startHeartbeat(ws: WebSocket): NodeJS.Timeout {
  let isAlive = true;
  ws.on('pong', () => { isAlive = true; });
  const HEARTBEAT_INTERVAL_MS = 2000;
  return setInterval(() => {
    if (!isAlive) {
      try { ws.terminate(); } catch { /* ignore */ }
      return;
    }
    isAlive = false;
    try { ws.ping(); } catch { /* ignore */ }
  }, HEARTBEAT_INTERVAL_MS);
}

/**
 * Sprint B / MEDIUM-11-C-06 + HAZOP H-07 deadman. If the disconnecting
 * client was the last operator AND a mission is running, pause the mission
 * and emit a crit event. The mission executor loops on state.missionPause
 * with a 500 ms tick, so the pause takes effect within ~1 s. The mission is
 * NOT aborted — position is preserved so the operator can click Resume
 * (POST /api/missions/resume) after the WiFi recovers.
 */
function pauseMissionIfUnattended(deps: AppDeps): void {
  const missionRunning =
    deps.state.missionProgress != null &&
    deps.state.missionProgress.status === 'running';
  if (deps.state.activeClients === 0 && missionRunning && !deps.state.missionPause) {
    deps.state.missionPause = true;
    deps.eventLog.emit('crit', 'MISSION',
      'Mission paused: last operator disconnected. ' +
      'Resume from the dashboard after reconnecting.');
  }
}

export function setupControlWs(server: http.Server, deps: AppDeps): void {
  const wss = new WebSocketServer({ noServer: true, perMessageDeflate: false });

  server.on('upgrade', (request, socket, head) => {
    const { pathname } = new URL(request.url || '', `http://${request.headers.host || 'localhost'}`);
    if (pathname === '/ws/control') {
      wss.handleUpgrade(request, socket, head, (ws) => {
        wss.emit('connection', ws, request);
      });
    }
  });

  wss.on('connection', (ws: WebSocket, req: http.IncomingMessage) => {
    const role = verifyWsAuth(req, deps);
    if (role === null) {
      ws.close(4401, 'Unauthorized — pass ?token=<jwt> in WebSocket URL');
      return;
    }
    const canCommand = role === 'operator' || role === 'engineer';

    deps.state.activeClients++;
    console.log(`Dashboard client connected (${deps.state.activeClients})`);

    const heartbeatInterval = startHeartbeat(ws);

    let lastPathSnapshot = '';
    let clientMapVersion = deps.state.mapVersion;  // start at current so we don't re-send on connect
    let clientLiveMapVersion = deps.state.liveMapVersion;
    const sentPendingApriltags = new Set<number>();  // Track which pending IDs this client has been notified about

    // Send current maps immediately on connect so reconnecting clients don't see blank
    if (deps.state.mapPng && deps.state.mapMeta) {
      try {
        ws.send(JSON.stringify({
          type: 'map_update',
          png_base64: deps.state.mapPng.toString('base64'),
          ...deps.state.mapMeta,
        }));
      } catch { /* ignore */ }
    }
    if (deps.state.liveMapPng && deps.state.liveMapMeta) {
      try {
        ws.send(JSON.stringify({
          type: 'acc_map',
          png_base64: deps.state.liveMapPng.toString('base64'),
          ...deps.state.liveMapMeta,
        }));
      } catch { /* ignore */ }
    }
    // Send current nav path on connect so reconnecting clients see the active route
    if (deps.state.navPathPoints.length > 0) {
      try {
        const pathStr = JSON.stringify(deps.state.navPathPoints);
        lastPathSnapshot = pathStr;
        ws.send(JSON.stringify({ type: 'path', points: deps.state.navPathPoints }));
      } catch { /* ignore */ }
    }

    // 5Hz status broadcast
    const statusInterval = setInterval(() => {
      if (ws.readyState !== WebSocket.OPEN) return;
      try {
        ws.send(JSON.stringify({ type: 'status', ...getStatus(deps) }));

        if (deps.state.scanPoints.length > 0) {
          ws.send(JSON.stringify({ type: 'scan', points: deps.state.scanPoints }));
        }

        // Path is broadcast whenever the per-client snapshot differs from the
        // current state — no global dirty flag (the previous flag pattern was
        // never reset and corrupted multi-client/refresh scenarios).
        const pathStr = JSON.stringify(deps.state.navPathPoints);
        if (pathStr !== lastPathSnapshot) {
          lastPathSnapshot = pathStr;
          ws.send(JSON.stringify({ type: 'path', points: deps.state.navPathPoints }));
        }

        // Per-client map version tracking (fixes multi-client race condition)
        if (clientMapVersion < deps.state.mapVersion && deps.state.mapPng) {
          clientMapVersion = deps.state.mapVersion;
          ws.send(JSON.stringify({
            type: 'map_update',
            png_base64: deps.state.mapPng.toString('base64'),
            ...deps.state.mapMeta,
          }));
        }

        // Live map from scan_grid_mapper (direct rclnodejs subscription)
        if (clientLiveMapVersion < deps.state.liveMapVersion && deps.state.liveMapPng) {
          clientLiveMapVersion = deps.state.liveMapVersion;
          ws.send(JSON.stringify({
            type: 'acc_map',
            png_base64: deps.state.liveMapPng.toString('base64'),
            ...deps.state.liveMapMeta,
          }));
        }

        // AprilTag pending detections — notify client about new unassigned hardware IDs
        const pending = deps.apriltagManager.getPendingDetections();
        // Clean up sent set: remove IDs no longer pending (assigned or dismissed)
        const currentPendingIds = new Set(pending.map(p => p.hardware_id));
        for (const sentId of sentPendingApriltags) {
          if (!currentPendingIds.has(sentId)) sentPendingApriltags.delete(sentId);
        }
        // Send any new pending detections
        for (const det of pending) {
          if (!sentPendingApriltags.has(det.hardware_id)) {
            sentPendingApriltags.add(det.hardware_id);
            ws.send(JSON.stringify({
              type: 'apriltag_pending',
              hardware_id: det.hardware_id,
              first_seen: det.first_seen,
            }));
          }
        }

        for (const evt of deps.eventLog.popPending()) {
          ws.send(JSON.stringify({ type: 'event', ...evt }));
        }
      } catch { /* client disconnected */ }
    }, 200);

    // Message handlers
    ws.on('message', (data: Buffer) => {
      // Only the JSON.parse + shape check is allowed to fail silently —
      // malformed frames from a flaky client are expected and benign.
      let msg: any;
      try {
        msg = JSON.parse(data.toString());
      } catch { /* ignore malformed frames */ return; }
      if (typeof msg !== 'object' || msg === null || typeof msg.type !== 'string') return;

      try {
        // Viewers can only receive status; reject commands
        if (!canCommand) {
          ws.send(JSON.stringify({ type: 'error', message: 'Viewer role cannot send commands' }));
          return;
        }

        switch (msg.type) {
          case 'cmd_vel':
            // Number() coercion: non-numeric input becomes NaN, which
            // sendCmdVel rejects (never published as a Twist).
            deps.ros.sendCmdVel(Number(msg.linear ?? 0), Number(msg.angular ?? 0));
            break;
          case 'e_stop':
            deps.ros.sendEStop(!!msg.active);
            break;
          case 'motor_enable':
            deps.ros.sendMotorEnable(!!msg.active);
            break;
          case 'mode':
            if (['teleop', 'mapping', 'nav'].includes(msg.mode)) {
              // setMode is async now (nav transition validates Nav2 lifecycle
              // active). Fire-and-forget at the WS layer; the result is
              // surfaced via eventLog, and the next status tick reflects the
              // actual currentMode (unchanged if the transition was rejected).
              deps.setMode(msg.mode).catch((e) => {
                console.warn('[ws] setMode failed:', e);
              });
            }
            break;
          case 'nav_goal':
            if (deps.state.currentMode === 'nav') {
              deps.ros.sendNavGoal(
                Number(msg.x ?? 0),
                Number(msg.y ?? 0),
                Number(msg.theta ?? 0),
              );
            }
            break;
          case 'nav_cancel':
            deps.ros.cancelNavGoal();
            break;
          case 'recording': {
            const action = msg.action;
            (async () => {
              let result;
              if (action === 'start') {
                result = await deps.ros.callTriggerService(deps.ros.startRecClient, 'start_recording');
                if (result.success) deps.state.recordingActive = true;
              } else if (action === 'stop') {
                result = await deps.ros.callTriggerService(deps.ros.stopRecClient, 'stop_recording');
                if (result.success) deps.state.recordingActive = false;
              } else {
                result = { success: false, message: 'Unknown action' };
              }
              if (ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({ type: 'recording_result', ...result }));
              }
            })().catch(() => { /* ignore */ });
            break;
          }
        }
      } catch (e: any) {
        // A failure while dispatching a command (e.g. an e-stop) must not be
        // silent — it would be invisible in the field.
        console.warn(`[ws/control] command '${msg.type}' failed:`, e?.message || e);
      }
    });

    ws.on('close', () => {
      clearInterval(statusInterval);
      clearInterval(heartbeatInterval);
      deps.state.activeClients = Math.max(0, deps.state.activeClients - 1);
      deps.ros.sendCmdVel(0, 0);
      pauseMissionIfUnattended(deps);
    });
  });
}

export function setupTeleopWs(server: http.Server, deps: AppDeps): void {
  const wss = new WebSocketServer({ noServer: true, perMessageDeflate: false });

  server.on('upgrade', (request, socket, head) => {
    const { pathname } = new URL(request.url || '', `http://${request.headers.host || 'localhost'}`);
    if (pathname === '/ws/teleop') {
      wss.handleUpgrade(request, socket, head, (ws) => {
        wss.emit('connection', ws, request);
      });
    }
  });

  wss.on('connection', (ws: WebSocket, req: http.IncomingMessage) => {
    const role = verifyWsAuth(req, deps);
    if (role === null) {
      ws.close(4401, 'Unauthorized');
      return;
    }
    if (role === 'viewer') {
      ws.close(4403, 'Viewer role cannot use teleop');
      return;
    }

    deps.state.activeClients++;
    // Heartbeat matters even more here: a half-open teleop socket means the
    // joystick's cmd_vel stream stopped arriving, but the close-handler's
    // zero-velocity send only fires once the close is actually detected.
    const heartbeatInterval = startHeartbeat(ws);
    ws.on('message', (data: Buffer) => {
      let msg: any;
      try {
        msg = JSON.parse(data.toString());
      } catch { /* ignore malformed frames */ return; }
      if (typeof msg !== 'object' || msg === null || typeof msg.type !== 'string') return;
      try {
        if (msg.type === 'cmd_vel') deps.ros.sendCmdVel(Number(msg.linear ?? 0), Number(msg.angular ?? 0));
        else if (msg.type === 'e_stop') deps.ros.sendEStop(!!msg.active);
      } catch (e: any) {
        console.warn(`[ws/teleop] command '${msg.type}' failed:`, e?.message || e);
      }
    });
    ws.on('close', () => {
      clearInterval(heartbeatInterval);
      deps.state.activeClients = Math.max(0, deps.state.activeClients - 1);
      deps.ros.sendCmdVel(0, 0);
      pauseMissionIfUnattended(deps);
    });
  });
}
