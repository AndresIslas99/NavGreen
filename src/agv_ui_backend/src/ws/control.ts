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
    mission_progress: s.missionProgress,
    mapping_coverage: 0,
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

    // 5Hz status broadcast
    const statusInterval = setInterval(() => {
      if (ws.readyState !== WebSocket.OPEN) return;
      try {
        ws.send(JSON.stringify({ type: 'status', ...getStatus(deps) }));

        if (deps.state.scanPoints.length > 0) {
          ws.send(JSON.stringify({ type: 'scan', points: deps.state.scanPoints }));
        }

        if (deps.state.navPathChanged) {
          const pathStr = JSON.stringify(deps.state.navPathPoints);
          if (pathStr !== lastPathSnapshot) {
            lastPathSnapshot = pathStr;
            ws.send(JSON.stringify({ type: 'path', points: deps.state.navPathPoints }));
          }
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

        // Live map from scan_grid_mapper (proper Bayesian grid via live_map_bridge)
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
      try {
        const msg = JSON.parse(data.toString());

        // Viewers can only receive status; reject commands
        if (!canCommand) {
          ws.send(JSON.stringify({ type: 'error', message: 'Viewer role cannot send commands' }));
          return;
        }

        switch (msg.type) {
          case 'cmd_vel':
            deps.ros.sendCmdVel(msg.linear || 0, msg.angular || 0);
            break;
          case 'e_stop':
            deps.ros.sendEStop(!!msg.active);
            break;
          case 'motor_enable':
            deps.ros.sendMotorEnable(!!msg.active);
            break;
          case 'mode':
            if (['teleop', 'mapping', 'nav'].includes(msg.mode)) {
              deps.setMode(msg.mode);
            }
            break;
          case 'nav_goal':
            if (deps.state.currentMode === 'nav') {
              deps.ros.sendNavGoal(
                parseFloat(msg.x || 0),
                parseFloat(msg.y || 0),
                parseFloat(msg.theta || 0),
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
      } catch { /* ignore parse errors */ }
    });

    ws.on('close', () => {
      clearInterval(statusInterval);
      deps.state.activeClients = Math.max(0, deps.state.activeClients - 1);
      deps.ros.sendCmdVel(0, 0);
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
    ws.on('message', (data: Buffer) => {
      try {
        const msg = JSON.parse(data.toString());
        if (msg.type === 'cmd_vel') deps.ros.sendCmdVel(msg.linear || 0, msg.angular || 0);
        else if (msg.type === 'e_stop') deps.ros.sendEStop(!!msg.active);
      } catch { /* ignore */ }
    });
    ws.on('close', () => {
      deps.state.activeClients = Math.max(0, deps.state.activeClients - 1);
      deps.ros.sendCmdVel(0, 0);
    });
  });
}
