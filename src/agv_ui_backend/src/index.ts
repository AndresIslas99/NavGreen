#!/usr/bin/env node
/**
 * AGV Operator Backend — TypeScript + rclnodejs + Express
 *
 * Production replacement for the Python teleop_server.py.
 * Provides REST API, WebSocket, and ROS2 bridge for the operator dashboard.
 */

import * as http from 'http';
import * as path from 'path';
import * as fs from 'fs';
import { execFile } from 'child_process';
import express from 'express';
import { WebSocketServer, WebSocket } from 'ws';
import * as rclnodejs from 'rclnodejs';

import { deriveState, allowedActions, RobotState, MotorState, NavState, MissionProgress } from './state_machine';
import { EventLog, LogEntry } from './event_log';
import { ScanAccumulator } from './scan_accumulator';
import { TelemetryStore } from './telemetry_store';
import { AuthManager } from './auth';

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

const PORT = parseInt(process.env.AGV_PORT || '8090');
const DATA_DIR = process.env.AGV_DATA_DIR || '/tmp/agv_data';
const NAMESPACE = process.env.AGV_NAMESPACE || 'agv';
const MAPS_DIR = process.env.AGV_MAPS_DIR || path.join(DATA_DIR, 'maps');

fs.mkdirSync(DATA_DIR, { recursive: true });
fs.mkdirSync(path.join(DATA_DIR, 'missions'), { recursive: true });
fs.mkdirSync(MAPS_DIR, { recursive: true });

const MISSIONS_FILE = path.join(DATA_DIR, 'missions', 'missions.json');
if (!fs.existsSync(MISSIONS_FILE)) fs.writeFileSync(MISSIONS_FILE, '[]');

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

let eStopActive = false;
let currentMode = 'teleop';
let robotPose = { x: 0, y: 0, theta: 0 };
let latestVelocity = { linear: 0, angular: 0 };
let motorState: MotorState = { armed: false, left_state: 0, right_state: 0, left_errors: 0, right_errors: 0 };
let navState: NavState = { active: false, distance_remaining: 0, status: 'idle' };
let missionProgress: MissionProgress | null = null;
let robotState: RobotState = 'offline';
let prevRobotState: RobotState = 'offline';
let slamTracking = 'unknown';
let scanPoints: Array<{ x: number; y: number }> = [];
let navPathPoints: Array<{ x: number; y: number }> = [];
let navPathChanged = false;
let odomTimes: number[] = [];
let activeClients = 0;
let lastCmdTime = 0;
let mapPng: Buffer | null = null;
let mapMeta: any = null;
let mapChanged = false;

// Mission execution state
let missionCancel = false;
let missionPause = false;

// Recording state
let recordingActive = false;

// Nav2 action client state
let navGoalHandle: any = null;

const health: Record<string, any> = {
  drive: { status: 'unknown', detail: 'waiting', updated: 0 },
  imu: { status: 'unknown', detail: 'waiting', updated: 0 },
  slam: { status: 'unknown', detail: 'waiting', updated: 0 },
  nav: { status: 'unknown', detail: 'waiting', updated: 0 },
  network: { status: 'ok', detail: '', updated: Date.now() / 1000 },
};

const eventLog = new EventLog(DATA_DIR);
const scanAccumulator = new ScanAccumulator();
const RETENTION_DAYS = parseInt(process.env.AGV_RETENTION_DAYS || '30');
const telemetryStore = new TelemetryStore(DATA_DIR, RETENTION_DAYS);

const authManager = new AuthManager(DATA_DIR);

// Persist events to SQLite for analytics
eventLog.onEvent((entry) => {
  try { telemetryStore.recordEvent(entry.timestamp, entry.severity, entry.subsystem, entry.text); }
  catch { /* ignore */ }
});

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

function calcHz(times: number[]): number {
  if (times.length < 2) return 0;
  const dt = times[times.length - 1] - times[0];
  return dt > 0 ? Math.round(((times.length - 1) / dt) * 10) / 10 : 0;
}

function yawFromQuat(q: { x: number; y: number; z: number; w: number }): number {
  return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}

function updateState(): void {
  const old = robotState;
  robotState = deriveState(eStopActive, motorState, calcHz(odomTimes), currentMode, missionProgress, navState);
  if (old !== robotState) {
    eventLog.emit(
      robotState === 'e_stop' || robotState === 'fault' ? 'crit' : 'info',
      'SYSTEM',
      `State: ${old} → ${robotState}`
    );
    prevRobotState = old;
  }
}

function getStatus() {
  return {
    robot_state: robotState,
    allowed_actions: allowedActions(robotState),
    wheel_odom_hz: calcHz(odomTimes),
    velocity: latestVelocity,
    e_stop: eStopActive,
    motors_armed: motorState.armed,
    left_state: motorState.left_state,
    right_state: motorState.right_state,
    motor_errors: motorState.left_errors !== 0 || motorState.right_errors !== 0,
    drive_online: calcHz(odomTimes) > 1.0,
    slam_tracking: slamTracking,
    recording: recordingActive,
    clients: activeClients,
    mode: currentMode,
    pose: robotPose,
    nav_state: navState,
    health,
    mission_progress: missionProgress,
  };
}

function readMissions(): any[] {
  try {
    return JSON.parse(fs.readFileSync(MISSIONS_FILE, 'utf-8'));
  } catch { return []; }
}

function writeMissions(missions: any[]): void {
  fs.writeFileSync(MISSIONS_FILE, JSON.stringify(missions, null, 2));
}

function normalizeMission(m: any): any {
  if (!m.nodes && m.waypoints) {
    m.nodes = m.waypoints.map((wp: any, i: number) => ({
      id: `n${i}`, type: 'waypoint', action: 'none', ...wp,
    }));
    m.edges = [];
  }
  return m;
}

/** List .yaml map files from MAPS_DIR */
function listMapFiles(): Array<{ name: string; modified: number }> {
  const result: Array<{ name: string; modified: number }> = [];
  try {
    for (const f of fs.readdirSync(MAPS_DIR).sort()) {
      if (!f.endsWith('.yaml')) continue;
      const fullPath = path.join(MAPS_DIR, f);
      const text = fs.readFileSync(fullPath, 'utf-8');
      if (!text.includes('image:')) continue;
      const stat = fs.statSync(fullPath);
      result.push({ name: f.replace('.yaml', ''), modified: stat.mtimeMs / 1000 });
    }
  } catch { /* ignore */ }
  return result;
}

// ---------------------------------------------------------------------------
// ROS2 Node
// ---------------------------------------------------------------------------

async function main() {
  await rclnodejs.init();
  const node = new rclnodejs.Node('teleop_server');

  // Publishers
  const cmdVelPub = node.createPublisher('geometry_msgs/msg/Twist', `/${NAMESPACE}/cmd_vel`);
  const eStopPub = node.createPublisher('std_msgs/msg/Bool', `/${NAMESPACE}/e_stop`);
  const motorEnablePub = node.createPublisher('std_msgs/msg/Bool', `/${NAMESPACE}/motor_enable`);

  // -------------------------------------------------------------------------
  // Nav2 Action Client (P1.1)
  // -------------------------------------------------------------------------
  const navActionClient = new rclnodejs.ActionClient(
    node, 'nav2_msgs/action/NavigateToPose', `/${NAMESPACE}/navigate_to_pose`
  );

  function sendNavGoal(x: number, y: number, theta: number = 0): { success: boolean; message: string } {
    if (currentMode !== 'nav') {
      return { success: false, message: 'Not in nav mode' };
    }
    if (!navActionClient.isActionServerAvailable()) {
      health.nav = { status: 'error', detail: 'Nav2 server not available', updated: Date.now() / 1000, action: 'Check Nav2 lifecycle' };
      return { success: false, message: 'Nav2 action server not available' };
    }

    const goal: any = {
      pose: {
        header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
        pose: {
          position: { x, y, z: 0 },
          orientation: { x: 0, y: 0, z: Math.sin(theta / 2.0), w: Math.cos(theta / 2.0) },
        },
      },
    };

    navActionClient.sendGoal(goal, (feedback: any) => {
      // Feedback callback
      navState.distance_remaining = Math.round((feedback.distance_remaining || 0) * 1000) / 1000;
      navState.status = 'active';
    }).then((goalHandle: any) => {
      if (!goalHandle.isAccepted()) {
        navState = { active: false, distance_remaining: 0, status: 'rejected' };
        eventLog.emit('warn', 'NAV', 'Goal rejected');
        updateState();
        return;
      }
      navGoalHandle = goalHandle;
      navState.status = 'active';
      health.nav = { status: 'ok', detail: 'Navigating', updated: Date.now() / 1000 };
      updateState();

      // Get result
      goalHandle.getResult().then((result: any) => {
        const status = goalHandle.status;
        // STATUS_SUCCEEDED=4, STATUS_CANCELED=5, STATUS_ABORTED=6
        if (status === 4) {
          navState = { active: false, distance_remaining: 0, status: 'succeeded' };
          eventLog.emit('info', 'NAV', 'Goal reached');
        } else if (status === 5) {
          navState = { active: false, distance_remaining: 0, status: 'canceled' };
          eventLog.emit('info', 'NAV', 'Goal canceled');
        } else {
          navState = { active: false, distance_remaining: 0, status: 'aborted' };
          eventLog.emit('warn', 'NAV', 'Goal aborted');
        }
        navGoalHandle = null;
        navPathPoints = [];
        navPathChanged = true;
        updateState();
      }).catch(() => {
        navState = { active: false, distance_remaining: 0, status: 'aborted' };
        navGoalHandle = null;
        updateState();
      });
    }).catch((err: any) => {
      navState = { active: false, distance_remaining: 0, status: 'aborted' };
      eventLog.emit('warn', 'NAV', `Goal send failed: ${err?.message || err}`);
      updateState();
    });

    navState = { active: true, distance_remaining: 0, status: 'sending' };
    eventLog.emit('info', 'NAV', `Goal sent: (${x.toFixed(2)}, ${y.toFixed(2)})`);
    updateState();
    return { success: true, message: 'Goal sent' };
  }

  function cancelNavGoal(): void {
    if (navGoalHandle !== null) {
      navGoalHandle.cancelGoal().catch(() => { /* ignore */ });
      navState.status = 'canceling';
    }
    missionCancel = true;
  }

  // -------------------------------------------------------------------------
  // Service Clients (P1.2)
  // -------------------------------------------------------------------------
  const startRecClient = node.createClient('std_srvs/srv/Trigger', '/session/start_recording');
  const stopRecClient = node.createClient('std_srvs/srv/Trigger', '/session/stop_recording');
  const loadMapClient = node.createClient('nav2_msgs/srv/LoadMap', `/${NAMESPACE}/map_server/load_map`);

  async function callTriggerService(client: any, name: string): Promise<{ success: boolean; message: string }> {
    if (!client.isServiceServerAvailable()) {
      return { success: false, message: `${name} service not available` };
    }
    try {
      const response = await client.sendRequestAsync({}, { timeout: 5000 });
      return { success: response.success, message: response.message || '' };
    } catch (e: any) {
      return { success: false, message: e?.message || 'Service call failed' };
    }
  }

  async function loadMapFile(yamlPath: string): Promise<{ success: boolean; message: string }> {
    if (!loadMapClient.isServiceServerAvailable()) {
      return { success: false, message: 'map_server/load_map service not available' };
    }
    try {
      const response = await loadMapClient.sendRequestAsync({ map_url: yamlPath }, { timeout: 10000 });
      return { success: response.result === 0, message: `result=${response.result}` };
    } catch (e: any) {
      return { success: false, message: e?.message || 'LoadMap timed out' };
    }
  }

  async function saveMap(name: string): Promise<{ success: boolean; message: string; name?: string }> {
    if (!name || name.includes('/') || name.includes('..')) {
      return { success: false, message: 'Invalid map name' };
    }
    const outPath = path.join(MAPS_DIR, name);
    return new Promise((resolve) => {
      execFile('ros2', [
        'run', 'nav2_map_server', 'map_saver_cli',
        '-f', outPath, '-t', `/${NAMESPACE}/map`,
        '--ros-args', '-p', 'save_map_timeout:=10.0',
      ], { timeout: 15000 }, (error, stdout, stderr) => {
        if (error) {
          eventLog.emit('warn', 'MAPPING', `Map save failed: ${name}`);
          resolve({ success: false, message: stderr?.trim() || 'map_saver failed' });
        } else {
          eventLog.emit('info', 'MAPPING', `Map "${name}" saved`);
          resolve({ success: true, message: 'Map saved', name });
        }
      });
    });
  }

  // -------------------------------------------------------------------------
  // Mission Execution (P1.3)
  // -------------------------------------------------------------------------

  async function executeMission(missionId: string): Promise<{ success: boolean; message?: string; nodes?: number }> {
    const missions = readMissions();
    let mission = missions.find((m: any) => m.id === missionId);
    if (!mission) return { success: false, message: 'Mission not found' };
    mission = normalizeMission(mission);

    if (currentMode !== 'nav') return { success: false, message: 'Not in nav mode' };
    const nodes = mission.nodes || [];
    if (nodes.length === 0) return { success: false, message: 'Mission has no nodes' };

    const edges: Record<string, any> = {};
    for (const e of (mission.edges || [])) {
      edges[`${e.from}->${e.to}`] = e;
    }

    missionCancel = false;
    missionPause = false;
    const runId = `run_${Date.now()}`;
    missionProgress = {
      mission_id: missionId,
      mission_name: mission.name || '',
      current_node: 0,
      total_nodes: nodes.length,
      status: 'running',
    };
    eventLog.emit('info', 'MISSION', `Mission "${mission.name}" started (${nodes.length} nodes)`);
    telemetryStore.startMissionRun(runId, missionId, mission.name || '', nodes.length);
    updateState();

    // Run asynchronously
    (async () => {
      let completed = true;
      for (let i = 0; i < nodes.length; i++) {
        if (missionCancel) {
          missionProgress!.status = 'canceled';
          eventLog.emit('info', 'MISSION', 'Mission canceled');
          completed = false;
          break;
        }

        // Pause support
        while (missionPause) {
          await new Promise(r => setTimeout(r, 500));
          if (missionCancel) break;
        }
        if (missionCancel) {
          missionProgress!.status = 'canceled';
          eventLog.emit('info', 'MISSION', 'Mission canceled');
          completed = false;
          break;
        }

        missionProgress!.current_node = i;
        missionProgress!.status = 'running';

        const nd = nodes[i];
        sendNavGoal(
          parseFloat(nd.x || 0),
          parseFloat(nd.y || 0),
          parseFloat(nd.theta || 0)
        );

        // Wait for nav completion
        while (navState.active) {
          await new Promise(r => setTimeout(r, 500));
          if (missionCancel) {
            cancelNavGoal();
            break;
          }
        }

        if (navState.status !== 'succeeded') {
          missionProgress!.status = 'failed';
          eventLog.emit('warn', 'MISSION', `Mission failed at node ${i}: ${navState.status}`);
          completed = false;
          break;
        }

        // Node action
        const action = nd.action || 'none';
        if (action === 'pause') {
          const pauseSec = parseFloat(nd.pause_sec || 3);
          eventLog.emit('info', 'MISSION', `Pausing ${pauseSec}s at node ${i}`);
          await new Promise(r => setTimeout(r, pauseSec * 1000));
        } else if (action === 'signal') {
          eventLog.emit('info', 'MISSION', `Signal at node ${i}`);
        }
      }

      if (completed) {
        missionProgress!.status = 'completed';
        eventLog.emit('info', 'MISSION', `Mission "${mission.name}" completed`);
        telemetryStore.endMissionRun(runId, 'completed', nodes.length);
      } else {
        telemetryStore.endMissionRun(runId, missionProgress!.status, missionProgress!.current_node);
      }
      updateState();
    })().catch((err) => {
      eventLog.emit('warn', 'MISSION', `Mission error: ${err?.message || err}`);
      if (missionProgress) missionProgress.status = 'failed';
      telemetryStore.endMissionRun(runId, 'failed', missionProgress?.current_node || 0);
      updateState();
    });

    return { success: true, nodes: nodes.length };
  }

  // -------------------------------------------------------------------------
  // Subscribers
  // -------------------------------------------------------------------------

  node.createSubscription('nav_msgs/msg/Odometry', `/${NAMESPACE}/wheel_odom`, (msg: any) => {
    odomTimes.push(Date.now() / 1000);
    if (odomTimes.length > 50) odomTimes.shift();
    latestVelocity = {
      linear: Math.round(msg.twist.twist.linear.x * 1000) / 1000,
      angular: Math.round(msg.twist.twist.angular.z * 1000) / 1000,
    };
  });

  node.createSubscription('std_msgs/msg/String', `/${NAMESPACE}/motor_state`, (msg: any) => {
    try {
      const prev = motorState.armed;
      motorState = JSON.parse(msg.data);
      if (prev !== motorState.armed) {
        eventLog.emit('info', 'DRIVE', motorState.armed ? 'Motors armed' : 'Motors disarmed');
        updateState();
      }
    } catch { /* ignore */ }
  });

  node.createSubscription('std_msgs/msg/String', '/slam/quality', (msg: any) => {
    try {
      const data = JSON.parse(msg.data);
      slamTracking = data?.tracking?.confidence || 'unknown';
    } catch { /* ignore */ }
  });

  node.createSubscription('nav_msgs/msg/Odometry', `/${NAMESPACE}/odometry/global`, (msg: any) => {
    const p = msg.pose.pose;
    robotPose = {
      x: Math.round(p.position.x * 10000) / 10000,
      y: Math.round(p.position.y * 10000) / 10000,
      theta: Math.round(yawFromQuat(p.orientation) * 10000) / 10000,
    };
  });

  // Path subscription (P1.4 fix)
  node.createSubscription('nav_msgs/msg/Path', `/${NAMESPACE}/plan`, (msg: any) => {
    navPathPoints = (msg.poses || []).map((ps: any) => ({
      x: Math.round(ps.pose.position.x * 1000) / 1000,
      y: Math.round(ps.pose.position.y * 1000) / 1000,
    }));
    navPathChanged = true;
  });

  node.createSubscription('sensor_msgs/msg/LaserScan', `/${NAMESPACE}/scan`, (msg: any) => {
    scanPoints = scanAccumulator.addScan(
      robotPose.x, robotPose.y, robotPose.theta,
      Array.from(msg.ranges), msg.angle_min, msg.angle_increment,
      msg.range_min, msg.range_max,
    );
  });

  // OccupancyGrid subscription (P1.4)
  const transientLocalQos = new rclnodejs.QoS(
    rclnodejs.QoS.HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1,
    rclnodejs.QoS.ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    rclnodejs.QoS.DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
  );

  node.createSubscription('nav_msgs/msg/OccupancyGrid', `/${NAMESPACE}/map`,
    { qos: transientLocalQos },
    (msg: any) => {
      const w = msg.info.width;
      const h = msg.info.height;
      const data: Int8Array | number[] = msg.data;

      // Convert OccupancyGrid to grayscale PNG
      const pixels = Buffer.alloc(w * h);
      for (let i = 0; i < w * h; i++) {
        const v = typeof data[i] === 'number' ? data[i] : 0;
        if (v === -1 || v === 255) pixels[i] = 205;  // unknown
        else if (v === 0) pixels[i] = 254;            // free
        else if (v === 100) pixels[i] = 0;            // occupied
        else pixels[i] = Math.max(0, Math.min(255, 255 - Math.round(v * 2.55)));
      }

      // Flip vertically (ROS maps have origin at bottom-left)
      const flipped = Buffer.alloc(w * h);
      for (let row = 0; row < h; row++) {
        pixels.copy(flipped, (h - 1 - row) * w, row * w, (row + 1) * w);
      }

      const sharp = require('sharp');
      sharp(flipped, { raw: { width: w, height: h, channels: 1 } })
        .png()
        .toBuffer()
        .then((buf: Buffer) => {
          mapPng = buf;
          mapMeta = {
            resolution: msg.info.resolution,
            origin_x: msg.info.origin.position.x,
            origin_y: msg.info.origin.position.y,
            width: w,
            height: h,
          };
          mapChanged = true;
        })
        .catch(() => { /* sharp error */ });
    });

  // Health update timer
  setInterval(() => {
    const now = Date.now() / 1000;
    const hz = calcHz(odomTimes);
    health.drive = hz > 10
      ? { status: 'ok', detail: `${hz} Hz`, updated: now }
      : hz > 1
        ? { status: 'warn', detail: `${hz} Hz (low)`, updated: now }
        : { status: 'error', detail: 'No odom', updated: now, action: 'Check CAN connection' };

    health.slam = slamTracking === 'good'
      ? { status: 'ok', detail: 'Tracking: good', updated: now }
      : { status: 'warn', detail: `Tracking: ${slamTracking}`, updated: now };

    // Nav2 health
    const navReady = navActionClient.isActionServerAvailable();
    health.nav = navReady
      ? { status: 'ok', detail: navState.active ? `Active (${navState.distance_remaining}m)` : 'Ready', updated: now }
      : { status: 'warn', detail: 'Nav2 server not ready', updated: now, action: 'Check Nav2 lifecycle' };

    health.network = { status: 'ok', detail: `${activeClients} client(s)`, updated: now };
    updateState();
  }, 1000);

  // Scan accumulator PNG update
  setInterval(() => scanAccumulator.updatePng(), 2000);

  // Telemetry recording — 1Hz sample to SQLite (P2.2)
  setInterval(() => {
    try {
      telemetryStore.recordSample({
        timestamp: Date.now() / 1000,
        pose_x: robotPose.x,
        pose_y: robotPose.y,
        pose_theta: robotPose.theta,
        linear_vel: latestVelocity.linear,
        angular_vel: latestVelocity.angular,
        odom_hz: calcHz(odomTimes),
        slam_confidence: slamTracking,
        robot_state: robotState,
        battery_pct: -1, // No BMS hardware yet
      });
    } catch { /* ignore write errors */ }
  }, 1000);

  // Daily telemetry prune
  setInterval(() => {
    try { telemetryStore.prune(); } catch { /* ignore */ }
  }, 86400_000);

  // Cmd vel helper
  function sendCmdVel(linear: number, angular: number) {
    if (eStopActive) return;
    if (currentMode !== 'teleop' && currentMode !== 'mapping') return;
    const msg = rclnodejs.createMessageObject('geometry_msgs/msg/Twist') as any;
    msg.linear.x = Math.max(-0.5, Math.min(0.5, linear));
    msg.angular.z = Math.max(-1.0, Math.min(1.0, angular));
    cmdVelPub.publish(msg);
    lastCmdTime = Date.now() / 1000;
  }

  function sendZero() {
    const msg = rclnodejs.createMessageObject('geometry_msgs/msg/Twist') as any;
    cmdVelPub.publish(msg);
  }

  // Watchdog
  setInterval(() => {
    if (activeClients > 0 && !eStopActive && lastCmdTime > 0) {
      if (Date.now() / 1000 - lastCmdTime > 0.5) sendZero();
    }
  }, 100);

  // ---------------------------------------------------------------------------
  // Express + REST
  // ---------------------------------------------------------------------------

  const app = express();
  app.use(express.json());

  // Serve dashboard
  const dashboardDir = path.resolve(__dirname, '../../web/agv_dashboard/dist');
  if (fs.existsSync(dashboardDir)) {
    app.use('/dashboard', express.static(dashboardDir));
  }

  // Serve legacy teleop
  const staticDir = path.resolve(__dirname, '../static');
  if (fs.existsSync(staticDir)) {
    app.get('/', (_req, res) => res.sendFile(path.join(staticDir, 'index.html')));
  }

  // Auth endpoints (P2.6)
  app.get('/api/auth/status', (_req, res) => {
    res.json({ enabled: authManager.enabled });
  });

  app.post('/api/auth/login', (req, res) => {
    const { username, password } = req.body || {};
    if (!username || !password) return res.status(400).json({ error: 'Missing credentials' });
    const result = authManager.login(username, password);
    if (!result) return res.status(401).json({ error: 'Invalid credentials' });
    res.json(result);
  });

  app.get('/api/auth/me', authManager.requireAuth(), (req, res) => {
    res.json((req as any).user);
  });

  app.get('/api/auth/users', authManager.requireAuth('engineer'), (_req, res) => {
    res.json(authManager.listUsers());
  });

  app.post('/api/auth/users', authManager.requireAuth('engineer'), (req, res) => {
    const { username, password, role } = req.body || {};
    if (!username || !password || !role) return res.status(400).json({ error: 'Missing fields' });
    if (authManager.addUser(username, password, role)) {
      res.json({ success: true });
    } else {
      res.status(409).json({ error: 'User already exists' });
    }
  });

  app.delete('/api/auth/users/:username', authManager.requireAuth('engineer'), (req, res) => {
    if (authManager.removeUser(String(req.params.username))) {
      res.json({ success: true });
    } else {
      res.status(404).json({ error: 'User not found' });
    }
  });

  app.get('/api/status', (_req, res) => res.json(getStatus()));
  app.get('/api/health', (_req, res) => res.json(health));
  app.get('/api/mode', (_req, res) => res.json({ mode: currentMode }));

  app.put('/api/mode', (req, res) => {
    const mode = req.body?.mode;
    if (!['teleop', 'mapping', 'nav'].includes(mode)) {
      return res.status(400).json({ success: false, message: `Invalid mode: ${mode}` });
    }
    if (mode !== currentMode) {
      if (mode !== 'nav' && navState.active) cancelNavGoal();
      eventLog.emit('info', 'SYSTEM', `Mode: ${currentMode} → ${mode}`);
      if (mode !== 'teleop') sendZero();
      currentMode = mode;
      updateState();
    }
    res.json({ success: true, mode: currentMode });
  });

  // Events
  app.get('/api/events', (req, res) => {
    const limit = parseInt(req.query.limit as string) || 100;
    const offset = parseInt(req.query.offset as string) || 0;
    res.json(eventLog.getEntries(limit, offset));
  });
  app.delete('/api/events', (_req, res) => { eventLog.clear(); res.json({ success: true }); });

  // Recording (P1.2)
  app.post('/api/recording/start', async (_req, res) => {
    const result = await callTriggerService(startRecClient, 'start_recording');
    if (result.success) recordingActive = true;
    res.json(result);
  });
  app.post('/api/recording/stop', async (_req, res) => {
    const result = await callTriggerService(stopRecClient, 'stop_recording');
    if (result.success) recordingActive = false;
    res.json(result);
  });

  // Maps (P1.2)
  app.get('/api/maps', (_req, res) => {
    res.json(listMapFiles());
  });

  app.get('/api/maps/:name/image', (req, res) => {
    const name = req.params.name;
    const yamlPath = path.join(MAPS_DIR, `${name}.yaml`);
    if (!fs.existsSync(yamlPath)) return res.status(404).json({ error: 'Map not found' });

    let pgmName: string | null = null;
    for (const line of fs.readFileSync(yamlPath, 'utf-8').split('\n')) {
      if (line.trim().startsWith('image:')) {
        pgmName = line.split(':').slice(1).join(':').trim();
        break;
      }
    }
    if (!pgmName) return res.status(400).json({ error: 'No image in map YAML' });

    const pgmPath = path.join(MAPS_DIR, pgmName);
    if (!fs.existsSync(pgmPath)) return res.status(404).json({ error: `PGM not found: ${pgmName}` });

    const sharp = require('sharp');
    sharp(pgmPath).png().toBuffer()
      .then((buf: Buffer) => res.type('image/png').send(buf))
      .catch((err: any) => res.status(500).json({ error: err?.message || 'Conversion failed' }));
  });

  app.post('/api/maps/save', async (req, res) => {
    const name = (req.body?.name || '').trim();
    const result = await saveMap(name);
    if (result.success) res.json(result);
    else res.status(500).json(result);
  });

  app.post('/api/maps/load', async (req, res) => {
    const name = (req.body?.name || '').trim();
    const yamlPath = path.join(MAPS_DIR, `${name}.yaml`);
    if (!fs.existsSync(yamlPath)) return res.status(404).json({ error: 'Map not found' });
    const result = await loadMapFile(yamlPath);
    if (result.success) eventLog.emit('info', 'MAPPING', `Map "${name}" loaded`);
    res.json(result);
  });

  // Missions
  app.get('/api/missions', (_req, res) => {
    res.json(readMissions().map(normalizeMission));
  });

  app.post('/api/missions', (req, res) => {
    try {
      const missions = readMissions();
      let nodes = req.body.nodes || [];
      if (!nodes.length && req.body.waypoints) {
        nodes = req.body.waypoints.map((wp: any, i: number) => ({
          id: `n${i}`, type: 'waypoint', action: 'none', ...wp,
        }));
      }
      const mission = {
        id: req.body.id || `m${Date.now() % 100000000}`,
        name: req.body.name || 'Untitled',
        nodes,
        edges: req.body.edges || [],
        repeat: req.body.repeat || false,
        waypoints: req.body.waypoints || [],
        created: Date.now() / 1000,
      };
      missions.push(mission);
      writeMissions(missions);
      eventLog.emit('info', 'MISSION', `Mission "${mission.name}" created (${nodes.length} nodes)`);
      res.json(mission);
    } catch (e) { res.status(500).json({ error: String(e) }); }
  });

  app.delete('/api/missions/:id', (req, res) => {
    try {
      let missions = readMissions();
      const name = missions.find((m: any) => m.id === req.params.id)?.name || '?';
      missions = missions.filter((m: any) => m.id !== req.params.id);
      writeMissions(missions);
      eventLog.emit('info', 'MISSION', `Mission "${name}" deleted`);
      res.json({ success: true });
    } catch (e) { res.status(500).json({ error: String(e) }); }
  });

  // Mission execution (P1.3)
  app.post('/api/missions/:id/execute', async (req, res) => {
    const result = await executeMission(req.params.id);
    if (result.success) res.json(result);
    else res.status(400).json(result);
  });

  app.post('/api/missions/pause', (_req, res) => {
    missionPause = true;
    eventLog.emit('info', 'MISSION', 'Mission paused');
    res.json({ success: true });
  });

  app.post('/api/missions/resume', (_req, res) => {
    missionPause = false;
    eventLog.emit('info', 'MISSION', 'Mission resumed');
    res.json({ success: true });
  });

  // Nav
  app.post('/api/nav/goal', (req, res) => {
    const result = sendNavGoal(
      parseFloat(req.body?.x || 0),
      parseFloat(req.body?.y || 0),
      parseFloat(req.body?.theta || 0)
    );
    if (result.success) res.json(result);
    else res.status(400).json(result);
  });

  app.post('/api/nav/cancel', (_req, res) => {
    cancelNavGoal();
    res.json({ success: true });
  });

  // MJPEG camera stream (P1.7)
  const CAMERA_TOPIC = process.env.AGV_CAMERA_TOPIC || `/${NAMESPACE}/zed/left/image_rect_color/compressed`;
  let cameraJpeg: Buffer | null = null;
  let cameraClients: Set<http.ServerResponse> = new Set();

  // Subscribe to compressed image topic only when there are camera clients
  let cameraSubCreated = false;
  function ensureCameraSub() {
    if (cameraSubCreated) return;
    cameraSubCreated = true;
    try {
      node.createSubscription('sensor_msgs/msg/CompressedImage', CAMERA_TOPIC, (msg: any) => {
        cameraJpeg = Buffer.from(msg.data);
        // Push to all MJPEG clients
        for (const client of cameraClients) {
          try {
            client.write(`--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ${cameraJpeg.length}\r\n\r\n`);
            client.write(cameraJpeg);
            client.write('\r\n');
          } catch {
            cameraClients.delete(client);
          }
        }
      });
    } catch { /* topic may not exist */ }
  }

  app.get('/api/camera/stream', (req, res) => {
    ensureCameraSub();
    res.writeHead(200, {
      'Content-Type': 'multipart/x-mixed-replace; boundary=frame',
      'Cache-Control': 'no-cache',
      'Connection': 'keep-alive',
    });
    cameraClients.add(res);
    req.on('close', () => cameraClients.delete(res));
  });

  app.get('/api/camera/snapshot', (_req, res) => {
    ensureCameraSub();
    if (cameraJpeg) {
      res.type('image/jpeg').send(cameraJpeg);
    } else {
      res.status(404).json({ error: 'No camera frame available' });
    }
  });

  // Accumulated map
  app.get('/api/acc_map/image', (_req, res) => {
    if (scanAccumulator.pngBuffer) {
      res.type('image/png').send(scanAccumulator.pngBuffer);
    } else {
      res.status(404).json({ error: 'No accumulated map' });
    }
  });
  app.delete('/api/acc_map', (_req, res) => {
    scanAccumulator.clear();
    eventLog.emit('info', 'MAPPING', 'Accumulated map cleared');
    res.json({ success: true });
  });

  // Analytics (P2.3)
  app.get('/api/analytics/summary', (req, res) => {
    const period = req.query.period as string || '24h';
    let seconds = 86400; // default 24h
    if (period.endsWith('h')) seconds = parseInt(period) * 3600;
    else if (period.endsWith('d')) seconds = parseInt(period) * 86400;
    else if (period.endsWith('m')) seconds = parseInt(period) * 60;
    try {
      res.json(telemetryStore.getSummary(seconds));
    } catch (e) { res.status(500).json({ error: String(e) }); }
  });

  app.get('/api/analytics/timeseries', (req, res) => {
    const metric = (req.query.metric as string) || 'odom_hz';
    const validMetrics = ['odom_hz', 'linear_vel', 'pose_x', 'pose_y', 'slam_confidence'];
    if (!validMetrics.includes(metric)) {
      return res.status(400).json({ error: `Invalid metric. Valid: ${validMetrics.join(', ')}` });
    }
    const now = Date.now() / 1000;
    const from = parseFloat(req.query.from as string) || (now - 86400);
    const to = parseFloat(req.query.to as string) || now;
    const resolution = parseInt(req.query.resolution as string) || 60;
    try {
      res.json(telemetryStore.getTimeseries(metric as any, from, to, resolution));
    } catch (e) { res.status(500).json({ error: String(e) }); }
  });

  app.get('/api/analytics/missions', (req, res) => {
    const now = Date.now() / 1000;
    const from = parseFloat(req.query.from as string) || (now - 86400 * 7);
    const to = parseFloat(req.query.to as string) || now;
    try {
      res.json(telemetryStore.getMissionRuns(from, to));
    } catch (e) { res.status(500).json({ error: String(e) }); }
  });

  // Replay data (P2.4)
  app.get('/api/replay/samples', (req, res) => {
    const from = parseFloat(req.query.from as string) || 0;
    const to = parseFloat(req.query.to as string) || (Date.now() / 1000);
    try {
      res.json(telemetryStore.getReplaySamples(from, to));
    } catch (e) { res.status(500).json({ error: String(e) }); }
  });

  app.get('/api/replay/events', (req, res) => {
    const from = parseFloat(req.query.from as string) || 0;
    const to = parseFloat(req.query.to as string) || (Date.now() / 1000);
    const limit = parseInt(req.query.limit as string) || 500;
    try {
      res.json(telemetryStore.getEvents(from, to, limit));
    } catch (e) { res.status(500).json({ error: String(e) }); }
  });

  // ---------------------------------------------------------------------------
  // WebSocket
  // ---------------------------------------------------------------------------

  const server = http.createServer(app);
  const wss = new WebSocketServer({ server, path: '/ws/control' });

  wss.on('connection', (ws: WebSocket) => {
    activeClients++;
    console.log(`Dashboard client connected (${activeClients})`);

    let lastPathSnapshot: string = '';

    const statusInterval = setInterval(() => {
      if (ws.readyState !== WebSocket.OPEN) return;
      try {
        ws.send(JSON.stringify({ type: 'status', ...getStatus() }));

        if (scanPoints.length > 0) {
          ws.send(JSON.stringify({ type: 'scan', points: scanPoints }));
        }

        // Path updates (P1.4)
        if (navPathChanged) {
          const pathStr = JSON.stringify(navPathPoints);
          if (pathStr !== lastPathSnapshot) {
            lastPathSnapshot = pathStr;
            ws.send(JSON.stringify({ type: 'path', points: navPathPoints }));
          }
        }

        // OccupancyGrid map updates (P1.4)
        if (mapChanged && mapPng) {
          mapChanged = false;
          ws.send(JSON.stringify({
            type: 'map_update',
            png_base64: mapPng.toString('base64'),
            ...mapMeta,
          }));
        }

        if (scanAccumulator.pngBuffer && scanAccumulator.changed) {
          ws.send(JSON.stringify({
            type: 'acc_map',
            png_base64: scanAccumulator.pngBuffer.toString('base64'),
            ...scanAccumulator.meta,
          }));
        }

        for (const evt of eventLog.popPending()) {
          ws.send(JSON.stringify({ type: 'event', ...evt }));
        }
      } catch { /* client disconnected */ }
    }, 200); // 5Hz

    ws.on('message', (data: Buffer) => {
      try {
        const msg = JSON.parse(data.toString());
        switch (msg.type) {
          case 'cmd_vel':
            sendCmdVel(msg.linear || 0, msg.angular || 0);
            break;
          case 'e_stop': {
            eStopActive = !!msg.active;
            const eMsg = rclnodejs.createMessageObject('std_msgs/msg/Bool') as any;
            eMsg.data = eStopActive;
            eStopPub.publish(eMsg);
            if (eStopActive) {
              sendZero();
              cancelNavGoal();
            }
            eventLog.emit(eStopActive ? 'crit' : 'info', 'SAFETY',
              eStopActive ? 'E-STOP ACTIVATED' : 'E-stop cleared');
            updateState();
            break;
          }
          case 'motor_enable': {
            const mMsg = rclnodejs.createMessageObject('std_msgs/msg/Bool') as any;
            mMsg.data = !!msg.active;
            motorEnablePub.publish(mMsg);
            break;
          }
          case 'mode':
            if (['teleop', 'mapping', 'nav'].includes(msg.mode)) {
              if (msg.mode !== currentMode) {
                if (msg.mode !== 'nav' && navState.active) cancelNavGoal();
                eventLog.emit('info', 'SYSTEM', `Mode: ${currentMode} → ${msg.mode}`);
                if (msg.mode !== 'teleop') sendZero();
                currentMode = msg.mode;
                updateState();
              }
            }
            break;
          case 'nav_goal':
            if (currentMode === 'nav') {
              sendNavGoal(
                parseFloat(msg.x || 0),
                parseFloat(msg.y || 0),
                parseFloat(msg.theta || 0)
              );
            }
            break;
          case 'nav_cancel':
            cancelNavGoal();
            break;
          case 'recording': {
            const action = msg.action;
            (async () => {
              let result;
              if (action === 'start') {
                result = await callTriggerService(startRecClient, 'start_recording');
                if (result.success) recordingActive = true;
              } else if (action === 'stop') {
                result = await callTriggerService(stopRecClient, 'stop_recording');
                if (result.success) recordingActive = false;
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
      activeClients = Math.max(0, activeClients - 1);
      sendZero();
    });
  });

  // Legacy teleop WS
  const wssTeleop = new WebSocketServer({ server, path: '/ws/teleop' });
  wssTeleop.on('connection', (ws: WebSocket) => {
    activeClients++;
    ws.on('message', (data: Buffer) => {
      try {
        const msg = JSON.parse(data.toString());
        if (msg.type === 'cmd_vel') sendCmdVel(msg.linear || 0, msg.angular || 0);
        else if (msg.type === 'e_stop') {
          eStopActive = !!msg.active;
          const eMsg = rclnodejs.createMessageObject('std_msgs/msg/Bool') as any;
          eMsg.data = eStopActive;
          eStopPub.publish(eMsg);
        }
      } catch { /* ignore */ }
    });
    ws.on('close', () => { activeClients = Math.max(0, activeClients - 1); sendZero(); });
  });

  // Start
  rclnodejs.spin(node);
  server.listen(PORT, '0.0.0.0', () => {
    console.log(`AGV Operator Backend (TypeScript) listening on http://0.0.0.0:${PORT}`);
    eventLog.emit('info', 'SYSTEM', 'Operator backend started (TypeScript)');
  });
}

main().catch(console.error);
