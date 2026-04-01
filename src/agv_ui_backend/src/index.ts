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
import express from 'express';
import { WebSocketServer, WebSocket } from 'ws';
import * as rclnodejs from 'rclnodejs';

import { deriveState, allowedActions, RobotState, MotorState, NavState, MissionProgress } from './state_machine';
import { EventLog, LogEntry } from './event_log';
import { ScanAccumulator } from './scan_accumulator';

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

const PORT = parseInt(process.env.AGV_PORT || '8090');
const DATA_DIR = process.env.AGV_DATA_DIR || '/tmp/agv_data';
const NAMESPACE = process.env.AGV_NAMESPACE || 'agv';

fs.mkdirSync(DATA_DIR, { recursive: true });
fs.mkdirSync(path.join(DATA_DIR, 'missions'), { recursive: true });

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
let odomTimes: number[] = [];
let activeClients = 0;
let lastCmdTime = 0;
let mapPng: Buffer | null = null;
let mapMeta: any = null;
let mapChanged = false;

const health: Record<string, any> = {
  drive: { status: 'unknown', detail: 'waiting', updated: 0 },
  imu: { status: 'unknown', detail: 'waiting', updated: 0 },
  slam: { status: 'unknown', detail: 'waiting', updated: 0 },
  nav: { status: 'unknown', detail: 'waiting', updated: 0 },
  network: { status: 'ok', detail: '', updated: Date.now() / 1000 },
};

const eventLog = new EventLog(DATA_DIR);
const scanAccumulator = new ScanAccumulator();

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
    recording: false,
    clients: activeClients,
    mode: currentMode,
    pose: robotPose,
    nav_state: navState,
    health,
    mission_progress: missionProgress,
  };
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

  // Subscribers
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

  node.createSubscription('nav_msgs/msg/Path', `/${NAMESPACE}/plan`, (msg: any) => {
    // path points stored for WS push
  });

  node.createSubscription('sensor_msgs/msg/LaserScan', `/${NAMESPACE}/scan`, (msg: any) => {
    scanPoints = scanAccumulator.addScan(
      robotPose.x, robotPose.y, robotPose.theta,
      Array.from(msg.ranges), msg.angle_min, msg.angle_increment,
      msg.range_min, msg.range_max,
    );
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

    health.network = { status: 'ok', detail: `${activeClients} client(s)`, updated: now };
    updateState();
  }, 1000);

  // Scan accumulator PNG update
  setInterval(() => scanAccumulator.updatePng(), 2000);

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

  app.get('/api/status', (_req, res) => res.json(getStatus()));
  app.get('/api/health', (_req, res) => res.json(health));
  app.get('/api/mode', (_req, res) => res.json({ mode: currentMode }));

  app.put('/api/mode', (req, res) => {
    const mode = req.body?.mode;
    if (!['teleop', 'mapping', 'nav'].includes(mode)) {
      return res.status(400).json({ success: false, message: `Invalid mode: ${mode}` });
    }
    if (mode !== currentMode) {
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

  // Maps
  app.get('/api/maps', (_req, res) => {
    // List .yaml files in map dir
    // This would need a configurable map_dir — for now use agv_navigation maps
    res.json([]); // TODO: implement with configurable map directory
  });

  // Missions
  app.get('/api/missions', (_req, res) => {
    try {
      res.json(JSON.parse(fs.readFileSync(MISSIONS_FILE, 'utf-8')));
    } catch { res.json([]); }
  });

  app.post('/api/missions', (req, res) => {
    try {
      const missions = JSON.parse(fs.readFileSync(MISSIONS_FILE, 'utf-8'));
      const mission = {
        id: req.body.id || `m${Date.now() % 100000000}`,
        name: req.body.name || 'Untitled',
        waypoints: req.body.waypoints || [],
        nodes: req.body.nodes || [],
        created: Date.now() / 1000,
      };
      missions.push(mission);
      fs.writeFileSync(MISSIONS_FILE, JSON.stringify(missions, null, 2));
      eventLog.emit('info', 'MISSION', `Mission "${mission.name}" created`);
      res.json(mission);
    } catch (e) { res.status(500).json({ error: String(e) }); }
  });

  app.delete('/api/missions/:id', (req, res) => {
    try {
      let missions = JSON.parse(fs.readFileSync(MISSIONS_FILE, 'utf-8'));
      missions = missions.filter((m: any) => m.id !== req.params.id);
      fs.writeFileSync(MISSIONS_FILE, JSON.stringify(missions, null, 2));
      res.json({ success: true });
    } catch (e) { res.status(500).json({ error: String(e) }); }
  });

  // Nav
  app.post('/api/nav/goal', (req, res) => {
    if (currentMode !== 'nav') return res.status(400).json({ success: false, message: 'Not in nav mode' });
    eventLog.emit('info', 'NAV', `Goal sent: (${req.body.x?.toFixed(2)}, ${req.body.y?.toFixed(2)})`);
    navState = { active: true, distance_remaining: 0, status: 'sending' };
    updateState();
    res.json({ success: true, message: 'Goal sent' });
    // TODO: actual NavigateToPose action client
  });

  app.post('/api/nav/cancel', (_req, res) => {
    navState = { active: false, distance_remaining: 0, status: 'canceled' };
    updateState();
    res.json({ success: true });
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

  // ---------------------------------------------------------------------------
  // WebSocket
  // ---------------------------------------------------------------------------

  const server = http.createServer(app);
  const wss = new WebSocketServer({ server, path: '/ws/control' });

  wss.on('connection', (ws: WebSocket) => {
    activeClients++;
    console.log(`Dashboard client connected (${activeClients})`);

    const statusInterval = setInterval(() => {
      if (ws.readyState !== WebSocket.OPEN) return;
      try {
        ws.send(JSON.stringify({ type: 'status', ...getStatus() }));

        if (scanPoints.length > 0) {
          ws.send(JSON.stringify({ type: 'scan', points: scanPoints }));
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
            if (eStopActive) sendZero();
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
                eventLog.emit('info', 'SYSTEM', `Mode: ${currentMode} → ${msg.mode}`);
                if (msg.mode !== 'teleop') sendZero();
                currentMode = msg.mode;
                updateState();
              }
            }
            break;
          case 'nav_goal':
            if (currentMode === 'nav') {
              eventLog.emit('info', 'NAV', `Goal: (${msg.x?.toFixed(2)}, ${msg.y?.toFixed(2)})`);
              navState = { active: true, distance_remaining: 0, status: 'sending' };
              updateState();
            }
            break;
          case 'nav_cancel':
            navState = { active: false, distance_remaining: 0, status: 'canceled' };
            updateState();
            break;
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
