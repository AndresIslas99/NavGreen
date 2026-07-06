#!/usr/bin/env node
/**
 * AGV Fleet Manager — VDA 5050 Master Control
 *
 * Subscribes to all robots' VDA 5050 state/visualization/connection topics.
 * Provides REST API + WebSocket for the fleet dashboard.
 * Publishes orders and instant actions to individual robots.
 */

import * as crypto from 'crypto';
import * as http from 'http';
import express from 'express';
import { WebSocketServer, WebSocket } from 'ws';
import * as mqtt from 'mqtt';
import { TrafficManager } from './traffic';

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

// 8091 belongs to agv_image_server (specs/project.yaml default_image_server_port)
// — both services run on the same Jetson, so the fleet manager defaults to 8092.
const PORT = parseInt(process.env.FLEET_PORT || '8092');
const BIND_ADDR = process.env.FLEET_BIND_ADDR || '0.0.0.0';
const MQTT_BROKER = process.env.VDA_MQTT_BROKER || 'mqtt://localhost:1883';
// Broker credentials — required once the broker enables password_file/acl_file
// (see fleet/mosquitto/config/mosquitto.conf). The shipped ACL grants the
// username 'fleet-manager' fleet-wide access.
const MQTT_USERNAME = process.env.VDA_MQTT_USERNAME || (process.env.VDA_MQTT_PASSWORD ? 'fleet-manager' : '');
const MQTT_PASSWORD = process.env.VDA_MQTT_PASSWORD || '';
// Shared secret for the REST + WebSocket control surface. When unset the API
// is open — acceptable only on an isolated greenhouse LAN (see SECURITY.md).
const API_TOKEN = process.env.FLEET_API_TOKEN || '';
const VDA_VERSION = '2.0.0';

// ---------------------------------------------------------------------------
// Fleet state
// ---------------------------------------------------------------------------

interface RobotState {
  manufacturer: string;
  serialNumber: string;
  lastState: any;         // Full VDA 5050 state message
  lastViz: any;           // Last visualization message
  connectionState: string;
  lastSeen: number;       // timestamp
  position: { x: number; y: number; theta: number };
  driving: boolean;
  batteryCharge: number;
  operatingMode: string;
  errors: any[];
  orderId: string;
}

const fleet = new Map<string, RobotState>();
let headerId = 0;

// Traffic management (P3.4)
// VDA 5050 pause semantics: startPause ACTIVATES pause mode, stopPause
// DEACTIVATES it (the adapter used to have these swapped and aliased to the
// e-stop; keep master and adapter in lockstep).
const trafficManager = new TrafficManager(
  // onPause: activate pause mode
  (robotId: string) => {
    const robot = fleet.get(robotId);
    if (robot) {
      sendInstantAction(robot.manufacturer, robot.serialNumber, [
        { actionId: `traffic_pause_${Date.now()}`, actionType: 'startPause', blockingType: 'HARD' },
      ]);
    }
  },
  // onResume: deactivate pause mode
  (robotId: string) => {
    const robot = fleet.get(robotId);
    if (robot) {
      sendInstantAction(robot.manufacturer, robot.serialNumber, [
        { actionId: `traffic_resume_${Date.now()}`, actionType: 'stopPause', blockingType: 'HARD' },
      ]);
    }
  },
);

function robotKey(manufacturer: string, serial: string): string {
  return `${manufacturer}/${serial}`;
}

function nextHeader(manufacturer: string, serialNumber: string) {
  return {
    headerId: headerId++,
    timestamp: new Date().toISOString(),
    version: VDA_VERSION,
    manufacturer,
    serialNumber,
  };
}

// ---------------------------------------------------------------------------
// MQTT
// ---------------------------------------------------------------------------

const mqttClient = mqtt.connect(MQTT_BROKER, {
  clientId: 'agv-fleet-manager',
  clean: true,
  ...(MQTT_USERNAME ? { username: MQTT_USERNAME, password: MQTT_PASSWORD } : {}),
});

mqttClient.on('connect', () => {
  console.log(`Fleet manager connected to MQTT: ${MQTT_BROKER}`);

  // Subscribe to all robots' topics using wildcards
  mqttClient.subscribe('uagv/v2/+/+/state', { qos: 1 });
  mqttClient.subscribe('uagv/v2/+/+/visualization', { qos: 0 });
  mqttClient.subscribe('uagv/v2/+/+/connection', { qos: 1 });
});

mqttClient.on('message', (topic: string, payload: Buffer) => {
  try {
    const msg = JSON.parse(payload.toString());
    const parts = topic.split('/');
    // uagv/v2/{manufacturer}/{serialNumber}/{subtopic}
    if (parts.length < 5) return;
    const manufacturer = parts[2];
    const serial = parts[3];
    const subtopic = parts[4];
    const key = robotKey(manufacturer, serial);

    // Ensure robot entry exists
    if (!fleet.has(key)) {
      fleet.set(key, {
        manufacturer,
        serialNumber: serial,
        lastState: null,
        lastViz: null,
        connectionState: 'ONLINE',
        lastSeen: Date.now(),
        position: { x: 0, y: 0, theta: 0 },
        driving: false,
        batteryCharge: -1,
        operatingMode: 'AUTOMATIC',
        errors: [],
        orderId: '',
      });
      console.log(`New robot discovered: ${key}`);
      broadcastFleetUpdate();
    }

    const robot = fleet.get(key)!;
    robot.lastSeen = Date.now();

    switch (subtopic) {
      case 'state':
        robot.lastState = msg;
        robot.driving = msg.driving || false;
        robot.operatingMode = msg.operatingMode || 'AUTOMATIC';
        robot.errors = msg.errors || [];
        robot.orderId = msg.orderId || '';
        if (msg.agvPosition) {
          robot.position = {
            x: msg.agvPosition.x,
            y: msg.agvPosition.y,
            theta: msg.agvPosition.theta,
          };
        }
        if (msg.batteryState) {
          robot.batteryCharge = msg.batteryState.batteryCharge;
        }
        break;

      case 'visualization':
        robot.lastViz = msg;
        if (msg.agvPosition) {
          robot.position = {
            x: msg.agvPosition.x,
            y: msg.agvPosition.y,
            theta: msg.agvPosition.theta,
          };
          // Traffic zone check on every position update
          trafficManager.updateRobotPosition(key, robot.position.x, robot.position.y, robot.position.theta);
        }
        break;

      case 'connection':
        robot.connectionState = msg.connectionState || 'ONLINE';
        break;
    }
  } catch { /* ignore parse errors */ }
});

// Check for stale connections every 30s
setInterval(() => {
  const now = Date.now();
  for (const [key, robot] of fleet) {
    if (now - robot.lastSeen > 60000) {
      if (robot.connectionState !== 'CONNECTIONBROKEN') {
        robot.connectionState = 'CONNECTIONBROKEN';
        // Remove disconnected robot from traffic zones
        trafficManager.removeRobot(key);
      }
    }
  }
}, 30000);

// Deadlock detection every 10s
setInterval(() => {
  const deadlocked = trafficManager.detectDeadlocks();
  if (deadlocked.length > 0) {
    console.warn(`Deadlock detected: ${deadlocked.join(', ')}`);
    trafficManager.resolveDeadlock(deadlocked);
  }
}, 10000);

// ---------------------------------------------------------------------------
// Fleet order dispatch
// ---------------------------------------------------------------------------

function sendOrder(manufacturer: string, serial: string, order: any): boolean {
  const topic = `uagv/v2/${manufacturer}/${serial}/order`;
  const payload = JSON.stringify({
    ...nextHeader(manufacturer, serial),
    ...order,
  });
  mqttClient.publish(topic, payload, { qos: 1 });
  return true;
}

function sendInstantAction(manufacturer: string, serial: string, actions: any[]): boolean {
  const topic = `uagv/v2/${manufacturer}/${serial}/instantActions`;
  const payload = JSON.stringify({
    ...nextHeader(manufacturer, serial),
    instantActions: actions,
  });
  mqttClient.publish(topic, payload, { qos: 1 });
  return true;
}

function sendNavigateOrder(
  manufacturer: string, serial: string,
  orderId: string, x: number, y: number, theta: number = 0, mapId: string = 'default',
): boolean {
  // Use robot's current position as start node (not origin)
  const key = robotKey(manufacturer, serial);
  const robot = fleet.get(key);
  const startPos = robot?.position || { x: 0, y: 0, theta: 0 };

  return sendOrder(manufacturer, serial, {
    orderId,
    orderUpdateId: 0,
    nodes: [
      {
        nodeId: 'start',
        sequenceId: 0,
        released: true,
        nodePosition: { x: startPos.x, y: startPos.y, mapId },
        actions: [],
      },
      {
        nodeId: 'goal',
        sequenceId: 2,
        released: true,
        nodePosition: { x, y, theta, mapId, allowedDeviationXY: 0.15 },
        actions: [],
      },
    ],
    edges: [
      {
        edgeId: 'e0',
        sequenceId: 1,
        released: true,
        startNodeId: 'start',
        endNodeId: 'goal',
        actions: [],
      },
    ],
  });
}

// ---------------------------------------------------------------------------
// Express API
// ---------------------------------------------------------------------------

const app = express();
app.use(express.json());

// Shared-secret auth. Every /api route (fleet e-stop, navigate, traffic
// zones, ...) commands physical robots, so when FLEET_API_TOKEN is set all
// of them — and the WebSocket — require it.
function tokenMatches(provided: string): boolean {
  const a = Buffer.from(provided);
  const b = Buffer.from(API_TOKEN);
  return a.length === b.length && crypto.timingSafeEqual(a, b);
}

app.use('/api', (req, res, next) => {
  if (!API_TOKEN) return next();
  const auth = String(req.headers.authorization || '');
  const provided = auth.startsWith('Bearer ') ? auth.slice(7) : '';
  if (tokenMatches(provided)) return next();
  res.status(401).json({ error: 'unauthorized: send Authorization: Bearer <FLEET_API_TOKEN>' });
});

// Fleet overview
app.get('/api/fleet', (_req, res) => {
  const robots = Array.from(fleet.values()).map(r => ({
    id: robotKey(r.manufacturer, r.serialNumber),
    manufacturer: r.manufacturer,
    serialNumber: r.serialNumber,
    connectionState: r.connectionState,
    position: r.position,
    driving: r.driving,
    batteryCharge: r.batteryCharge,
    operatingMode: r.operatingMode,
    errorCount: r.errors.length,
    orderId: r.orderId,
    lastSeen: r.lastSeen,
  }));
  res.json(robots);
});

// Individual robot state
app.get('/api/fleet/:manufacturer/:serial/state', (req, res) => {
  const key = robotKey(String(req.params.manufacturer), String(req.params.serial));
  const robot = fleet.get(key);
  if (!robot) return res.status(404).json({ error: 'Robot not found' });
  res.json(robot.lastState || {});
});

// Send navigation order
app.post('/api/fleet/:manufacturer/:serial/navigate', (req, res) => {
  const { x, y, theta, mapId } = req.body;
  const orderId = `order_${Date.now()}`;
  sendNavigateOrder(
    String(req.params.manufacturer), String(req.params.serial),
    orderId, x, y, theta || 0, mapId || 'default'
  );
  res.json({ success: true, orderId });
});

// Send instant action (e-stop, cancel, etc.)
app.post('/api/fleet/:manufacturer/:serial/action', (req, res) => {
  const { actionType, actionId } = req.body;
  sendInstantAction(
    String(req.params.manufacturer), String(req.params.serial),
    [{ actionId: actionId || `ia_${Date.now()}`, actionType, blockingType: 'HARD' }]
  );
  res.json({ success: true });
});

// Fleet-wide e-stop. emergencyStop is a manufacturer-specific instant action
// that latches the robot's /agv/e_stop topic — a real stop, not a VDA 5050
// pause (see fleet/README.md).
app.post('/api/fleet/estop', (_req, res) => {
  for (const [, robot] of fleet) {
    sendInstantAction(robot.manufacturer, robot.serialNumber, [
      { actionId: `estop_${Date.now()}`, actionType: 'emergencyStop', blockingType: 'HARD' },
    ]);
  }
  res.json({ success: true, robotCount: fleet.size });
});

// Fleet-wide resume: release the latched e-stop, then deactivate pause mode
// so suspended orders continue.
app.post('/api/fleet/resume', (_req, res) => {
  for (const [, robot] of fleet) {
    sendInstantAction(robot.manufacturer, robot.serialNumber, [
      { actionId: `estop_clear_${Date.now()}`, actionType: 'clearEmergencyStop', blockingType: 'HARD' },
      { actionId: `resume_${Date.now()}`, actionType: 'stopPause', blockingType: 'HARD' },
    ]);
  }
  res.json({ success: true, robotCount: fleet.size });
});

// Fleet KPIs (P3.3)
app.get('/api/fleet/kpis', (_req, res) => {
  const robots = Array.from(fleet.values());
  const total = robots.length;
  const online = robots.filter(r => r.connectionState === 'ONLINE').length;
  const driving = robots.filter(r => r.driving).length;
  const idle = robots.filter(r => !r.driving && r.connectionState === 'ONLINE').length;
  const errors = robots.filter(r => r.errors.length > 0).length;
  const avgBattery = robots.length > 0
    ? robots.reduce((sum, r) => sum + (r.batteryCharge >= 0 ? r.batteryCharge : 0), 0) / Math.max(1, robots.filter(r => r.batteryCharge >= 0).length)
    : -1;

  res.json({
    total,
    online,
    driving,
    idle,
    errors,
    avgBattery: Math.round(avgBattery * 10) / 10,
    utilization: total > 0 ? Math.round((driving / total) * 100) : 0,
  });
});

// Traffic Management (P3.4)
app.get('/api/traffic/zones', (_req, res) => {
  res.json(trafficManager.getZones());
});

app.post('/api/traffic/zones', (req, res) => {
  const zone = req.body;
  if (!zone?.id || !zone?.polygon || !zone?.type) {
    return res.status(400).json({ error: 'Missing required fields: id, type, polygon' });
  }
  trafficManager.addZone({
    id: zone.id,
    type: zone.type,
    polygon: zone.polygon,
    direction: zone.direction,
    directionTolerance: zone.directionTolerance,
    maxRobots: zone.maxRobots || 1,
    priority: zone.priority,
  });
  res.json({ success: true });
});

app.delete('/api/traffic/zones/:id', (req, res) => {
  trafficManager.removeZone(String(req.params.id));
  res.json({ success: true });
});

app.get('/api/traffic/occupancy', (_req, res) => {
  res.json(trafficManager.getOccupancy());
});

app.get('/api/traffic/events', (req, res) => {
  const limit = parseInt(req.query.limit as string) || 100;
  res.json(trafficManager.getEvents(limit));
});

// ---------------------------------------------------------------------------
// WebSocket — real-time fleet updates
// ---------------------------------------------------------------------------

const server = http.createServer(app);
const wss = new WebSocketServer({ server, path: '/ws/fleet' });
const wsClients = new Set<WebSocket>();

wss.on('connection', (ws: WebSocket, req: http.IncomingMessage) => {
  if (API_TOKEN) {
    // Browsers cannot set headers on WebSocket upgrades — accept ?token=.
    const token = new URL(req.url || '/', 'http://internal').searchParams.get('token') || '';
    if (!tokenMatches(token)) {
      ws.close(1008, 'unauthorized');
      return;
    }
  }
  wsClients.add(ws);
  console.log(`Fleet dashboard connected (${wsClients.size})`);

  // Send initial fleet state (includes batteryCharge)
  const robots = Array.from(fleet.values()).map(r => ({
    id: robotKey(r.manufacturer, r.serialNumber),
    position: r.position,
    driving: r.driving,
    connectionState: r.connectionState,
    operatingMode: r.operatingMode,
    errorCount: r.errors.length,
    orderId: r.orderId,
    batteryCharge: r.batteryCharge,
  }));
  ws.send(JSON.stringify({ type: 'fleet_state', robots }));

  ws.on('close', () => wsClients.delete(ws));
});

// Broadcast fleet updates at 2Hz
setInterval(() => {
  if (wsClients.size === 0) return;
  const robots = Array.from(fleet.values()).map(r => ({
    id: robotKey(r.manufacturer, r.serialNumber),
    position: r.position,
    driving: r.driving,
    connectionState: r.connectionState,
    operatingMode: r.operatingMode,
    errorCount: r.errors.length,
    orderId: r.orderId,
    batteryCharge: r.batteryCharge,
  }));
  const msg = JSON.stringify({ type: 'fleet_state', robots });
  for (const ws of wsClients) {
    if (ws.readyState === WebSocket.OPEN) {
      ws.send(msg);
    }
  }
}, 500);

function broadcastFleetUpdate() {
  // Triggered on new robot discovery
  const robots = Array.from(fleet.values()).map(r => ({
    id: robotKey(r.manufacturer, r.serialNumber),
    position: r.position,
    driving: r.driving,
    connectionState: r.connectionState,
    operatingMode: r.operatingMode,
    errorCount: r.errors.length,
    orderId: r.orderId,
  }));
  const msg = JSON.stringify({ type: 'fleet_update', robots });
  for (const ws of wsClients) {
    if (ws.readyState === WebSocket.OPEN) ws.send(msg);
  }
}

// Start
server.listen(PORT, BIND_ADDR, () => {
  console.log(`AGV Fleet Manager listening on http://${BIND_ADDR}:${PORT}`);
  if (!API_TOKEN) {
    console.warn(
      '[SECURITY] FLEET_API_TOKEN is not set: the fleet control API (e-stop, ' +
      'navigate, traffic zones) and /ws/fleet are UNAUTHENTICATED. Run only ' +
      'on an isolated LAN, or set FLEET_API_TOKEN (see SECURITY.md).'
    );
  }
});
