#!/usr/bin/env node
/**
 * VDA 5050 MQTT Adapter
 *
 * Runs on each robot alongside the ROS2 stack.
 * Bridges ROS2 topics/actions ↔ VDA 5050 MQTT messages.
 *
 * Publishes: state, visualization, connection
 * Subscribes: order, instantActions
 */

import * as mqtt from 'mqtt';
import * as rclnodejs from 'rclnodejs';
import type {
  AgvState, Visualization, Connection, Order, InstantActions,
  AgvPosition, Velocity, BatteryState, SafetyState, AgvError,
  NodeState, EdgeState, ActionState, OperatingMode,
} from './vda5050_types';

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

const MQTT_BROKER = process.env.VDA_MQTT_BROKER || 'mqtt://localhost:1883';
const MANUFACTURER = process.env.VDA_MANUFACTURER || 'agv-greenhouse';
const SERIAL_NUMBER = process.env.VDA_SERIAL_NUMBER || 'agv-001';
const NAMESPACE = process.env.AGV_NAMESPACE || 'agv';
const VDA_VERSION = '2.0.0';

const TOPIC_PREFIX = `uagv/v2/${MANUFACTURER}/${SERIAL_NUMBER}`;

// Publish rates
const STATE_INTERVAL_MS = 1000;      // 1Hz state
const VIZ_INTERVAL_MS = 200;         // 5Hz visualization
const CONNECTION_INTERVAL_MS = 15000; // heartbeat every 15s

// ---------------------------------------------------------------------------
// Robot state (from ROS2 subscriptions)
// ---------------------------------------------------------------------------

let robotPose: AgvPosition = {
  x: 0, y: 0, theta: 0, mapId: 'default',
  positionInitialized: false, localizationScore: 0,
};
let robotVelocity: Velocity = { vx: 0, vy: 0, omega: 0 };
let eStopActive = false;
let motorsArmed = false;
let driving = false;
let slamConfidence = 'unknown';
let odomHz = 0;
let odomTimes: number[] = [];
let currentMode = 'teleop';
let navActive = false;
let navStatus = 'idle';
// Per-topic header IDs (VDA 5050 requires incrementing per topic, not globally)
const headerIds: Record<string, number> = {
  state: 0,
  visualization: 0,
  connection: 0,
};

// VDA 5050 order state
let currentOrderId = '';
let currentOrderUpdateId = 0;
let lastNodeId = '';
let lastNodeSequenceId = 0;
let nodeStates: NodeState[] = [];
let edgeStates: EdgeState[] = [];
let actionStates: ActionState[] = [];
let errors: AgvError[] = [];

function nextHeader(topic: string = 'state'): { headerId: number; timestamp: string; version: string; manufacturer: string; serialNumber: string } {
  if (!(topic in headerIds)) headerIds[topic] = 0;
  return {
    headerId: headerIds[topic]++,
    timestamp: new Date().toISOString(),
    version: VDA_VERSION,
    manufacturer: MANUFACTURER,
    serialNumber: SERIAL_NUMBER,
  };
}

function calcHz(times: number[]): number {
  if (times.length < 2) return 0;
  const dt = times[times.length - 1] - times[0];
  return dt > 0 ? Math.round(((times.length - 1) / dt) * 10) / 10 : 0;
}

function deriveOperatingMode(): OperatingMode {
  if (currentMode === 'teleop') return 'MANUAL';
  if (currentMode === 'mapping') return 'TEACHIN';
  return 'AUTOMATIC';
}

function deriveSafetyState(): SafetyState {
  return {
    eStop: eStopActive ? 'REMOTE' : 'NONE',
    fieldViolation: false,
  };
}

function deriveBatteryState(): BatteryState {
  return {
    batteryCharge: -1, // No BMS yet
    charging: false,
  };
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

async function main() {
  // Connect to MQTT
  const mqttClient = mqtt.connect(MQTT_BROKER, {
    clientId: `vda5050-${SERIAL_NUMBER}`,
    clean: true,
    will: {
      topic: `${TOPIC_PREFIX}/connection`,
      payload: JSON.stringify({
        ...nextHeader('connection'),
        connectionState: 'CONNECTIONBROKEN',
      }),
      qos: 1,
      retain: true,
    },
  });

  mqttClient.on('connect', () => {
    console.log(`VDA 5050 adapter connected to ${MQTT_BROKER}`);

    // Subscribe FIRST, then publish ONLINE after subscriptions confirmed
    // This prevents race where orders arrive before we're ready
    let subsConfirmed = 0;
    const onSubConfirmed = () => {
      subsConfirmed++;
      if (subsConfirmed === 2) {
        // Both subscriptions confirmed — now publish ONLINE
        mqttClient.publish(`${TOPIC_PREFIX}/connection`, JSON.stringify({
          ...nextHeader('connection'),
          connectionState: 'ONLINE',
        } satisfies Connection), { qos: 1, retain: true });
        console.log('VDA 5050: subscriptions confirmed, published ONLINE');
      }
    };

    mqttClient.subscribe(`${TOPIC_PREFIX}/order`, { qos: 1 }, onSubConfirmed);
    mqttClient.subscribe(`${TOPIC_PREFIX}/instantActions`, { qos: 1 }, onSubConfirmed);
  });

  mqttClient.on('error', (err) => {
    console.error('MQTT error:', err.message);
  });

  // Initialize ROS2
  await rclnodejs.init();
  const node = new rclnodejs.Node('vda5050_adapter');

  // Publishers (for executing received orders)
  const eStopPub = node.createPublisher('std_msgs/msg/Bool', `/${NAMESPACE}/e_stop`);
  const motorEnablePub = node.createPublisher('std_msgs/msg/Bool', `/${NAMESPACE}/motor_enable`);

  // Nav2 action client for executing VDA 5050 navigation orders
  const navActionClient = new rclnodejs.ActionClient(
    node, 'nav2_msgs/action/NavigateToPose', `/${NAMESPACE}/navigate_to_pose`
  );

  // -------------------------------------------------------------------------
  // ROS2 Subscriptions (robot state → VDA 5050)
  // -------------------------------------------------------------------------

  node.createSubscription('nav_msgs/msg/Odometry', `/${NAMESPACE}/wheel_odom`, (msg: any) => {
    odomTimes.push(Date.now() / 1000);
    if (odomTimes.length > 50) odomTimes.shift();
    odomHz = calcHz(odomTimes);
    robotVelocity.vx = msg.twist.twist.linear.x;
    robotVelocity.omega = msg.twist.twist.angular.z;
  });

  node.createSubscription('nav_msgs/msg/Odometry', `/${NAMESPACE}/odometry/global`, (msg: any) => {
    const p = msg.pose.pose;
    const q = p.orientation;
    robotPose.x = p.position.x;
    robotPose.y = p.position.y;
    robotPose.theta = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    robotPose.positionInitialized = true;
  });

  node.createSubscription('std_msgs/msg/String', `/${NAMESPACE}/motor_state`, (msg: any) => {
    try {
      const state = JSON.parse(msg.data);
      motorsArmed = state.armed;
      if (state.left_errors || state.right_errors) {
        errors = [{ errorType: 'MOTOR_ERROR', errorLevel: 'FATAL', errorDescription: 'Motor fault detected' }];
      } else {
        errors = errors.filter(e => e.errorType !== 'MOTOR_ERROR');
      }
    } catch { /* ignore */ }
  });

  node.createSubscription('std_msgs/msg/String', '/slam/quality', (msg: any) => {
    try {
      const data = JSON.parse(msg.data);
      slamConfidence = data?.tracking?.confidence || 'unknown';
      robotPose.localizationScore = slamConfidence === 'good' ? 1.0 : slamConfidence === 'medium' ? 0.6 : 0.3;
    } catch { /* ignore */ }
  });

  // -------------------------------------------------------------------------
  // MQTT message handlers (VDA 5050 → ROS2)
  // -------------------------------------------------------------------------

  mqttClient.on('message', (topic: string, payload: Buffer) => {
    try {
      const msg = JSON.parse(payload.toString());

      if (topic.endsWith('/order')) {
        handleOrder(msg as Order);
      } else if (topic.endsWith('/instantActions')) {
        handleInstantActions(msg as InstantActions);
      }
    } catch (e) {
      console.error('Failed to parse MQTT message:', e);
    }
  });

  function handleOrder(order: Order) {
    console.log(`Received order: ${order.orderId} (update ${order.orderUpdateId})`);
    currentOrderId = order.orderId;
    currentOrderUpdateId = order.orderUpdateId;

    // Update node/edge states
    nodeStates = order.nodes.map(n => ({
      nodeId: n.nodeId,
      sequenceId: n.sequenceId,
      released: n.released,
      nodePosition: n.nodePosition,
    }));
    edgeStates = order.edges.map(e => ({
      edgeId: e.edgeId,
      sequenceId: e.sequenceId,
      released: e.released,
    }));

    // Build released node queue and execute sequentially (C3: multi-node)
    const releasedNodes = order.nodes.filter(n => n.released);
    nodeQueue = releasedNodes.map(n => ({
      nodeId: n.nodeId,
      sequenceId: n.sequenceId,
      x: n.nodePosition.x,
      y: n.nodePosition.y,
      theta: n.nodePosition.theta || 0,
    }));

    // Start executing first node in queue
    executeNextInQueue();
  }

  function handleInstantActions(msg: InstantActions) {
    for (const action of msg.instantActions) {
      console.log(`Instant action: ${action.actionType} (${action.actionId})`);

      switch (action.actionType) {
        case 'cancelOrder':
          cancelCurrentNavGoal();
          currentOrderId = '';
          nodeStates = [];
          edgeStates = [];
          break;

        case 'stopPause': {
          eStopActive = true;
          const stopMsg = rclnodejs.createMessageObject('std_msgs/msg/Bool') as any;
          stopMsg.data = true;
          eStopPub.publish(stopMsg);
          break;
        }

        case 'startPause': {
          // Resume from pause
          eStopActive = false;
          const resumeMsg = rclnodejs.createMessageObject('std_msgs/msg/Bool') as any;
          resumeMsg.data = false;
          eStopPub.publish(resumeMsg);
          break;
        }

        default:
          console.warn(`Unknown instant action: ${action.actionType}`);
      }

      actionStates.push({
        actionId: action.actionId,
        actionType: action.actionType,
        actionStatus: 'FINISHED',
      });
    }
  }

  // -------------------------------------------------------------------------
  // Navigation execution
  // -------------------------------------------------------------------------

  let currentGoalHandle: any = null;
  let nodeQueue: Array<{ nodeId: string; sequenceId: number; x: number; y: number; theta: number }> = [];

  function executeNextInQueue() {
    if (nodeQueue.length === 0) {
      driving = false;
      return;
    }
    const next = nodeQueue[0];
    executeNavGoal(next.x, next.y, next.theta);
  }

  function onNodeCompleted(succeeded: boolean) {
    if (succeeded && nodeQueue.length > 0) {
      const completed = nodeQueue.shift()!;
      lastNodeId = completed.nodeId;
      lastNodeSequenceId = completed.sequenceId;
      // Remove corresponding node/edge states
      nodeStates = nodeStates.filter(n => n.sequenceId > completed.sequenceId);
      if (edgeStates.length > 0) edgeStates.shift();
      // Execute next
      executeNextInQueue();
    } else {
      nodeQueue = [];
      driving = false;
    }
  }

  function executeNavGoal(x: number, y: number, theta: number) {
    if (!navActionClient.isActionServerAvailable()) {
      console.warn('Nav2 action server not available');
      return;
    }

    driving = true;
    navActive = true;
    navStatus = 'active';

    const goal: any = {
      pose: {
        header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
        pose: {
          position: { x, y, z: 0 },
          orientation: { x: 0, y: 0, z: Math.sin(theta / 2), w: Math.cos(theta / 2) },
        },
      },
    };

    navActionClient.sendGoal(goal, () => {
      // feedback
    }).then((goalHandle: any) => {
      if (!goalHandle.isAccepted()) {
        driving = false;
        navActive = false;
        return;
      }
      currentGoalHandle = goalHandle;

      goalHandle.getResult().then(() => {
        const status = goalHandle.status;
        navActive = false;
        currentGoalHandle = null;

        if (status === 4) {
          // Succeeded — advance to next node in queue
          onNodeCompleted(true);
        } else {
          onNodeCompleted(false);
        }
      }).catch(() => {
        navActive = false;
        currentGoalHandle = null;
        onNodeCompleted(false);
      });
    }).catch(() => {
      navActive = false;
      onNodeCompleted(false);
    });
  }

  function cancelCurrentNavGoal() {
    if (currentGoalHandle) {
      currentGoalHandle.cancelGoal().catch(() => { /* ignore */ });
    }
    driving = false;
    navActive = false;
  }

  // -------------------------------------------------------------------------
  // Periodic MQTT publishing
  // -------------------------------------------------------------------------

  // State at 1Hz
  setInterval(() => {
    const state: AgvState = {
      ...nextHeader('state'),
      orderId: currentOrderId,
      orderUpdateId: currentOrderUpdateId,
      lastNodeId,
      lastNodeSequenceId,
      driving,
      paused: eStopActive,
      newBaseRequest: false,
      distanceSinceLastNode: 0,
      operatingMode: deriveOperatingMode(),
      nodeStates,
      edgeStates,
      actionStates,
      batteryState: deriveBatteryState(),
      errors,
      safetyState: deriveSafetyState(),
      agvPosition: robotPose,
      velocity: robotVelocity,
    };

    mqttClient.publish(`${TOPIC_PREFIX}/state`, JSON.stringify(state), { qos: 1 });

    // Clean up finished action states
    actionStates = actionStates.filter(a => a.actionStatus !== 'FINISHED' && a.actionStatus !== 'FAILED');
  }, STATE_INTERVAL_MS);

  // Visualization at 5Hz
  setInterval(() => {
    const viz: Visualization = {
      ...nextHeader('visualization'),
      agvPosition: robotPose,
      velocity: robotVelocity,
    };
    mqttClient.publish(`${TOPIC_PREFIX}/visualization`, JSON.stringify(viz), { qos: 0 });
  }, VIZ_INTERVAL_MS);

  // Connection heartbeat
  setInterval(() => {
    mqttClient.publish(`${TOPIC_PREFIX}/connection`, JSON.stringify({
      ...nextHeader('visualization'),
      connectionState: 'ONLINE',
    } satisfies Connection), { qos: 1, retain: true });
  }, CONNECTION_INTERVAL_MS);

  // Start ROS2 spinning
  rclnodejs.spin(node);
  console.log(`VDA 5050 adapter running: ${MANUFACTURER}/${SERIAL_NUMBER}`);
  console.log(`MQTT topics: ${TOPIC_PREFIX}/*`);
}

main().catch(console.error);
