#!/usr/bin/env node
/**
 * AGV Operator Backend — TypeScript + rclnodejs + Express
 *
 * Thin coordinator that wires together:
 * - ROS2 node (subscriptions, publishers, action clients)
 * - Express REST API (route modules in ./routes/)
 * - WebSocket handlers (./ws/)
 * - Shared state (AppDeps)
 */

import * as http from 'http';
import * as path from 'path';
import * as fs from 'fs';
import { execFile } from 'child_process';
import express from 'express';
import * as rclnodejs from 'rclnodejs';

import { deriveState, allowedActions, RobotState, MotorState, NavState, MissionProgress } from './state_machine';
import { EventLog } from './event_log';
// ScanAccumulator removed — live map comes directly from scan_grid_mapper via rclnodejs
import { AprilTagManager } from './apriltag_manager';
import { TelemetryStore } from './telemetry_store';
import { AuthManager } from './auth';
import { registerAllRoutes } from './routes';
import { setupControlWs, setupTeleopWs } from './ws/control';
import type { AppDeps, AppState, RosBridge } from './app_deps';

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

const PORT = parseInt(process.env.AGV_PORT || '8090');
const DATA_DIR = process.env.AGV_DATA_DIR || '/tmp/agv_data';
const NAMESPACE = process.env.AGV_NAMESPACE || 'agv';
const MAPS_DIR = process.env.AGV_MAPS_DIR || path.join(DATA_DIR, 'maps');
const MISSIONS_FILE = path.join(DATA_DIR, 'missions', 'missions.json');

fs.mkdirSync(DATA_DIR, { recursive: true });
fs.mkdirSync(path.join(DATA_DIR, 'missions'), { recursive: true });
fs.mkdirSync(MAPS_DIR, { recursive: true });
if (!fs.existsSync(MISSIONS_FILE)) fs.writeFileSync(MISSIONS_FILE, '[]');

// ---------------------------------------------------------------------------
// Shared instances
// ---------------------------------------------------------------------------

const eventLog = new EventLog(DATA_DIR);
// scanAccumulator removed — live map pipeline is now direct rclnodejs subscription
const apriltagManager = new AprilTagManager(DATA_DIR);
const telemetryStore = new TelemetryStore(DATA_DIR, parseInt(process.env.AGV_RETENTION_DAYS || '30'));
const authManager = new AuthManager(DATA_DIR);

eventLog.onEvent((entry) => {
  try { telemetryStore.recordEvent(entry.timestamp, entry.severity, entry.subsystem, entry.text); }
  catch { /* ignore */ }
});

// ---------------------------------------------------------------------------
// Mutable state
// ---------------------------------------------------------------------------

const state: AppState = {
  eStopActive: false,
  currentMode: 'teleop',
  robotPose: { x: 0, y: 0, theta: 0 },
  latestVelocity: { linear: 0, angular: 0 },
  motorState: { armed: false, left_state: 0, right_state: 0, left_errors: 0, right_errors: 0 },
  navState: { active: false, distance_remaining: 0, status: 'idle' },
  missionProgress: null,
  robotState: 'offline',
  slamTracking: 'unknown',
  scanPoints: [],
  navPathPoints: [],
  navPathChanged: false,
  odomTimes: [],
  activeClients: 0,
  recordingActive: false,
  missionCancel: false,
  missionPause: false,
  batteryPct: -1,
  lastImuTime: 0,
  mapPng: null,
  mapMeta: null,
  mapChanged: false,
  mapVersion: 0,
  liveMapPng: null,
  liveMapMeta: null,
  liveMapVersion: 0,
  pendingRailApproach: null,
  railApproachState: 'idle',
  health: {
    drive: { status: 'unknown', detail: 'waiting', updated: 0 },
    imu: { status: 'unknown', detail: 'waiting', updated: 0 },
    slam: { status: 'unknown', detail: 'waiting', updated: 0 },
    nav: { status: 'unknown', detail: 'waiting', updated: 0 },
    network: { status: 'ok', detail: '', updated: Date.now() / 1000 },
  },
};

let prevRobotState: RobotState = 'offline';
let navGoalHandle: any = null;
let lastCmdTime = 0;

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
  const old = state.robotState;
  state.robotState = deriveState(state.eStopActive, state.motorState, calcHz(state.odomTimes),
    state.currentMode, state.missionProgress, state.navState);
  if (old !== state.robotState) {
    eventLog.emit(
      state.robotState === 'e_stop' || state.robotState === 'fault' ? 'crit' : 'info',
      'SYSTEM', `State: ${old} → ${state.robotState}`
    );
    prevRobotState = old;
  }
}

function readMissions(): any[] {
  try { return JSON.parse(fs.readFileSync(MISSIONS_FILE, 'utf-8')); } catch { return []; }
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

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

async function main() {
  await rclnodejs.init();
  const node = new rclnodejs.Node('teleop_server');

  // --- Publishers ---
  const cmdVelPub = node.createPublisher('geometry_msgs/msg/Twist', `/${NAMESPACE}/cmd_vel`);
  const cmdVelSafePub = node.createPublisher('geometry_msgs/msg/Twist', `/${NAMESPACE}/cmd_vel_safe`);
  const eStopPub = node.createPublisher('std_msgs/msg/Bool', `/${NAMESPACE}/e_stop`);
  const motorEnablePub = node.createPublisher('std_msgs/msg/Bool', `/${NAMESPACE}/motor_enable`);
  const modePub = node.createPublisher('std_msgs/msg/String', `/${NAMESPACE}/mode`);
  // AprilTag registry reload trigger (transient_local so marker_correction picks it up after restart)
  const markerReloadPub = node.createPublisher('std_msgs/msg/Empty',
    `/${NAMESPACE}/markers/registry_reload`,
    { qos: new rclnodejs.QoS(rclnodejs.QoS.HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1,
      rclnodejs.QoS.ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
      rclnodejs.QoS.DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) });

  // When AprilTagManager regenerates the registry YAML, trigger marker_correction reload
  apriltagManager.onRegistryChanged(() => {
    const empty = rclnodejs.createMessageObject('std_msgs/msg/Empty') as any;
    markerReloadPub.publish(empty);
  });

  // --- Nav2 Action Client ---
  const navActionClient = new rclnodejs.ActionClient(
    node, 'nav2_msgs/action/NavigateToPose', `/${NAMESPACE}/navigate_to_pose`
  );

  // --- Service Clients ---
  const startRecClient = node.createClient('std_srvs/srv/Trigger', '/session/start_recording');
  const stopRecClient = node.createClient('std_srvs/srv/Trigger', '/session/stop_recording');
  const loadMapClient = node.createClient('nav2_msgs/srv/LoadMap', `/${NAMESPACE}/map_server/load_map`);

  // --- ROS Bridge (passed to route modules via AppDeps) ---
  const ros: RosBridge = {
    sendCmdVel(linear: number, angular: number) {
      if (state.eStopActive) return;
      if (state.currentMode !== 'teleop' && state.currentMode !== 'mapping') return;
      const msg = rclnodejs.createMessageObject('geometry_msgs/msg/Twist') as any;
      // Mapping mode: tighter limits to keep cuVSLAM tracking stable
      const maxLin = state.currentMode === 'mapping' ? 0.4 : 0.5;
      const maxAng = state.currentMode === 'mapping' ? 0.2 : 0.5;
      msg.linear.x = Math.max(-maxLin, Math.min(maxLin, linear));
      msg.angular.z = Math.max(-maxAng, Math.min(maxAng, angular));
      cmdVelPub.publish(msg);
      cmdVelSafePub.publish(msg);
      lastCmdTime = Date.now() / 1000;
    },

    sendNavGoal(x: number, y: number, theta: number = 0) {
      if (state.currentMode !== 'nav') return { success: false, message: 'Not in nav mode' };
      if (!navActionClient.isActionServerAvailable()) {
        state.health.nav = { status: 'error', detail: 'Nav2 not available', updated: Date.now() / 1000 };
        return { success: false, message: 'Nav2 action server not available' };
      }
      const goal: any = {
        pose: {
          header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
          pose: {
            position: { x, y, z: 0 },
            orientation: { x: 0, y: 0, z: Math.sin(theta / 2), w: Math.cos(theta / 2) },
          },
        },
      };
      navActionClient.sendGoal(goal, (fb: any) => {
        state.navState.distance_remaining = Math.round((fb.distance_remaining || 0) * 1000) / 1000;
        state.navState.status = 'active';
      }).then((gh: any) => {
        if (!gh.isAccepted()) {
          state.navState = { active: false, distance_remaining: 0, status: 'rejected' };
          updateState(); return;
        }
        navGoalHandle = gh;
        state.navState.status = 'active';
        state.health.nav = { status: 'ok', detail: 'Navigating', updated: Date.now() / 1000 };
        updateState();
        gh.getResult().then(() => {
          const s = gh.status;
          const succeeded = s === 4;
          state.navState = { active: false, distance_remaining: 0,
            status: succeeded ? 'succeeded' : s === 5 ? 'canceled' : 'aborted' };
          navGoalHandle = null; state.navPathPoints = []; state.navPathChanged = true;
          updateState();

          // Auto-trigger rail_approach if this nav goal targeted a rail_start tag
          if (succeeded && state.pendingRailApproach) {
            const { hardware_id, defined_id } = state.pendingRailApproach;
            state.pendingRailApproach = null;
            eventLog.emit('info', 'NAV',
              `Nav2 reached rail_start vicinity, triggering precision approach for tag ${hardware_id}`);
            const { execFile } = require('child_process');
            execFile('ros2', ['service', 'call',
              `/${NAMESPACE}/rail_approach/execute`,
              'agv_interfaces/srv/RailApproach',
              `{tag_id: ${hardware_id}, offset_x: 0.3, offset_y: 0.0}`],
              { env: process.env, timeout: 10000 }, (err: any) => {
                if (err) {
                  eventLog.emit('warn', 'NAV', `rail_approach service call failed: ${err.message}`);
                }
              });
            void defined_id;  // available for future logging
          }
        }).catch(() => { state.navState = { active: false, distance_remaining: 0, status: 'aborted' }; navGoalHandle = null; state.pendingRailApproach = null; updateState(); });
      }).catch(() => { state.navState = { active: false, distance_remaining: 0, status: 'aborted' }; updateState(); });

      state.navState = { active: true, distance_remaining: 0, status: 'sending' };
      eventLog.emit('info', 'NAV', `Goal: (${x.toFixed(2)}, ${y.toFixed(2)})`);
      updateState();
      return { success: true, message: 'Goal sent' };
    },

    cancelNavGoal() {
      if (navGoalHandle) { navGoalHandle.cancelGoal().catch(() => {}); state.navState.status = 'canceling'; }
      state.missionCancel = true;
    },

    sendEStop(active: boolean) {
      state.eStopActive = active;
      const msg = rclnodejs.createMessageObject('std_msgs/msg/Bool') as any;
      msg.data = active; eStopPub.publish(msg);
      if (active) { ros.sendCmdVel(0, 0); ros.cancelNavGoal(); }
      eventLog.emit(active ? 'crit' : 'info', 'SAFETY', active ? 'E-STOP ACTIVATED' : 'E-stop cleared');
      updateState();
    },

    sendMotorEnable(active: boolean) {
      const msg = rclnodejs.createMessageObject('std_msgs/msg/Bool') as any;
      msg.data = active; motorEnablePub.publish(msg);
    },

    async callTriggerService(client: any, name: string) {
      if (!client.isServiceServerAvailable()) return { success: false, message: `${name} not available` };
      try {
        const r = await client.sendRequestAsync({}, { timeout: 5000 });
        return { success: r.success, message: r.message || '' };
      } catch (e: any) { return { success: false, message: e?.message || 'failed' }; }
    },

    startRecClient, stopRecClient, loadMapClient,

    async saveMap(name: string, mapDir: string, mapTopic: string) {
      if (!name || name.includes('/') || name.includes('..')) return { success: false, message: 'Invalid name' };
      const outPath = path.join(mapDir, name);
      return new Promise((resolve) => {
        execFile('ros2', ['run', 'nav2_map_server', 'map_saver_cli', '-f', outPath, '-t', mapTopic,
          '--ros-args', '-p', 'save_map_timeout:=10.0'], { timeout: 15000 }, (error, _stdout, stderr) => {
          if (error) { eventLog.emit('warn', 'MAPPING', `Save failed: ${name}`); resolve({ success: false, message: stderr?.trim() || 'failed' }); }
          else { eventLog.emit('info', 'MAPPING', `Map "${name}" saved`); resolve({ success: true, message: 'Saved', name }); }
        });
      });
    },
  };

  // --- Mission executor ---
  async function executeMission(missionId: string): Promise<{ success: boolean; message?: string; nodes?: number }> {
    const missions = readMissions();
    let mission = missions.find((m: any) => m.id === missionId);
    if (!mission) return { success: false, message: 'Mission not found' };
    mission = normalizeMission(mission);
    if (state.currentMode !== 'nav') return { success: false, message: 'Not in nav mode' };
    const nodes = mission.nodes || [];
    if (!nodes.length) return { success: false, message: 'No nodes' };

    state.missionCancel = false; state.missionPause = false;
    const runId = `run_${Date.now()}`;
    state.missionProgress = { mission_id: missionId, mission_name: mission.name || '', current_node: 0, total_nodes: nodes.length, status: 'running' };
    eventLog.emit('info', 'MISSION', `"${mission.name}" started (${nodes.length} nodes)`);
    telemetryStore.startMissionRun(runId, missionId, mission.name || '', nodes.length);
    updateState();

    (async () => {
      let completed = true;
      for (let i = 0; i < nodes.length; i++) {
        if (state.missionCancel) { state.missionProgress!.status = 'canceled'; completed = false; break; }
        while (state.missionPause) { await new Promise(r => setTimeout(r, 500)); if (state.missionCancel) break; }
        if (state.missionCancel) { state.missionProgress!.status = 'canceled'; completed = false; break; }
        state.missionProgress!.current_node = i; state.missionProgress!.status = 'running';
        const nd = nodes[i];

        // If waypoint is snapped to an AprilTag, check if it's a rail_start
        // and set up auto-trigger for rail_approach precision alignment.
        let railTag: any = null;
        if (nd.apriltag_id) {
          const tag = apriltagManager.getDefinedTag(nd.apriltag_id);
          if (tag && tag.type === 'rail_start') {
            // Find hardware ID assigned to this defined tag
            for (const [hw, def] of Object.entries(apriltagManager.getHardwareAssignments())) {
              if (def === nd.apriltag_id) {
                state.pendingRailApproach = { hardware_id: parseInt(hw, 10), defined_id: nd.apriltag_id };
                railTag = tag;
                break;
              }
            }
          }
        }

        ros.sendNavGoal(parseFloat(nd.x || 0), parseFloat(nd.y || 0), parseFloat(nd.theta || 0));
        while (state.navState.active) { await new Promise(r => setTimeout(r, 500)); if (state.missionCancel) { ros.cancelNavGoal(); break; } }
        if (state.navState.status !== 'succeeded') { state.missionProgress!.status = 'failed'; completed = false; break; }

        // If this waypoint snapped to a rail_start tag, wait for rail_approach to settle
        if (railTag) {
          eventLog.emit('info', 'MISSION', `Waiting for rail_approach alignment at "${railTag.label}"`);
          // Wait up to 30s for FINE_SERVOING → settle (state returns to 'idle' after success)
          const start = Date.now();
          while (Date.now() - start < 30000) {
            if (state.missionCancel) break;
            // After rail_approach finishes, status returns to 'idle'
            // We need to wait for it to LEAVE idle (meaning service was called) and return
            if (state.railApproachState === 'idle' && Date.now() - start > 2000) break;
            await new Promise(r => setTimeout(r, 200));
          }
          if (state.missionCancel) { completed = false; break; }
        }

        // Execute action after navigation (and rail alignment if applicable)
        if (nd.action === 'pause') {
          await new Promise(r => setTimeout(r, (parseFloat(nd.pause_sec || 3)) * 1000));
        } else if (nd.action === 'start_recording') {
          eventLog.emit('info', 'MISSION', `Starting recording at waypoint ${i + 1}`);
          await ros.callTriggerService(ros.startRecClient, 'start_recording').catch(() => {});
          state.recordingActive = true;
        } else if (nd.action === 'stop_recording') {
          eventLog.emit('info', 'MISSION', `Stopping recording at waypoint ${i + 1}`);
          await ros.callTriggerService(ros.stopRecClient, 'stop_recording').catch(() => {});
          state.recordingActive = false;
        }
      }
      if (completed) { state.missionProgress!.status = 'completed'; telemetryStore.endMissionRun(runId, 'completed', nodes.length); }
      else { telemetryStore.endMissionRun(runId, state.missionProgress!.status, state.missionProgress!.current_node); }
      updateState();
    })().catch((err) => {
      if (state.missionProgress) state.missionProgress.status = 'failed';
      telemetryStore.endMissionRun(runId, 'failed', state.missionProgress?.current_node || 0);
      updateState();
    });

    return { success: true, nodes: nodes.length };
  }

  // --- Subscribers ---
  //
  // DDS late-joiner fix: subscriptions are created AFTER rclnodejs.spin()
  // starts (see bottom of main()). This gives the DDS participant time to
  // discover existing publishers before registering subscriber endpoints.
  // A deferred creation with a small delay ensures reliable discovery
  // of all publishers, even low-frequency ones like motor_state (2 Hz).

  function createAllSubscriptions() {
    node.createSubscription('nav_msgs/msg/Odometry', `/${NAMESPACE}/wheel_odom`, (msg: any) => {
      state.odomTimes.push(Date.now() / 1000);
      if (state.odomTimes.length > 50) state.odomTimes.shift();
      state.latestVelocity = {
        linear: Math.round(msg.twist.twist.linear.x * 1000) / 1000,
        angular: Math.round(msg.twist.twist.angular.z * 1000) / 1000,
      };
    });

    // motor_state is handled by the subprocess bridge (see below)
    // because rclnodejs DDS discovery is unreliable for this topic.
    // The rclnodejs subscriber is kept as fallback but the bridge is primary.

    node.createSubscription('nav_msgs/msg/Odometry', `/${NAMESPACE}/odometry/global`, (msg: any) => {
      const p = msg.pose.pose;
      state.robotPose = {
        x: Math.round(p.position.x * 1e4) / 1e4,
        y: Math.round(p.position.y * 1e4) / 1e4,
        theta: Math.round(yawFromQuat(p.orientation) * 1e4) / 1e4,
      };
    });

    node.createSubscription('std_msgs/msg/String', '/slam/quality', (msg: any) => {
      try { state.slamTracking = JSON.parse(msg.data)?.tracking?.confidence || 'unknown'; } catch {}
    });

    node.createSubscription('nav_msgs/msg/Path', `/${NAMESPACE}/plan`, (msg: any) => {
      state.navPathPoints = (msg.poses || []).map((ps: any) => ({
        x: Math.round(ps.pose.position.x * 1000) / 1000, y: Math.round(ps.pose.position.y * 1000) / 1000,
      }));
      state.navPathChanged = true;
    });

    node.createSubscription('sensor_msgs/msg/LaserScan', `/${NAMESPACE}/scan`, (msg: any) => {
      // Extract scan points for real-time visualization (red dots on map)
      const points: Array<{x: number; y: number}> = [];
      const cosT = Math.cos(state.robotPose.theta);
      const sinT = Math.sin(state.robotPose.theta);
      const ranges: number[] = Array.from(msg.ranges);
      let angle = msg.angle_min as number;
      const inc = msg.angle_increment as number;
      const rMin = msg.range_min as number;
      const rMax = msg.range_max as number;
      for (let i = 0; i < ranges.length; i++) {
        const r = ranges[i];
        if (r > rMin && r < rMax) {
          const lx = r * Math.cos(angle);
          const ly = r * Math.sin(angle);
          points.push({
            x: Math.round((state.robotPose.x + cosT * lx - sinT * ly) * 1000) / 1000,
            y: Math.round((state.robotPose.y + sinT * lx + cosT * ly) * 1000) / 1000,
          });
        }
        angle += inc;
      }
      state.scanPoints = points;
    });

    node.createSubscription('sensor_msgs/msg/Imu', `/${NAMESPACE}/zed/imu/data`, () => {
      state.lastImuTime = Date.now() / 1000;
    });

    // AprilTag raw detections — listen on /marker_raw_detected ("tag_<id>") which
    // marker_correction_node publishes for ALL detected tags (regardless of registry).
    // Avoids the need for apriltag_msgs custom types which rclnodejs can't load.
    node.createSubscription('std_msgs/msg/String',
      `/${NAMESPACE}/marker_raw_detected`, (msg: any) => {
      const data: string = typeof msg.data === 'string' ? msg.data : '';
      const m = data.match(/^tag_(\d+)$/);
      if (!m) return;
      const id = parseInt(m[1], 10);
      if (isNaN(id) || id < 0) return;
      apriltagManager.recordPendingDetection(id);
    });

    // Rail approach status — subscribed to track state for waypoint action gating.
    // Format: '{"state":"settled","target_tag":7}'
    node.createSubscription('std_msgs/msg/String',
      `/${NAMESPACE}/rail_approach/status`, (msg: any) => {
      try {
        const parsed = JSON.parse(msg.data);
        state.railApproachState = parsed.state || 'unknown';
      } catch { /* ignore parse errors */ }
    });

    console.log('[ROS] All subscriptions created');
  }

  // Create all subscriptions BEFORE spin — DDS requires subscriptions
  // to be registered before spinning so they are announced to the network.
  createAllSubscriptions();

  // Battery state — extract percentage for dashboard display
  node.createSubscription('sensor_msgs/msg/BatteryState', `/${NAMESPACE}/battery`, (msg: any) => {
    state.batteryPct = typeof msg.percentage === 'number' ? Math.round(msg.percentage * 100) / 100 : -1;
  });

  // OccupancyGrid (transient local QoS)
  const tlQos = new rclnodejs.QoS(rclnodejs.QoS.HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1,
    rclnodejs.QoS.ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    rclnodejs.QoS.DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  node.createSubscription('nav_msgs/msg/OccupancyGrid', `/${NAMESPACE}/map`, { qos: tlQos }, (msg: any) => {
    const w = msg.info.width, h = msg.info.height, data = msg.data;
    const pixels = Buffer.alloc(w * h);
    for (let i = 0; i < w * h; i++) {
      const v = typeof data[i] === 'number' ? data[i] : 0;
      pixels[i] = v === -1 || v === 255 ? 205 : v === 0 ? 254 : v === 100 ? 0 : Math.max(0, Math.min(255, 255 - Math.round(v * 2.55)));
    }
    const flipped = Buffer.alloc(w * h);
    for (let row = 0; row < h; row++) pixels.copy(flipped, (h - 1 - row) * w, row * w, (row + 1) * w);
    const sharp = require('sharp');
    sharp(flipped, { raw: { width: w, height: h, channels: 1 } }).png().toBuffer()
      .then((buf: Buffer) => { state.mapPng = buf; state.mapMeta = { resolution: msg.info.resolution,
        origin_x: msg.info.origin.position.x, origin_y: msg.info.origin.position.y, width: w, height: h }; state.mapChanged = true; state.mapVersion++; })
      .catch(() => {});
  });

  // --- Live occupancy grid (direct rclnodejs subscription) ---
  // Same pattern as static map subscription above — no Python subprocess needed.
  // Uses a sequence counter to prevent race conditions: sharp PNG compression is
  // async, so a newer OccupancyGrid may arrive before the previous PNG finishes.
  // Without the guard, a stale PNG could overwrite the current metadata, causing
  // the frontend to display the map at incorrect bounds.
  let liveMapSeq = 0;
  let liveMapCompressing = false;
  node.createSubscription('nav_msgs/msg/OccupancyGrid', `/${NAMESPACE}/live_map`,
    { qos: tlQos }, (msg: any) => {
    // Drop this frame if previous PNG is still compressing — prevents queue buildup
    if (liveMapCompressing) return;

    const seq = ++liveMapSeq;
    const w = msg.info.width, h = msg.info.height, data = msg.data;
    const meta = { resolution: msg.info.resolution,
      origin_x: msg.info.origin.position.x, origin_y: msg.info.origin.position.y,
      width: w, height: h };

    const pixels = Buffer.alloc(w * h * 4);
    for (let i = 0; i < w * h; i++) {
      const v = typeof data[i] === 'number' ? data[i] : -1;
      const p = i * 4;
      if (v === -1 || v === 255) {
        pixels[p] = 0; pixels[p+1] = 0; pixels[p+2] = 0; pixels[p+3] = 0;
      } else if (v === 0) {
        pixels[p] = 240; pixels[p+1] = 245; pixels[p+2] = 240; pixels[p+3] = 180;
      } else if (v >= 80) {
        pixels[p] = 20; pixels[p+1] = 20; pixels[p+2] = 25; pixels[p+3] = 240;
      } else {
        const gray = Math.max(40, 220 - v * 2);
        const a = Math.min(220, 80 + v * 2);
        pixels[p] = gray; pixels[p+1] = gray; pixels[p+2] = gray; pixels[p+3] = a;
      }
    }
    const rowBytes = w * 4;
    const flipped = Buffer.alloc(w * h * 4);
    for (let row = 0; row < h; row++)
      pixels.copy(flipped, (h - 1 - row) * rowBytes, row * rowBytes, (row + 1) * rowBytes);

    liveMapCompressing = true;
    const sharp = require('sharp');
    sharp(flipped, { raw: { width: w, height: h, channels: 4 } }).png().toBuffer()
      .then((buf: Buffer) => {
        liveMapCompressing = false;
        // Only apply if this is still the latest frame (discard stale completions)
        if (seq !== liveMapSeq) return;
        state.liveMapPng = buf;
        state.liveMapMeta = meta;
        state.liveMapVersion++;
      }).catch(() => { liveMapCompressing = false; });
  });

  // --- motor_state bridge via subprocess ---
  // rclnodejs has a DDS discovery bug where its subscriber fails to
  // connect to existing C++ publishers on low-frequency topics.
  // Workaround: use `ros2 topic echo` as a subprocess and parse stdout.
  // This is reliable because the ros2 CLI creates its own DDS participant
  // that properly discovers all publishers.
  const { spawn } = require('child_process');
  function startMotorStateBridge() {
    const proc = spawn('ros2', ['topic', 'echo', '--no-arr', '--full-length', `/${NAMESPACE}/motor_state`, 'std_msgs/msg/String'], {
      env: process.env,
      stdio: ['ignore', 'pipe', 'pipe'],
    });
    let buf = '';
    proc.stdout.on('data', (chunk: Buffer) => {
      buf += chunk.toString();
      // Each message ends with "---\n"
      const parts = buf.split('---\n');
      buf = parts.pop() || '';
      for (const part of parts) {
        const match = part.match(/data:\s*'(.+)'/s);
        if (!match) continue;
        try {
          const parsed = JSON.parse(match[1].replace(/\\n/g, '').replace(/\bnan\b/g, 'null'));
          if (parsed._keepalive) continue;
          const prev = state.motorState.armed;
          state.motorState = parsed;
          if (prev !== state.motorState.armed) {
            console.log(`[motor_state bridge] armed=${parsed.armed}`);
            eventLog.emit('info', 'DRIVE', state.motorState.armed ? 'Armed' : 'Disarmed');
            updateState();
          }
        } catch { /* ignore parse errors */ }
      }
    });
    proc.stderr.on('data', (chunk: Buffer) => {
      const line = chunk.toString().trim();
      if (line && !line.includes('selected interface')) console.warn('[motor_state bridge stderr]', line);
    });
    proc.on('exit', (code: number) => {
      console.warn(`[motor_state bridge] Exited code=${code}, restarting in 2s...`);
      setTimeout(startMotorStateBridge, 2000);
    });
  }
  setTimeout(startMotorStateBridge, 3000); // start after DDS discovery settles

  // --- Timers ---
  setInterval(() => { // Health 1Hz
    const now = Date.now() / 1000, hz = calcHz(state.odomTimes);
    state.health.drive = hz > 10 ? { status: 'ok', detail: `${hz} Hz`, updated: now }
      : hz > 1 ? { status: 'warn', detail: `${hz} Hz (low)`, updated: now }
      : { status: 'error', detail: 'No odom', updated: now, action: 'Check CAN' };
    const imuAge = state.lastImuTime > 0 ? now - state.lastImuTime : Infinity;
    state.health.imu = imuAge < 2 ? { status: 'ok', detail: 'Receiving', updated: now }
      : imuAge < 10 ? { status: 'warn', detail: `Stale (${imuAge.toFixed(0)}s)`, updated: now }
      : { status: 'error', detail: 'No IMU data', updated: now, action: 'Check ZED/IMU' };
    state.health.slam = state.slamTracking === 'good'
      ? { status: 'ok', detail: 'Tracking: good', updated: now }
      : { status: 'warn', detail: `Tracking: ${state.slamTracking}`, updated: now };
    state.health.nav = navActionClient.isActionServerAvailable()
      ? { status: 'ok', detail: state.navState.active ? `Active (${state.navState.distance_remaining}m)` : 'Ready', updated: now }
      : { status: 'warn', detail: 'Nav2 not ready', updated: now };
    state.health.network = { status: 'ok', detail: `${state.activeClients} client(s)`, updated: now };
    updateState();
  }, 1000);
  setInterval(() => { try { telemetryStore.recordSample({ timestamp: Date.now() / 1000,
    pose_x: state.robotPose.x, pose_y: state.robotPose.y, pose_theta: state.robotPose.theta,
    linear_vel: state.latestVelocity.linear, angular_vel: state.latestVelocity.angular,
    odom_hz: calcHz(state.odomTimes), slam_confidence: state.slamTracking,
    robot_state: state.robotState, battery_pct: state.batteryPct }); } catch {} }, 1000);
  setInterval(() => { try { telemetryStore.prune(); } catch {} }, 86400_000);
  setInterval(() => { // Watchdog
    if (state.activeClients > 0 && !state.eStopActive && lastCmdTime > 0 && Date.now() / 1000 - lastCmdTime > 0.5) {
      const msg = rclnodejs.createMessageObject('geometry_msgs/msg/Twist') as any; cmdVelPub.publish(msg); cmdVelSafePub.publish(msg);
    }
  }, 100);

  // --- Build AppDeps ---
  const deps: AppDeps = {
    state, ros, eventLog, telemetryStore, authManager, apriltagManager,
    config: { port: PORT, dataDir: DATA_DIR, namespace: NAMESPACE, mapsDir: MAPS_DIR, missionsFile: MISSIONS_FILE },
    updateState,
    setMode(mode: string) {
      if (mode !== state.currentMode) {
        if (mode !== 'nav' && state.navState.active) ros.cancelNavGoal();
        eventLog.emit('info', 'SYSTEM', `Mode: ${state.currentMode} → ${mode}`);
        state.currentMode = mode;
        // Publish mode to ROS2 topic (interfaces.yaml compliance)
        const modeMsg = rclnodejs.createMessageObject('std_msgs/msg/String') as any;
        modeMsg.data = mode;
        modePub.publish(modeMsg);
        updateState();
      }
    },
    executeMission,
  };

  // --- Express + Routes ---
  const app = express();
  app.use(express.json());

  const dashboardDir = path.resolve(__dirname, '../../../web/agv_dashboard/dist');
  if (fs.existsSync(dashboardDir)) app.use('/dashboard', express.static(dashboardDir));
  app.get('/', (_req, res) => {
    if (fs.existsSync(dashboardDir)) res.redirect('/dashboard');
    else res.status(404).send('Dashboard not built');
  });

  registerAllRoutes(app, deps, node);

  // --- WebSocket + Server ---
  const server = http.createServer(app);
  setupControlWs(server, deps);
  setupTeleopWs(server, deps);

  // Start DDS spin BEFORE server listen so the event loop processes
  // participant discovery for publishers that already exist.
  rclnodejs.spin(node);

  server.listen(PORT, '0.0.0.0', () => {
    console.log(`AGV Backend (TS) on http://0.0.0.0:${PORT}`);
    eventLog.emit('info', 'SYSTEM', 'Backend started');
  });
}

main().catch(console.error);
