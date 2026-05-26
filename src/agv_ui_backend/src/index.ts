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
import { readHomePoint } from './routes/home_point';
import { setupControlWs, setupTeleopWs } from './ws/control';
import { deriveBatteryTte } from './battery_tte';
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
  odomTimes: [],
  activeClients: 0,
  recordingActive: false,
  missionCancel: false,
  missionPause: false,
  batteryPct: -1,
  batterySamples: [],
  batteryTteS: null,
  homePoint: null,
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
  // Iter-37 Phase 2 defaults — updated from /agv/{mode,zone,rail_driver}/state.
  modeArbiterState: {
    mode: 'unknown', source: 'NONE', zone: 'unknown',
    operator_mode: 'nav', transitions: 0, updated: 0,
  },
  zoneDetectorState: {
    zone: 'unknown', section: '', aisle_y_center: null,
    rail_offset_lat: null, rail_yaw_error: null,
    approach_tag_id: -1, confidence: 0, source: 'pose', updated: 0,
  },
  railDriverState: {
    state: 'idle', linear_x: 0, remaining_m: 0,
    in_rail_zone: false, collision_stop: false, updated: 0,
  },
  // 'IDLE' (gray) at boot before collision_monitor has published any state.
  // The collision_monitor only publishes state_topic when it processes a
  // cmd_vel_smoothed, so a fresh boot with no nav active → no state messages.
  // 'OFFLINE' is reserved for the STALE case (updated>0 and age>2s).
  collisionMonitor: { action: 'IDLE', polygon: '', updated: 0 },
  localization: { action: 'UNKNOWN', detail: '', map: '', updated: 0 },
  currentMapName: null,
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
  // cmd_vel is the SINGLE entry point into the safety chain:
  //   /agv/cmd_vel → velocity_smoother → /agv/cmd_vel_smoothed
  //                                    → collision_monitor → /agv/cmd_vel_safe → odrive
  // The backend NEVER publishes to cmd_vel_safe directly — only collision_monitor
  // does, ensuring single-source-of-truth and giving teleop/mapping the same
  // collision protection as nav.
  const cmdVelPub = node.createPublisher('geometry_msgs/msg/Twist', `/${NAMESPACE}/cmd_vel`);
  const eStopPub = node.createPublisher('std_msgs/msg/Bool', `/${NAMESPACE}/e_stop`);
  const motorEnablePub = node.createPublisher('std_msgs/msg/Bool', `/${NAMESPACE}/motor_enable`);
  const modePub = node.createPublisher('std_msgs/msg/String', `/${NAMESPACE}/mode`);
  // Phase-2 mode_arbiter consumes this to pick its operator_mode (nav|teleop|idle).
  // Without it the arbiter stays in its default 'nav' and stomps the cmd_vel topic
  // at 20 Hz with zero-Twist while teleop_server also publishes joystick commands.
  const operatorModePub = node.createPublisher('std_msgs/msg/String', `/${NAMESPACE}/mode/set`);
  // AprilTag registry reload trigger (transient_local so marker_correction picks it up after restart)
  const markerReloadPub = node.createPublisher('std_msgs/msg/Empty',
    `/${NAMESPACE}/markers/registry_reload`,
    { qos: new rclnodejs.QoS(rclnodejs.QoS.HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1,
      rclnodejs.QoS.ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
      rclnodejs.QoS.DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) });
  // maps/loaded event — fired from routes/maps.ts after any successful Nav2
  // load_map call, and at boot with AGV_BOOT_MAP_NAME if the launch loaded a
  // default map. Transient_local so the auto_init_orchestrator receives the
  // last published value even if it subscribes later. QoS must match the
  // orchestrator's subscription (rclcpp::QoS(1).transient_local().reliable()).
  const mapsLoadedPub = node.createPublisher('std_msgs/msg/String',
    `/${NAMESPACE}/maps/loaded`,
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
  // Nav2 lifecycle liveness check. lifecycle_manager_navigation exposes
  // is_active (std_srvs/Trigger) that returns success=true iff all of its
  // managed nodes (map_server, controller_server, planner_server,
  // behavior_server, bt_navigator, velocity_smoother, collision_monitor)
  // are in the 'active' lifecycle state. Used to gate setMode('nav').
  const nav2IsActiveClient = node.createClient(
    'std_srvs/srv/Trigger',
    `/${NAMESPACE}/lifecycle_manager_navigation/is_active`);

  // rail_approach service client. Used by /api/apriltags/:hw/align (the
  // pure fine-servoing path that skips Nav2) and by the post-Nav2
  // pendingRailApproach trigger. Replaces the previous execFile path
  // so we can pass skip_coarse_approach and read the response cleanly.
  // Cast to `any` because rclnodejs's TypeScript declaration map is
  // generated from a static catalogue that doesn't yet include
  // agv_interfaces/srv/RailApproach. The runtime resolution is dynamic
  // and works fine.
  const railApproachClient = (node as any).createClient(
    'agv_interfaces/srv/RailApproach',
    `/${NAMESPACE}/rail_approach/execute`);

  // list_rail_starts: empty Request, Response carries RailStartPoint[].
  // Same cast-to-any pattern as railApproachClient — rclnodejs's static type
  // map doesn't include agv_interfaces. Used by routes/rails.ts to drive the
  // data-driven rail-label overlay (replaces hardcoded RAIL_AISLE_Y).
  const listRailStartsClient = (node as any).createClient(
    'agv_interfaces/srv/ListRailStarts',
    `/${NAMESPACE}/rail_approach/list_rail_starts`);

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
      // Single chain entry point — smoother + collision_monitor handle the rest.
      cmdVelPub.publish(msg);
      lastCmdTime = Date.now() / 1000;
    },

    sendNavGoal(x: number, y: number, theta: number = 0) {
      if (state.currentMode !== 'nav') return { success: false, message: 'Not in nav mode' };
      if (!state.motorState.armed) {
        eventLog.emit('warn', 'NAV',
          'Goal rejected: motors not armed. Arm motors first (Recovery panel)');
        return { success: false, message: 'Motors not armed. Arm motors before navigating.' };
      }
      // Fase 7 F4: gate nav goals when the localization cascade has exhausted
      // all 4 paths (A0 Area Memory, A cuVSLAM+AprilTag, B AprilTag only, C
      // last-known pose). With localization in FAILED, map→odom is stale
      // (frozen from a prior session) and the robot navigates to phantom
      // coordinates. Blocks only FAILED; allows UNKNOWN (boot), INITIALIZING
      // (cascade running), LOCALIZED (ideal), DEGRADED (operator accepts
      // drift risk). See specs/state_machine.yaml invariant mode_coherence
      // and docs/audit/2026-04-13-full-audit.md bug #1 history.
      //
      // KNOWN GAP: agv_waypoint_manager sends NavigateToPose goals directly
      // to Nav2, bypassing this gate. Mission execution remains unprotected.
      // Tracked for Fase 8 (mission executor refactor to route via backend).
      if (state.localization?.action === 'FAILED') {
        eventLog.emit('crit', 'NAV',
          `Goal rejected: localization FAILED (${state.localization.detail || 'cascade exhausted'}). ` +
          `Drive to an AprilTag via teleop and call /agv/localization/reinitialize.`);
        return { success: false, message: 'Localization failed. Reinitialize with AprilTag before navigating.' };
      }
      // Safety chain liveness gate. Nav2's collision_monitor only publishes
      // state_topic WHILE it's processing cmd_vel_smoothed (i.e., during an
      // active navigation). On cold boot we have NEVER seen a state message
      // (updated == 0), so the first goal is allowed through; the STALE
      // watchdog will catch it within 2s if the chain silently fails.
      //
      // For subsequent goals: if we previously received state but it has aged
      // more than 2s without updates while we're navigating, the chain is
      // considered broken and we reject new goals with a clear message.
      const cm = state.collisionMonitor;
      if (cm.updated > 0) {
        const cmAge = Date.now() / 1000 - cm.updated;
        if (cm.action === 'STALE' || cm.action === 'OFFLINE' || cmAge > 2.0) {
          eventLog.emit('crit', 'SAFETY',
            `Goal rejected: collision_monitor ${cm.action} (last update ${cmAge.toFixed(1)}s ago)`);
          return { success: false, message: 'Safety chain offline (collision_monitor not publishing). Restart nav stack.' };
        }
      }
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
          navGoalHandle = null; state.navPathPoints = [];
          updateState();

          // Auto-trigger rail_approach if this nav goal targeted a rail_start tag
          if (succeeded && state.pendingRailApproach) {
            const { hardware_id, defined_id } = state.pendingRailApproach;
            state.pendingRailApproach = null;
            eventLog.emit('info', 'NAV',
              `Nav2 reached rail_start vicinity, triggering precision approach for tag ${hardware_id}`);
            // Use the rclnodejs client (replaces the previous execFile path).
            // skip_coarse_approach=false because Nav2 just delivered us to
            // the standoff — coarse and fine want to compose normally here.
            ros.callRailApproach({
              tag_id: hardware_id, offset_x: 0.3, offset_y: 0.0,
              skip_coarse_approach: false,
            }).then((r) => {
              if (!r.success) {
                eventLog.emit('warn', 'NAV',
                  `rail_approach service call failed: ${r.message}`);
              }
            }).catch((e: any) => {
              eventLog.emit('warn', 'NAV',
                `rail_approach service call threw: ${e?.message || e}`);
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
      // Inject (0,0) into the chain. velocity_smoother decelerates linearly via
      // max_decel: -1.0 m/s² → 0.5→0 m/s in ~500ms, graceful (no jerk).
      // Don't wait for Nav2 to process the cancellation.
      const zero = rclnodejs.createMessageObject('geometry_msgs/msg/Twist') as any;
      zero.linear.x = 0; zero.linear.y = 0; zero.linear.z = 0;
      zero.angular.x = 0; zero.angular.y = 0; zero.angular.z = 0;
      cmdVelPub.publish(zero);
      eventLog.emit('warn', 'NAV', 'Navigation canceled — robot decelerating');
    },

    sendEStop(active: boolean) {
      state.eStopActive = active;
      const msg = rclnodejs.createMessageObject('std_msgs/msg/Bool') as any;
      msg.data = active; eStopPub.publish(msg);
      if (active) {
        // Primary E-Stop path is /agv/e_stop → odrive::on_e_stop, which sets
        // e_stop_active_=true sticky and sends CAN-level zero immediately.
        // odrive ignores ALL subsequent cmd_vel until cleared, so publishing
        // additional cmd_vel(0,0) here is redundant — and would create a race
        // with the chain. We rely on the dedicated topic + cancelNavGoal.
        ros.cancelNavGoal();
      }
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

    /**
     * Call /agv/rail_approach/execute with the new skip_coarse_approach
     * field. Returns the response message; throws on transport failure.
     * Used by /api/apriltags/:hw/align (skip=true, pure fine-servoing)
     * and by the post-Nav2 pendingRailApproach trigger (skip=false,
     * traditional coarse+fine flow). The earlier execFile-based call
     * was replaced with this client because it gives us access to the
     * response and lets us pass the new boolean field.
     */
    async callRailApproach(req: {
      tag_id: number; offset_x: number; offset_y: number;
      skip_coarse_approach: boolean;
    }): Promise<{ success: boolean; message: string }> {
      if (!railApproachClient.isServiceServerAvailable()) {
        return { success: false, message: 'rail_approach service not available' };
      }
      try {
        const r: any = await railApproachClient.sendRequestAsync({
          tag_id: req.tag_id,
          offset_x: req.offset_x,
          offset_y: req.offset_y,
          skip_coarse_approach: req.skip_coarse_approach,
        }, { timeout: 10000 });
        return { success: !!r.success, message: r.message || '' };
      } catch (e: any) {
        return { success: false, message: e?.message || 'rail_approach call failed' };
      }
    },

    async listRailStarts(): Promise<Array<{
      tag_id: number; x: number; y: number; approach_yaw: number; tag_size: number;
    }>> {
      if (!listRailStartsClient.isServiceServerAvailable()) return [];
      try {
        const r: any = await listRailStartsClient.sendRequestAsync({}, { timeout: 2000 });
        return Array.isArray(r?.rail_starts) ? r.rail_starts : [];
      } catch {
        return [];
      }
    },

    publishMapLoaded(name: string) {
      if (!name) return;
      const msg = rclnodejs.createMessageObject('std_msgs/msg/String') as any;
      msg.data = name;
      mapsLoadedPub.publish(msg);
      eventLog.emit('info', 'NAV', `Published maps/loaded: ${name} (auto-localize starting)`);
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

    // motor_state: native rclnodejs subscription. ODrive publishes at 10 Hz
    // (odrive_can_node.cpp:135 — 100 ms wall timer), so the old "low-frequency
    // publisher discovery bug" workaround (ros2 topic echo subprocess) no longer
    // applies. The subprocess approach was masking arm/disarm transitions
    // because stdout buffering delayed the first messages by several seconds,
    // so the UI required a page reload to see armed=true.
    node.createSubscription('std_msgs/msg/String', `/${NAMESPACE}/motor_state`, (msg: any) => {
      try {
        const parsed = JSON.parse(String(msg.data).replace(/\bnan\b/g, 'null'));
        if (parsed._keepalive) return;
        const prev = state.motorState.armed;
        state.motorState = parsed;
        if (prev !== state.motorState.armed) {
          eventLog.emit('info', 'DRIVE', state.motorState.armed ? 'Armed' : 'Disarmed');
          updateState();
        }
      } catch { /* ignore parse errors */ }
    });

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
    });

    // Auto-localization orchestrator state (from agv_localization_init).
    // Published as JSON string on /agv/localization/state with
    // { action, detail, map }. Purely informational for the dashboard LOC
    // pill — nav goal gating is NOT tied to this. The orchestrator is
    // authoritative: cuVSLAM → AprilTag → last-known-pose cascade runs
    // automatically at boot and on every map load.
    node.createSubscription('std_msgs/msg/String',
      `/${NAMESPACE}/localization/state`, (msg: any) => {
        try {
          const p = JSON.parse(msg.data);
          const prevAction = state.localization.action;
          state.localization = {
            action: p.action || 'UNKNOWN',
            detail: p.detail || '',
            map: p.map || '',
            updated: Date.now() / 1000,
          };
          if (prevAction !== state.localization.action) {
            const loc = state.localization.action;
            const sev = loc === 'FAILED' ? 'crit'
                      : loc === 'DEGRADED' ? 'warn' : 'info';
            eventLog.emit(sev as any, 'NAV',
              `Localization ${loc}: ${state.localization.detail}`);
          }
        } catch (e: any) {
          console.warn('[localization] parse error:', e?.message);
        }
      });

    // collision_monitor liveness watchdog. The collision_monitor publishes a
    // CollisionMonitorState message every cycle (driven by cmd_vel_smoothed).
    // We track the action and the wall-clock arrival time. A separate periodic
    // check (below, in the watchdog interval) marks the chain STALE if no
    // message has arrived in 2s. canSendGoal in sendNavGoal blocks new goals
    // when STALE/OFFLINE.
    node.createSubscription('nav2_msgs/msg/CollisionMonitorState',
      `/${NAMESPACE}/collision_monitor_state`, (msg: any) => {
        const actionMap: Record<number, string> = {
          0: 'OK',         // DO_NOTHING
          1: 'STOP',
          2: 'SLOWDOWN',
          3: 'APPROACH',
        };
        const prevAction = state.collisionMonitor.action;
        state.collisionMonitor = {
          action: actionMap[msg.action_type] ?? 'unknown',
          polygon: msg.polygon_name || '',
          updated: Date.now() / 1000,
        };
        // Log only on STOP/SLOWDOWN entry edges to avoid spam
        if (state.collisionMonitor.action === 'STOP' && prevAction !== 'STOP') {
          eventLog.emit('warn', 'SAFETY',
            `Collision STOP triggered (polygon: ${state.collisionMonitor.polygon})`);
        } else if (state.collisionMonitor.action === 'SLOWDOWN' && prevAction !== 'SLOWDOWN') {
          eventLog.emit('info', 'SAFETY',
            `Collision SLOWDOWN (polygon: ${state.collisionMonitor.polygon})`);
        }
      });

    // Sprint 1 Fase A3 (2026-04-24): throttle /scan callback to 5 Hz.
    // The publisher (pointcloud_to_laserscan) ships at ~30 Hz; each callback
    // does an O(n) trig loop over ranges to project points into world frame.
    // The WS broadcast is already 5 Hz, so processing 6× more frames per
    // second is wasted CPU on the Node event loop. Under simultaneous load
    // (Nav2 + perception + multiple WS clients), this saturation showed up
    // as WS jitter and delayed status broadcasts. /wheel_odom is NOT
    // throttled here: its callback is trivial (timestamp push + two
    // roundings) and the wheel_odom_hz reported to the dashboard would
    // misrepresent the underlying publisher rate.
    let lastScanProcessed = 0;
    const SCAN_THROTTLE_MS = 200;
    node.createSubscription('sensor_msgs/msg/LaserScan', `/${NAMESPACE}/scan`, (msg: any) => {
      const now = Date.now();
      if (now - lastScanProcessed < SCAN_THROTTLE_MS) return;
      lastScanProcessed = now;

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

    // ── Iter-37 Phase 2 state — HIL iter-33/34 stack (19/20) ──
    // Mode arbiter: FSM over {CORRIDOR_NAV, RAIL_APPROACH_*, RAIL_DRIVE,
    // RAIL_EXIT, BLOCKED_HANDOFF, TELEOP, IDLE}. 20 Hz.
    node.createSubscription('std_msgs/msg/String',
      `/${NAMESPACE}/mode/state`, (msg: any) => {
      try {
        const parsed = JSON.parse(msg.data);
        state.modeArbiterState = {
          mode: parsed.mode || 'unknown',
          source: parsed.source || 'NONE',
          zone: parsed.zone || 'unknown',
          operator_mode: parsed.operator_mode || 'nav',
          transitions: Number(parsed.transitions) || 0,
          updated: Date.now() / 1000,
        };
      } catch { /* parse tolerant */ }
    });

    // Zone detector: robot position + rail-aisle classification. 10 Hz.
    node.createSubscription('std_msgs/msg/String',
      `/${NAMESPACE}/zone/state`, (msg: any) => {
      try {
        const parsed = JSON.parse(msg.data);
        const numOrNull = (v: any) =>
          v === null || v === undefined || Number.isNaN(Number(v)) ? null : Number(v);
        state.zoneDetectorState = {
          zone: parsed.zone || 'unknown',
          section: parsed.section || '',
          aisle_y_center: numOrNull(parsed.aisle_y_center),
          rail_offset_lat: numOrNull(parsed.rail_offset_lat),
          rail_yaw_error: numOrNull(parsed.rail_yaw_error),
          approach_tag_id: Number(parsed.approach_tag_id ?? -1),
          confidence: Number(parsed.confidence) || 0,
          source: parsed.source || 'pose',
          updated: Date.now() / 1000,
        };
      } catch { /* parse tolerant */ }
    });

    // Rail driver: longitudinal progress + BLOCKED_* gates. 20 Hz.
    node.createSubscription('std_msgs/msg/String',
      `/${NAMESPACE}/rail_driver/state`, (msg: any) => {
      try {
        const parsed = JSON.parse(msg.data);
        state.railDriverState = {
          state: parsed.state || 'idle',
          linear_x: Number(parsed.linear_x) || 0,
          remaining_m: Number(parsed.remaining_m) || 0,
          in_rail_zone: Boolean(parsed.in_rail_zone),
          collision_stop: Boolean(parsed.collision_stop),
          updated: Date.now() / 1000,
        };
      } catch { /* parse tolerant */ }
    });

    // Latched current map name (transient_local) published by map_manager_node
    // on every successful maps/loaded event. Feeds the map header pill in the
    // dashboard so the operator always sees which map is active.
    const tlQosCurrentMap = new rclnodejs.QoS(
      rclnodejs.QoS.HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1,
      rclnodejs.QoS.ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
      rclnodejs.QoS.DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    node.createSubscription('std_msgs/msg/String',
      `/${NAMESPACE}/current_map`, { qos: tlQosCurrentMap }, (msg: any) => {
      const name = typeof msg.data === 'string' ? msg.data.trim() : '';
      state.currentMapName = name.length > 0 ? name : null;
    });

    console.log('[ROS] All subscriptions created');
  }

  // Create all subscriptions BEFORE spin — DDS requires subscriptions
  // to be registered before spinning so they are announced to the network.
  createAllSubscriptions();

  // Battery state — extract percentage for dashboard display + rolling sample
  // buffer for the time-to-empty (TTE) heuristic. The TTE itself is computed
  // in the 5 Hz broadcast loop, not here, to avoid recomputing the slope on
  // every BatteryState message.
  node.createSubscription('sensor_msgs/msg/BatteryState', `/${NAMESPACE}/battery`, (msg: any) => {
    const pct = typeof msg.percentage === 'number' ? Math.round(msg.percentage * 100) / 100 : -1;
    state.batteryPct = pct;
    if (pct >= 0) {
      state.batterySamples.push({ t_s: Date.now() / 1000, pct });
      if (state.batterySamples.length > 30) state.batterySamples.shift();
    }
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
    // Recompute battery time-to-empty heuristic at 1 Hz. The result feeds the
    // 5 Hz status broadcast; computing it more often is pure waste because the
    // input window is 30 samples / ≥60 s.
    state.batteryTteS = deriveBatteryTte(state.batterySamples);
    updateState();
  }, 1000);
  setInterval(() => { try { telemetryStore.recordSample({ timestamp: Date.now() / 1000,
    pose_x: state.robotPose.x, pose_y: state.robotPose.y, pose_theta: state.robotPose.theta,
    linear_vel: state.latestVelocity.linear, angular_vel: state.latestVelocity.angular,
    odom_hz: calcHz(state.odomTimes), slam_confidence: state.slamTracking,
    robot_state: state.robotState, battery_pct: state.batteryPct }); } catch {} }, 1000);
  setInterval(() => { try { telemetryStore.prune(); } catch {} }, 86400_000);
  setInterval(() => { // Watchdog: stale-command stop, defense in depth
    if (state.activeClients > 0 && !state.eStopActive && lastCmdTime > 0 && Date.now() / 1000 - lastCmdTime > 0.5) {
      const msg = rclnodejs.createMessageObject('geometry_msgs/msg/Twist') as any;
      cmdVelPub.publish(msg);
    }
  }, 100);

  // collision_monitor liveness watchdog. The state_topic is published only when
  // the chain processes a cmd_vel_smoothed; if the operator is idle (mode=teleop
  // joystick at zero), nav2 emits no cmd_vel and collision_monitor stays silent.
  // We mark STALE only after 2s without ANY message AND only when the chain is
  // expected to be live (mode=nav OR backend is sending cmd_vel actively).
  setInterval(() => {
    const cm = state.collisionMonitor;
    const age = Date.now() / 1000 - cm.updated;
    const wasOnline = cm.action !== 'STALE' && cm.action !== 'OFFLINE';
    if (cm.updated === 0) {
      // Never received — keep as OFFLINE
      return;
    }
    if (age > 2.0 && wasOnline) {
      state.collisionMonitor = { ...cm, action: 'STALE' };
      eventLog.emit('warn', 'SAFETY',
        `Collision monitor STALE — no state_topic for ${age.toFixed(1)}s`);
      updateState();
    }
  }, 500);

  // --- Build AppDeps ---
  const HOME_POINT_PATH = path.join(DATA_DIR, 'home_point.json');
  const ZONES_YAML_PATH = path.join(DATA_DIR, 'zones.yaml');
  const GREENHOUSE_GEOMETRY_YAML_PATH = path.join(DATA_DIR, 'greenhouse_geometry.yaml');
  // Load persisted home point at boot. If absent/malformed, state.homePoint
  // stays null — the dashboard's IR A BASE button stays disabled, no default.
  state.homePoint = readHomePoint(HOME_POINT_PATH);

  const deps: AppDeps = {
    state, ros, eventLog, telemetryStore, authManager, apriltagManager,
    config: {
      port: PORT, dataDir: DATA_DIR, namespace: NAMESPACE,
      mapsDir: MAPS_DIR, missionsFile: MISSIONS_FILE,
      homePointPath: HOME_POINT_PATH, zonesYamlPath: ZONES_YAML_PATH,
      greenhouseGeometryYamlPath: GREENHOUSE_GEOMETRY_YAML_PATH,
    },
    updateState,
    async setMode(mode: string): Promise<{ok: boolean; reason?: string}> {
      if (mode === state.currentMode) return {ok: true};
      // Transition-into-nav precondition: Nav2 lifecycle must be active.
      // Without this check the dashboard can claim 'nav' while Nav2 has
      // crashed or never reached active, and goal dispatch fails silently.
      // See docs/audit/2026-04-13-full-audit.md bug #3 and
      // specs/state_machine.yaml invariant mode_coherence.
      if (mode === 'nav') {
        try {
          if (!nav2IsActiveClient.isServiceServerAvailable()) {
            const reason = 'Nav2 lifecycle service not available — stack likely in mapping-first mode or still booting';
            eventLog.emit('warn', 'NAV', `Mode transition to nav rejected: ${reason}`);
            return {ok: false, reason};
          }
          const r = await nav2IsActiveClient.sendRequestAsync({}, {timeout: 1500});
          if (!r || r.success !== true) {
            const reason = (r && r.message) ? r.message : 'lifecycle_manager returned not-active';
            eventLog.emit('warn', 'NAV', `Mode transition to nav rejected: ${reason}`);
            return {ok: false, reason};
          }
        } catch (e: any) {
          const reason = `lifecycle_manager check failed: ${e?.message || e}`;
          eventLog.emit('warn', 'NAV', `Mode transition to nav rejected: ${reason}`);
          return {ok: false, reason};
        }
      }
      if (mode !== 'nav' && state.navState.active) ros.cancelNavGoal();
      eventLog.emit('info', 'SYSTEM', `Mode: ${state.currentMode} → ${mode}`);
      state.currentMode = mode;
      // Publish mode to ROS2 topic (interfaces.yaml compliance)
      const modeMsg = rclnodejs.createMessageObject('std_msgs/msg/String') as any;
      modeMsg.data = mode;
      modePub.publish(modeMsg);
      // Phase-2 operator_mode for mode_arbiter. Arbiter accepts nav|teleop|idle.
      // Mapping uses the operator joystick end-to-end → arbiter must treat it
      // as teleop so Nav2 is never selected as the cmd_vel source.
      const operatorModeMsg = rclnodejs.createMessageObject('std_msgs/msg/String') as any;
      operatorModeMsg.data = (mode === 'mapping') ? 'teleop' : mode;
      operatorModePub.publish(operatorModeMsg);
      updateState();
      return {ok: true};
    },
    executeMission,
  };

  // --- Express + Routes ---
  const app = express();
  app.use(express.json());

  // CORS for externally-hosted frontend (Sprint 1 Fase 1a). When the dashboard
  // runs on a different origin than this backend (laptop x86 serving the
  // build, dev box on a different host:port), the browser blocks fetch/WS
  // unless we explicitly allow that origin. Default empty = same-origin only.
  // Set AGV_UI_ALLOWED_ORIGINS to a comma-separated list of origins (each
  // origin is scheme://host[:port], e.g. http://laptop.lan:5173).
  const allowedOriginsRaw = process.env.AGV_UI_ALLOWED_ORIGINS || '';
  const allowedOrigins = new Set(
    allowedOriginsRaw.split(',').map(s => s.trim()).filter(Boolean)
  );
  if (allowedOrigins.size > 0) {
    app.use((req, res, next) => {
      const origin = req.headers.origin;
      if (origin && allowedOrigins.has(origin)) {
        res.setHeader('Access-Control-Allow-Origin', origin);
        res.setHeader('Vary', 'Origin');
        res.setHeader('Access-Control-Allow-Credentials', 'true');
        res.setHeader('Access-Control-Allow-Headers', 'Content-Type, Authorization');
        res.setHeader('Access-Control-Allow-Methods', 'GET, POST, PUT, DELETE, OPTIONS');
      }
      if (req.method === 'OPTIONS') {
        res.status(204).end();
        return;
      }
      next();
    });
    console.log(`[cors] Allowed origins: ${[...allowedOrigins].join(', ')}`);
  }

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

  // Boot-time maps/loaded: if the launch file provided AGV_BOOT_MAP_NAME (the
  // basename of the map arg), publish it to /agv/maps/loaded after a short
  // delay so the auto_init_orchestrator (which starts at t=7s per the launch)
  // is definitely up and subscribed before we publish. Without this, the
  // boot-time load via Nav2 map_server bypasses the orchestrator entirely.
  const bootMapName = process.env.AGV_BOOT_MAP_NAME || '';
  if (bootMapName) {
    setTimeout(() => {
      try {
        ros.publishMapLoaded(bootMapName);
        console.log(`[boot] Published maps/loaded for '${bootMapName}'`);
      } catch (e: any) {
        console.warn(`[boot] publishMapLoaded failed: ${e?.message}`);
      }
    }, 10000);
  }

  server.listen(PORT, '0.0.0.0', () => {
    console.log(`AGV Backend (TS) on http://0.0.0.0:${PORT}`);
    eventLog.emit('info', 'SYSTEM', 'Backend started');
    if (bootMapName) {
      eventLog.emit('info', 'MAPPING',
        `Boot map detected: '${bootMapName}' — will auto-localize in 10s`);
    }
    // Seed mode_arbiter with the default operator_mode at boot. Defaults
    // to 'teleop' (state.currentMode) so the arbiter does not spend the
    // first minute in 'nav' and fight the backend for /agv/cmd_vel.
    setTimeout(() => {
      const bootModeMsg = rclnodejs.createMessageObject('std_msgs/msg/String') as any;
      bootModeMsg.data = (state.currentMode === 'mapping') ? 'teleop' : state.currentMode;
      operatorModePub.publish(bootModeMsg);
    }, 2000);
  });
}

main().catch(console.error);
