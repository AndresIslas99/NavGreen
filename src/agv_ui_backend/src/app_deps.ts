/**
 * Shared dependencies interface — passed to all route modules and WS handlers.
 * Prevents circular imports and centralizes access to shared state.
 */

import type { RobotState, MotorState, NavState, MissionProgress } from './state_machine';
import type { EventLog } from './event_log';
import type { TelemetryStore } from './telemetry_store';
import type { AuthManager } from './auth';
import type { AprilTagManager } from './apriltag_manager';

export interface RobotPose {
  x: number;
  y: number;
  theta: number;
}

/** Occupancy-grid metadata attached to map PNGs sent to the dashboard. */
export interface MapMeta {
  resolution: number;   // m/cell
  origin_x: number;     // map frame, meters
  origin_y: number;     // map frame, meters
  width: number;        // cells
  height: number;       // cells
}

export interface HealthEntry {
  status: string;       // ok | warn | error | unknown
  detail: string;
  updated: number;      // wall-clock seconds
  action?: string;      // operator hint shown by the dashboard
}

export interface AppState {
  eStopActive: boolean;
  currentMode: string;
  robotPose: RobotPose;
  latestVelocity: { linear: number; angular: number };
  motorState: MotorState;
  navState: NavState;
  missionProgress: MissionProgress | null;
  robotState: RobotState;
  slamTracking: string;
  scanPoints: Array<{ x: number; y: number }>;
  navPathPoints: Array<{ x: number; y: number }>;
  odomTimes: number[];
  activeClients: number;
  recordingActive: boolean;
  missionCancel: boolean;
  missionPause: boolean;
  batteryPct: number;
  lastImuTime: number;
  mapPng: Buffer | null;
  mapMeta: MapMeta | null;
  mapChanged: boolean;
  mapVersion: number;
  liveMapPng: Buffer | null;
  liveMapMeta: MapMeta | null;
  liveMapVersion: number;
  health: Record<string, HealthEntry>;
  // Set when a nav goal is sent to a rail_start tag — backend triggers
  // rail_approach/execute when Nav2 succeeds.
  pendingRailApproach: { hardware_id: number; defined_id: number } | null;
  // Latest rail_approach state for waypoint action gating
  railApproachState: string;
  // Iter-37 Phase 2 state exposure (from HIL iter-33/34 19/20 stack).
  // These mirror the /agv/{mode,zone,rail_driver}/state topics so the
  // dashboard can render the arbiter FSM, the zone classifier, and the
  // rail_driver status without subscribing to every topic from the SPA.
  // All three are JSON strings published at 10–20 Hz by Phase 2 nodes
  // (mode_arbiter_node, zone_detector_node, rail_driver_node).
  modeArbiterState: {
    mode: string;            // corridor_nav | rail_approach_pend | ... | blocked_handoff
    source: string;          // NAV | APPROACH | RAIL | NONE
    zone: string;            // echoed zone label the arbiter is seeing
    operator_mode: string;   // nav | teleop | idle
    transitions: number;     // monotonic count since arbiter start
    updated: number;         // wall-clock seconds of last message
  };
  zoneDetectorState: {
    zone: string;            // rail_aisle_* | rail_approach_* | gap | corridor_* | unknown
    section: string;         // REAR | GAP | FRONT | OUTSIDE
    aisle_y_center: number | null;
    rail_offset_lat: number | null;
    rail_yaw_error: number | null;
    approach_tag_id: number; // -1 when not in an approach strip
    confidence: number;      // 0..1
    source: string;          // pose (phase 1)
    updated: number;
  };
  railDriverState: {
    state: string;           // idle | driving | reached | blocked_wait | blocked_misaligned | blocked_lateral | canceled
    linear_x: number;        // last commanded body-x velocity (m/s)
    remaining_m: number;     // body-frame distance to goal
    in_rail_zone: boolean;
    collision_stop: boolean;
    updated: number;
  };
  // Liveness + state of the collision_monitor safety chain. updated is the
  // wall-clock seconds when the last state_topic message was received; if it
  // ages > 2s the chain is considered STALE and nav goals are rejected.
  // action: OK | SLOWDOWN | STOP | STALE | OFFLINE
  collisionMonitor: {
    action: string;
    polygon: string;
    updated: number;
  };
  // Auto-localization orchestrator state from agv_localization_init.
  // Informational only — nav goals are NOT gated on this. The orchestrator
  // is the authoritative source of truth for localization; this field
  // mirrors its reported state for dashboard display.
  // action: INITIALIZING | LOCALIZED | DEGRADED | FAILED | UNKNOWN
  localization: {
    action: string;
    detail: string;
    map: string;
    updated: number;
  };
  // Name of the map currently loaded into Nav2 (no extension). Sourced from
  // the latched /{ns}/current_map topic published by map_manager_node. null
  // when no map is loaded (mapping-first mode) or before the latched value
  // has been received at dashboard boot.
  currentMapName: string | null;
}

/**
 * Who is dispatching a nav goal. While a mission is running, only the
 * mission executor may send goals — operator goals (REST/WS) are rejected
 * so they cannot corrupt the mission's shared navState/goal handle.
 */
export type NavGoalSource = 'operator' | 'mission';

export interface RosBridge {
  sendCmdVel(linear: number, angular: number): void;
  sendNavGoal(x: number, y: number, theta: number, source?: NavGoalSource): { success: boolean; message: string };
  cancelNavGoal(): void;
  sendEStop(active: boolean): void;
  sendMotorEnable(active: boolean): void;
  callTriggerService(client: any, name: string): Promise<{ success: boolean; message: string }>;
  // Calls /agv/rail_approach/execute with the skip_coarse_approach flag.
  // skip_coarse_approach=true bypasses Nav2 entirely and goes directly
  // to fine-servoing — used for the /api/apriltags/:hw/align endpoint
  // when the operator only needs precision alignment with a visible tag.
  callRailApproach(req: {
    tag_id: number; offset_x: number; offset_y: number;
    skip_coarse_approach: boolean;
  }): Promise<{ success: boolean; message: string }>;
  // Fires the /agv/maps/loaded event so auto_init_orchestrator starts its
  // relocalization sequence. Must be called after any successful Nav2
  // load_map. The name is the map stem (without path or .yaml extension).
  publishMapLoaded(name: string): void;
  startRecClient: any;
  stopRecClient: any;
  loadMapClient: any;
  saveMap(name: string, mapDir: string, mapTopic: string): Promise<{ success: boolean; message: string; name?: string }>;
}

export interface AppDeps {
  state: AppState;
  ros: RosBridge;
  eventLog: EventLog;
  telemetryStore: TelemetryStore;
  authManager: AuthManager;
  apriltagManager: AprilTagManager;
  config: {
    port: number;
    dataDir: string;
    namespace: string;
    mapsDir: string;
    missionsFile: string;
  };
  // Functions that modify state
  updateState(): void;
  setMode(mode: string): Promise<{ok: boolean; reason?: string}>;
  executeMission(missionId: string): Promise<{ success: boolean; message?: string; nodes?: number }>;
}
