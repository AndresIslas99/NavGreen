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
  mapMeta: any;
  mapChanged: boolean;
  mapVersion: number;
  liveMapPng: Buffer | null;
  liveMapMeta: any;
  liveMapVersion: number;
  health: Record<string, any>;
  // Set when a nav goal is sent to a rail_start tag — backend triggers
  // rail_approach/execute when Nav2 succeeds.
  pendingRailApproach: { hardware_id: number; defined_id: number } | null;
  // Latest rail_approach state for waypoint action gating
  railApproachState: string;
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
}

export interface RosBridge {
  sendCmdVel(linear: number, angular: number): void;
  sendNavGoal(x: number, y: number, theta: number): { success: boolean; message: string };
  cancelNavGoal(): void;
  sendEStop(active: boolean): void;
  sendMotorEnable(active: boolean): void;
  callTriggerService(client: any, name: string): Promise<{ success: boolean; message: string }>;
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
  setMode(mode: string): void;
  executeMission(missionId: string): Promise<{ success: boolean; message?: string; nodes?: number }>;
}
