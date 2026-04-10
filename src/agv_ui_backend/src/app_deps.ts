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
  navPathChanged: boolean;
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
}

export interface RosBridge {
  sendCmdVel(linear: number, angular: number): void;
  sendNavGoal(x: number, y: number, theta: number): { success: boolean; message: string };
  cancelNavGoal(): void;
  sendEStop(active: boolean): void;
  sendMotorEnable(active: boolean): void;
  callTriggerService(client: any, name: string): Promise<{ success: boolean; message: string }>;
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
