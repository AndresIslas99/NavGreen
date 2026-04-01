// ---------------------------------------------------------------------------
// Robot state (authoritative from backend)
// ---------------------------------------------------------------------------

export type RobotState =
  | 'offline'
  | 'idle'
  | 'ready'
  | 'mapping'
  | 'navigating'
  | 'executing_mission'
  | 'blocked'
  | 'e_stop'
  | 'fault'

export type ModeRail = 'operate' | 'map' | 'missions' | 'recovery'

// ---------------------------------------------------------------------------
// Allowed actions (computed by backend)
// ---------------------------------------------------------------------------

export interface AllowedActions {
  canTeleop: boolean
  canStartMapping: boolean
  canStopMapping: boolean
  canSendGoal: boolean
  canExecuteMission: boolean
  canSaveMap: boolean
  canLoadMap: boolean
  canMotorEnable: boolean
  canPauseMission: boolean
  canCancelNav: boolean
}

// ---------------------------------------------------------------------------
// Event log
// ---------------------------------------------------------------------------

export type Severity = 'info' | 'warn' | 'crit'
export type Subsystem = 'DRIVE' | 'NAV' | 'SLAM' | 'SAFETY' | 'MAPPING' | 'MISSION' | 'SYSTEM'

export interface LogEntry {
  timestamp: number
  severity: Severity
  subsystem: Subsystem
  text: string
}

// ---------------------------------------------------------------------------
// Subsystem health
// ---------------------------------------------------------------------------

export interface SubsystemHealth {
  status: 'ok' | 'warn' | 'error' | 'unknown'
  detail: string
  updated: number
  action?: string
}

export type HealthMap = Record<string, SubsystemHealth>

// ---------------------------------------------------------------------------
// Mission progress
// ---------------------------------------------------------------------------

export interface MissionProgress {
  mission_id: string
  mission_name: string
  current_node: number
  total_nodes: number
  status: 'running' | 'paused' | 'completed' | 'failed' | 'canceled'
}

// ---------------------------------------------------------------------------
// WebSocket payloads
// ---------------------------------------------------------------------------

export interface RobotStatus {
  robot_state: RobotState
  allowed_actions: AllowedActions
  wheel_odom_hz: number
  velocity: { linear: number; angular: number }
  e_stop: boolean
  motors_armed: boolean
  left_state: number
  right_state: number
  motor_errors: boolean
  drive_online: boolean
  slam_tracking: string
  recording: boolean
  clients: number
  mode: 'teleop' | 'mapping' | 'nav'
  pose: { x: number; y: number; theta: number }
  nav_state: { active: boolean; distance_remaining: number; status: string }
  health: HealthMap
  mission_progress: MissionProgress | null
}

export interface MapInfo {
  name: string
  modified: number
}

export interface MapMeta {
  resolution: number
  origin_x: number
  origin_y: number
  width: number
  height: number
}

export interface MapUpdate extends MapMeta {
  png_base64: string
}

export interface PathPoint {
  x: number
  y: number
}

export interface Waypoint {
  x: number
  y: number
  theta: number
}

export interface MissionNode {
  id: string
  type: string
  x: number
  y: number
  theta: number
  action: 'none' | 'pause' | 'signal'
  pause_sec?: number
}

export interface MissionEdge {
  from: string
  to: string
  max_speed?: number
}

export interface Mission {
  id: string
  name: string
  nodes: MissionNode[]
  edges: MissionEdge[]
  waypoints?: Waypoint[]  // legacy compat
  repeat: boolean
  created: number
}

export type WsMessage =
  | ({ type: 'status' } & RobotStatus)
  | { type: 'path'; points: PathPoint[] }
  | { type: 'scan'; points: PathPoint[] }
  | ({ type: 'map_update' } & MapUpdate)
  | ({ type: 'acc_map' } & MapUpdate)
  | ({ type: 'event' } & LogEntry)
  | { type: 'recording_result'; success: boolean; message: string }
