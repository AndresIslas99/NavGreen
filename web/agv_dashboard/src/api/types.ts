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

export type ModeRail = 'operate' | 'map' | 'missions' | 'recovery' | 'analytics' | 'apriltags'

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
  // Name of the map currently loaded into Nav2 (no extension). null when the
  // robot is in mapping-first mode or has never been told which map is active.
  current_map_name: string | null
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
  battery_pct: number | null
  nav_state: { active: boolean; distance_remaining: number; status: string }
  health: HealthMap
  mapping_coverage: number
  mission_progress: MissionProgress | null
  // Live state of the safety chain. action: OK | SLOWDOWN | STOP | STALE | OFFLINE
  // age_s: seconds since last collision_monitor_state message (null if never).
  collision_monitor: {
    action: string
    polygon: string
    age_s: number | null
  }
  // Auto-localization orchestrator state (from agv_localization_init).
  // action: INITIALIZING | LOCALIZED | DEGRADED | FAILED | UNKNOWN
  localization: {
    action: string
    detail: string
    map: string
  }
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

export type WaypointAction = 'none' | 'pause' | 'signal' | 'start_recording' | 'stop_recording'

export interface MissionNode {
  id: string
  type: string
  x: number
  y: number
  theta: number
  action: WaypointAction
  pause_sec?: number
  apriltag_id?: number  // Optional: snap to a defined AprilTag
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
  | { type: 'apriltag_pending'; hardware_id: number; first_seen: number }

// ---------------------------------------------------------------------------
// AprilTags
// ---------------------------------------------------------------------------

export type TagType = 'wall' | 'rail_start'

export interface DefinedTag {
  id: number
  label: string
  description: string
  type: TagType
  x: number
  y: number
  z: number
  yaw: number
  created_at: number
}

export interface AprilTagState {
  defined_tags: DefinedTag[]
  hardware_assignments: Record<string, number>
  pending_detections: Array<{ hardware_id: number; first_seen: number }>
}

// ---------------------------------------------------------------------------
// Analytics
// ---------------------------------------------------------------------------

export interface AnalyticsSummary {
  uptime_pct: number
  distance_m: number
  mission_success_rate: number
  mission_count: number
  avg_mission_duration_s: number
  avg_odom_hz: number
  min_odom_hz: number
  max_odom_hz: number
  slam_good_pct: number
}

export interface TimeseriesPoint {
  timestamp: number
  value: number
}

// ---------------------------------------------------------------------------
// Auth
// ---------------------------------------------------------------------------

export interface AuthStatus {
  enabled: boolean
}

export interface AuthSession {
  token: string
  username: string
  role: 'viewer' | 'operator' | 'engineer'
}

// ---------------------------------------------------------------------------
// Traffic zones
// ---------------------------------------------------------------------------

export interface TrafficZone {
  id: string
  type: 'exclusion' | 'one_way' | 'yield'
  polygon: Array<{ x: number; y: number }>
  direction?: number
  directionTolerance?: number
  maxRobots: number
  priority?: number
}

export interface ZoneOccupancy {
  zoneId: string
  robotIds: string[]
  waitingRobots: string[]
}

// ---------------------------------------------------------------------------
// Mission run (analytics)
// ---------------------------------------------------------------------------

export interface MissionRun {
  id: string
  mission_id: string
  mission_name: string
  started: number
  ended: number | null
  status: string
  nodes_completed: number
  total_nodes: number
}
