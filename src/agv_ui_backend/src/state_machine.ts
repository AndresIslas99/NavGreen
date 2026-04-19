// Backend-authoritative robot state machine
// Mirrors the Python implementation's state derivation and allowed actions

export type RobotState =
  | 'offline' | 'idle' | 'ready' | 'mapping' | 'navigating'
  | 'executing_mission' | 'blocked' | 'e_stop' | 'fault';

export interface AllowedActions {
  canTeleop: boolean;
  canStartMapping: boolean;
  canStopMapping: boolean;
  canSendGoal: boolean;
  canExecuteMission: boolean;
  canSaveMap: boolean;
  canLoadMap: boolean;
  canMotorEnable: boolean;
  canPauseMission: boolean;
  canCancelNav: boolean;
}

export interface MotorState {
  armed: boolean;
  left_state: number;
  right_state: number;
  left_errors: number;
  right_errors: number;
}

export interface NavState {
  active: boolean;
  distance_remaining: number;
  status: string;
}

export interface MissionProgress {
  mission_id: string;
  mission_name: string;
  current_node: number;
  total_nodes: number;
  status: string;
}

export interface SubsystemHealth {
  status: 'ok' | 'warn' | 'error' | 'unknown';
  detail: string;
  updated: number;
  action?: string;
}

export function deriveState(
  eStop: boolean,
  motorState: MotorState,
  odomHz: number,
  mode: string,
  missionProgress: MissionProgress | null,
  navState: NavState,
): RobotState {
  if (eStop) return 'e_stop';
  if (motorState.left_errors !== 0 || motorState.right_errors !== 0) return 'fault';
  if (odomHz < 1.0 && !motorState.armed) return 'idle';
  if (mode === 'mapping') return 'mapping';
  if (missionProgress && missionProgress.status === 'running') return 'executing_mission';
  if (navState.active && navState.status === 'active') return 'navigating';
  if (motorState.armed) return 'ready';
  return 'idle';
}

export function allowedActions(state: RobotState): AllowedActions {
  return {
    canTeleop: state === 'ready' || state === 'mapping',
    canStartMapping: state === 'ready' || state === 'idle',
    canStopMapping: state === 'mapping',
    canSendGoal: state === 'ready' || state === 'navigating',
    canExecuteMission: state === 'ready',
    canSaveMap: state !== 'offline' && state !== 'fault',
    canLoadMap: state === 'idle' || state === 'ready',
    canMotorEnable: state === 'idle' || state === 'ready' || state === 'e_stop',
    canPauseMission: state === 'executing_mission',
    canCancelNav: state === 'navigating' || state === 'executing_mission',
  };
}
