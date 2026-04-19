/**
 * VDA 5050 v2.0 message types (subset for MVP).
 *
 * Full spec: https://github.com/VDA5050/VDA5050/blob/main/VDA5050_EN.md
 * We implement: state, visualization, connection, order, instantActions.
 * Deferred: factsheet.
 */

// ---------------------------------------------------------------------------
// Common
// ---------------------------------------------------------------------------

export interface HeaderId {
  headerId: number;
  timestamp: string;     // ISO 8601
  version: string;       // "2.0.0"
  manufacturer: string;
  serialNumber: string;
}

export interface AgvPosition {
  x: number;
  y: number;
  theta: number;
  mapId: string;
  positionInitialized: boolean;
  localizationScore: number;      // 0.0 - 1.0
  deviationRange?: number;
}

export interface Velocity {
  vx: number;
  vy: number;
  omega: number;
}

export interface BatteryState {
  batteryCharge: number;          // 0-100%
  batteryVoltage?: number;
  batteryHealth?: number;
  charging: boolean;
  reach?: number;                 // estimated range in meters
}

export type OperatingMode =
  | 'AUTOMATIC'
  | 'SEMIAUTOMATIC'
  | 'MANUAL'
  | 'SERVICE'
  | 'TEACHIN';

export type EStopType = 'AUTOACK' | 'MANUAL' | 'REMOTE' | 'NONE';

export interface SafetyState {
  eStop: EStopType;
  fieldViolation: boolean;
}

// ---------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------

export interface AgvError {
  errorType: string;
  errorLevel: 'WARNING' | 'FATAL';
  errorDescription?: string;
  errorReferences?: Array<{ referenceKey: string; referenceValue: string }>;
}

// ---------------------------------------------------------------------------
// State (AGV → Master)
// ---------------------------------------------------------------------------

export interface NodeState {
  nodeId: string;
  sequenceId: number;
  nodeDescription?: string;
  released: boolean;
  nodePosition?: { x: number; y: number; theta?: number; mapId: string };
}

export interface EdgeState {
  edgeId: string;
  sequenceId: number;
  edgeDescription?: string;
  released: boolean;
}

export interface ActionState {
  actionId: string;
  actionType: string;
  actionStatus: 'WAITING' | 'INITIALIZING' | 'RUNNING' | 'PAUSED' | 'FINISHED' | 'FAILED';
  actionDescription?: string;
  resultDescription?: string;
}

export interface AgvState extends HeaderId {
  orderId: string;
  orderUpdateId: number;
  lastNodeId: string;
  lastNodeSequenceId: number;
  driving: boolean;
  paused: boolean;
  newBaseRequest: boolean;
  distanceSinceLastNode: number;
  operatingMode: OperatingMode;
  nodeStates: NodeState[];
  edgeStates: EdgeState[];
  actionStates: ActionState[];
  batteryState: BatteryState;
  errors: AgvError[];
  safetyState: SafetyState;
  agvPosition?: AgvPosition;
  velocity?: Velocity;
}

// ---------------------------------------------------------------------------
// Visualization (AGV → Viz, high frequency)
// ---------------------------------------------------------------------------

export interface Visualization extends HeaderId {
  agvPosition: AgvPosition;
  velocity: Velocity;
}

// ---------------------------------------------------------------------------
// Connection (AGV → Master, heartbeat + LWT)
// ---------------------------------------------------------------------------

export interface Connection extends HeaderId {
  connectionState: 'ONLINE' | 'OFFLINE' | 'CONNECTIONBROKEN';
}

// ---------------------------------------------------------------------------
// Order (Master → AGV)
// ---------------------------------------------------------------------------

export interface OrderNode {
  nodeId: string;
  sequenceId: number;
  released: boolean;
  nodePosition: { x: number; y: number; theta?: number; mapId: string; allowedDeviationXY?: number };
  actions: OrderAction[];
}

export interface OrderEdge {
  edgeId: string;
  sequenceId: number;
  released: boolean;
  startNodeId: string;
  endNodeId: string;
  maxSpeed?: number;
  actions: OrderAction[];
}

export interface OrderAction {
  actionId: string;
  actionType: string;
  blockingType: 'NONE' | 'SOFT' | 'HARD';
  actionDescription?: string;
  actionParameters?: Array<{ key: string; value: string | number | boolean }>;
}

export interface Order extends HeaderId {
  orderId: string;
  orderUpdateId: number;
  nodes: OrderNode[];
  edges: OrderEdge[];
}

// ---------------------------------------------------------------------------
// Instant Actions (Master → AGV)
// ---------------------------------------------------------------------------

export interface InstantActions extends HeaderId {
  instantActions: OrderAction[];
}
