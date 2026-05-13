/**
 * Health monitor — Sub-fase 1.1.c.
 *
 * Reads the static component list from `config/health_monitor.json` and
 * answers `/api/health/*` queries by sampling the existing `AppState`
 * (for topic_alive checks the backend already subscribes to), plus a
 * handful of system probes (`systemctl is-active`, `ip link`, chrony).
 *
 * The module is intentionally NOT a separate process — it lives inside
 * the backend, reads from `state` directly, and reuses the auth + WS
 * infrastructure.
 *
 * Status semantics
 * ----------------
 *   green     — last update within `deadline_ms`, or systemd unit
 *               active, or interface up.
 *   amber     — last update older than `deadline_ms` but < 3× that,
 *               or warning condition reported by the source.
 *   red       — never seen (and not in a documented idle state) OR
 *               last update > 3× deadline OR check failed.
 *   idle      — explicit zero-update state on a topic that only
 *               publishes during active nav (collision_monitor before
 *               any goal in flight). NOT a fault.
 *   unknown   — backend can't sample the source yet (boot grace, etc.).
 */

import * as fs from 'fs';
import * as path from 'path';
import { execFile } from 'child_process';
import type { AppState } from './app_deps';

// ── Types ────────────────────────────────────────────────────────────────

export type ComponentStatus = 'green' | 'amber' | 'red' | 'idle' | 'unknown';

export interface ComponentDef {
  id: string;
  name: string;
  type: 'sensor' | 'ros_node' | 'systemd' | 'network' | 'process';
  section: string;
  critical: boolean;
  check?: string;
  topic?: string;
  deadline_ms?: number;
  unit?: string;
  interface?: string;
  restart?: string | null;
  restart_help?: string;
}

export interface RestartTarget {
  command: string;
  args: string[];
  description: string;
  self_terminating?: boolean;
}

interface VerifierDef {
  id: string;
  name: string;
  script: string;
  blocking: boolean;
}

interface ConfigSchema {
  schema_version: number;
  components: ComponentDef[];
  verifiers: VerifierDef[];
  restart_targets?: Record<string, RestartTarget>;
}

export interface ComponentSample {
  id: string;
  name: string;
  section: string;
  critical: boolean;
  status: ComponentStatus;
  detail: string;
  last_seen_ms_ago: number | null;
}

export interface HealthEvent {
  ts: number;
  type: 'component_status' | 'verifier_run';
  id: string;
  payload: Record<string, any>;
}

// ── Config loading ───────────────────────────────────────────────────────

let CONFIG: ConfigSchema | null = null;

function loadConfig(): ConfigSchema {
  if (CONFIG) return CONFIG;
  const p = path.resolve(__dirname, '..', 'config', 'health_monitor.json');
  CONFIG = JSON.parse(fs.readFileSync(p, 'utf-8'));
  return CONFIG!;
}

export function getComponents(): ComponentDef[] {
  return loadConfig().components;
}

export function getVerifiers(): VerifierDef[] {
  return loadConfig().verifiers;
}

export function getRestartTargets(): Record<string, RestartTarget> {
  return loadConfig().restart_targets ?? {};
}

// ── State-derived freshness tracker ──────────────────────────────────────
//
// The backend already subscribes to a number of AGV topics and stamps
// `state.X.updated` / `state.lastImuTime` / `state.odomTimes[-1]`. The
// health monitor reads those instead of opening duplicate subscriptions.
// `topicFreshnessMap` resolves each YAML-declared topic name to the
// `state` field that carries its latest timestamp.
//
// Anything not in this map gets a "no health probe wired yet" detail
// and renders `unknown` — surfaces a spec/code mismatch instead of
// silently rendering green.

function topicLastSeenMsAgo(state: AppState, topic: string): number | null {
  const nowS = Date.now() / 1000;
  switch (topic) {
    case '/agv/wheel_odom': {
      const last = state.odomTimes.length ? state.odomTimes[state.odomTimes.length - 1] : 0;
      return last > 0 ? (nowS - last) * 1000 : null;
    }
    case '/agv/imu/filtered':
      return state.lastImuTime > 0 ? (nowS - state.lastImuTime) * 1000 : null;
    case '/agv/collision_monitor_state':
      return state.collisionMonitor.updated > 0
        ? (nowS - state.collisionMonitor.updated) * 1000 : null;
    case '/agv/mode/state':
      return state.modeArbiterState.updated > 0
        ? (nowS - state.modeArbiterState.updated) * 1000 : null;
    case '/agv/localization/state':
      return state.localization.updated > 0
        ? (nowS - state.localization.updated) * 1000 : null;
    case '/agv/scan':
      return state.lastScanTime > 0 ? (nowS - state.lastScanTime) * 1000 : null;
    case '/agv/odometry/global':
      return state.lastGlobalOdomTime > 0 ? (nowS - state.lastGlobalOdomTime) * 1000 : null;
    case '/agv/odometry/local':
      return state.lastLocalOdomTime > 0 ? (nowS - state.lastLocalOdomTime) * 1000 : null;
    case '/visual_slam/tracking/odometry':
      return state.lastVslamTime > 0 ? (nowS - state.lastVslamTime) * 1000 : null;
    case '/agv/marker_pose':
      return state.lastMarkerPoseTime > 0 ? (nowS - state.lastMarkerPoseTime) * 1000 : null;
    case '/agv/safety/status':
      return state.lastSafetyStatusTime > 0 ? (nowS - state.lastSafetyStatusTime) * 1000 : null;
    default:
      return null;
  }
}

// ── Per-check evaluators ─────────────────────────────────────────────────

function statusFromAge(ageMs: number | null, deadlineMs: number): {
  status: ComponentStatus; detail: string; last_seen_ms_ago: number | null;
} {
  if (ageMs === null) {
    return { status: 'unknown', detail: 'no health probe wired', last_seen_ms_ago: null };
  }
  if (ageMs <= deadlineMs) {
    return { status: 'green', detail: `${(ageMs / 1000).toFixed(2)}s ago`, last_seen_ms_ago: Math.round(ageMs) };
  }
  if (ageMs <= deadlineMs * 3) {
    return { status: 'amber', detail: `late: ${(ageMs / 1000).toFixed(1)}s ago (deadline ${(deadlineMs / 1000).toFixed(1)}s)`, last_seen_ms_ago: Math.round(ageMs) };
  }
  return { status: 'red', detail: `silent for ${(ageMs / 1000).toFixed(1)}s`, last_seen_ms_ago: Math.round(ageMs) };
}

async function checkSystemd(unit: string): Promise<{ status: ComponentStatus; detail: string }> {
  return new Promise(resolve => {
    execFile('systemctl', ['is-active', unit], { timeout: 2000 }, (err, stdout) => {
      const s = (stdout || '').trim();
      if (s === 'active') resolve({ status: 'green', detail: 'active' });
      else if (s === 'inactive' || s === 'failed' || s === 'unknown') resolve({ status: 'red', detail: s });
      else resolve({ status: 'amber', detail: s || `is-active: ${err?.message ?? 'unknown'}` });
    });
  });
}

async function checkInterface(iface: string): Promise<{ status: ComponentStatus; detail: string }> {
  return new Promise(resolve => {
    fs.readFile(`/sys/class/net/${iface}/operstate`, 'utf-8', (err, data) => {
      if (err) return resolve({ status: 'red', detail: `interface ${iface} not present` });
      const s = data.trim();
      if (s === 'up') resolve({ status: 'green', detail: 'up' });
      else if (s === 'unknown') resolve({ status: 'amber', detail: 'unknown (loopback/bridge)' });
      else resolve({ status: 'red', detail: s });
    });
  });
}

async function checkCanLink(iface: string): Promise<{ status: ComponentStatus; detail: string }> {
  return new Promise(resolve => {
    execFile('ip', ['-details', 'link', 'show', iface], { timeout: 2000 }, (err, stdout) => {
      if (err) return resolve({ status: 'red', detail: `link not present: ${err.message}` });
      const out = stdout || '';
      const upMatch = /state\s+([A-Z\-]+)/.exec(out);
      const errMatch = /berr-counter\s+tx\s+(\d+)\s+rx\s+(\d+)/.exec(out);
      const state = upMatch ? upMatch[1] : 'UNKNOWN';
      const errs = errMatch ? `tx_err=${errMatch[1]}, rx_err=${errMatch[2]}` : '';
      if (state === 'UP') return resolve({ status: 'green', detail: `state=UP ${errs}`.trim() });
      return resolve({ status: 'red', detail: `state=${state} ${errs}`.trim() });
    });
  });
}

async function checkChrony(): Promise<{ status: ComponentStatus; detail: string }> {
  return new Promise(resolve => {
    execFile('chronyc', ['tracking'], { timeout: 2000 }, (err, stdout) => {
      if (err) return resolve({ status: 'amber', detail: `chronyc unavailable: ${err.message}` });
      const out = stdout || '';
      const m = /System time\s*:\s*([0-9.e+\-]+)/.exec(out);
      const refMatch = /Reference ID\s*:\s*([0-9A-F]+)/.exec(out);
      if (m && refMatch && refMatch[1] !== '00000000') {
        const offsetS = parseFloat(m[1]);
        const offsetMs = Math.round(offsetS * 1000);
        if (Math.abs(offsetMs) < 100) return resolve({ status: 'green', detail: `synced, offset=${offsetMs}ms` });
        if (Math.abs(offsetMs) < 1000) return resolve({ status: 'amber', detail: `synced, offset=${offsetMs}ms` });
        return resolve({ status: 'red', detail: `unsync, offset=${offsetMs}ms` });
      }
      return resolve({ status: 'red', detail: 'no reference (unsynced)' });
    });
  });
}

// ── Main evaluation ──────────────────────────────────────────────────────

export async function evaluateComponent(comp: ComponentDef, state: AppState): Promise<ComponentSample> {
  let s: { status: ComponentStatus; detail: string; last_seen_ms_ago?: number | null };

  if (comp.check === 'topic_alive' && comp.topic) {
    const ageMs = topicLastSeenMsAgo(state, comp.topic);
    s = statusFromAge(ageMs, comp.deadline_ms ?? 1000);
  } else if (comp.type === 'systemd' && comp.unit) {
    s = await checkSystemd(comp.unit);
  } else if (comp.type === 'network' && comp.interface) {
    s = await checkInterface(comp.interface);
  } else if (comp.check === 'can_link_up' && comp.interface) {
    s = await checkCanLink(comp.interface);
  } else if (comp.check === 'chrony_synced') {
    s = await checkChrony();
  } else {
    s = { status: 'unknown', detail: 'no check wired' };
  }
  return {
    id: comp.id,
    name: comp.name,
    section: comp.section,
    critical: comp.critical,
    status: s.status,
    detail: s.detail,
    last_seen_ms_ago: s.last_seen_ms_ago ?? null,
  };
}

export async function evaluateAll(state: AppState): Promise<ComponentSample[]> {
  return Promise.all(getComponents().map(c => evaluateComponent(c, state)));
}

// ── Event persistence ────────────────────────────────────────────────────

let eventsDir: string | null = null;
let currentDay: string | null = null;
let currentLogStream: fs.WriteStream | null = null;

function initEventsLog(dataDir: string) {
  eventsDir = path.join(dataDir, 'events');
  fs.mkdirSync(eventsDir, { recursive: true });
}

function rotateIfNeeded() {
  const day = new Date().toISOString().slice(0, 10);
  if (day === currentDay && currentLogStream) return;
  if (currentLogStream) currentLogStream.end();
  currentDay = day;
  if (!eventsDir) return;
  currentLogStream = fs.createWriteStream(
    path.join(eventsDir, `health-${day}.jsonl`),
    { flags: 'a' },
  );
  // Best-effort: cleanup files older than 7 days
  try {
    const cutoffMs = Date.now() - 7 * 86400_000;
    for (const f of fs.readdirSync(eventsDir)) {
      if (!f.startsWith('health-') || !f.endsWith('.jsonl')) continue;
      const fp = path.join(eventsDir, f);
      const st = fs.statSync(fp);
      if (st.mtimeMs < cutoffMs) fs.unlinkSync(fp);
    }
  } catch { /* ignore */ }
}

export function recordEvent(dataDir: string, event: HealthEvent) {
  if (!eventsDir) initEventsLog(dataDir);
  rotateIfNeeded();
  if (currentLogStream) {
    currentLogStream.write(JSON.stringify(event) + '\n');
  }
}

export function readRecentEvents(dataDir: string, maxLines: number = 100): HealthEvent[] {
  const dir = path.join(dataDir, 'events');
  if (!fs.existsSync(dir)) return [];
  const files = fs.readdirSync(dir)
    .filter(f => f.startsWith('health-') && f.endsWith('.jsonl'))
    .sort()
    .reverse();
  const out: HealthEvent[] = [];
  for (const f of files) {
    const lines = fs.readFileSync(path.join(dir, f), 'utf-8').split('\n').reverse();
    for (const line of lines) {
      if (!line.trim()) continue;
      try {
        out.push(JSON.parse(line));
        if (out.length >= maxLines) return out.reverse();
      } catch { /* skip malformed */ }
    }
  }
  return out.reverse();
}
