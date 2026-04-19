/**
 * Persistent telemetry storage using SQLite (WAL mode).
 *
 * Stores:
 * - telemetry_samples: 1/sec pose, velocity, state, SLAM confidence
 * - mission_runs: mission execution history
 * - events: migrated from JSONL for unified querying
 *
 * ~3.5 MB/day at 1Hz write rate. 30-day default retention.
 */

import Database from 'better-sqlite3';
import * as path from 'path';

export interface TelemetrySample {
  timestamp: number;
  pose_x: number;
  pose_y: number;
  pose_theta: number;
  linear_vel: number;
  angular_vel: number;
  odom_hz: number;
  slam_confidence: string;
  robot_state: string;
  battery_pct: number;
}

export interface MissionRun {
  id: string;
  mission_id: string;
  mission_name: string;
  started: number;
  ended: number | null;
  status: string;
  nodes_completed: number;
  total_nodes: number;
}

export interface AnalyticsSummary {
  uptime_pct: number;
  distance_m: number;
  mission_success_rate: number;
  mission_count: number;
  avg_mission_duration_s: number;
  avg_odom_hz: number;
  min_odom_hz: number;
  max_odom_hz: number;
  slam_good_pct: number;
}

export interface TimeseriesPoint {
  timestamp: number;
  value: number;
}

const SCHEMA = `
CREATE TABLE IF NOT EXISTS telemetry_samples (
  timestamp REAL NOT NULL,
  pose_x REAL NOT NULL,
  pose_y REAL NOT NULL,
  pose_theta REAL NOT NULL,
  linear_vel REAL NOT NULL,
  angular_vel REAL NOT NULL,
  odom_hz REAL NOT NULL,
  slam_confidence TEXT NOT NULL DEFAULT 'unknown',
  robot_state TEXT NOT NULL DEFAULT 'offline',
  battery_pct REAL NOT NULL DEFAULT -1
);

CREATE INDEX IF NOT EXISTS idx_telemetry_ts ON telemetry_samples(timestamp);

CREATE TABLE IF NOT EXISTS mission_runs (
  id TEXT PRIMARY KEY,
  mission_id TEXT NOT NULL,
  mission_name TEXT NOT NULL DEFAULT '',
  started REAL NOT NULL,
  ended REAL,
  status TEXT NOT NULL DEFAULT 'running',
  nodes_completed INTEGER NOT NULL DEFAULT 0,
  total_nodes INTEGER NOT NULL DEFAULT 0
);

CREATE INDEX IF NOT EXISTS idx_mission_started ON mission_runs(started);

CREATE TABLE IF NOT EXISTS events (
  timestamp REAL NOT NULL,
  severity TEXT NOT NULL,
  subsystem TEXT NOT NULL,
  text TEXT NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_events_ts ON events(timestamp);
`;

export class TelemetryStore {
  private db: Database.Database;
  private retentionDays: number;

  // Prepared statements for hot-path operations
  private insertSample: Database.Statement;
  private insertEvent: Database.Statement;
  private insertMission: Database.Statement;
  private updateMission: Database.Statement;

  constructor(dataDir: string, retentionDays = 30) {
    const dbPath = path.join(dataDir, 'telemetry.db');
    this.db = new Database(dbPath);
    this.retentionDays = retentionDays;

    // WAL mode for concurrent reads during writes
    this.db.pragma('journal_mode = WAL');
    this.db.pragma('synchronous = NORMAL');

    this.db.exec(SCHEMA);

    // Prepare statements
    this.insertSample = this.db.prepare(`
      INSERT INTO telemetry_samples (timestamp, pose_x, pose_y, pose_theta, linear_vel, angular_vel, odom_hz, slam_confidence, robot_state, battery_pct)
      VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
    `);

    this.insertEvent = this.db.prepare(`
      INSERT INTO events (timestamp, severity, subsystem, text)
      VALUES (?, ?, ?, ?)
    `);

    this.insertMission = this.db.prepare(`
      INSERT INTO mission_runs (id, mission_id, mission_name, started, status, total_nodes)
      VALUES (?, ?, ?, ?, 'running', ?)
    `);

    this.updateMission = this.db.prepare(`
      UPDATE mission_runs SET ended = ?, status = ?, nodes_completed = ? WHERE id = ?
    `);
  }

  // ---------------------------------------------------------------------------
  // Write operations
  // ---------------------------------------------------------------------------

  recordSample(s: TelemetrySample): void {
    this.insertSample.run(
      s.timestamp, s.pose_x, s.pose_y, s.pose_theta,
      s.linear_vel, s.angular_vel, s.odom_hz,
      s.slam_confidence, s.robot_state, s.battery_pct
    );
  }

  recordEvent(timestamp: number, severity: string, subsystem: string, text: string): void {
    this.insertEvent.run(timestamp, severity, subsystem, text);
  }

  startMissionRun(runId: string, missionId: string, missionName: string, totalNodes: number): void {
    this.insertMission.run(runId, missionId, missionName, Date.now() / 1000, totalNodes);
  }

  endMissionRun(runId: string, status: string, nodesCompleted: number): void {
    this.updateMission.run(Date.now() / 1000, status, nodesCompleted, runId);
  }

  // ---------------------------------------------------------------------------
  // Read operations (analytics)
  // ---------------------------------------------------------------------------

  /** Get aggregated KPIs for a time period (in seconds from now) */
  getSummary(periodSeconds: number): AnalyticsSummary {
    const since = Date.now() / 1000 - periodSeconds;

    // Uptime: percentage of samples not in offline/fault states
    const uptimeRow = this.db.prepare(`
      SELECT
        COUNT(*) as total,
        SUM(CASE WHEN robot_state NOT IN ('offline', 'fault', 'e_stop') THEN 1 ELSE 0 END) as up
      FROM telemetry_samples WHERE timestamp > ?
    `).get(since) as any;

    const uptime_pct = uptimeRow?.total > 0 ? (uptimeRow.up / uptimeRow.total) * 100 : 0;

    // Distance: sum of position deltas
    const distRow = this.db.prepare(`
      SELECT SUM(dist) as total_dist FROM (
        SELECT SQRT(
          (pose_x - LAG(pose_x) OVER (ORDER BY timestamp)) *
          (pose_x - LAG(pose_x) OVER (ORDER BY timestamp)) +
          (pose_y - LAG(pose_y) OVER (ORDER BY timestamp)) *
          (pose_y - LAG(pose_y) OVER (ORDER BY timestamp))
        ) as dist
        FROM telemetry_samples WHERE timestamp > ?
      )
    `).get(since) as any;

    const distance_m = Math.round((distRow?.total_dist || 0) * 100) / 100;

    // Mission stats
    const missionRow = this.db.prepare(`
      SELECT
        COUNT(*) as total,
        SUM(CASE WHEN status = 'completed' THEN 1 ELSE 0 END) as succeeded,
        AVG(CASE WHEN ended IS NOT NULL THEN ended - started END) as avg_dur
      FROM mission_runs WHERE started > ?
    `).get(since) as any;

    const mission_count = missionRow?.total || 0;
    const mission_success_rate = mission_count > 0
      ? Math.round((missionRow.succeeded / mission_count) * 100) : 0;
    const avg_mission_duration_s = Math.round(missionRow?.avg_dur || 0);

    // Odom Hz stats
    const odomRow = this.db.prepare(`
      SELECT AVG(odom_hz) as avg_hz, MIN(odom_hz) as min_hz, MAX(odom_hz) as max_hz
      FROM telemetry_samples WHERE timestamp > ? AND odom_hz > 0
    `).get(since) as any;

    // SLAM confidence
    const slamRow = this.db.prepare(`
      SELECT
        COUNT(*) as total,
        SUM(CASE WHEN slam_confidence = 'good' THEN 1 ELSE 0 END) as good
      FROM telemetry_samples WHERE timestamp > ?
    `).get(since) as any;

    const slam_good_pct = slamRow?.total > 0
      ? Math.round((slamRow.good / slamRow.total) * 100) : 0;

    return {
      uptime_pct: Math.round(uptime_pct * 10) / 10,
      distance_m,
      mission_success_rate,
      mission_count,
      avg_mission_duration_s,
      avg_odom_hz: Math.round((odomRow?.avg_hz || 0) * 10) / 10,
      min_odom_hz: Math.round((odomRow?.min_hz || 0) * 10) / 10,
      max_odom_hz: Math.round((odomRow?.max_hz || 0) * 10) / 10,
      slam_good_pct,
    };
  }

  /** Get downsampled time series for a metric */
  getTimeseries(
    metric: 'odom_hz' | 'linear_vel' | 'pose_x' | 'pose_y' | 'slam_confidence',
    fromTs: number,
    toTs: number,
    resolutionSeconds: number = 60,
  ): TimeseriesPoint[] {
    const column = metric === 'slam_confidence'
      ? "CASE WHEN slam_confidence = 'good' THEN 1.0 WHEN slam_confidence = 'low' THEN 0.3 ELSE 0.0 END"
      : metric;

    const rows = this.db.prepare(`
      SELECT
        CAST(timestamp / ? AS INTEGER) * ? as bucket,
        AVG(${column}) as value
      FROM telemetry_samples
      WHERE timestamp BETWEEN ? AND ?
      GROUP BY bucket
      ORDER BY bucket
    `).all(resolutionSeconds, resolutionSeconds, fromTs, toTs) as any[];

    return rows.map(r => ({ timestamp: r.bucket, value: Math.round(r.value * 100) / 100 }));
  }

  /** Get telemetry samples for replay */
  getReplaySamples(fromTs: number, toTs: number): TelemetrySample[] {
    return this.db.prepare(`
      SELECT * FROM telemetry_samples
      WHERE timestamp BETWEEN ? AND ?
      ORDER BY timestamp
    `).all(fromTs, toTs) as TelemetrySample[];
  }

  /** Get mission runs for a time period */
  getMissionRuns(fromTs: number, toTs: number): MissionRun[] {
    return this.db.prepare(`
      SELECT * FROM mission_runs
      WHERE started BETWEEN ? AND ?
      ORDER BY started DESC
    `).all(fromTs, toTs) as MissionRun[];
  }

  /** Get events for a time range */
  getEvents(fromTs: number, toTs: number, limit = 500): any[] {
    return this.db.prepare(`
      SELECT * FROM events
      WHERE timestamp BETWEEN ? AND ?
      ORDER BY timestamp DESC
      LIMIT ?
    `).all(fromTs, toTs, limit);
  }

  // ---------------------------------------------------------------------------
  // Maintenance
  // ---------------------------------------------------------------------------

  /** Prune data older than retention period */
  prune(): { deleted: number } {
    const cutoff = Date.now() / 1000 - this.retentionDays * 86400;
    const r1 = this.db.prepare('DELETE FROM telemetry_samples WHERE timestamp < ?').run(cutoff);
    const r2 = this.db.prepare('DELETE FROM events WHERE timestamp < ?').run(cutoff);
    const r3 = this.db.prepare('DELETE FROM mission_runs WHERE started < ?').run(cutoff);
    return { deleted: r1.changes + r2.changes + r3.changes };
  }

  close(): void {
    this.db.close();
  }
}
