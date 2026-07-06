import * as fs from 'fs';
import type { Express } from 'express';
import type { AppDeps } from '../app_deps';

function readMissions(file: string): any[] {
  try { return JSON.parse(fs.readFileSync(file, 'utf-8')); }
  catch { return []; }
}

function writeMissions(file: string, missions: any[]): void {
  fs.writeFileSync(file, JSON.stringify(missions, null, 2));
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

export function register(app: Express, deps: AppDeps): void {
  const { config, eventLog, state } = deps;
  const file = config.missionsFile;
  const requireOperator = deps.authManager.requireAuth('operator');

  app.get('/api/missions', (_req, res) => {
    res.json(readMissions(file).map(normalizeMission));
  });

  app.post('/api/missions', requireOperator, (req, res) => {
    try {
      const missions = readMissions(file);
      let nodes = req.body.nodes || [];
      if (!nodes.length && req.body.waypoints) {
        nodes = req.body.waypoints.map((wp: any, i: number) => ({
          id: `n${i}`, type: 'waypoint', action: 'none', ...wp,
        }));
      }
      const mission = {
        id: req.body.id || `m${Date.now() % 100000000}`,
        name: req.body.name || 'Untitled',
        nodes,
        edges: req.body.edges || [],
        repeat: req.body.repeat || false,
        waypoints: req.body.waypoints || [],
        created: Date.now() / 1000,
      };
      missions.push(mission);
      writeMissions(file, missions);
      eventLog.emit('info', 'MISSION', `Mission "${mission.name}" created (${nodes.length} nodes)`);
      res.json(mission);
    } catch (e: any) {
      console.warn('[missions] create failed:', e?.message || e);
      res.status(500).json({ error: 'Failed to save mission' });
    }
  });

  app.delete('/api/missions/:id', requireOperator, (req, res) => {
    try {
      let missions = readMissions(file);
      const name = missions.find((m: any) => m.id === req.params.id)?.name || '?';
      missions = missions.filter((m: any) => m.id !== req.params.id);
      writeMissions(file, missions);
      eventLog.emit('info', 'MISSION', `Mission "${name}" deleted`);
      res.json({ success: true });
    } catch (e: any) {
      console.warn('[missions] delete failed:', e?.message || e);
      res.status(500).json({ error: 'Failed to delete mission' });
    }
  });

  app.post('/api/missions/:id/execute', requireOperator, async (req, res) => {
    const result = await deps.executeMission(String(req.params.id));
    if (result.success) res.json(result);
    else res.status(400).json(result);
  });

  // Stop-type action — intentionally unauthenticated (pausing halts motion).
  app.post('/api/missions/pause', (_req, res) => {
    state.missionPause = true;
    eventLog.emit('info', 'MISSION', 'Mission paused');
    res.json({ success: true });
  });

  app.post('/api/missions/resume', requireOperator, (_req, res) => {
    state.missionPause = false;
    eventLog.emit('info', 'MISSION', 'Mission resumed');
    res.json({ success: true });
  });
}
