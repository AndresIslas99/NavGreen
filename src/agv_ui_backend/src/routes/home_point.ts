/**
 * Home point (operator-defined base/dock) REST endpoints.
 *
 * The home point is a single saved pose (x, y, theta) the operator marks as
 * the "base" — typically the docking/charging spot. POST /api/home_point/go
 * dispatches a navigate_to_pose action to this pose by reusing the existing
 * sendNavGoal codepath (no new DDS contract).
 *
 * Persistence: AGV_DATA_DIR/home_point.json (atomic tmp+rename writes).
 * Schema declared in specs/persistence.yaml#home_point_json and
 * specs/hmi_api.yaml.
 *
 * Safety: no implicit default. If the file is absent, GET returns null and
 * POST /go returns 409. The dashboard's IR A BASE button stays disabled so
 * the robot can't drive to map origin (which is rarely the dock) by accident.
 */

import type { Express } from 'express';
import * as fs from 'fs';
import * as path from 'path';
import type { AppDeps, HomePoint } from '../app_deps';

function readHomePoint(filePath: string): HomePoint | null {
  if (!fs.existsSync(filePath)) return null;
  try {
    const raw = JSON.parse(fs.readFileSync(filePath, 'utf-8'));
    if (
      typeof raw?.x !== 'number' ||
      typeof raw?.y !== 'number' ||
      typeof raw?.theta !== 'number'
    ) return null;
    return {
      x: raw.x,
      y: raw.y,
      theta: raw.theta,
      set_at: typeof raw.set_at === 'number' ? raw.set_at : 0,
      name: typeof raw.name === 'string' ? raw.name : 'Base',
    };
  } catch {
    return null;
  }
}

function writeHomePoint(filePath: string, hp: HomePoint): void {
  fs.mkdirSync(path.dirname(filePath), { recursive: true });
  const tmp = filePath + '.tmp';
  fs.writeFileSync(tmp, JSON.stringify(hp, null, 2));
  fs.renameSync(tmp, filePath);
}

export function register(app: Express, deps: AppDeps): void {
  const { state, ros, config, eventLog } = deps;

  app.get('/api/home_point', (_req, res) => {
    res.json(state.homePoint);
  });

  app.put('/api/home_point', (req, res) => {
    const { x, y, theta, name } = req.body || {};
    if (typeof x !== 'number' || typeof y !== 'number' || typeof theta !== 'number') {
      return res.status(400).json({ success: false, message: 'Invalid coordinates' });
    }
    const hp: HomePoint = {
      x, y, theta,
      set_at: Math.floor(Date.now() / 1000),
      name: typeof name === 'string' && name.trim() ? name.trim() : 'Base',
    };
    try {
      writeHomePoint(config.homePointPath, hp);
      state.homePoint = hp;
      eventLog.emit('info', 'NAV',
        `Home point set: "${hp.name}" at (${x.toFixed(2)}, ${y.toFixed(2)}, ${theta.toFixed(2)})`);
      res.json({ success: true, home_point: hp });
    } catch (e: any) {
      res.status(500).json({ success: false, message: e?.message || 'Write failed' });
    }
  });

  app.post('/api/home_point/go', (_req, res) => {
    const hp = state.homePoint;
    if (!hp) {
      return res.status(409).json({ success: false, message: 'No home point defined' });
    }
    const result = ros.sendNavGoal(hp.x, hp.y, hp.theta);
    if (!result.success) {
      // Nav rejected — surface as 503 (action gate failed) so the dashboard
      // can distinguish from the 409 (no home defined) case.
      return res.status(503).json(result);
    }
    eventLog.emit('info', 'NAV', `Dispatched nav goal: home "${hp.name}"`);
    res.json(result);
  });
}

export { readHomePoint };
