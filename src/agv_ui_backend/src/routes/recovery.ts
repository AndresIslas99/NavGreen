// Recovery routes — exposes operator-recovery actions via REST so they
// can be invoked from CLI / scripts as well as the dashboard. The
// dashboard already has WS-driven counterparts for some of these
// (sendEStop, motorEnable); the REST endpoints are convenient for
// runbooks and CI smoke tests.
//
// 2026-04-25: added /api/recovery/clear_estop because publishing
// /agv/e_stop=false from the CLI does NOT update the backend's
// internal state.eStopActive flag. That flag is what the state
// machine uses to gate sendCmdVel and sendNavGoal; without resetting
// it, the dashboard reports robot_state="e_stop" indefinitely even
// after the topic is cleared.

import type { Express } from 'express';
import type { AppDeps } from '../app_deps';

export function register(app: Express, deps: AppDeps): void {
  const { ros, eventLog } = deps;

  // POST /api/recovery/clear_estop
  // Clears state.eStopActive in the backend AND publishes /agv/e_stop=false.
  // Idempotent: safe to call when e-stop is already cleared.
  app.post('/api/recovery/clear_estop', (_req, res) => {
    ros.sendEStop(false);
    eventLog.emit('info', 'SAFETY', 'E-stop cleared via /api/recovery/clear_estop');
    res.json({ success: true, message: 'E-stop cleared' });
  });

  // POST /api/recovery/trigger_estop
  // Mirrors the dashboard E-Stop button. Provided for CLI / automation.
  app.post('/api/recovery/trigger_estop', (_req, res) => {
    ros.sendEStop(true);
    eventLog.emit('crit', 'SAFETY', 'E-stop triggered via /api/recovery/trigger_estop');
    res.json({ success: true, message: 'E-stop activated' });
  });
}
