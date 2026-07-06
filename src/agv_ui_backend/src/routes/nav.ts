import type { Express } from 'express';
import type { AppDeps } from '../app_deps';

export function register(app: Express, deps: AppDeps): void {
  // Motion-starting endpoint — operator role required when auth is enabled.
  const requireOperator = deps.authManager.requireAuth('operator');

  app.post('/api/nav/goal', requireOperator, (req, res) => {
    const x = Number(req.body?.x ?? 0);
    const y = Number(req.body?.y ?? 0);
    const theta = Number(req.body?.theta ?? 0);
    if (![x, y, theta].every((v) => Number.isFinite(v))) {
      return res.status(400).json({ success: false, message: 'x, y, theta must be finite numbers' });
    }
    res.json(deps.ros.sendNavGoal(x, y, theta));
  });

  // Stop-type action — intentionally unauthenticated so anyone on the local
  // network can always stop the robot (mirrors a physical stop control).
  app.post('/api/nav/cancel', (_req, res) => {
    deps.ros.cancelNavGoal();
    res.json({ success: true });
  });
}
