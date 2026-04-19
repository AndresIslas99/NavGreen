import type { Express } from 'express';
import type { AppDeps } from '../app_deps';

export function register(app: Express, deps: AppDeps): void {
  app.post('/api/nav/goal', (req, res) => {
    const result = deps.ros.sendNavGoal(
      parseFloat(req.body?.x || 0),
      parseFloat(req.body?.y || 0),
      parseFloat(req.body?.theta || 0)
    );
    res.json(result);
  });

  app.post('/api/nav/cancel', (_req, res) => {
    deps.ros.cancelNavGoal();
    res.json({ success: true });
  });
}
