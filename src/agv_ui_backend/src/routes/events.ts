import type { Express } from 'express';
import type { AppDeps } from '../app_deps';

export function register(app: Express, deps: AppDeps): void {
  const requireOperator = deps.authManager.requireAuth('operator');

  app.get('/api/events', (req, res) => {
    const limit = parseInt(req.query.limit as string) || 100;
    const offset = parseInt(req.query.offset as string) || 0;
    res.json(deps.eventLog.getEntries(limit, offset));
  });

  app.delete('/api/events', requireOperator, (_req, res) => {
    deps.eventLog.clear();
    res.json({ success: true });
  });
}
