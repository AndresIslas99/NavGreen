import type { Express } from 'express';
import type { AppDeps } from '../app_deps';

export function register(app: Express, deps: AppDeps): void {
  const requireOperator = deps.authManager.requireAuth('operator');

  app.post('/api/recording/start', requireOperator, async (_req, res) => {
    const result = await deps.ros.callTriggerService(deps.ros.startRecClient, 'start_recording');
    if (result.success) deps.state.recordingActive = true;
    res.json(result);
  });

  app.post('/api/recording/stop', requireOperator, async (_req, res) => {
    const result = await deps.ros.callTriggerService(deps.ros.stopRecClient, 'stop_recording');
    if (result.success) deps.state.recordingActive = false;
    res.json(result);
  });
}
