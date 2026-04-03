import type { Express } from 'express';
import type { AppDeps } from '../app_deps';

export function register(app: Express, deps: AppDeps): void {
  app.post('/api/recording/start', async (_req, res) => {
    const result = await deps.ros.callTriggerService(deps.ros.startRecClient, 'start_recording');
    if (result.success) deps.state.recordingActive = true;
    res.json(result);
  });

  app.post('/api/recording/stop', async (_req, res) => {
    const result = await deps.ros.callTriggerService(deps.ros.stopRecClient, 'stop_recording');
    if (result.success) deps.state.recordingActive = false;
    res.json(result);
  });
}
