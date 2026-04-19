import type { Express } from 'express';
import type { AppDeps } from '../app_deps';

export function register(app: Express, deps: AppDeps): void {
  const { telemetryStore } = deps;

  app.get('/api/analytics/summary', (req, res) => {
    const period = req.query.period as string || '24h';
    let seconds = 86400;
    if (period.endsWith('h')) seconds = parseInt(period) * 3600;
    else if (period.endsWith('d')) seconds = parseInt(period) * 86400;
    else if (period.endsWith('m')) seconds = parseInt(period) * 60;
    try {
      res.json(telemetryStore.getSummary(seconds));
    } catch (e) { res.status(500).json({ error: String(e) }); }
  });

  app.get('/api/analytics/timeseries', (req, res) => {
    const metric = (req.query.metric as string) || 'odom_hz';
    const validMetrics = ['odom_hz', 'linear_vel', 'pose_x', 'pose_y', 'slam_confidence'];
    if (!validMetrics.includes(metric)) {
      return res.status(400).json({ error: `Invalid metric. Valid: ${validMetrics.join(', ')}` });
    }
    const now = Date.now() / 1000;
    const from = parseFloat(req.query.from as string) || (now - 86400);
    const to = parseFloat(req.query.to as string) || now;
    const resolution = parseInt(req.query.resolution as string) || 60;
    try {
      res.json(telemetryStore.getTimeseries(metric as any, from, to, resolution));
    } catch (e) { res.status(500).json({ error: String(e) }); }
  });

  app.get('/api/analytics/missions', (req, res) => {
    const now = Date.now() / 1000;
    const from = parseFloat(req.query.from as string) || (now - 86400 * 7);
    const to = parseFloat(req.query.to as string) || now;
    try {
      res.json(telemetryStore.getMissionRuns(from, to));
    } catch (e) { res.status(500).json({ error: String(e) }); }
  });

  app.get('/api/replay/samples', (req, res) => {
    const from = parseFloat(req.query.from as string) || 0;
    const to = parseFloat(req.query.to as string) || (Date.now() / 1000);
    try {
      res.json(telemetryStore.getReplaySamples(from, to));
    } catch (e) { res.status(500).json({ error: String(e) }); }
  });

  app.get('/api/replay/events', (req, res) => {
    const from = parseFloat(req.query.from as string) || 0;
    const to = parseFloat(req.query.to as string) || (Date.now() / 1000);
    const limit = parseInt(req.query.limit as string) || 500;
    try {
      res.json(telemetryStore.getEvents(from, to, limit));
    } catch (e) { res.status(500).json({ error: String(e) }); }
  });
}
