import * as path from 'path';
import * as fs from 'fs';
import type { Express } from 'express';
import type { AppDeps } from '../app_deps';

export function register(app: Express, deps: AppDeps): void {
  const { config, eventLog, ros, state } = deps;

  function listMapFiles() {
    try {
      return fs.readdirSync(config.mapsDir)
        .filter(f => f.endsWith('.yaml'))
        .map(f => ({
          name: f.replace('.yaml', ''),
          modified: fs.statSync(path.join(config.mapsDir, f)).mtimeMs / 1000,
        }))
        .sort((a, b) => b.modified - a.modified);
    } catch { return []; }
  }

  app.get('/api/maps', (_req, res) => {
    res.json(listMapFiles());
  });

  app.get('/api/maps/:name/image', (req, res) => {
    const name = req.params.name;
    const yamlPath = path.join(config.mapsDir, `${name}.yaml`);
    if (!fs.existsSync(yamlPath)) return res.status(404).json({ error: 'Map not found' });

    let pgmName: string | null = null;
    for (const line of fs.readFileSync(yamlPath, 'utf-8').split('\n')) {
      if (line.trim().startsWith('image:')) {
        pgmName = line.split(':').slice(1).join(':').trim();
        break;
      }
    }
    if (!pgmName) return res.status(400).json({ error: 'No image in map YAML' });

    const pgmPath = path.join(config.mapsDir, pgmName);
    if (!fs.existsSync(pgmPath)) return res.status(404).json({ error: `PGM not found: ${pgmName}` });

    const sharp = require('sharp');
    sharp(pgmPath).png().toBuffer()
      .then((buf: Buffer) => res.type('image/png').send(buf))
      .catch((err: any) => res.status(500).json({ error: err?.message || 'Conversion failed' }));
  });

  app.post('/api/maps/save', async (req, res) => {
    const name = (req.body?.name || '').trim();
    const result = await ros.saveMap(name, config.mapsDir, `/${config.namespace}/live_map`);
    if (result.success) res.json(result);
    else res.status(500).json(result);
  });

  app.post('/api/maps/load', async (req, res) => {
    const name = (req.body?.name || '').trim();
    const yamlPath = path.join(config.mapsDir, `${name}.yaml`);
    if (!fs.existsSync(yamlPath)) return res.status(404).json({ error: 'Map not found' });
    // LoadMap service expects {map_url: string}, not Trigger's empty request
    if (!ros.loadMapClient.isServiceServerAvailable()) {
      return res.status(503).json({ success: false, message: 'map_server not available' });
    }
    try {
      const r = await ros.loadMapClient.sendRequestAsync({ map_url: yamlPath }, { timeout: 10000 });
      const success = r.result === 0; // nav2_msgs/srv/LoadMap: RESULT_SUCCESS = 0
      if (success) eventLog.emit('info', 'MAPPING', `Map "${name}" loaded`);
      else eventLog.emit('warn', 'MAPPING', `Map load failed: ${name}`);
      res.json({ success, message: success ? 'Loaded' : 'Load failed' });
    } catch (e: any) {
      res.status(500).json({ success: false, message: e?.message || 'Load service call failed' });
    }
  });

  // Live accumulated map (from scan_grid_mapper via rclnodejs)
  app.get('/api/acc_map/image', (_req, res) => {
    if (state.liveMapPng) {
      res.type('image/png').send(state.liveMapPng);
    } else {
      res.status(404).json({ error: 'No accumulated map' });
    }
  });

  app.delete('/api/acc_map', (_req, res) => {
    // Clear the ROS scan_grid_mapper via topic
    const { execFile } = require('child_process');
    execFile('ros2', ['topic', 'pub', '--once',
      `/${config.namespace}/clear_map`, 'std_msgs/msg/Bool', '{data: true}'],
      { env: process.env, timeout: 5000 }, () => {});
    eventLog.emit('info', 'MAPPING', 'Accumulated map cleared');
    res.json({ success: true });
  });
}
