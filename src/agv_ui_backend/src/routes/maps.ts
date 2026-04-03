import * as path from 'path';
import * as fs from 'fs';
import type { Express } from 'express';
import type { AppDeps } from '../app_deps';

export function register(app: Express, deps: AppDeps): void {
  const { config, eventLog, ros, scanAccumulator } = deps;

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
    const result = await ros.saveMap(name, config.mapsDir, `/${config.namespace}/map`);
    if (result.success) res.json(result);
    else res.status(500).json(result);
  });

  app.post('/api/maps/load', async (req, res) => {
    const name = (req.body?.name || '').trim();
    const yamlPath = path.join(config.mapsDir, `${name}.yaml`);
    if (!fs.existsSync(yamlPath)) return res.status(404).json({ error: 'Map not found' });
    // Use ROS service client to load
    const result = await ros.callTriggerService(ros.loadMapClient, 'load_map');
    if (result.success) eventLog.emit('info', 'MAPPING', `Map "${name}" loaded`);
    res.json(result);
  });

  // Accumulated map
  app.get('/api/acc_map/image', (_req, res) => {
    if (scanAccumulator.pngBuffer) {
      res.type('image/png').send(scanAccumulator.pngBuffer);
    } else {
      res.status(404).json({ error: 'No accumulated map' });
    }
  });

  app.delete('/api/acc_map', (_req, res) => {
    scanAccumulator.clear();
    eventLog.emit('info', 'MAPPING', 'Accumulated map cleared');
    res.json({ success: true });
  });
}
