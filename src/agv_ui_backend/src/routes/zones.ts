/**
 * Semantic zones REST endpoints — operator-facing labeled map overlay.
 *
 * Distinct from agv_map_manager's traffic-zone JSON (specs/persistence.yaml
 * #zones_json). This is the human-readable overlay the dashboard paints over
 * the SLAM map: BASE DE CARGA, ZONA DE TRABAJO A, ESTACIONAMIENTO, etc.
 *
 * Persistence: AGV_DATA_DIR/zones.yaml. If the file is missing or malformed,
 * GET returns {zones: []} with 200 so the dashboard renders an empty overlay
 * instead of crashing. Schema declared in specs/persistence.yaml
 * #semantic_zones_yaml and specs/hmi_api.yaml.
 */

import type { Express } from 'express';
import * as fs from 'fs';
import * as path from 'path';
import * as yaml from 'js-yaml';
import type { AppDeps } from '../app_deps';

export interface ZonePoint { x: number; y: number }
export interface SemanticZone {
  name: string;
  label: string;
  polygon: ZonePoint[];
  color: string;
  kind: 'home' | 'work' | 'parking' | 'other';
}

function validateZone(z: any): { ok: true; zone: SemanticZone } | { ok: false; reason: string } {
  if (!z || typeof z !== 'object') return { ok: false, reason: 'not an object' };
  if (typeof z.name !== 'string' || !z.name.trim()) return { ok: false, reason: 'missing name' };
  if (typeof z.label !== 'string' || !z.label.trim()) return { ok: false, reason: 'missing label' };
  if (!Array.isArray(z.polygon) || z.polygon.length < 3) {
    return { ok: false, reason: 'polygon must be an array of ≥3 points' };
  }
  const polygon: ZonePoint[] = [];
  for (const pt of z.polygon) {
    if (typeof pt?.x !== 'number' || typeof pt?.y !== 'number') {
      return { ok: false, reason: 'polygon points must be {x, y} numbers' };
    }
    polygon.push({ x: pt.x, y: pt.y });
  }
  const kind: SemanticZone['kind'] =
    z.kind === 'home' || z.kind === 'work' || z.kind === 'parking' ? z.kind : 'other';
  const color = typeof z.color === 'string' ? z.color : '#7a9d8e';
  return { ok: true, zone: { name: z.name.trim(), label: z.label.trim(), polygon, color, kind } };
}

function readZones(filePath: string): { zones: SemanticZone[]; error?: string } {
  if (!fs.existsSync(filePath)) return { zones: [] };
  try {
    const raw = yaml.load(fs.readFileSync(filePath, 'utf-8'));
    if (!Array.isArray(raw)) {
      return { zones: [], error: 'Top-level value must be an array of zones' };
    }
    const zones: SemanticZone[] = [];
    for (const z of raw) {
      const v = validateZone(z);
      if (v.ok) zones.push(v.zone);
      // Skip invalid entries silently — graceful degradation per spec.
    }
    return { zones };
  } catch (e: any) {
    return { zones: [], error: e?.message || 'YAML parse failed' };
  }
}

function writeZones(filePath: string, zones: SemanticZone[]): void {
  fs.mkdirSync(path.dirname(filePath), { recursive: true });
  const tmp = filePath + '.tmp';
  fs.writeFileSync(tmp, yaml.dump(zones, { indent: 2 }));
  fs.renameSync(tmp, filePath);
}

export function register(app: Express, deps: AppDeps): void {
  const { config, eventLog } = deps;

  // Boot-time log so operators know whether the overlay file is present.
  // Logged once during setup, not on every request.
  if (!fs.existsSync(config.zonesYamlPath)) {
    eventLog.emit('info', 'SYSTEM',
      `No semantic zones file at ${config.zonesYamlPath} — overlay starts empty`);
  }

  app.get('/api/zones', (_req, res) => {
    res.json(readZones(config.zonesYamlPath));
  });

  app.put('/api/zones', (req, res) => {
    const incoming = req.body?.zones;
    if (!Array.isArray(incoming)) {
      return res.status(400).json({
        success: false, message: "Body must be {zones: [...]}",
      });
    }
    const valid: SemanticZone[] = [];
    for (let i = 0; i < incoming.length; i++) {
      const v = validateZone(incoming[i]);
      if (!v.ok) {
        return res.status(400).json({
          success: false, message: `Schema validation failed`,
          detail: `zones[${i}]: ${v.reason}`,
        });
      }
      valid.push(v.zone);
    }
    try {
      writeZones(config.zonesYamlPath, valid);
      eventLog.emit('info', 'SYSTEM', `Semantic zones updated (${valid.length} zone(s))`);
      res.json({ success: true, zones: valid });
    } catch (e: any) {
      res.status(500).json({ success: false, message: e?.message || 'Write failed' });
    }
  });
}

export { readZones };
