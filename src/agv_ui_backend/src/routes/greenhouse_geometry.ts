/**
 * Greenhouse geometry REST endpoints — SSOT for the physical layout.
 *
 * The greenhouse's structural constants — number of aisles, aisle spacing in
 * Y, section X-bounds, rail gauge — used to live as C++ hardcoded values
 * across `zone_classifier_impl.hpp`, `rail_controller.hpp`, and the FSM in
 * `mode_fsm.hpp`. This route gives the operator a single editable file
 * (${AGV_DATA_DIR}/greenhouse_geometry.yaml) that those nodes will read at
 * boot. The dashboard polls GET every 30 s and re-renders.
 *
 * Persistence schema declared in specs/persistence.yaml
 * #greenhouse_geometry_yaml; endpoints in specs/hmi_api.yaml.
 *
 * Engineer role is required for PUT because geometry edits affect navigation
 * correctness (the C++ classifier uses these bounds for mode-arbiter
 * transitions). Backend enforces sane ranges so an off-by-three operator
 * input can't render the layout unparseable.
 */

import type { Express } from 'express';
import * as fs from 'fs';
import * as path from 'path';
import * as yaml from 'js-yaml';
import type { AppDeps } from '../app_deps';

export interface GreenhouseGeometry {
  aisles: {
    count: number;
    spacing_m: number;
    half_width_m: number;
  };
  sections: {
    rear:  { x_start: number; x_end: number };
    front: { x_start: number; x_end: number };
  };
  corridor: { x_start: number; x_end: number };
  rail: { gauge_m: number };
}

// Fallback defaults — match the C++ hardcoded constants in
// zone_classifier_impl.hpp so a missing YAML file produces the same
// behavior as the legacy hardcoded path.
export const DEFAULT_GEOMETRY: GreenhouseGeometry = {
  aisles:   { count: 5, spacing_m: 2.2, half_width_m: 0.35 },
  sections: {
    rear:  { x_start: -16.5, x_end:  3.5 },
    front: { x_start:   7.5, x_end: 27.5 },
  },
  corridor: { x_start: 3.5, x_end: 7.5 },
  rail:     { gauge_m: 0.57 },
};

// Sane ranges for validation. Outside these, navigation behavior is
// undefined; the operator typed something wrong.
const RANGES = {
  count:        { min: 3,    max: 9 },
  spacing_m:    { min: 1.5,  max: 3.5 },
  half_width_m: { min: 0.20, max: 0.60 },
  section_len:  { min: 10,   max: 40 },   // x_end - x_start
  corridor_w:   { min: 2.0,  max: 6.0 },
  gauge_m:      { min: 0.30, max: 1.00 },
};

function inRange(v: unknown, lo: number, hi: number): boolean {
  return typeof v === 'number' && Number.isFinite(v) && v >= lo && v <= hi;
}

type ValidationResult =
  | { ok: true; geometry: GreenhouseGeometry }
  | { ok: false; reason: string };

function validate(raw: any): ValidationResult {
  if (!raw || typeof raw !== 'object') {
    return { ok: false, reason: 'body must be an object' };
  }
  const a = raw.aisles;
  if (!a || typeof a !== 'object') return { ok: false, reason: 'missing aisles' };
  if (!Number.isInteger(a.count) || !inRange(a.count, RANGES.count.min, RANGES.count.max)) {
    return { ok: false, reason: `aisles.count must be integer in [${RANGES.count.min}, ${RANGES.count.max}]` };
  }
  if (!inRange(a.spacing_m, RANGES.spacing_m.min, RANGES.spacing_m.max)) {
    return { ok: false, reason: `aisles.spacing_m must be in [${RANGES.spacing_m.min}, ${RANGES.spacing_m.max}]` };
  }
  if (!inRange(a.half_width_m, RANGES.half_width_m.min, RANGES.half_width_m.max)) {
    return { ok: false, reason: `aisles.half_width_m must be in [${RANGES.half_width_m.min}, ${RANGES.half_width_m.max}]` };
  }

  const s = raw.sections;
  if (!s || typeof s !== 'object') return { ok: false, reason: 'missing sections' };
  for (const which of ['rear', 'front'] as const) {
    const sec = s[which];
    if (!sec || typeof sec !== 'object'
        || typeof sec.x_start !== 'number' || typeof sec.x_end !== 'number'
        || !Number.isFinite(sec.x_start) || !Number.isFinite(sec.x_end)) {
      return { ok: false, reason: `sections.${which} must be {x_start, x_end}` };
    }
    const len = sec.x_end - sec.x_start;
    if (!inRange(len, RANGES.section_len.min, RANGES.section_len.max)) {
      return { ok: false, reason: `sections.${which} length (${len.toFixed(2)} m) must be in [${RANGES.section_len.min}, ${RANGES.section_len.max}]` };
    }
  }

  const c = raw.corridor;
  if (!c || typeof c !== 'object'
      || typeof c.x_start !== 'number' || typeof c.x_end !== 'number') {
    return { ok: false, reason: 'corridor must be {x_start, x_end}' };
  }
  const cw = c.x_end - c.x_start;
  if (!inRange(cw, RANGES.corridor_w.min, RANGES.corridor_w.max)) {
    return { ok: false, reason: `corridor width (${cw.toFixed(2)} m) must be in [${RANGES.corridor_w.min}, ${RANGES.corridor_w.max}]` };
  }
  // Continuity check: rear ends where corridor starts, corridor ends where front starts.
  if (Math.abs(s.rear.x_end - c.x_start) > 0.05) {
    return { ok: false, reason: 'rear.x_end must equal corridor.x_start' };
  }
  if (Math.abs(c.x_end - s.front.x_start) > 0.05) {
    return { ok: false, reason: 'corridor.x_end must equal front.x_start' };
  }

  const r = raw.rail;
  if (!r || typeof r !== 'object' || !inRange(r.gauge_m, RANGES.gauge_m.min, RANGES.gauge_m.max)) {
    return { ok: false, reason: `rail.gauge_m must be in [${RANGES.gauge_m.min}, ${RANGES.gauge_m.max}]` };
  }

  return {
    ok: true,
    geometry: {
      aisles:   { count: a.count, spacing_m: a.spacing_m, half_width_m: a.half_width_m },
      sections: {
        rear:  { x_start: s.rear.x_start,  x_end: s.rear.x_end  },
        front: { x_start: s.front.x_start, x_end: s.front.x_end },
      },
      corridor: { x_start: c.x_start, x_end: c.x_end },
      rail:     { gauge_m: r.gauge_m },
    },
  };
}

function readGeometry(filePath: string): { geometry: GreenhouseGeometry; source: 'yaml' | 'default'; error?: string } {
  if (!fs.existsSync(filePath)) {
    return { geometry: DEFAULT_GEOMETRY, source: 'default' };
  }
  try {
    const raw = yaml.load(fs.readFileSync(filePath, 'utf-8'));
    const v = validate(raw);
    if (!v.ok) {
      return { geometry: DEFAULT_GEOMETRY, source: 'default', error: v.reason };
    }
    return { geometry: v.geometry, source: 'yaml' };
  } catch (e: any) {
    return { geometry: DEFAULT_GEOMETRY, source: 'default', error: e?.message || 'YAML parse failed' };
  }
}

function writeGeometry(filePath: string, geom: GreenhouseGeometry): void {
  fs.mkdirSync(path.dirname(filePath), { recursive: true });
  const tmp = filePath + '.tmp';
  fs.writeFileSync(tmp, yaml.dump(geom, { indent: 2, lineWidth: 100 }));
  fs.renameSync(tmp, filePath);
}

export function register(app: Express, deps: AppDeps): void {
  const { config, eventLog } = deps;
  const filePath = config.greenhouseGeometryYamlPath;

  if (!fs.existsSync(filePath)) {
    eventLog.emit('info', 'SYSTEM',
      `No greenhouse_geometry.yaml at ${filePath} — using defaults`);
  }

  app.get('/api/greenhouse/geometry', (_req, res) => {
    const result = readGeometry(filePath);
    res.json({
      geometry: result.geometry,
      source: result.source,
      ...(result.error ? { error: result.error } : {}),
    });
  });

  app.put('/api/greenhouse/geometry', (req, res) => {
    const v = validate(req.body);
    if (!v.ok) {
      return res.status(400).json({
        success: false,
        message: 'Schema validation failed',
        detail: v.reason,
      });
    }
    try {
      writeGeometry(filePath, v.geometry);
      eventLog.emit('info', 'SYSTEM',
        `Greenhouse geometry updated (aisles=${v.geometry.aisles.count}, ` +
        `spacing=${v.geometry.aisles.spacing_m} m, ` +
        `corridor=${(v.geometry.corridor.x_end - v.geometry.corridor.x_start).toFixed(2)} m)`);
      res.json({ success: true, geometry: v.geometry });
    } catch (e: any) {
      res.status(500).json({ success: false, message: e?.message || 'Write failed' });
    }
  });
}

export { readGeometry };
