/**
 * /api/tags/* — Sub-fase 1.2 Tag Layout Loader (Modo 1 YAML Import + Modo 3 Probe).
 *
 * Extends the existing /api/apriltags/* CRUD with bulk import/export
 * and an in-situ probe flow. apriltag_manager remains the source of
 * truth — these endpoints adapt the operator-facing YAML to the
 * manager's DefinedTag + hardware_assignments model.
 *
 * Endpoints
 * ---------
 *   POST /api/tags/layout/validate    parse + validate (no persistence)
 *   POST /api/tags/layout/apply       parse + validate + persist
 *   GET  /api/tags/layout/current     current layout YAML (text/yaml)
 *   GET  /api/tags/layout/example     sample YAML (text/yaml)
 *   GET  /api/tags/probe/status       localization gate + live detection
 *   POST /api/tags/probe/save         save a probed tag to the layout
 *
 * Auth: GET endpoints accept any role. POST apply / probe save require
 * `engineer` or `operator` role. Auth gating mirrors the existing
 * /api/apriltags/ routes — open the layout to anyone with a token.
 */

import type { Express, Request, Response } from 'express';
import type { AppDeps } from '../app_deps';
import type { LayoutRole } from '../apriltag_manager';

const VALID_ROLES: LayoutRole[] = ['charging', 'rail_entry', 'central_aisle_beacon', 'handoff', 'other'];

function requireAuth(deps: AppDeps, req: Request, res: Response, role?: 'engineer' | 'operator'): boolean {
  if (!deps.authManager.enabled) return true;
  const auth = req.headers.authorization || '';
  const token = auth.startsWith('Bearer ') ? auth.slice(7) : '';
  const claims = deps.authManager.verify(token);
  if (!claims) {
    res.status(401).json({ error: 'unauthenticated' });
    return false;
  }
  if (role === 'engineer' && claims.role !== 'engineer') {
    res.status(403).json({ error: 'requires engineer role' });
    return false;
  }
  if (role === 'operator' && claims.role === 'viewer') {
    res.status(403).json({ error: 'requires operator or engineer role' });
    return false;
  }
  return true;
}

// Receive raw text body for YAML uploads. Express's default json
// parser would reject the YAML payload, so we accept text/yaml,
// text/plain, and application/x-yaml.
function rawBody(req: Request): Promise<string> {
  return new Promise((resolve, reject) => {
    let buf = '';
    req.setEncoding('utf-8');
    req.on('data', chunk => { buf += chunk; if (buf.length > 1_000_000) { reject(new Error('payload too large')); } });
    req.on('end', () => resolve(buf));
    req.on('error', reject);
  });
}

export function register(app: Express, deps: AppDeps): void {
  // ── Layout: validate ─────────────────────────────────────────────────────
  app.post('/api/tags/layout/validate', async (req, res) => {
    if (!requireAuth(deps, req, res, 'operator')) return;
    let yamlText: string;
    try { yamlText = await rawBody(req); }
    catch (e: any) { res.status(413).json({ error: e?.message ?? 'payload error' }); return; }
    const result = deps.apriltagManager.validateLayoutYaml(yamlText);
    if (!result.valid) {
      res.status(400).json({ valid: false, errors: result.errors });
      return;
    }
    res.json({
      valid: true,
      tag_count: result.parsed!.tags.length,
      tags: result.parsed!.tags,
    });
  });

  // ── Layout: apply ────────────────────────────────────────────────────────
  app.post('/api/tags/layout/apply', async (req, res) => {
    if (!requireAuth(deps, req, res, 'operator')) return;
    let yamlText: string;
    try { yamlText = await rawBody(req); }
    catch (e: any) { res.status(413).json({ error: e?.message ?? 'payload error' }); return; }
    const validate = deps.apriltagManager.validateLayoutYaml(yamlText);
    if (!validate.valid) {
      res.status(400).json({ valid: false, errors: validate.errors });
      return;
    }
    try {
      const replace = (req.query.replace ?? 'true') !== 'false';
      const result = deps.apriltagManager.applyLayout(validate.parsed!, replace);
      deps.eventLog.emit('info', 'SYSTEM',
        `Tag layout applied: ${result.applied_count} tags (replace=${result.replaced})`);
      res.json({
        applied: true,
        tag_count: result.applied_count,
        replaced: result.replaced,
        runtime_reloaded: true,    // regenerateRegistryYaml fires the publisher chain
      });
    } catch (e: any) {
      res.status(500).json({ error: e?.message ?? String(e) });
    }
  });

  // ── Layout: current export ───────────────────────────────────────────────
  app.get('/api/tags/layout/current', (req, res) => {
    if (!requireAuth(deps, req, res)) return;
    const yamlText = deps.apriltagManager.getCurrentLayoutYaml();
    res.set('Content-Type', 'text/yaml');
    res.send(yamlText);
  });

  // ── Layout: example ──────────────────────────────────────────────────────
  app.get('/api/tags/layout/example', (req, res) => {
    if (!requireAuth(deps, req, res)) return;
    const yamlText = deps.apriltagManager.getExampleYaml();
    res.set('Content-Type', 'text/yaml');
    res.send(yamlText);
  });

  // ── Probe: live status (localization gate + current detection) ──────────
  app.get('/api/tags/probe/status', (req, res) => {
    if (!requireAuth(deps, req, res, 'operator')) return;
    // Pull from the existing AppState fields the rest of the backend
    // populates. The "current_detection" piece comes from the most
    // recent /agv/marker_pose + /agv/marker_raw_detected pair held in
    // state.probeState (added below) — that's a small inline subscriber
    // sufficient for the Probe modal poll.
    const localization = deps.state.localization?.action ?? 'UNKNOWN';
    const probe = deps.state.probeState;
    res.json({
      localization_state: localization,
      localization_detail: deps.state.localization?.detail ?? '',
      current_detection: probe?.tag_id ? {
        tag_id: probe.tag_id,
        decision_margin: probe.decision_margin,
        range_m: probe.range_m,
        pose_in_map: probe.pose_in_map,
        last_seen_ms_ago: probe.updated > 0
          ? Math.round((Date.now() / 1000 - probe.updated) * 1000)
          : null,
      } : null,
    });
  });

  // ── Probe: save current detection with operator-supplied metadata ───────
  app.post('/api/tags/probe/save', async (req, res) => {
    if (!requireAuth(deps, req, res, 'operator')) return;
    const body = req.body || {};
    const { tag_id, role, rail_id, size } = body;
    if (!Number.isInteger(tag_id) || tag_id < 0) {
      res.status(400).json({ error: 'tag_id must be a non-negative integer' });
      return;
    }
    if (!VALID_ROLES.includes(role)) {
      res.status(400).json({ error: `role must be one of ${VALID_ROLES.join('|')}` });
      return;
    }
    if (role === 'rail_entry' && (typeof rail_id !== 'string' || !rail_id.trim())) {
      res.status(400).json({ error: 'rail_id is required when role=rail_entry' });
      return;
    }
    // Gate: localized + recent detection of this exact tag_id.
    const loc = deps.state.localization?.action ?? 'UNKNOWN';
    if (loc !== 'LOCALIZED' && loc !== 'DEGRADED') {
      res.status(409).json({ error: `cannot probe while localization is ${loc}` });
      return;
    }
    const probe = deps.state.probeState;
    if (!probe || probe.tag_id !== tag_id || probe.updated === 0) {
      res.status(409).json({ error: `no recent detection of tag ${tag_id}` });
      return;
    }
    const ageMs = (Date.now() / 1000 - probe.updated) * 1000;
    if (ageMs > 2000) {
      res.status(409).json({ error: `last detection of tag ${tag_id} is ${(ageMs/1000).toFixed(1)}s old; point camera at tag and retry` });
      return;
    }
    try {
      const { x, y, z, yaw_rad } = probe.pose_in_map;
      const result = deps.apriltagManager.addOrUpdateProbedTag(
        tag_id, role, rail_id, x, y, z, yaw_rad, size,
      );
      deps.eventLog.emit('info', 'SYSTEM',
        `Tag ${tag_id} ${result.updated ? 'updated' : 'added'} via probe (role=${role}${rail_id ? `, rail=${rail_id}` : ''})`);
      const allTags = deps.apriltagManager.getDefinedTags();
      res.json({
        added: result.added,
        updated: result.updated,
        total_tags: allTags.length,
      });
    } catch (e: any) {
      res.status(500).json({ error: e?.message ?? String(e) });
    }
  });
}
