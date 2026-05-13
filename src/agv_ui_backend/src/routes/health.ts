/**
 * /api/health/* — Sub-fase 1.1.c System Health Panel backend.
 *
 * Endpoints
 * ---------
 *   GET  /api/health/components            list of all components + status
 *   GET  /api/health/components/:id        single component detail + recent events
 *   GET  /api/health/verifiers             list of verifiers from health_monitor.json
 *   POST /api/health/verifiers/:id/run     execute verifier; returns stdout/stderr/code
 *   GET  /api/health/events?lines=N        recent JSONL events (default 100)
 *
 * Auth: every endpoint requires a valid token (any role) for the GETs.
 * `verifiers/:id/run` requires `engineer` role.
 *
 * Restart and journalctl endpoints from §4.3 of the prompt are deferred
 * to a follow-up sub-phase (see future_work).
 */

import type { Express, Request, Response } from 'express';
import { execFile } from 'child_process';
import * as path from 'path';
import type { AppDeps } from '../app_deps';
import {
  evaluateAll, evaluateComponent, getComponents, getVerifiers,
  getRestartTargets, recordEvent, readRecentEvents,
} from '../health_monitor';

const WS_ROOT = path.resolve(__dirname, '../../../..');

function requireAuth(deps: AppDeps, req: Request, res: Response, role?: string): boolean {
  if (!deps.authManager.enabled) return true;
  const auth = req.headers.authorization || '';
  const token = auth.startsWith('Bearer ') ? auth.slice(7) : '';
  const claims = deps.authManager.verify(token);
  if (!claims) {
    res.status(401).json({ error: 'unauthenticated' });
    return false;
  }
  if (role && claims.role !== role && claims.role !== 'engineer') {
    res.status(403).json({ error: `requires role ${role}` });
    return false;
  }
  return true;
}

export function register(app: Express, deps: AppDeps): void {
  // List components with current status
  app.get('/api/health/components', async (req, res) => {
    if (!requireAuth(deps, req, res)) return;
    const samples = await evaluateAll(deps.state);
    res.json({ components: samples });
  });

  // Single component detail
  app.get('/api/health/components/:id', async (req, res) => {
    if (!requireAuth(deps, req, res)) return;
    const comp = getComponents().find(c => c.id === req.params.id);
    if (!comp) {
      res.status(404).json({ error: `unknown component '${req.params.id}'` });
      return;
    }
    const sample = await evaluateComponent(comp, deps.state);
    // Filter recent events to this component
    const all = readRecentEvents(deps.config.dataDir, 500);
    const events = all.filter(e => e.id === req.params.id).slice(-100);
    res.json({ component: sample, events });
  });

  // List verifiers
  app.get('/api/health/verifiers', (req, res) => {
    if (!requireAuth(deps, req, res)) return;
    res.json({ verifiers: getVerifiers() });
  });

  // Run a verifier
  app.post('/api/health/verifiers/:id/run', (req, res) => {
    if (!requireAuth(deps, req, res, 'engineer')) return;
    const v = getVerifiers().find(x => x.id === req.params.id);
    if (!v) {
      res.status(404).json({ error: `unknown verifier '${req.params.id}'` });
      return;
    }
    const scriptPath = path.resolve(WS_ROOT, v.script);
    const isPy = scriptPath.endsWith('.py');
    const cmd = isPy ? 'python3' : 'bash';
    const args = isPy ? [scriptPath] : [scriptPath];
    const started = Date.now();
    execFile(cmd, args, { cwd: WS_ROOT, timeout: 60_000 }, (err, stdout, stderr) => {
      const durationMs = Date.now() - started;
      const exitCode = err && typeof (err as any).code === 'number' ? (err as any).code : (err ? 1 : 0);
      const result = exitCode === 0 ? 'pass' : 'fail';
      recordEvent(deps.config.dataDir, {
        ts: Date.now() / 1000,
        type: 'verifier_run',
        id: v.id,
        payload: { result, exit_code: exitCode, duration_ms: durationMs },
      });
      res.json({
        verifier: v.id,
        exit_code: exitCode,
        duration_ms: durationMs,
        result,
        stdout: stdout || '',
        stderr: stderr || '',
      });
    });
  });

  // Restart a component (only if its `restart` field names a target in
  // `restart_targets`). Caller must send {"confirmation":"yes"} in the body
  // to acknowledge the destructive nature. Engineer role required.
  app.post('/api/health/components/:id/restart', (req, res) => {
    if (!requireAuth(deps, req, res, 'engineer')) return;
    const comp = getComponents().find(c => c.id === req.params.id);
    if (!comp) {
      res.status(404).json({ error: `unknown component '${req.params.id}'` });
      return;
    }
    if (!comp.restart) {
      res.status(400).json({
        error: 'component is not restartable',
        help: (comp as any).restart_help ?? null,
      });
      return;
    }
    const targets = getRestartTargets();
    const target = targets[comp.restart];
    if (!target) {
      res.status(500).json({ error: `restart target '${comp.restart}' not declared in restart_targets` });
      return;
    }
    if ((req.body || {}).confirmation !== 'yes') {
      res.status(400).json({
        error: 'restart requires explicit confirmation',
        hint: 'POST {"confirmation":"yes"}',
        target: target.description,
        self_terminating: !!target.self_terminating,
      });
      return;
    }
    recordEvent(deps.config.dataDir, {
      ts: Date.now() / 1000,
      type: 'verifier_run',                    // reuse the JSONL channel
      id: comp.id,
      payload: { action: 'restart', target: comp.restart },
    });
    // If the target is self-terminating (e.g., restart agv.service),
    // respond 202 FIRST, then exec — otherwise the backend dies before
    // the response flushes.
    if (target.self_terminating) {
      res.status(202).json({
        accepted: true,
        target: target.description,
        note: 'Backend will terminate; reconnect in ~15s.',
      });
      // Give the response a moment to flush before exec.
      setTimeout(() => {
        execFile(target.command, target.args, { timeout: 10_000 }, () => { /* unreachable */ });
      }, 200);
      return;
    }
    execFile(target.command, target.args, { timeout: 30_000 }, (err, stdout, stderr) => {
      const exitCode = err && typeof (err as any).code === 'number' ? (err as any).code : (err ? 1 : 0);
      res.json({
        exit_code: exitCode,
        target: target.description,
        stdout: stdout || '',
        stderr: stderr || '',
      });
    });
  });

  // Recent events (across all components/verifiers)
  app.get('/api/health/events', (req, res) => {
    if (!requireAuth(deps, req, res)) return;
    const lines = Math.max(1, Math.min(parseInt((req.query.lines as string) || '100'), 500));
    const events = readRecentEvents(deps.config.dataDir, lines);
    res.json({ events });
  });
}
