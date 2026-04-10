/**
 * AprilTag REST endpoints — operator-driven tag definition and assignment.
 */

import type { Express } from 'express';
import type { AppDeps } from '../app_deps';

export function register(app: Express, deps: AppDeps): void {
  const { apriltagManager, eventLog } = deps;

  // GET full state: defined tags + assignments + pending detections
  app.get('/api/apriltags', (_req, res) => {
    res.json({
      defined_tags: apriltagManager.getDefinedTags(),
      hardware_assignments: apriltagManager.getHardwareAssignments(),
      pending_detections: apriltagManager.getPendingDetections(),
    });
  });

  // POST create a new defined tag
  app.post('/api/apriltags/defined', (req, res) => {
    const { label, description, x, y, yaw, z } = req.body || {};
    if (typeof label !== 'string' || !label.trim()) {
      return res.status(400).json({ error: 'label required' });
    }
    if (typeof x !== 'number' || typeof y !== 'number' || typeof yaw !== 'number') {
      return res.status(400).json({ error: 'x, y, yaw must be numbers' });
    }
    const tag = apriltagManager.addDefinedTag(
      label.trim(),
      typeof description === 'string' ? description : '',
      x, y, yaw,
      typeof z === 'number' ? z : 0.145,
    );
    eventLog.emit('info', 'MARKERS', `Tag defined: "${tag.label}" at (${x.toFixed(2)}, ${y.toFixed(2)})`);
    res.json(tag);
  });

  // PUT update existing defined tag
  app.put('/api/apriltags/defined/:id', (req, res) => {
    const id = parseInt(req.params.id, 10);
    if (isNaN(id)) return res.status(400).json({ error: 'invalid id' });
    const fields: Record<string, unknown> = {};
    for (const k of ['label', 'description', 'x', 'y', 'z', 'yaw']) {
      if (k in req.body) fields[k] = req.body[k];
    }
    const ok = apriltagManager.updateDefinedTag(id, fields as any);
    if (!ok) return res.status(404).json({ error: 'tag not found' });
    res.json({ success: true });
  });

  // DELETE defined tag (also removes any hardware assignments)
  app.delete('/api/apriltags/defined/:id', (req, res) => {
    const id = parseInt(req.params.id, 10);
    if (isNaN(id)) return res.status(400).json({ error: 'invalid id' });
    const ok = apriltagManager.deleteDefinedTag(id);
    if (!ok) return res.status(404).json({ error: 'tag not found' });
    eventLog.emit('info', 'MARKERS', `Defined tag #${id} removed`);
    res.json({ success: true });
  });

  // POST assign a hardware ID to a defined tag
  app.post('/api/apriltags/assign', (req, res) => {
    const { hardware_id, defined_id } = req.body || {};
    if (typeof hardware_id !== 'number' || typeof defined_id !== 'number') {
      return res.status(400).json({ error: 'hardware_id and defined_id required (numbers)' });
    }
    const ok = apriltagManager.assignHardware(hardware_id, defined_id);
    if (!ok) return res.status(404).json({ error: 'defined_id not found' });
    eventLog.emit('info', 'MARKERS', `Hardware AprilTag #${hardware_id} → defined #${defined_id}`);
    res.json({ success: true });
  });

  // DELETE remove a hardware assignment
  app.delete('/api/apriltags/assignment/:hardware_id', (req, res) => {
    const id = parseInt(req.params.hardware_id, 10);
    if (isNaN(id)) return res.status(400).json({ error: 'invalid hardware_id' });
    const ok = apriltagManager.unassignHardware(id);
    if (!ok) return res.status(404).json({ error: 'assignment not found' });
    res.json({ success: true });
  });

  // POST dismiss pending detection (until next time it's seen)
  app.post('/api/apriltags/dismiss/:hardware_id', (req, res) => {
    const id = parseInt(req.params.hardware_id, 10);
    if (isNaN(id)) return res.status(400).json({ error: 'invalid hardware_id' });
    apriltagManager.dismissPending(id);
    res.json({ success: true });
  });
}
