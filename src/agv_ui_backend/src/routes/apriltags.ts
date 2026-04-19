/**
 * AprilTag REST endpoints — operator-driven tag definition and assignment.
 */

import type { Express } from 'express';
import type { AppDeps } from '../app_deps';

export function register(app: Express, deps: AppDeps): void {
  const { apriltagManager, eventLog, ros, state } = deps;

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
    const { label, description, x, y, yaw, z, type } = req.body || {};
    if (typeof label !== 'string' || !label.trim()) {
      return res.status(400).json({ error: 'label required' });
    }
    if (typeof x !== 'number' || typeof y !== 'number' || typeof yaw !== 'number') {
      return res.status(400).json({ error: 'x, y, yaw must be numbers' });
    }
    const tagType = (type === 'rail_start') ? 'rail_start' : 'wall';
    const tag = apriltagManager.addDefinedTag(
      label.trim(),
      typeof description === 'string' ? description : '',
      x, y, yaw, tagType,
      typeof z === 'number' ? z : undefined,
    );
    eventLog.emit('info', 'MARKERS',
      `Tag defined: "${tag.label}" (${tagType}) at (${x.toFixed(2)}, ${y.toFixed(2)})`);
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

  // POST send nav goal to a defined tag's coordinates.
  // If the tag is rail_start, the backend will auto-trigger rail_approach
  // after Nav2 succeeds (handled in index.ts via pendingRailApproach).
  app.post('/api/apriltags/:id/navigate', (req, res) => {
    const id = parseInt(req.params.id, 10);
    if (isNaN(id)) return res.status(400).json({ error: 'invalid id' });
    const tag = apriltagManager.getDefinedTag(id);
    if (!tag) return res.status(404).json({ error: 'tag not found' });

    // Find the hardware ID for this defined tag (needed to call rail_approach service)
    let hwId: number | null = null;
    for (const [hw, def] of Object.entries(apriltagManager.getHardwareAssignments())) {
      if (def === id) { hwId = parseInt(hw, 10); break; }
    }

    if (state.currentMode !== 'nav') {
      return res.status(409).json({ error: 'Switch to nav mode first' });
    }

    // For rail_start tags, the goal is the tag position itself (yaw aligned with tag).
    // For wall tags, the goal is in front of the tag (0.5m standoff).
    let goal_x: number, goal_y: number, goal_yaw: number;
    if (tag.type === 'rail_start') {
      // Goal: standoff position before the rail start, facing the tag.
      // The rail_approach service will take over for fine alignment.
      const standoff = 0.5;
      goal_x = tag.x - standoff * Math.cos(tag.yaw);
      goal_y = tag.y - standoff * Math.sin(tag.yaw);
      goal_yaw = tag.yaw;
    } else {
      // Wall tag: stand 1m in front of it
      const standoff = 1.0;
      goal_x = tag.x - standoff * Math.cos(tag.yaw);
      goal_y = tag.y - standoff * Math.sin(tag.yaw);
      goal_yaw = tag.yaw;
    }

    const result = ros.sendNavGoal(goal_x, goal_y, goal_yaw);
    if (result.success) {
      // Mark this nav goal as targeting a rail_start so the backend can
      // auto-trigger rail_approach when Nav2 completes.
      if (tag.type === 'rail_start' && hwId !== null) {
        (state as any).pendingRailApproach = { hardware_id: hwId, defined_id: id };
      } else {
        (state as any).pendingRailApproach = null;
      }
      eventLog.emit('info', 'NAV',
        `Sent nav goal to tag "${tag.label}" (${tag.type}) at (${goal_x.toFixed(2)}, ${goal_y.toFixed(2)})`);
    }
    res.json(result);
  });
}
