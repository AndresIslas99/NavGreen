/**
 * System routes — non-ROS-dependent.
 *
 * Currently exposes /api/system/ros_status (Sub-fase 1.1.b). Reads
 * the RosBridgeProxy status injected via deps.ros — that proxy is
 * stable for the lifetime of the process, so this endpoint can
 * answer even when the ROS bridge is offline.
 */

import type { Express, Request, Response } from 'express';
import type { AppDeps } from '../app_deps';
import { RosBridgeProxy } from '../ros_lifecycle';

export function register(app: Express, deps: AppDeps): void {
  app.get('/api/system/ros_status', (_req: Request, res: Response) => {
    // deps.ros is a RosBridgeProxy in the new bootstrap (Sub-fase 1.1.b).
    // If the old direct-impl is still in use, default to 'online' since
    // the process couldn't have started without a working ROS bridge.
    const proxy = deps.ros as unknown as RosBridgeProxy;
    const status = (typeof proxy.status === 'string') ? proxy.status : 'online';
    const detail = (typeof proxy.detail === 'string')
      ? proxy.detail
      : 'ROS bridge active (legacy bootstrap)';
    res.json({ status, detail });
  });
}
