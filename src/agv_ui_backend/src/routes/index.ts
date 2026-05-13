/**
 * Route module registry — registers all API endpoints on the Express app.
 */

import type { Express } from 'express';
import type { AppDeps } from '../app_deps';

import * as statusRoutes from './status';
import * as authRoutes from './auth';
import * as eventsRoutes from './events';
import * as recordingRoutes from './recording';
import * as recoveryRoutes from './recovery';
import * as mapsRoutes from './maps';
import * as missionsRoutes from './missions';
import * as navRoutes from './nav';
import * as analyticsRoutes from './analytics';
import * as apriltagRoutes from './apriltags';
import * as systemRoutes from './system';
import * as healthRoutes from './health';

export function registerAllRoutes(app: Express, deps: AppDeps, _rosNode: any): void {
  statusRoutes.register(app, deps);
  authRoutes.register(app, deps);
  eventsRoutes.register(app, deps);
  recordingRoutes.register(app, deps);
  recoveryRoutes.register(app, deps);
  mapsRoutes.register(app, deps);
  missionsRoutes.register(app, deps);
  navRoutes.register(app, deps);
  // Camera streams served directly by C++ image_server_node on port 8091
  analyticsRoutes.register(app, deps);
  apriltagRoutes.register(app, deps);
  systemRoutes.register(app, deps);
  healthRoutes.register(app, deps);
}
