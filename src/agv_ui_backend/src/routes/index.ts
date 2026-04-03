/**
 * Route module registry — registers all API endpoints on the Express app.
 */

import type { Express } from 'express';
import type { AppDeps } from '../app_deps';

import * as statusRoutes from './status';
import * as authRoutes from './auth';
import * as eventsRoutes from './events';
import * as recordingRoutes from './recording';
import * as mapsRoutes from './maps';
import * as missionsRoutes from './missions';
import * as navRoutes from './nav';
import * as cameraRoutes from './camera';
import * as analyticsRoutes from './analytics';

export function registerAllRoutes(app: Express, deps: AppDeps, rosNode: any): void {
  statusRoutes.register(app, deps);
  authRoutes.register(app, deps);
  eventsRoutes.register(app, deps);
  recordingRoutes.register(app, deps);
  mapsRoutes.register(app, deps);
  missionsRoutes.register(app, deps);
  navRoutes.register(app, deps);
  cameraRoutes.register(app, deps, rosNode);
  analyticsRoutes.register(app, deps);
}
