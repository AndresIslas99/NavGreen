import type { Express } from 'express';
import type { AppDeps } from '../app_deps';
import { deriveState, allowedActions } from '../state_machine';

function calcHz(times: number[]): number {
  if (times.length < 2) return 0;
  const dt = times[times.length - 1] - times[0];
  return dt > 0 ? Math.round(((times.length - 1) / dt) * 10) / 10 : 0;
}

export function register(app: Express, deps: AppDeps): void {
  const { state, eventLog } = deps;

  app.get('/api/status', (_req, res) => {
    const s = state;
    res.json({
      robot_state: s.robotState,
      allowed_actions: allowedActions(s.robotState),
      wheel_odom_hz: calcHz(s.odomTimes),
      velocity: s.latestVelocity,
      e_stop: s.eStopActive,
      motors_armed: s.motorState.armed,
      left_state: s.motorState.left_state,
      right_state: s.motorState.right_state,
      motor_errors: (s.motorState.left_errors !== 0 || s.motorState.right_errors !== 0),
      drive_online: s.motorState.armed || s.motorState.left_state > 0,
      slam_tracking: s.slamTracking,
      recording: s.recordingActive,
      clients: s.activeClients,
      mode: s.currentMode,
      pose: s.robotPose,
      nav_state: s.navState,
      health: s.health,
      mission_progress: s.missionProgress,
    });
  });

  app.get('/api/health', (_req, res) => res.json(state.health));
  app.get('/api/mode', (_req, res) => res.json({ mode: state.currentMode }));

  // ── Iter-37 Phase 2 state endpoints — HIL iter-33/34 stack (19/20) ──
  // Read-only mirrors of /agv/{mode,zone,rail_driver}/state + rail_approach.
  // The frontend polls these or receives the same snapshot via the 5 Hz
  // WebSocket broadcast under `type='rail_state'`.
  app.get('/api/rail/state', (_req, res) => {
    res.json({
      mode_arbiter: state.modeArbiterState,
      zone: state.zoneDetectorState,
      rail_driver: state.railDriverState,
      rail_approach: { state: state.railApproachState },
    });
  });
  app.get('/api/zone/state', (_req, res) => res.json(state.zoneDetectorState));
  app.get('/api/mode/arbiter', (_req, res) => res.json(state.modeArbiterState));
  app.get('/api/rail_driver/state', (_req, res) => res.json(state.railDriverState));

  app.put('/api/mode', async (req, res) => {
    const mode = req.body?.mode;
    if (!['teleop', 'mapping', 'nav'].includes(mode)) {
      return res.status(400).json({ success: false, message: `Invalid mode: ${mode}` });
    }
    if (mode === state.currentMode) {
      return res.json({ success: true, mode: state.currentMode });
    }
    // Mode transitions into 'nav' validate Nav2 lifecycle state first.
    // Rejections are surfaced as HTTP 409 so the dashboard can show the
    // reason; the state remains in the previous mode.
    const result = await deps.setMode(mode);
    if (!result.ok) {
      return res.status(409).json({
        success: false,
        mode: state.currentMode,
        message: result.reason || 'setMode rejected',
      });
    }
    res.json({ success: true, mode: state.currentMode });
  });
}
