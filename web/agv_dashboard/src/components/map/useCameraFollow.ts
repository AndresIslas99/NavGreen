/**
 * useCameraFollow — owns the "always-centered robot" follow logic.
 *
 * State machine derived from two booleans:
 *   followRobot — true unless operator manually panned/zoomed.
 *   poseStale   — true after 2 s of no pose updates (SLAM drop or disconnect).
 *
 * cameraMode = follow  → animated panTo on every pose update.
 * cameraMode = manual  → no auto-pan; recenter FAB + off-screen indicator visible.
 * cameraMode = frozen  → no auto-pan; status pill warns "Vista congelada".
 *
 * Key behaviors:
 * - Continuous follow uses panTo({ animate: true, duration: 0.30, easeLinearity: 0.25 })
 *   — 300 ms ease-out, matches Google Maps feel.
 * - Re-engage from manual uses flyTo (smoother retarget when zoom changed).
 * - User-pan detection is event-based: movestart + zoomstart events,
 *   gated by `programmaticMoveRef` so our own panTo/flyTo don't trigger
 *   disengagement.
 * - Throttle: pose updates are ~5 Hz; we skip new panTo while one is
 *   in flight via `panInFlightRef`. Effective rate ≈ 3 Hz, smooth.
 * - Centering target has a 25 % bottom bias so the operator sees more
 *   of what's ahead of the robot (Google Maps pattern).
 * - prefers-reduced-motion: collapses animations to instant.
 */
import { useEffect, useRef, useState } from 'react';
import type L from 'leaflet';
import type { RobotState } from '../../api/types';

export interface CameraFollowApi {
  cameraMode: 'follow' | 'manual' | 'frozen';
  followRobot: boolean;
  poseStale: boolean;
  /** Re-engage following: smooth flyTo robot pose + reset zoom. */
  recenter: () => void;
  /** Programmatic fit-to-bounds (e.g. "Fit greenhouse" control). */
  fitBounds: (bounds: L.LatLngBoundsExpression) => void;
  /**
   * Heading-up map rotation (Google Maps Nav-style). Returns the
   * degrees the leaflet-container should rotate so the robot's
   * heading appears UP on screen. 0 means "north-up, no rotation".
   * Active only while following the robot AND driving (navigating,
   * executing_mission, mapping). Hysteresis built in to avoid
   * flickering when theta drifts a fraction of a degree.
   */
  mapRotationDeg: number;
}

interface Options {
  /** Default zoom when entering follow mode. Higher = closer. */
  defaultZoom?: number;
  /** Fraction of viewport height to bias robot toward bottom (0=center, 0.25=below center). */
  bottomBias?: number;
  /** Stale-pose threshold (ms). */
  staleAfterMs?: number;
}

const DEFAULT_OPTIONS: Required<Options> = {
  defaultZoom: 4,
  bottomBias: 0.25,
  staleAfterMs: 2000,
};

function reducedMotion(): boolean {
  return typeof window !== 'undefined' &&
    window.matchMedia('(prefers-reduced-motion: reduce)').matches;
}

// States where the heading-up rotation is active. Mirrors Google Maps:
// rotate while actively navigating; stay north-up otherwise.
const DRIVING_STATES: ReadonlyArray<RobotState> = [
  'navigating', 'executing_mission', 'mapping',
];

export function useCameraFollow(
  map: L.Map | null,
  pose: { x: number; y: number; theta: number } | null,
  worldToLatLng: (x: number, y: number) => L.LatLng,
  state: RobotState | undefined,
  options: Options = {},
): CameraFollowApi {
  const opts = { ...DEFAULT_OPTIONS, ...options };

  const [followRobot, setFollowRobot] = useState(true);
  const [poseStale, setPoseStale] = useState(false);
  // Latched rotation value with hysteresis — only updates when the
  // robot's heading has changed by more than HEADING_DEAD_BAND_DEG so
  // tiny theta jitter while stationary doesn't cause visible spin.
  const [mapRotationDeg, setMapRotationDeg] = useState(0);
  const HEADING_DEAD_BAND_DEG = 2;
  const lastSignificantThetaRef = useRef<number | null>(null);

  // Guard refs — distinguish OUR panTo/flyTo from operator-initiated moves.
  const programmaticMoveRef = useRef(false);
  // Timestamp of last programmatic pan/fly. We treat any movestart within
  // GRACE_MS after a programmatic op as "still ours" — defends against
  // Leaflet firing movestart late (after moveend has already reset
  // programmaticMoveRef) or for follow-on internal events like setView's
  // post-pan refresh, which would otherwise flip followRobot=false.
  const lastProgrammaticAtRef = useRef<number>(0);
  // Skip new panTo if a previous animation is still running.
  const panInFlightRef = useRef(false);
  // Track last pose timestamp for stale detection.
  const lastPoseAtRef = useRef<number>(0);
  // Grace window: movestart within this many ms of a programmatic op is ours.
  const GRACE_MS = 400;
  const isWithinProgrammaticGrace = () =>
    programmaticMoveRef.current ||
    Date.now() - lastProgrammaticAtRef.current < GRACE_MS;

  // Compute the centering target with bottom-bias: the robot sits below
  // the geometric center of the viewport so the operator sees more of the
  // "forward" direction. The bias is a fraction of viewport height in pixels.
  const computeBiasedTarget = (latlng: L.LatLng): L.LatLng | null => {
    if (!map) return latlng;
    const containerH = map.getSize().y;
    const biasPx = containerH * opts.bottomBias;
    const point = map.latLngToContainerPoint(latlng);
    // Robot drawn at (containerH/2 + biasPx) → camera centered ABOVE the robot
    // by `biasPx`. So target latlng's container-y should be at the geometric
    // center MINUS biasPx (move target up so robot ends up below center).
    const targetPoint = { x: point.x, y: point.y - biasPx } as L.Point;
    return map.containerPointToLatLng(targetPoint as L.Point);
  };

  // Event listeners — disengage on operator-initiated pan/zoom.
  useEffect(() => {
    if (!map) return;
    const onMoveStart = () => {
      if (isWithinProgrammaticGrace()) return;
      setFollowRobot(false);
    };
    const onZoomStart = () => {
      if (isWithinProgrammaticGrace()) return;
      setFollowRobot(false);
    };
    const onMoveEnd = () => {
      programmaticMoveRef.current = false;
      panInFlightRef.current = false;
    };
    map.on('movestart', onMoveStart);
    map.on('zoomstart', onZoomStart);
    map.on('moveend', onMoveEnd);
    return () => {
      map.off('movestart', onMoveStart);
      map.off('zoomstart', onZoomStart);
      map.off('moveend', onMoveEnd);
    };
  }, [map]);

  // Pose-stale watchdog — flips poseStale to true after staleAfterMs of silence.
  useEffect(() => {
    if (!pose) return;
    lastPoseAtRef.current = Date.now();
    if (poseStale) setPoseStale(false);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [pose]);
  useEffect(() => {
    const id = window.setInterval(() => {
      const last = lastPoseAtRef.current;
      if (last === 0) return;
      const age = Date.now() - last;
      const stale = age > opts.staleAfterMs;
      if (stale !== poseStale) setPoseStale(stale);
    }, 500);
    return () => window.clearInterval(id);
  }, [poseStale, opts.staleAfterMs]);

  // Continuous follow — pan only when the pose VALUE changes (App re-creates
  // the pose object literal on every render, so depending on identity would
  // call panTo every ~500 ms even when the robot is stationary, renewing the
  // programmatic-grace window and breaking user-pan detection).
  const lastPoseValueRef = useRef<{ x: number; y: number } | null>(null);
  useEffect(() => {
    if (!map || !pose) return;
    if (!followRobot) return;
    if (poseStale) return;
    if (panInFlightRef.current) return;

    const last = lastPoseValueRef.current;
    if (last && Math.abs(last.x - pose.x) < 0.001 && Math.abs(last.y - pose.y) < 0.001) {
      return;   // pose value unchanged — skip pan, leave grace window alone
    }
    lastPoseValueRef.current = { x: pose.x, y: pose.y };

    const latlng = worldToLatLng(pose.x, pose.y);
    const target = computeBiasedTarget(latlng) ?? latlng;

    programmaticMoveRef.current = true;
    panInFlightRef.current = true;
    lastProgrammaticAtRef.current = Date.now();
    if (reducedMotion()) {
      map.panTo(target, { animate: false });
    } else {
      map.panTo(target, {
        animate: true,
        duration: 0.30,
        easeLinearity: 0.25,
        noMoveStart: true,
      });
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [pose, followRobot, poseStale]);

  // Initial mount: set zoom + center on robot (or fit-to-bounds fallback).
  // Triggered when map first exists AND we have a pose.
  const didInitialCenterRef = useRef(false);
  useEffect(() => {
    if (!map || didInitialCenterRef.current) return;
    if (!pose) return;
    const latlng = worldToLatLng(pose.x, pose.y);
    programmaticMoveRef.current = true;
    lastProgrammaticAtRef.current = Date.now();
    map.setView(latlng, opts.defaultZoom, { animate: false });
    didInitialCenterRef.current = true;
  }, [map, pose, worldToLatLng, opts.defaultZoom]);

  // Public actions
  const recenter = () => {
    if (!map || !pose) return;
    const latlng = worldToLatLng(pose.x, pose.y);
    const target = computeBiasedTarget(latlng) ?? latlng;
    programmaticMoveRef.current = true;
    panInFlightRef.current = true;
    lastProgrammaticAtRef.current = Date.now();
    if (reducedMotion()) {
      map.setView(target, opts.defaultZoom, { animate: false });
    } else {
      map.flyTo(target, opts.defaultZoom, {
        animate: true,
        duration: 0.50,
        easeLinearity: 0.25,
      });
    }
    setFollowRobot(true);
  };

  const fitBounds = (bounds: L.LatLngBoundsExpression) => {
    if (!map) return;
    programmaticMoveRef.current = true;
    lastProgrammaticAtRef.current = Date.now();
    map.fitBounds(bounds, { padding: [48, 48] });
    // Programmatic operation — operator may want overview, so don't auto-follow.
    setFollowRobot(false);
  };

  const cameraMode: 'follow' | 'manual' | 'frozen' =
    poseStale ? 'frozen' : followRobot ? 'follow' : 'manual';

  // ── Heading-up rotation ────────────────────────────────────────────
  // Active only while the camera is following AND the robot is in a
  // driving state. Outside of that (idle, blocked, e_stop, fault,
  // manual pan, frozen pose) the map snaps back to north-up.
  //
  // Formula (see plan): mapRotationDeg = theta_deg - 90, because the
  // icon is already rotated by `-theta` and we want the composed
  // visual rotation to be -90° (wedge pointing UP).
  const headingUpActive =
    followRobot && !poseStale &&
    state != null && DRIVING_STATES.includes(state) &&
    pose != null;
  useEffect(() => {
    if (!headingUpActive || !pose) {
      // Return to north-up.
      if (mapRotationDeg !== 0) setMapRotationDeg(0);
      lastSignificantThetaRef.current = null;
      return;
    }
    const thetaDeg = pose.theta * 180 / Math.PI;
    const last = lastSignificantThetaRef.current;
    if (last != null && Math.abs(thetaDeg - last) < HEADING_DEAD_BAND_DEG) {
      // Within dead band — keep current rotation.
      return;
    }
    lastSignificantThetaRef.current = thetaDeg;
    setMapRotationDeg(thetaDeg - 90);
  }, [headingUpActive, pose, mapRotationDeg]);

  return { cameraMode, followRobot, poseStale, recenter, fitBounds, mapRotationDeg };
}
