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

export interface CameraFollowApi {
  cameraMode: 'follow' | 'manual' | 'frozen';
  followRobot: boolean;
  poseStale: boolean;
  /** Re-engage following: smooth flyTo robot pose + reset zoom. */
  recenter: () => void;
  /** Programmatic fit-to-bounds (e.g. "Fit greenhouse" control). */
  fitBounds: (bounds: L.LatLngBoundsExpression) => void;
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

export function useCameraFollow(
  map: L.Map | null,
  pose: { x: number; y: number; theta: number } | null,
  worldToLatLng: (x: number, y: number) => L.LatLng,
  options: Options = {},
): CameraFollowApi {
  const opts = { ...DEFAULT_OPTIONS, ...options };

  const [followRobot, setFollowRobot] = useState(true);
  const [poseStale, setPoseStale] = useState(false);

  // Guard refs — distinguish OUR panTo/flyTo from operator-initiated moves.
  const programmaticMoveRef = useRef(false);
  // Skip new panTo if a previous animation is still running.
  const panInFlightRef = useRef(false);
  // Track last pose timestamp for stale detection.
  const lastPoseAtRef = useRef<number>(0);

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
      if (programmaticMoveRef.current) return;
      setFollowRobot(false);
    };
    const onZoomStart = () => {
      if (programmaticMoveRef.current) return;
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

  // Continuous follow — pan on every pose update when in follow mode.
  useEffect(() => {
    if (!map || !pose) return;
    if (!followRobot) return;
    if (poseStale) return;
    if (panInFlightRef.current) return;   // skip if previous animation still running

    const latlng = worldToLatLng(pose.x, pose.y);
    const target = computeBiasedTarget(latlng) ?? latlng;

    programmaticMoveRef.current = true;
    panInFlightRef.current = true;
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
    map.fitBounds(bounds, { padding: [48, 48] });
    // Programmatic operation — operator may want overview, so don't auto-follow.
    setFollowRobot(false);
  };

  const cameraMode: 'follow' | 'manual' | 'frozen' =
    poseStale ? 'frozen' : followRobot ? 'follow' : 'manual';

  return { cameraMode, followRobot, poseStale, recenter, fitBounds };
}
