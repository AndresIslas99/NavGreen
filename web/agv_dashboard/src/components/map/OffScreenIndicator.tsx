/**
 * OffScreenIndicator — chevron pinned to the viewport edge that points
 * back to the robot when the operator has panned away and the robot is
 * no longer inside the visible map bounds.
 *
 * Click → recenter (re-engages camera follow via the same hook action).
 *
 * Hidden when:
 *   - cameraMode === 'follow'  (robot is auto-centered, never off-screen)
 *   - robot is inside the current viewport bounds
 *   - pose is null
 *
 * The chevron is anchored at the projected intersection of the line
 * (viewport-center → robot) with the viewport rectangle, padded inward
 * by 28 px so the icon stays fully visible. The rotation matches the
 * direction of the robot relative to the viewport center.
 */
import { useEffect, useState } from 'react';
import { createPortal } from 'react-dom';
import type L from 'leaflet';
import { Navigation } from '../ui/icons';

interface Props {
  map: L.Map | null;
  pose: { x: number; y: number; theta: number } | null;
  cameraMode: 'follow' | 'manual' | 'frozen';
  worldToLatLng: (x: number, y: number) => L.LatLng;
  /** Click handler — call hook's recenter(). */
  onRecenter: () => void;
}

interface Projection {
  /** Container-relative pixel position (top-left origin). */
  x: number;
  y: number;
  /** Rotation in degrees: 0 = up, 90 = right, 180 = down, 270 = left. */
  rotationDeg: number;
}

const EDGE_PAD = 28;   // pixels of inset so the chevron isn't clipped by viewport

function project(map: L.Map, pose: { x: number; y: number }, worldToLatLng: (x: number, y: number) => L.LatLng): Projection | null {
  const size = map.getSize();
  if (size.x <= 0 || size.y <= 0) return null;
  const robotPx = map.latLngToContainerPoint(worldToLatLng(pose.x, pose.y));
  const cx = size.x / 2;
  const cy = size.y / 2;
  // Vector from viewport center to robot.
  const dx = robotPx.x - cx;
  const dy = robotPx.y - cy;

  // Inside viewport with reasonable margin → no indicator needed.
  // Caller decides whether to render at all (see useState below).
  const margin = 12;
  if (
    robotPx.x >= margin && robotPx.x <= size.x - margin &&
    robotPx.y >= margin && robotPx.y <= size.y - margin
  ) {
    return null;
  }

  // Project the line from center along (dx, dy) onto the padded rectangle.
  // Parametric: (cx + t*dx, cy + t*dy) hits an edge for the smallest t > 0
  // where the result lies inside the padded inner rect.
  const halfW = cx - EDGE_PAD;
  const halfH = cy - EDGE_PAD;
  if (halfW <= 0 || halfH <= 0) return null;
  const tx = dx === 0 ? Infinity : halfW / Math.abs(dx);
  const ty = dy === 0 ? Infinity : halfH / Math.abs(dy);
  const t = Math.min(tx, ty);
  const px = cx + dx * t;
  const py = cy + dy * t;

  // Rotation: Navigation icon points up by default. atan2(dx, -dy)
  // gives clockwise degrees with 0 = up.
  const rotationDeg = (Math.atan2(dx, -dy) * 180) / Math.PI;

  return { x: px, y: py, rotationDeg };
}

export function OffScreenIndicator({ map, pose, cameraMode, worldToLatLng, onRecenter }: Props) {
  const [proj, setProj] = useState<Projection | null>(null);

  useEffect(() => {
    if (!map || !pose) { setProj(null); return; }
    if (cameraMode === 'follow') { setProj(null); return; }

    const update = () => setProj(project(map, pose, worldToLatLng));
    update();

    // Recompute on pan/zoom/resize so the chevron stays anchored.
    map.on('move zoom resize', update);
    return () => { map.off('move zoom resize', update); };
  }, [map, pose, cameraMode, worldToLatLng]);

  if (!proj || !map) return null;

  // Convert map-container-relative pixel coords to viewport-fixed coords.
  const containerRect = map.getContainer().getBoundingClientRect();
  const vx = containerRect.left + proj.x;
  const vy = containerRect.top + proj.y;

  return createPortal(
    <button
      type="button"
      className="offscreen-indicator"
      onClick={onRecenter}
      onPointerDown={(e) => e.stopPropagation()}
      style={{
        left: `${vx}px`,
        top: `${vy}px`,
        transform: `translate(-50%, -50%) rotate(${proj.rotationDeg}deg)`,
      }}
      aria-label="El robot está fuera de la vista. Toca para centrar."
      title="Robot fuera de la vista — toca para centrar"
    >
      <Navigation size={20} strokeWidth={2.2} aria-hidden />
    </button>,
    document.body,
  );
}
