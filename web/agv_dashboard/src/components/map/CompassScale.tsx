/**
 * CompassScale — top-right floating chip showing two operator references:
 *   - compass arrow rotating with the robot's heading (N=up convention)
 *   - dynamic scale bar that grows/shrinks as the map zooms
 *
 * Rendered via createPortal into document.body so it lives above Leaflet's
 * compositor (same trick as RecenterButton — paint suppression inside
 * .map-bg compositor layers).
 */
import { useEffect, useState } from 'react';
import { createPortal } from 'react-dom';
import L from 'leaflet';

interface Props {
  map: L.Map | null;
  /** Robot heading in radians. */
  theta: number;
}

interface ScaleInfo {
  /** Width in pixels the bar should occupy. */
  px: number;
  /** Human-readable distance for the label (e.g. "5 m", "20 m"). */
  label: string;
}

/**
 * Choose a "nice" round distance that produces a bar between 60 and 120 px
 * at the current map zoom. CRS.Simple in MapView treats 1 world unit ≈
 * `2 ** zoom` container pixels at the equator. We sample two world points
 * and measure their container distance, then pick the nearest of a
 * round-number set (0.5, 1, 2, 5, 10, 20, 50 m) whose bar lands in range.
 */
function computeScale(map: L.Map): ScaleInfo {
  const center = map.getCenter();
  const p1 = map.latLngToContainerPoint(center);
  // 1 world unit east of center.
  const p2 = map.latLngToContainerPoint(L.latLng(center.lat, center.lng + 1));
  const pxPerMeter = Math.abs(p2.x - p1.x);

  const candidates = [0.5, 1, 2, 5, 10, 20, 50, 100];
  let best = candidates[0];
  let bestPx = pxPerMeter * best;
  for (const c of candidates) {
    const cpx = pxPerMeter * c;
    if (cpx >= 60 && cpx <= 130) { best = c; bestPx = cpx; break; }
    if (cpx < 60 && cpx > bestPx) { best = c; bestPx = cpx; }
  }
  const label = best >= 1 ? `${best} m` : `${(best * 100).toFixed(0)} cm`;
  return { px: Math.max(20, Math.min(160, bestPx)), label };
}

export function CompassScale({ map, theta }: Props) {
  const [scale, setScale] = useState<ScaleInfo | null>(null);

  useEffect(() => {
    if (!map) return;
    const update = () => setScale(computeScale(map));
    update();
    map.on('zoom move resize', update);
    return () => { map.off('zoom move resize', update); };
  }, [map]);

  if (!map) return null;

  // Compass arrow rotates with the robot heading. Convention matches
  // RobotIcon: theta=0 → robot faces east. The arrow points the same
  // direction as the robot, so the operator sees "robot is heading
  // this way" relative to true map north (up).
  const deg = -(theta * 180 / Math.PI) + 90;

  return createPortal(
    <div className="compass-scale" aria-hidden>
      <div className="compass-scale__compass" title="Brújula — apunta hacia donde mira el robot">
        <svg width="40" height="40" viewBox="0 0 40 40">
          <circle cx="20" cy="20" r="17" fill="none"
                  stroke="rgba(26, 36, 33, 0.18)" strokeWidth="1" />
          <text x="20" y="9" textAnchor="middle" fontSize="8"
                fill="var(--text-secondary)" fontWeight="700"
                style={{ letterSpacing: '0.5px' }}>N</text>
          <g style={{ transform: `rotate(${deg}deg)`, transformOrigin: '20px 20px' }}>
            <path d="M20 7 L24 22 L20 19 L16 22 Z"
                  fill="var(--accent)" stroke="var(--accent-hover)" strokeWidth="0.5"
                  strokeLinejoin="round" />
          </g>
          <circle cx="20" cy="20" r="1.4" fill="var(--text)" />
        </svg>
      </div>
      {scale && (
        <div className="compass-scale__bar" title="Escala del mapa">
          <div className="compass-scale__bar-line" style={{ width: `${scale.px}px` }} />
          <span className="compass-scale__bar-label">{scale.label}</span>
        </div>
      )}
    </div>,
    document.body,
  );
}
