/**
 * RobotIcon — top-down vehicle outline replacing the legacy circle+arrow.
 *
 * Real-world AGV dimensions from `src/agv_navigation/config/agv_geometry.yaml`:
 *   front +0.50 m, rear −0.30 m, half-width ±0.37 m
 *   → footprint 0.80 m × 0.74 m (aspect 1.08:1)
 *
 * The icon uses a fixed CSS pixel size (36×24) so it stays legible at any
 * zoom level. The 36:24 ≈ 1.5 ratio exaggerates the longitudinal axis
 * slightly so the operator can read the heading at a glance.
 *
 * Coloring is state-aware:
 *   - default → accent (forest green) stroke
 *   - blocked → warn (warm tan) stroke
 *   - e_stop, fault → crit (deep red) stroke
 *
 * This is a pure factory of `L.divIcon`; it does NOT mount React inside
 * Leaflet. The SVG is rendered as an inline HTML string, with theta
 * applied via CSS transform so updates are cheap.
 *
 * Heading convention (preserved from the original robotIcon):
 *   ROS theta=0 → robot faces world +X (east in CRS.Simple).
 *   CSS rotation: deg = -(theta * 180/π) + 90 (because SVG default is "up").
 */
import L from 'leaflet';
import type { RobotState } from '../../api/types';

type StrokeTone = 'accent' | 'warn' | 'crit';

const STROKE_BY_STATE: Record<RobotState, StrokeTone> = {
  offline:           'accent',
  idle:              'accent',
  ready:             'accent',
  mapping:           'accent',
  navigating:        'accent',
  executing_mission: 'accent',
  blocked:           'warn',
  e_stop:            'crit',
  fault:             'crit',
};

const STROKE_HEX: Record<StrokeTone, string> = {
  accent: '#2f6f2a',
  warn:   '#b8612e',
  crit:   '#a8392a',
};

const FILL_HEX: Record<StrokeTone, string> = {
  accent: '#e2eedc',   // accent-soft
  warn:   '#f6e7d4',   // warn-soft
  crit:   '#f5d8d2',   // crit-soft
};

export function robotIcon(theta: number, state: RobotState = 'idle'): L.DivIcon {
  const deg = -(theta * 180 / Math.PI) + 90;
  const tone = STROKE_BY_STATE[state] ?? 'accent';
  const stroke = STROKE_HEX[tone];
  const fill   = FILL_HEX[tone];

  // SVG with the body horizontal (long axis along +X). The container is
  // rotated as a whole, so the body always points in the heading direction.
  // 4 wheels visible as small dark rectangles at the corners.
  const svg = `
    <svg viewBox="0 0 36 24" width="36" height="24"
         style="transform: rotate(${deg}deg); transform-origin: 18px 12px; overflow: visible;">
      <defs>
        <filter id="rs" x="-30%" y="-30%" width="160%" height="160%">
          <feDropShadow dx="0" dy="1" stdDeviation="1" flood-opacity="0.18"/>
        </filter>
      </defs>
      <g filter="url(#rs)">
        <!-- Wheels (4 corners) — drawn first so the body sits over them -->
        <rect x="2"  y="0"  width="7" height="4" rx="1" fill="#1a2421" />
        <rect x="2"  y="20" width="7" height="4" rx="1" fill="#1a2421" />
        <rect x="27" y="0"  width="7" height="4" rx="1" fill="#1a2421" />
        <rect x="27" y="20" width="7" height="4" rx="1" fill="#1a2421" />
        <!-- Body — rounded rectangle, tone-tinted -->
        <rect x="3" y="4" width="30" height="16" rx="3"
              fill="${fill}" stroke="${stroke}" stroke-width="1.5"/>
        <!-- Heading wedge (front of vehicle) -->
        <path d="M28 8 L34 12 L28 16 Z" fill="${stroke}" />
        <!-- Center dot for base_link origin debugging -->
        <circle cx="18" cy="12" r="1.2" fill="${stroke}" opacity="0.6" />
      </g>
    </svg>`;

  return L.divIcon({
    className: 'robot-marker',
    html: svg,
    iconSize: [36, 24],
    iconAnchor: [18, 12],
  });
}
