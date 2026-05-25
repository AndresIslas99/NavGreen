/**
 * RobotIcon — top-down vehicle outline replacing the legacy circle+arrow.
 *
 * Real-world AGV dimensions from `src/agv_navigation/config/agv_geometry.yaml`:
 *   front +0.50 m, rear −0.30 m, half-width ±0.37 m
 *   → footprint 0.80 m × 0.74 m (aspect 1.08:1)
 *
 * The icon uses a fixed CSS pixel size (48×40 with breathing room for the
 * state ring + wedge pulse) so it stays legible at any zoom level. The
 * vehicle body itself stays at 36×24 within that canvas.
 *
 * "Robot vivo" detail — the icon now reads as a character with mood:
 *   - State-aware glow ring around the body (accent/warn/crit tone)
 *   - Subtle wobble when the robot is actively moving (CSS @keyframes)
 *   - Heading wedge pulses outward while navigating
 *   - Low-battery red blinking dot when battery_pct < 15
 *
 * Coloring is state-aware:
 *   - default / idle / ready / mapping / navigating → accent (forest green)
 *   - blocked → warn (warm tan)
 *   - e_stop, fault → crit (deep red)
 *
 * This is still a pure factory of `L.divIcon`; it does NOT mount React inside
 * Leaflet. SVG is rendered as an inline HTML string with CSS data-attributes
 * driving animations, so updates only re-key the divIcon (~cheap).
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

export interface RobotIconOptions {
  /** Triggers low-battery red dot blinking on top of the robot. */
  lowBattery?: boolean;
}

const MOVING_STATES: ReadonlyArray<RobotState> = [
  'navigating', 'executing_mission', 'mapping',
];

export function robotIcon(
  theta: number,
  state: RobotState = 'idle',
  opts: RobotIconOptions = {},
): L.DivIcon {
  const deg = -(theta * 180 / Math.PI) + 90;
  const tone = STROKE_BY_STATE[state] ?? 'accent';
  const stroke = STROKE_HEX[tone];
  const fill   = FILL_HEX[tone];

  const moving = MOVING_STATES.includes(state);
  const halted = state === 'e_stop' || state === 'fault';

  // Outer canvas leaves room for the glow ring + wedge pulse without
  // clipping. The body still occupies the centre 36×24 area.
  const W = 48, H = 40;
  const CX = W / 2, CY = H / 2;
  const BODY_X = CX - 18, BODY_Y = CY - 12;   // 36×24 centered

  const ringStrokeOpacity = halted ? 0.85 : 0.55;
  const ringDataState = halted ? 'halted' : (moving ? 'moving' : 'idle');

  const svg = `
    <div class="robot-icon" data-state="${ringDataState}" data-tone="${tone}">
      <svg viewBox="0 0 ${W} ${H}" width="${W}" height="${H}"
           style="overflow: visible;">
        <defs>
          <filter id="rs" x="-30%" y="-30%" width="160%" height="160%">
            <feDropShadow dx="0" dy="1" stdDeviation="1" flood-opacity="0.18"/>
          </filter>
          <radialGradient id="contact-shadow" cx="50%" cy="50%" r="50%">
            <stop offset="0%"  stop-color="rgba(26,36,33,0.45)"/>
            <stop offset="60%" stop-color="rgba(26,36,33,0.18)"/>
            <stop offset="100%" stop-color="rgba(26,36,33,0)"/>
          </radialGradient>
        </defs>
        <!-- Contact shadow — soft ellipse beneath the chassis. NOT inside the
             rotated group so the shadow stays oriented to the world's "down"
             axis (sun-from-above convention) rather than rotating with the
             robot. Gives the vehicle a sense of weight against the ground. -->
        <ellipse class="robot-icon__shadow"
                 cx="${CX}" cy="${CY + 3}" rx="17" ry="6"
                 fill="url(#contact-shadow)" />
        <!-- State glow ring (drawn next, beneath the chassis) -->
        <circle class="robot-icon__ring"
                cx="${CX}" cy="${CY}" r="20"
                fill="none"
                stroke="${stroke}" stroke-width="1.2"
                stroke-opacity="${ringStrokeOpacity}"
                stroke-dasharray="2 3" />
        <!-- Body group — rotated as a whole to align with heading. -->
        <g filter="url(#rs)"
           transform="rotate(${deg} ${CX} ${CY})"
           class="robot-icon__body">
          <!-- Wheels (4 corners) -->
          <rect x="${BODY_X + 2}"  y="${BODY_Y + 0}"  width="7" height="4" rx="1" fill="#1a2421" />
          <rect x="${BODY_X + 2}"  y="${BODY_Y + 20}" width="7" height="4" rx="1" fill="#1a2421" />
          <rect x="${BODY_X + 27}" y="${BODY_Y + 0}"  width="7" height="4" rx="1" fill="#1a2421" />
          <rect x="${BODY_X + 27}" y="${BODY_Y + 20}" width="7" height="4" rx="1" fill="#1a2421" />
          <!-- Body chassis -->
          <rect x="${BODY_X + 3}" y="${BODY_Y + 4}" width="30" height="16" rx="3"
                fill="${fill}" stroke="${stroke}" stroke-width="1.5"/>
          <!-- Heading wedge -->
          <path class="robot-icon__wedge"
                d="M${BODY_X + 28} ${BODY_Y + 8} L${BODY_X + 34} ${BODY_Y + 12} L${BODY_X + 28} ${BODY_Y + 16} Z"
                fill="${stroke}" />
          <!-- base_link origin dot -->
          <circle cx="${CX}" cy="${CY}" r="1.2" fill="${stroke}" opacity="0.6" />
        </g>
        ${opts.lowBattery ? `
          <!-- Low-battery indicator: red dot top-right of canvas, blinking -->
          <circle class="robot-icon__lowbat"
                  cx="${W - 5}" cy="5" r="3.4"
                  fill="#a8392a" stroke="#fefdfb" stroke-width="1.2" />
        ` : ''}
      </svg>
    </div>`;

  return L.divIcon({
    className: 'robot-marker',
    html: svg,
    iconSize: [W, H],
    iconAnchor: [CX, CY],
  });
}
