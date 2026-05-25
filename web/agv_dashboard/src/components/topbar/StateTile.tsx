/**
 * Hero state tile — big icon + bold label, color-coded by robot_state.
 *
 * The single most important indicator on the dashboard. Visible from across
 * the room. ISA-101 discipline: gray for "normal" (IDLE/READY), color only
 * for abnormal/active states.
 */
import type { RobotState } from '../../api/types';

const STATE_LABELS: Record<RobotState, string> = {
  offline: 'OFFLINE',
  idle: 'IDLE',
  ready: 'READY',
  mapping: 'MAPPING',
  navigating: 'NAVIGATING',
  executing_mission: 'MISSION',
  blocked: 'BLOCKED',
  e_stop: 'E-STOP',
  fault: 'FAULT',
};

// Inline SVG glyphs scaled to ~32 px. We don't pull in an icon library here
// because the hero tiles need to remain dependency-light (this file is loaded
// on every dashboard frame).
function stateIcon(state: RobotState) {
  switch (state) {
    case 'e_stop':
      return (
        <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.4" strokeLinecap="round">
          <circle cx="12" cy="12" r="9" />
          <path d="M8 8 L16 16 M16 8 L8 16" />
        </svg>
      );
    case 'fault':
      return (
        <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.4" strokeLinecap="round" strokeLinejoin="round">
          <path d="M12 3 L22 20 H2 Z" />
          <path d="M12 10 V14" /><circle cx="12" cy="17" r="0.6" fill="currentColor" />
        </svg>
      );
    case 'mapping':
      return (
        <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.2" strokeLinecap="round" strokeLinejoin="round">
          <path d="M3 6 L9 4 L15 6 L21 4 V18 L15 20 L9 18 L3 20 Z" />
          <path d="M9 4 V18 M15 6 V20" />
        </svg>
      );
    case 'navigating':
    case 'executing_mission':
      return (
        <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.2" strokeLinecap="round" strokeLinejoin="round">
          <path d="M3 12 L21 4 L13 22 L11 13 Z" />
        </svg>
      );
    case 'blocked':
      return (
        <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.2" strokeLinecap="round">
          <circle cx="12" cy="12" r="9" />
          <path d="M5 5 L19 19" />
        </svg>
      );
    case 'ready':
      return (
        <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.4" strokeLinecap="round" strokeLinejoin="round">
          <path d="M5 12 L10 17 L19 7" />
        </svg>
      );
    default:  // idle, offline
      return (
        <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round">
          <circle cx="12" cy="12" r="8" />
        </svg>
      );
  }
}

interface Props {
  state: RobotState;
}

export function StateTile({ state }: Props) {
  return (
    <div className={`hero-tile hero-tile--state hero-tile--state-${state}`} role="status" aria-live="polite">
      <span className="hero-tile-eyebrow">STATE</span>
      <div className="hero-tile-body">
        <span className="hero-tile-icon">{stateIcon(state)}</span>
        <span className="hero-tile-value hero-tile-value--bold">{STATE_LABELS[state]}</span>
      </div>
    </div>
  );
}
