/**
 * Hero localization tile — analog of the reference HMI's "RTK" tile.
 *
 * Greenhouse deployment uses SLAM (no GPS/RTK), so the tile reads the
 * auto_init_orchestrator's localization.action field:
 *   UNKNOWN | INITIALIZING | LOCALIZED | DEGRADED | FAILED
 *
 * We show one of four visual states (ok/warn/crit/unknown) with a glyph,
 * matching the topbar's existing LOC pill but at hero scale.
 */
import type { RobotStatus } from '../../api/types';

type Loc = RobotStatus['localization'];

interface Props {
  localization: Loc | undefined;
}

const LABEL: Record<string, string> = {
  UNKNOWN: '…',
  INITIALIZING: 'INIT',
  LOCALIZED: 'LOCALIZED',
  DEGRADED: 'DEGRADED',
  FAILED: 'FAILED',
};

function tier(action: string | undefined): 'ok' | 'warn' | 'crit' | 'unknown' {
  if (action === 'LOCALIZED') return 'ok';
  if (action === 'INITIALIZING') return 'warn';
  if (action === 'DEGRADED') return 'warn';
  if (action === 'FAILED') return 'crit';
  return 'unknown';
}

// Compass/lock combo: a chevron pointing to a point with a tiny lock when good.
function LocGlyph({ tier: t }: { tier: 'ok' | 'warn' | 'crit' | 'unknown' }) {
  if (t === 'crit') {
    return (
      <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.2" strokeLinecap="round">
        <circle cx="12" cy="12" r="9" />
        <path d="M6 6 L18 18 M18 6 L6 18" />
      </svg>
    );
  }
  return (
    <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
      <circle cx="12" cy="12" r="9" />
      <path d="M12 5 L15 12 L12 11 L9 12 Z" fill="currentColor" />
      {t === 'ok' && <circle cx="12" cy="12" r="1.4" fill="currentColor" />}
    </svg>
  );
}

export function LocalizationTile({ localization }: Props) {
  const action = localization?.action ?? 'UNKNOWN';
  const t = tier(action);
  const label = LABEL[action] ?? action;
  const detail = localization?.detail || (localization?.map ? `Mapa: ${localization.map}` : '');

  return (
    <div className={`hero-tile hero-tile--loc hero-tile--loc-${t}`} title={detail || `Localization: ${action}`} role="status">
      <span className="hero-tile-eyebrow">LOCALIZATION</span>
      <div className="hero-tile-body">
        <span className="hero-tile-icon"><LocGlyph tier={t} /></span>
        <div className="hero-tile-stack">
          <span className="hero-tile-value hero-tile-value--bold">{label}</span>
          {detail && <span className="hero-tile-sub">{detail}</span>}
        </div>
      </div>
    </div>
  );
}
