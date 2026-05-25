/**
 * Hero battery tile — % (big) + estimated time-to-empty (small).
 *
 * TTE comes from the backend's derived `battery_time_to_empty_s` field
 * (rolling EMA slope of pct over time). When the slope is positive/flat
 * or there's insufficient data, the backend returns null and the tile
 * shows only the percentage with no misleading secondary line.
 *
 * Color tiers follow the existing topbar BAT badge: <20% red, <40% amber,
 * otherwise dim/neutral (ISA-101).
 */

interface Props {
  batteryPct: number | null | undefined;
  tteSeconds: number | null | undefined;
}

function formatTte(s: number): string {
  if (s < 60) return `${Math.round(s)} s`;
  const totalMin = Math.round(s / 60);
  if (totalMin < 60) return `~${totalMin} min`;
  const hours = Math.floor(totalMin / 60);
  const mins = totalMin % 60;
  return mins === 0 ? `~${hours} h` : `~${hours} h ${mins} min`;
}

function batteryClass(pct: number | null | undefined): string {
  if (pct == null || pct < 0) return 'hero-tile--battery-unknown';
  if (pct < 20) return 'hero-tile--battery-crit';
  if (pct < 40) return 'hero-tile--battery-warn';
  return 'hero-tile--battery-ok';
}

// Inline bolt+battery glyph to avoid an icon dep.
function BatteryGlyph({ pct }: { pct: number | null | undefined }) {
  const fillPct = pct == null || pct < 0 ? 0 : Math.max(0, Math.min(100, pct));
  // Battery body 30×14 with a 2px nub on the right; fill width = (26 * pct/100)
  const fillW = (26 * fillPct) / 100;
  return (
    <svg width="36" height="20" viewBox="0 0 36 20" fill="none" stroke="currentColor" strokeWidth="1.8">
      <rect x="1" y="3" width="30" height="14" rx="2" />
      <rect x="32" y="7" width="3" height="6" rx="1" fill="currentColor" stroke="none" />
      {fillPct > 0 && (
        <rect x="3" y="5" width={fillW} height="10" rx="1" fill="currentColor" stroke="none" opacity="0.85" />
      )}
    </svg>
  );
}

export function BatteryTile({ batteryPct, tteSeconds }: Props) {
  const known = batteryPct != null && batteryPct >= 0;
  const label = known ? `${Math.round(batteryPct!)}%` : 'N/A';
  const tteLabel = tteSeconds != null && tteSeconds > 0 ? formatTte(tteSeconds) : null;

  return (
    <div className={`hero-tile hero-tile--battery ${batteryClass(batteryPct)}`} role="status">
      <span className="hero-tile-eyebrow">BATTERY</span>
      <div className="hero-tile-body">
        <span className="hero-tile-icon"><BatteryGlyph pct={batteryPct} /></span>
        <div className="hero-tile-stack">
          <span className="hero-tile-value hero-tile-value--bold">{label}</span>
          {tteLabel && <span className="hero-tile-sub">{tteLabel} restantes</span>}
        </div>
      </div>
    </div>
  );
}
