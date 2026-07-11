/**
 * HeroRow — the 3 at-a-glance indicators that sit in the dashboard's second
 * grid row, between the topbar and the body. State / Battery / Localization.
 *
 * Lives at the top of the components tree (not inside topbar/) because it
 * occupies its OWN grid row in the .app layout — not a sibling of the
 * topbar header. This avoids the original Block A bug where the strip was
 * a fragment sibling and stole the body's 1fr row.
 */
import type { RobotStatus, RobotState } from '../api/types';
import { StateTile } from './topbar/StateTile';
import { BatteryTile } from './topbar/BatteryTile';
import { LocalizationTile } from './topbar/LocalizationTile';

interface Props {
  status: RobotStatus | null;
  state: RobotState;
  /** True while the WS is connected but no status payload has arrived yet.
      Tiles render skeleton shimmers instead of stale placeholders. */
  loading?: boolean;
}

export function HeroRow({ status, state, loading = false }: Props) {
  return (
    <div className="hero-row" role="region" aria-label="Indicadores principales del robot">
      <StateTile state={state} loading={loading} />
      <BatteryTile
        batteryPct={status?.battery_pct ?? null}
        tteSeconds={status?.battery_time_to_empty_s ?? null}
        loading={loading}
      />
      <LocalizationTile localization={status?.localization} loading={loading} />
    </div>
  );
}
