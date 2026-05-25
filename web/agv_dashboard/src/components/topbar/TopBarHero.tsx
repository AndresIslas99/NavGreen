/**
 * TopBarHero — the three prominent hero tiles that sit below the topbar
 * header. State / Battery / Localization.
 *
 * Lives below the existing topbar (not inside it) so the topbar's existing
 * brand + state badge + metrics row + actions stay in place. The hero strip
 * is always visible — it's the at-a-glance status surface operators read
 * from across the room.
 */
import type { RobotStatus, RobotState } from '../../api/types';
import { StateTile } from './StateTile';
import { BatteryTile } from './BatteryTile';
import { LocalizationTile } from './LocalizationTile';

interface Props {
  status: RobotStatus | null;
  state: RobotState;
}

export function TopBarHero({ status, state }: Props) {
  return (
    <div className="topbar-hero" role="group" aria-label="Indicadores principales">
      <StateTile state={state} />
      <BatteryTile
        batteryPct={status?.battery_pct ?? null}
        tteSeconds={status?.battery_time_to_empty_s ?? null}
      />
      <LocalizationTile localization={status?.localization} />
    </div>
  );
}
