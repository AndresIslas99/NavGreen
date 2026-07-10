/**
 * BatteryTile — battery % + estimated time-to-empty.
 *
 * TTE comes from the backend's heuristic (rolling slope of battery_pct).
 * Backend returns null when charging, flat, or insufficient data; the tile
 * shows just "%" in that case so we never invent numbers.
 *
 * Color tiers: <20% crit, <40% warn, otherwise accent. Unknown → neutral.
 */
import { Tile } from '../ui/Tile';
import { Skeleton } from '../ui/Skeleton';
import { Battery, BatteryLow, BatteryWarning, BatteryFull } from '../ui/icons';
import type { LucideIcon } from '../ui/icons';

type Tone = 'neutral' | 'accent' | 'warn' | 'crit';

interface Props {
  batteryPct: number | null | undefined;
  tteSeconds: number | null | undefined;
  loading?: boolean;
}

function formatTte(seconds: number): string {
  if (seconds < 60) return `${Math.round(seconds)} s restantes`;
  const totalMin = Math.round(seconds / 60);
  if (totalMin < 60) return `~${totalMin} min restantes`;
  const hours = Math.floor(totalMin / 60);
  const mins = totalMin % 60;
  return mins === 0
    ? `~${hours} h restantes`
    : `~${hours} h ${mins} min restantes`;
}

function pickIconAndTone(pct: number | null | undefined): { Icon: LucideIcon; tone: Tone } {
  if (pct == null || pct < 0) return { Icon: Battery,          tone: 'neutral' };
  if (pct < 20)               return { Icon: BatteryWarning,   tone: 'crit'    };
  if (pct < 40)               return { Icon: BatteryLow,       tone: 'warn'    };
  if (pct >= 90)              return { Icon: BatteryFull,      tone: 'accent'  };
  return                              { Icon: Battery,          tone: 'accent'  };
}

export function BatteryTile({ batteryPct, tteSeconds, loading }: Props) {
  const known = batteryPct != null && batteryPct >= 0;
  const { Icon, tone } = pickIconAndTone(batteryPct);
  const valueLabel = known ? `${Math.round(batteryPct!)}%` : '—';
  const subLabel =
    !known
      ? 'Esperando datos de batería'
      : tteSeconds != null && tteSeconds > 0
        ? formatTte(tteSeconds)
        : 'Calculando autonomía…';

  return (
    <Tile tone={tone}>
      <Tile.Icon><Icon size={24} strokeWidth={1.8} /></Tile.Icon>
      <Tile.Content>
        <Tile.Eyebrow>Batería</Tile.Eyebrow>
        <Tile.Value>
          {loading && !known ? <Skeleton variant="bar" width={72} height={22} /> : valueLabel}
        </Tile.Value>
        <Tile.Sub>
          {loading && !known ? <Skeleton variant="text" width={140} height={11} /> : subLabel}
        </Tile.Sub>
      </Tile.Content>
    </Tile>
  );
}
