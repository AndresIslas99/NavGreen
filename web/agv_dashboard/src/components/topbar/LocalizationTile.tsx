/**
 * LocalizationTile — analog of the reference HMI's "Connectivity / RTK" tile.
 *
 * We use SLAM (no GNSS) so the tile binds to auto_init_orchestrator's
 * localization.action: UNKNOWN | INITIALIZING | LOCALIZED | DEGRADED | FAILED.
 * The sub line shows the detail message from the orchestrator, or the
 * current map name when there's no specific detail to share.
 */
import { Tile } from '../ui/Tile';
import { Skeleton } from '../ui/Skeleton';
import { Compass, Loader, LocateOff, AlertCircle } from '../ui/icons';
import type { LucideIcon } from '../ui/icons';
import type { RobotStatus } from '../../api/types';

type Loc = RobotStatus['localization'];
type Tone = 'neutral' | 'accent' | 'warn' | 'crit';

interface LocMeta {
  label: string;
  icon: LucideIcon;
  tone: Tone;
}

const LOC_META: Record<string, LocMeta> = {
  UNKNOWN:      { label: 'Sin datos',          icon: Compass,    tone: 'neutral' },
  INITIALIZING: { label: 'Localizando…',       icon: Loader,     tone: 'warn'    },
  LOCALIZED:    { label: 'Localizado',         icon: Compass,    tone: 'accent'  },
  DEGRADED:     { label: 'Localización débil', icon: AlertCircle, tone: 'warn'    },
  FAILED:       { label: 'Localización perdida', icon: LocateOff,  tone: 'crit'    },
};

const DEFAULT_META: LocMeta = { label: '—', icon: Compass, tone: 'neutral' };

interface Props {
  localization: Loc | undefined;
  loading?: boolean;
}

export function LocalizationTile({ localization, loading }: Props) {
  const action = localization?.action ?? 'UNKNOWN';
  const meta = LOC_META[action] ?? DEFAULT_META;
  const Icon = meta.icon;

  const sub =
    localization?.detail?.trim()
    || (localization?.map ? `Mapa: ${localization.map}` : 'Sin información');

  const isUnknown = !localization || action === 'UNKNOWN';

  return (
    <Tile tone={meta.tone}>
      <Tile.Icon>
        <Icon
          size={24}
          strokeWidth={1.8}
          className={action === 'INITIALIZING' ? 'ui-icon--spin' : ''}
        />
      </Tile.Icon>
      <Tile.Content>
        <Tile.Eyebrow>Localización</Tile.Eyebrow>
        <Tile.Value>
          {loading && isUnknown ? <Skeleton variant="bar" width={120} height={22} /> : meta.label}
        </Tile.Value>
        <Tile.Sub>
          {loading && isUnknown ? <Skeleton variant="text" width={160} height={11} /> : sub}
        </Tile.Sub>
      </Tile.Content>
    </Tile>
  );
}
