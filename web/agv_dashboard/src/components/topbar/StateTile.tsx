/**
 * StateTile — primary "what is the robot doing right now" indicator.
 *
 * Renders the canonical robot_state as a labeled hero tile with an icon
 * circle and a contextual sub-line. Sentence case for the value; the
 * descriptive sub keeps it human and calm.
 */
import { Tile } from '../ui/Tile';
import { Skeleton } from '../ui/Skeleton';
import {
  Circle, Play, MapPin, AlertOctagon, AlertTriangle, CheckCircle2, Power, Pause,
} from '../ui/icons';
import type { LucideIcon } from '../ui/icons';
import type { RobotState } from '../../api/types';

type Tone = 'neutral' | 'accent' | 'warn' | 'crit';

interface StateMeta {
  label: string;
  sub: string;
  icon: LucideIcon;
  tone: Tone;
}

const STATE_META: Record<RobotState, StateMeta> = {
  offline:           { label: 'Sin conexión', sub: 'Reintentando enlace…',           icon: Power,          tone: 'neutral' },
  idle:              { label: 'En espera',    sub: 'Sin tarea activa',               icon: Circle,         tone: 'neutral' },
  ready:             { label: 'Listo',        sub: 'Listo para comandos',            icon: CheckCircle2,   tone: 'accent'  },
  mapping:           { label: 'Mapeando',     sub: 'Construyendo mapa SLAM',         icon: MapPin,         tone: 'accent'  },
  navigating:        { label: 'Navegando',    sub: 'En camino a destino',            icon: Play,           tone: 'accent'  },
  executing_mission: { label: 'En misión',    sub: 'Ejecutando misión',              icon: Play,           tone: 'accent'  },
  blocked:           { label: 'Bloqueado',    sub: 'Esperando despeje',              icon: Pause,          tone: 'warn'    },
  e_stop:            { label: 'Paro activo',  sub: 'Paro de emergencia activo',      icon: AlertOctagon,   tone: 'crit'    },
  fault:             { label: 'Falla',        sub: 'Revisa el panel Recuperar',      icon: AlertTriangle,  tone: 'crit'    },
};

interface Props {
  state: RobotState;
  loading?: boolean;
}

export function StateTile({ state, loading }: Props) {
  const meta = STATE_META[state];
  const Icon = meta.icon;
  // Only show the skeleton when we genuinely don't know the state yet —
  // i.e. status hasn't loaded AND state is still the default 'offline'.
  // Showing it for a known offline state would feel like a glitch.
  const isPlaceholder = loading && state === 'offline';
  return (
    <Tile tone={meta.tone}>
      <Tile.Icon><Icon size={24} strokeWidth={1.8} /></Tile.Icon>
      <Tile.Content>
        <Tile.Eyebrow>Estado</Tile.Eyebrow>
        <Tile.Value>
          {isPlaceholder ? <Skeleton variant="bar" width={130} height={22} /> : meta.label}
        </Tile.Value>
        <Tile.Sub>
          {isPlaceholder ? <Skeleton variant="text" width={170} height={11} /> : meta.sub}
        </Tile.Sub>
      </Tile.Content>
    </Tile>
  );
}
