/**
 * StateTile — primary "what is the robot doing right now" indicator.
 *
 * Renders the canonical robot_state as a labeled hero tile with an icon
 * circle and a contextual sub-line. Sentence case for the value; the
 * descriptive sub keeps it human and calm.
 */
import { Tile } from '../ui/Tile';
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
  offline:           { label: 'Sin conexión', sub: 'Esperando enlace con el robot',  icon: Power,          tone: 'neutral' },
  idle:              { label: 'En espera',    sub: 'Todo en orden',                  icon: Circle,         tone: 'neutral' },
  ready:             { label: 'Listo',        sub: 'Listo para comandos',            icon: CheckCircle2,   tone: 'accent'  },
  mapping:           { label: 'Mapeando',     sub: 'Construyendo mapa SLAM',         icon: MapPin,         tone: 'accent'  },
  navigating:        { label: 'Navegando',    sub: 'En camino a destino',            icon: Play,           tone: 'accent'  },
  executing_mission: { label: 'En misión',    sub: 'Ejecutando misión',              icon: Play,           tone: 'accent'  },
  blocked:           { label: 'Bloqueado',    sub: 'Obstáculo detectado',            icon: Pause,          tone: 'warn'    },
  e_stop:            { label: 'Paro activo',  sub: 'Liberar para reanudar',          icon: AlertOctagon,   tone: 'crit'    },
  fault:             { label: 'Falla',        sub: 'Revisar diagnóstico',            icon: AlertTriangle,  tone: 'crit'    },
};

interface Props {
  state: RobotState;
}

export function StateTile({ state }: Props) {
  const meta = STATE_META[state];
  const Icon = meta.icon;
  return (
    <Tile tone={meta.tone}>
      <Tile.Icon><Icon size={24} strokeWidth={1.8} /></Tile.Icon>
      <Tile.Content>
        <Tile.Eyebrow>Estado</Tile.Eyebrow>
        <Tile.Value>{meta.label}</Tile.Value>
        <Tile.Sub>{meta.sub}</Tile.Sub>
      </Tile.Content>
    </Tile>
  );
}
