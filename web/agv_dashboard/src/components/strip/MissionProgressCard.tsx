/**
 * MissionProgressCard — left column of the bottom MissionStrip.
 *
 * Active: status pill + mission name + progress bar (accent green) + footer
 * with N/M nodes, distance, ETA. Empty: EmptyState with Route icon.
 *
 * ETA extrapolated client-side from elapsed time + nodes completed.
 */
import type { MissionProgress } from '../../api/types';
import { Card } from '../ui/Card';
import { Pill } from '../ui/Pill';
import { EmptyState } from '../ui/EmptyState';
import { Route, Play, Pause, CheckCircle2, AlertOctagon, X } from '../ui/icons';
import type { LucideIcon } from '../ui/icons';

interface Props {
  missionProgress: MissionProgress | null;
  distanceRemaining: number | null;
  startedAt: number | null;
}

function formatEta(s: number): string {
  if (s < 60) return `${Math.round(s)} s`;
  const m = Math.round(s / 60);
  if (m < 60) return `~${m} min`;
  const h = Math.floor(m / 60);
  const r = m % 60;
  return r ? `~${h} h ${r} min` : `~${h} h`;
}

type Tone = 'neutral' | 'accent' | 'warn' | 'crit' | 'info';

interface StatusMeta {
  label: string;
  tone: Tone;
  icon: LucideIcon;
}

const STATUS_META: Record<MissionProgress['status'], StatusMeta> = {
  running:   { label: 'En marcha',   tone: 'accent',  icon: Play },
  paused:    { label: 'Pausada',     tone: 'warn',    icon: Pause },
  completed: { label: 'Completada',  tone: 'accent',  icon: CheckCircle2 },
  failed:    { label: 'Falló',       tone: 'crit',    icon: AlertOctagon },
  canceled:  { label: 'Cancelada',   tone: 'neutral', icon: X },
};

export function MissionProgressCard({ missionProgress, distanceRemaining, startedAt }: Props) {
  if (!missionProgress) {
    return (
      <Card padding="compact" className="strip-card strip-card--mission">
        <EmptyState
          icon={Route}
          title="Sin misión activa"
          description="Crea una misión en el panel Misiones para arrancar."
          compact
        />
      </Card>
    );
  }

  const { mission_name, current_node, total_nodes, status } = missionProgress;
  const done = current_node + 1;
  const total = total_nodes || 1;
  const pct = (done / total) * 100;
  const meta = STATUS_META[status] ?? STATUS_META.running;

  let eta: string | null = null;
  if (startedAt && current_node > 0 && status === 'running') {
    const elapsed = Date.now() / 1000 - startedAt;
    const perNode = elapsed / current_node;
    const remaining = total - done;
    if (remaining > 0 && perNode > 0 && Number.isFinite(perNode)) {
      eta = formatEta(remaining * perNode);
    }
  }

  const barClass =
    status === 'paused' ? 'mission-bar mission-bar--paused' :
    status === 'failed' ? 'mission-bar mission-bar--failed' :
    'mission-bar';

  return (
    <Card padding="compact" className={`strip-card strip-card--mission strip-card--status-${status}`}>
      <div className="strip-card__header">
        <span className="strip-card__eyebrow">Misión</span>
        <span className="strip-card__title" title={mission_name}>{mission_name}</span>
        <Pill tone={meta.tone} size="xs" leadingIcon={meta.icon}>{meta.label}</Pill>
      </div>

      <div
        className={barClass}
        role="progressbar"
        aria-valuemin={0}
        aria-valuemax={100}
        aria-valuenow={Math.round(pct)}
      >
        <div className="mission-bar__fill" style={{ width: `${pct}%` }} />
      </div>

      <div className="strip-card__footer">
        <span>
          Progreso: <strong>{done}/{total}</strong> nodos ({Math.round(pct)}%)
        </span>
        {distanceRemaining != null && distanceRemaining > 0 && (
          <span className="strip-card__footer-dim">· {distanceRemaining.toFixed(1)} m al siguiente</span>
        )}
        {eta && (
          <span className="strip-card__footer-dim">· ETA {eta}</span>
        )}
      </div>
    </Card>
  );
}
