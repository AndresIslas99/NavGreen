/**
 * TaskInfoCard — visualizes the current task contextually:
 *   - mapping mode → coverage % doughnut
 *   - nav mode + mission → waypoints N/M doughnut + ETA
 *   - nav mode + goal only → distance remaining
 *   - else → "Sin tarea activa" empty state (calm, not alarming)
 *
 * Uses the Card + EmptyState primitives so the visual language is uniform
 * with the rest of the cockpit.
 */
import type { RobotStatus, MissionProgress } from '../../api/types';
import { Section } from '../ui/Section';
import { Card } from '../ui/Card';
import { EmptyState } from '../ui/EmptyState';
import { Route } from '../ui/icons';

interface Props {
  mode: string;
  status: RobotStatus | null;
  missionProgress: MissionProgress | null;
}

function Doughnut({ pct, primaryLabel, secondaryLabel }: {
  pct: number; primaryLabel: string; secondaryLabel?: string;
}) {
  const clamped = Math.max(0, Math.min(100, pct));
  const r = 36, c = 2 * Math.PI * r;
  const dash = (clamped / 100) * c;
  return (
    <svg className="task-doughnut" viewBox="0 0 96 96" width="92" height="92" role="img" aria-label={`${primaryLabel}, ${Math.round(clamped)}% completo`}>
      <circle cx="48" cy="48" r={r} fill="none" stroke="var(--surface-2)" strokeWidth="8" />
      <circle
        cx="48" cy="48" r={r} fill="none"
        stroke="var(--accent)" strokeWidth="8"
        strokeLinecap="round"
        strokeDasharray={`${dash} ${c}`}
        transform="rotate(-90 48 48)"
        style={{ transition: 'stroke-dasharray var(--dur-slower) var(--ease)' }}
      />
      <text x="48" y="50" textAnchor="middle" className="task-doughnut__value">{primaryLabel}</text>
      {secondaryLabel && (
        <text x="48" y="64" textAnchor="middle" className="task-doughnut__label">{secondaryLabel}</text>
      )}
    </svg>
  );
}

function LinearMetric({ value, sub }: { value: string; sub?: string }) {
  return (
    <div className="task-metric">
      <span className="task-metric__value">{value}</span>
      {sub && <span className="task-metric__sub">{sub}</span>}
    </div>
  );
}

export function TaskInfoCard({ mode, status, missionProgress }: Props) {
  let heading = 'Tarea actual';
  let body: React.ReactNode;

  if (mode === 'mapping') {
    const pct = Math.round((status?.mapping_coverage ?? 0) * 100);
    heading = 'Mapeo en curso';
    body = (
      <Card padding="comfortable" className="task-card">
        <Doughnut pct={pct} primaryLabel={`${pct}%`} secondaryLabel="cobertura" />
      </Card>
    );
  } else if (
    missionProgress &&
    (missionProgress.status === 'running' || missionProgress.status === 'paused')
  ) {
    const done = missionProgress.current_node + 1;
    const total = missionProgress.total_nodes || 1;
    const pct = (done / total) * 100;
    heading = missionProgress.status === 'paused' ? 'Misión pausada' : 'Misión en curso';
    const remaining = status?.nav_state?.distance_remaining;
    body = (
      <Card padding="comfortable" className="task-card task-card--mission">
        <Doughnut pct={pct} primaryLabel={`${done}/${total}`} secondaryLabel="nodos" />
        <div className="task-card__meta">
          <span className="task-card__name" title={missionProgress.mission_name}>
            {missionProgress.mission_name}
          </span>
          {typeof remaining === 'number' && remaining > 0 && (
            <LinearMetric
              value={`${remaining.toFixed(1)} m`}
              sub="al siguiente nodo"
            />
          )}
        </div>
      </Card>
    );
  } else if (mode === 'nav' && status?.nav_state?.active) {
    const remaining = status.nav_state.distance_remaining;
    heading = 'Navegando';
    body = (
      <Card padding="comfortable" className="task-card">
        <LinearMetric value={`${remaining.toFixed(1)} m`} sub="al destino" />
      </Card>
    );
  } else {
    body = (
      <EmptyState
        icon={Route}
        title="Sin tarea activa"
        description="Selecciona una misión o envía un goal desde el mapa."
        compact
      />
    );
  }

  return (
    <Section title={heading}>
      {body}
    </Section>
  );
}
