/**
 * TaskInfoCard — visual "what's happening right now" card.
 *
 * Picks a meaningful metric per mode (since we don't harvest yet — no
 * payload to visualize like the reference HMI's payload bar):
 *   - mapping → coverage % with a doughnut
 *   - nav + mission → waypoints N/M + ETA
 *   - nav + standalone goal → distance remaining
 *   - otherwise → idle state hint
 */

import type { RobotStatus, MissionProgress } from '../../api/types';

interface Props {
  mode: string;
  status: RobotStatus | null;
  missionProgress: MissionProgress | null;
}

function Doughnut({ pct, label }: { pct: number; label: string }) {
  const clamped = Math.max(0, Math.min(100, pct));
  const r = 38, c = 2 * Math.PI * r;
  const dash = (clamped / 100) * c;
  return (
    <svg className="task-doughnut" viewBox="0 0 96 96" width="96" height="96">
      <circle cx="48" cy="48" r={r} fill="none" stroke="var(--border)" strokeWidth="8" />
      <circle
        cx="48" cy="48" r={r} fill="none"
        stroke="var(--blue)" strokeWidth="8"
        strokeLinecap="round"
        strokeDasharray={`${dash} ${c}`}
        transform="rotate(-90 48 48)"
      />
      <text x="48" y="46" textAnchor="middle" className="task-doughnut-value">{Math.round(clamped)}%</text>
      <text x="48" y="62" textAnchor="middle" className="task-doughnut-label">{label}</text>
    </svg>
  );
}

function Linear({ valueLabel, sub }: { valueLabel: string; sub?: string }) {
  return (
    <div className="task-linear">
      <span className="task-linear-value">{valueLabel}</span>
      {sub && <span className="task-linear-sub">{sub}</span>}
    </div>
  );
}

export function TaskInfoCard({ mode, status, missionProgress }: Props) {
  let heading = 'TAREA ACTUAL';
  let body: React.ReactNode = (
    <div className="task-empty">Sin tarea activa</div>
  );

  if (mode === 'mapping') {
    const pct = Math.round((status?.mapping_coverage ?? 0) * 100);
    heading = 'MAPEO';
    body = <Doughnut pct={pct} label="cobertura" />;
  } else if (missionProgress && (missionProgress.status === 'running' || missionProgress.status === 'paused')) {
    const done = missionProgress.current_node + 1;
    const total = missionProgress.total_nodes || 1;
    const pct = (done / total) * 100;
    heading = `MISIÓN${missionProgress.status === 'paused' ? ' (PAUSADA)' : ''}`;
    const remaining = status?.nav_state?.distance_remaining;
    body = (
      <div className="task-mission-body">
        <Doughnut pct={pct} label={`${done}/${total}`} />
        <div className="task-mission-meta">
          <span className="task-mission-name">{missionProgress.mission_name}</span>
          {typeof remaining === 'number' && remaining > 0 && (
            <Linear valueLabel={`${remaining.toFixed(1)} m`} sub="al siguiente nodo" />
          )}
        </div>
      </div>
    );
  } else if (mode === 'nav' && status?.nav_state?.active) {
    heading = 'NAVEGANDO';
    const remaining = status.nav_state.distance_remaining;
    body = <Linear valueLabel={`${remaining.toFixed(1)} m`} sub="al destino" />;
  }

  return (
    <div className="cockpit-section">
      <div className="cockpit-eyebrow">{heading}</div>
      <div className="task-info-card">{body}</div>
    </div>
  );
}
