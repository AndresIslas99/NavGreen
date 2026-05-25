/**
 * MissionProgressCard — the left column of the bottom MissionStrip.
 *
 * Shows the current mission name + progress bar + N/M nodes + a heuristic
 * ETA based on average time per node. When no mission is running, shows a
 * neutral "Sin misión activa" line so the strip stays visually present.
 */

import type { MissionProgress } from '../../api/types';

interface Props {
  missionProgress: MissionProgress | null;
  distanceRemaining: number | null;   // metres to the next nav goal, when known
  startedAt: number | null;           // unix seconds when this mission started locally
}

function formatEta(s: number): string {
  if (s < 60) return `${Math.round(s)} s`;
  const m = Math.round(s / 60);
  if (m < 60) return `~${m} min`;
  const h = Math.floor(m / 60);
  const r = m % 60;
  return r ? `~${h} h ${r} min` : `~${h} h`;
}

export function MissionProgressCard({ missionProgress, distanceRemaining, startedAt }: Props) {
  if (!missionProgress) {
    return (
      <div className="mission-strip-progress mission-strip-progress--idle">
        <span className="mission-strip-eyebrow">MISIÓN</span>
        <span className="mission-strip-empty">Sin misión activa</span>
      </div>
    );
  }

  const { mission_name, current_node, total_nodes, status } = missionProgress;
  const done = current_node + 1;
  const total = total_nodes || 1;
  const pct = (done / total) * 100;

  // ETA: extrapolate from elapsed time vs nodes completed. Requires at least
  // one completed node and a startedAt timestamp; otherwise we just omit it.
  let eta: string | null = null;
  if (startedAt && current_node > 0 && status === 'running') {
    const elapsed = Date.now() / 1000 - startedAt;
    const perNode = elapsed / current_node;
    const remaining = total - done;
    if (remaining > 0 && perNode > 0 && Number.isFinite(perNode)) {
      eta = formatEta(remaining * perNode);
    }
  }

  return (
    <div className={`mission-strip-progress mission-strip-progress--${status}`}>
      <div className="mission-strip-progress-header">
        <span className="mission-strip-eyebrow">MISIÓN</span>
        <span className="mission-strip-name" title={mission_name}>{mission_name}</span>
        <span className="mission-strip-status">
          {status === 'running' ? '▶ EN MARCHA' :
           status === 'paused' ? '⏸ PAUSADA' :
           status === 'completed' ? '✓ COMPLETA' :
           status === 'failed' ? '⛔ FALLÓ' :
           status === 'canceled' ? '⊘ CANCELADA' : status}
        </span>
      </div>

      <div className="mission-strip-bar" role="progressbar" aria-valuemin={0} aria-valuemax={100} aria-valuenow={Math.round(pct)}>
        <div className="mission-strip-bar-fill" style={{ width: `${pct}%` }} />
      </div>

      <div className="mission-strip-progress-footer">
        <span className="mission-strip-progress-count">
          PROGRESO: <strong>{done}/{total}</strong> nodos ({Math.round(pct)}%)
        </span>
        {distanceRemaining != null && distanceRemaining > 0 && (
          <span className="mission-strip-progress-dist">
            · {distanceRemaining.toFixed(1)} m al siguiente
          </span>
        )}
        {eta && (
          <span className="mission-strip-progress-eta">· ETA {eta}</span>
        )}
      </div>
    </div>
  );
}
