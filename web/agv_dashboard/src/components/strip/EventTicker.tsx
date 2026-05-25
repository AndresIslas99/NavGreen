/**
 * EventTicker — the right column of the bottom MissionStrip.
 *
 * Shows the most recent N events (newest first), each row prefixed by a
 * severity-tagged category pill: [CRIT NAV], [WARN BAT], etc.
 *
 * Replaces the previous EventLog at the bottom of the dashboard. The cluster
 * dedupe and filter chips from EventLog were intentionally dropped — the
 * ticker is now a tight 3-row glance, and the operator can clear in bulk via
 * the Clear button. Forensic history still lives in events.jsonl on disk
 * (see specs/persistence.yaml#event_log).
 */

import type { LogEntry } from '../../api/types';
import { categoryFor } from '../../utils/eventCategory';

interface Props {
  entries: LogEntry[];
  onClear?: () => void;
  maxRows?: number;
}

function formatTime(ts: number): string {
  const d = new Date(ts * 1000);
  return d.toLocaleTimeString('en-US', { hour12: false, hour: '2-digit', minute: '2-digit', second: '2-digit' });
}

export function EventTicker({ entries, onClear, maxRows = 3 }: Props) {
  // Newest first. Backend already orders chronologically but we don't assume.
  const recent = [...entries]
    .sort((a, b) => b.timestamp - a.timestamp)
    .slice(0, maxRows);

  return (
    <div className="mission-strip-events">
      <div className="mission-strip-events-header">
        <span className="mission-strip-eyebrow">EVENTOS</span>
        <span className="mission-strip-events-count">{entries.length}</span>
        {onClear && (
          <button className="mission-strip-events-clear" onClick={onClear} title="Limpiar registro de eventos">
            Limpiar
          </button>
        )}
      </div>
      <div className="mission-strip-events-list">
        {recent.length === 0 ? (
          <div className="mission-strip-events-empty">Sin eventos recientes</div>
        ) : (
          recent.map((e, i) => {
            const cat = categoryFor(e.subsystem, e.severity);
            return (
              <div key={`${e.timestamp}-${i}`} className="mission-strip-event-row">
                <span className={`event-pill event-pill--${cat.color}`}>
                  {e.severity === 'crit' ? 'CRIT' :
                   e.severity === 'warn' ? 'WARN' : 'INFO'} {cat.short}
                </span>
                <span className="mission-strip-event-text" title={e.text}>{e.text}</span>
                <span className="mission-strip-event-time">{formatTime(e.timestamp)}</span>
              </div>
            );
          })
        )}
      </div>
    </div>
  );
}
