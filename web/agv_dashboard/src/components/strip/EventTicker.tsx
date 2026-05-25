/**
 * EventTicker — right column of the bottom MissionStrip.
 *
 * Newest 3 events with severity-tagged category pills (CRIT NAV, WARN BAT,
 * INFO MIS, ...). Uses Pill primitive for the category badge.
 *
 * Forensic full history lives in events.jsonl (see specs/persistence.yaml).
 * The Clear button here just clears in-memory ring buffer.
 */
import type { LogEntry } from '../../api/types';
import { categoryFor } from '../../utils/eventCategory';
import { Card } from '../ui/Card';
import { Pill } from '../ui/Pill';
import { Button } from '../ui/Button';
import { EmptyState } from '../ui/EmptyState';
import { Activity, X } from '../ui/icons';

interface Props {
  entries: LogEntry[];
  onClear?: () => void;
  maxRows?: number;
}

function formatTime(ts: number): string {
  const d = new Date(ts * 1000);
  return d.toLocaleTimeString('en-US', {
    hour12: false, hour: '2-digit', minute: '2-digit', second: '2-digit',
  });
}

export function EventTicker({ entries, onClear, maxRows = 3 }: Props) {
  const recent = [...entries]
    .sort((a, b) => b.timestamp - a.timestamp)
    .slice(0, maxRows);

  return (
    <Card padding="compact" className="strip-card strip-card--events">
      <div className="strip-card__header">
        <span className="strip-card__eyebrow">Eventos</span>
        <span className="strip-card__counter">{entries.length}</span>
        {onClear && (
          <Button
            variant="ghost"
            size="sm"
            leadingIcon={X}
            onClick={onClear}
            title="Limpiar registro de eventos"
          >
            Limpiar
          </Button>
        )}
      </div>

      {recent.length === 0 ? (
        <EmptyState
          icon={Activity}
          title="Sin eventos recientes"
          compact
        />
      ) : (
        <ul className="event-list" aria-label="Eventos recientes">
          {recent.map((e, i) => {
            const cat = categoryFor(e.subsystem, e.severity);
            const sevLabel = e.severity === 'crit' ? 'CRIT' :
                             e.severity === 'warn' ? 'WARN' : 'INFO';
            return (
              <li key={`${e.timestamp}-${i}`} className="event-row">
                <Pill tone={cat.color === 'muted' ? 'neutral' : cat.color} size="xs">
                  {sevLabel} {cat.short}
                </Pill>
                <span className="event-row__text" title={e.text}>{e.text}</span>
                <span className="event-row__time">{formatTime(e.timestamp)}</span>
              </li>
            );
          })}
        </ul>
      )}
    </Card>
  );
}
