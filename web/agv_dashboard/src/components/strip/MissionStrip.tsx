/**
 * MissionStrip — replaces the bottom EventLog row. Two-column strip:
 *   left  = MissionProgressCard (current mission + bar + ETA)
 *   right = EventTicker         (last 3 events with severity pills)
 *
 * Matches the reference HMI's bottom mission row. The two columns share
 * the --log-h height defined in :root.
 *
 * NOTE: mission start time is tracked client-side here so the ETA can be
 * computed without a backend change. The ref is reset whenever mission_id
 * or status transitions to 'running' from anything else.
 */
import { useEffect, useRef } from 'react';
import type { LogEntry, MissionProgress } from '../../api/types';
import { MissionProgressCard } from './MissionProgressCard';
import { EventTicker } from './EventTicker';

interface Props {
  events: LogEntry[];
  missionProgress: MissionProgress | null;
  distanceRemaining: number | null;
  onClear?: () => void;
}

export function MissionStrip({ events, missionProgress, distanceRemaining, onClear }: Props) {
  const startedAtRef = useRef<{ id: string | null; t: number | null }>({ id: null, t: null });

  // Reset startedAt whenever a new mission begins running.
  useEffect(() => {
    if (!missionProgress) {
      startedAtRef.current = { id: null, t: null };
      return;
    }
    const { mission_id, status } = missionProgress;
    if (status === 'running') {
      if (startedAtRef.current.id !== mission_id || startedAtRef.current.t == null) {
        startedAtRef.current = { id: mission_id, t: Date.now() / 1000 };
      }
    } else if (status !== 'paused') {
      // completed / failed / canceled — clear
      startedAtRef.current = { id: null, t: null };
    }
    // paused: keep timer (resume should continue from existing elapsed)
  }, [missionProgress]);

  return (
    <div className="mission-strip">
      <MissionProgressCard
        missionProgress={missionProgress}
        distanceRemaining={distanceRemaining}
        startedAt={startedAtRef.current.t}
      />
      <EventTicker entries={events} onClear={onClear} maxRows={3} />
    </div>
  );
}
