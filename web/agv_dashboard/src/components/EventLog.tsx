import { useState, useRef, useEffect } from 'react'
import type { LogEntry } from '../api/types'

interface Props {
  entries: LogEntry[]
  onClear?: () => void
}

type Filter = 'all' | 'warn' | 'crit'

function formatTime(ts: number) {
  const d = new Date(ts * 1000)
  return d.toLocaleTimeString('en-US', { hour12: false, hour: '2-digit', minute: '2-digit', second: '2-digit' })
}

// ISA-101: severity colors as pill badges
const SEV_STYLE: Record<string, React.CSSProperties> = {
  info: { background: 'var(--normal-bg)', color: 'var(--dim)' },
  warn: { background: 'rgba(239,108,0,0.15)', color: 'var(--orange)' },
  crit: { background: 'rgba(211,47,47,0.15)', color: 'var(--red)' },
}

export function EventLog({ entries, onClear }: Props) {
  const [filter, setFilter] = useState<Filter>('all')
  const [expanded, setExpanded] = useState(false)
  const scrollRef = useRef<HTMLDivElement>(null)

  const filtered = filter === 'all' ? entries
    : filter === 'warn' ? entries.filter(e => e.severity === 'warn' || e.severity === 'crit')
    : entries.filter(e => e.severity === 'crit')

  // Auto-scroll to top (newest first)
  useEffect(() => {
    if (scrollRef.current) scrollRef.current.scrollTop = 0
  }, [entries.length])

  return (
    <div className={`event-log ${expanded ? 'expanded' : ''}`}>
      <div className="log-header">
        <div className="log-filters">
          <button className={`log-filter ${filter === 'all' ? 'active' : ''}`}
            onClick={() => setFilter('all')}>All</button>
          <button className={`log-filter ${filter === 'warn' ? 'active' : ''}`}
            onClick={() => setFilter('warn')}>Warn+</button>
          <button className={`log-filter ${filter === 'crit' ? 'active' : ''}`}
            onClick={() => setFilter('crit')}>Crit</button>
        </div>
        <span className="log-count">{filtered.length}</span>
        {onClear && (
          <button className="log-clear" onClick={onClear} title="Clear events">Clear</button>
        )}
        <button className="log-expand" onClick={() => setExpanded(!expanded)}>
          {expanded ? '▼' : '▲'}
        </button>
      </div>
      <div className="log-scroll" ref={scrollRef}>
        {filtered.length === 0 && <div className="log-empty">No events</div>}
        {filtered.map((e, i) => (
          <div key={i} className={`log-entry ${i % 2 === 0 ? 'even' : ''}`}>
            <span className="log-time">{formatTime(e.timestamp)}</span>
            <span className="log-sev-pill" style={SEV_STYLE[e.severity] || SEV_STYLE.info}>
              {e.severity.toUpperCase()}
            </span>
            <span className="log-sub">{e.subsystem}</span>
            <span className="log-text">{e.text}</span>
          </div>
        ))}
      </div>
    </div>
  )
}
