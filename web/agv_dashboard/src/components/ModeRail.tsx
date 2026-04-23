import type { ModeRail as ModeRailType } from '../api/types'

interface Props {
  active: ModeRailType
  onChange: (mode: ModeRailType) => void
}

const MODES: { key: ModeRailType; icon: string; label: string }[] = [
  { key: 'operate', icon: '🎮', label: 'Operate' },
  { key: 'map', icon: '🗺', label: 'Map' },
  { key: 'missions', icon: '📋', label: 'Missions' },
  { key: 'apriltags', icon: '🏷', label: 'AprilTags' },
  { key: 'waypoint_battery', icon: '🎯', label: 'Battery' },
  { key: 'recovery', icon: '🔧', label: 'Recovery' },
  { key: 'analytics', icon: '📊', label: 'Analytics' },
]

export function ModeRail({ active, onChange }: Props) {
  return (
    <nav className="mode-rail">
      {MODES.map(m => (
        <button
          key={m.key}
          className={`rail-btn ${active === m.key ? 'active' : ''}`}
          onClick={() => onChange(m.key)}
          title={m.label}
        >
          <span className="rail-icon">{m.icon}</span>
          <span className="rail-label">{m.label}</span>
        </button>
      ))}
    </nav>
  )
}
