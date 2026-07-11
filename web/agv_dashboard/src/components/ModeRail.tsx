/**
 * ModeRail — left vertical navigation. One pill per panel.
 *
 * Lucide icons (no more emoji). Active state uses accent-soft bg +
 * accent text, with a 2px accent strip on the left edge as a visual
 * indicator. Sentence-case labels in Spanish.
 */
import type { ModeRail as ModeRailType } from '../api/types'
import {
  Gamepad2, MapIcon, Route, Tag, BatteryCharging, Wrench, BarChart3,
} from './ui/icons'
import type { LucideIcon } from './ui/icons'

interface Props {
  active: ModeRailType
  onChange: (mode: ModeRailType) => void
}

interface ModeDef {
  key: ModeRailType
  icon: LucideIcon
  label: string
}

const MODES: ModeDef[] = [
  { key: 'operate',          icon: Gamepad2,         label: 'Operar' },
  { key: 'map',              icon: MapIcon,          label: 'Mapa' },
  { key: 'missions',         icon: Route,            label: 'Misiones' },
  { key: 'apriltags',        icon: Tag,              label: 'AprilTags' },
  { key: 'waypoint_battery', icon: BatteryCharging,  label: 'Batería' },
  { key: 'recovery',         icon: Wrench,           label: 'Recuperar' },
  { key: 'analytics',        icon: BarChart3,        label: 'Análisis' },
]

export function ModeRail({ active, onChange }: Props) {
  return (
    <nav className="mode-rail" aria-label="Secciones del dashboard">
      {MODES.map(m => {
        const isActive = active === m.key
        const Icon = m.icon
        return (
          <button
            key={m.key}
            className={`rail-btn ${isActive ? 'rail-btn--active' : ''}`}
            onClick={() => onChange(m.key)}
            title={m.label}
            aria-current={isActive ? 'page' : undefined}
          >
            <span className="rail-btn__icon"><Icon size={22} strokeWidth={1.8} /></span>
            <span className="rail-btn__label">{m.label}</span>
          </button>
        )
      })}
    </nav>
  )
}
