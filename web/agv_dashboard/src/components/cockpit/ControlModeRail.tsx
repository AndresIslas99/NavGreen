/**
 * ControlModeRail — two big pills at the top of the mission cockpit.
 *
 * Sentence case: "Manual" / "Navegación". Active pill uses the accent
 * green; inactive sits on surface-2 cream. Disabled while a mission is
 * actively running (operator can't switch modes mid-mission).
 */
import { Section } from '../ui/Section';
import { Gamepad, Crosshair } from '../ui/icons';
import type { LucideIcon } from '../ui/icons';

interface Props {
  mode: string;
  canChange: boolean;
  onChange: (m: string) => void;
}

interface ModeDef {
  id: string;
  label: string;
  sub: string;
  icon: LucideIcon;
}

const MODES: ModeDef[] = [
  { id: 'teleop', label: 'Manual',     sub: 'Joystick',         icon: Gamepad },
  { id: 'nav',    label: 'Navegación', sub: 'Goals + misiones', icon: Crosshair },
];

export function ControlModeRail({ mode, canChange, onChange }: Props) {
  return (
    <Section title="Modo de control">
      <div className="control-mode-rail" role="tablist" aria-label="Modo de control">
        {MODES.map(m => {
          const active = mode === m.id;
          const Icon = m.icon;
          return (
            <button
              key={m.id}
              role="tab"
              aria-selected={active}
              className={`mode-pill ${active ? 'mode-pill--active' : ''}`}
              onClick={() => onChange(m.id)}
              disabled={!canChange && !active}
            >
              <Icon size={18} strokeWidth={1.8} />
              <span className="mode-pill__label">{m.label}</span>
              <span className="mode-pill__sub">{m.sub}</span>
            </button>
          );
        })}
      </div>
    </Section>
  );
}
