/**
 * ControlModeRail — two big pills (MANUAL / NAVEGACIÓN) at the top of the
 * mission cockpit. Replaces the small mode-toggle buttons that used to sit
 * in the OperatePanel.
 *
 * The pills are 56 px tall (--mode-pill-h) so they're comfortably clickable
 * with a gloved hand. Active state uses the existing --blue accent.
 */

interface Props {
  mode: string;
  canChange: boolean;
  onChange: (m: string) => void;
}

const MODES: Array<{ id: string; label: string; sub: string }> = [
  { id: 'teleop', label: 'MANUAL', sub: 'Joystick' },
  { id: 'nav',    label: 'NAVEGACIÓN', sub: 'Goals + misiones' },
];

export function ControlModeRail({ mode, canChange, onChange }: Props) {
  return (
    <div className="cockpit-section">
      <div className="cockpit-eyebrow">MODO DE CONTROL</div>
      <div className="control-mode-rail" role="tablist" aria-label="Modo de control">
        {MODES.map(m => {
          const active = mode === m.id;
          return (
            <button
              key={m.id}
              role="tab"
              aria-selected={active}
              className={`control-mode-pill ${active ? 'active' : ''}`}
              onClick={() => onChange(m.id)}
              disabled={!canChange && !active}
            >
              <span className="control-mode-pill-label">{m.label}</span>
              <span className="control-mode-pill-sub">{m.sub}</span>
            </button>
          );
        })}
      </div>
    </div>
  );
}
