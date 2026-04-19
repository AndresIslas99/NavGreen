import type { RobotStatus, RobotState, HealthMap } from '../../api/types'

interface Props {
  status: RobotStatus | null
  state: RobotState
  health: HealthMap
  onEStop: (active: boolean) => void
  onMotorEnable: (active: boolean) => void
  onNavCancel: () => void
}

// ISA-101: gray for normal, color only for abnormalities
const HEALTH_COLORS: Record<string, string> = {
  ok: 'var(--normal)',
  warn: 'var(--orange)',
  error: 'var(--red)',
  unknown: 'var(--dim)',
}

const SUBSYSTEM_LABELS: Record<string, string> = {
  drive: 'Drive / Odom',
  imu: 'IMU / Global Odom',
  slam: 'Visual SLAM',
  nav: 'Nav2 Stack',
  network: 'Network',
}

export function RecoveryPanel({ status, state, health, onEStop, onMotorEnable, onNavCancel }: Props) {
  const s = status

  return (
    <div className="context-panel">
      {/* Safety controls */}
      <div className="panel-section">
        <div className="section-title">Safety</div>
        <button
          className={`full-width ${s?.e_stop ? 'stop-btn' : 'action-btn'}`}
          onClick={() => onEStop(!s?.e_stop)}
        >
          {s?.e_stop ? 'Clear E-Stop' : 'Activate E-Stop'}
        </button>
      </div>

      <div className="panel-section">
        <div className="section-title">Motors</div>
        <button
          className="full-width"
          onClick={() => onMotorEnable(!s?.motors_armed)}
        >
          {s?.motors_armed ? 'Disarm Motors' : 'Arm Motors'}
        </button>
      </div>

      {s?.nav_state?.active && (
        <div className="panel-section">
          <div className="section-title">Navigation</div>
          <button className="full-width stop-btn" onClick={onNavCancel}>
            Cancel Navigation
          </button>
        </div>
      )}

      {/* Subsystem Health (Improvement 3) */}
      <div className="panel-section">
        <div className="section-title">System Health</div>
        <div className="health-grid">
          {Object.entries(health).map(([key, h]) => (
            <div key={key} className="health-row">
              <span
                className="health-dot"
                style={{ background: HEALTH_COLORS[h.status] || 'var(--dim)' }}
              />
              <div className="health-info">
                <span className="health-name">{SUBSYSTEM_LABELS[key] || key}</span>
                <span className="health-detail">{h.detail}</span>
                {h.action && h.status !== 'ok' && (
                  <span className="health-action">{h.action}</span>
                )}
              </div>
            </div>
          ))}
        </div>
      </div>

      {/* Raw state info */}
      <div className="panel-section">
        <div className="section-title">State</div>
        <div className="diag-grid">
          <span className="diag-label">Robot State</span>
          <span className="diag-value">{state}</span>
          <span className="diag-label">Mode</span>
          <span className="diag-value">{s?.mode || '?'}</span>
          <span className="diag-label">L/R Motor</span>
          <span className="diag-value">{s?.left_state} / {s?.right_state}</span>
          <span className="diag-label">Clients</span>
          <span className="diag-value">{s?.clients || 0}</span>
        </div>
      </div>
    </div>
  )
}
