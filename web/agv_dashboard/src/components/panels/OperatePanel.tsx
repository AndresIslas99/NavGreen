import { useCallback, useState } from 'react'
import { Joystick } from '../Joystick'
import type { AllowedActions, RobotState } from '../../api/types'

interface Props {
  actions: AllowedActions
  motorsArmed: boolean
  mode: string
  state: RobotState
  onCmdVel: (linear: number, angular: number) => void
  onMotorEnable: (active: boolean) => void
  onModeChange: (mode: string) => void
}

export function OperatePanel({ actions, motorsArmed, mode, state, onCmdVel, onMotorEnable, onModeChange }: Props) {
  const handleTeleop = useCallback(() => onModeChange('teleop'), [onModeChange])
  const handleNav = useCallback(() => onModeChange('nav'), [onModeChange])

  // Sprint E / MEDIUM-11-D-02. Disarming during an active navigation
  // or mission stops the wheels mid-traverse — the robot will coast and
  // any in-flight goal will fault. Force the operator to acknowledge
  // before we send the disarm command.
  const [showDisarmConfirm, setShowDisarmConfirm] = useState(false)
  const disarmIsDangerous = motorsArmed && (state === 'navigating' || state === 'executing_mission')

  return (
    <div className="context-panel">
      <div className="panel-section">
        <div className="section-title">Control Mode</div>
        <div className="btn-row">
          <button className={`mode-toggle ${mode === 'teleop' ? 'active' : ''}`} onClick={handleTeleop}>
            Teleop
          </button>
          <button className={`mode-toggle ${mode === 'nav' ? 'active' : ''}`} onClick={handleNav}>
            Nav
          </button>
        </div>
      </div>

      <div className="panel-section">
        <div className="section-title">Motors</div>
        <button
          className={`full-width ${motorsArmed ? 'armed' : ''}`}
          onClick={() => {
            if (disarmIsDangerous) {
              setShowDisarmConfirm(true)
            } else {
              onMotorEnable(!motorsArmed)
            }
          }}
          disabled={!actions.canMotorEnable && !motorsArmed}
        >
          {motorsArmed ? 'Disarm Motors' : 'Arm Motors'}
        </button>
      </div>

      {showDisarmConfirm && (
        <div className="modal-overlay" onClick={() => setShowDisarmConfirm(false)}>
          <div className="modal-body" onClick={e => e.stopPropagation()}>
            <h3 style={{ margin: '0 0 8px 0' }}>Disarm motors during active navigation?</h3>
            <p style={{ fontSize: 13, opacity: 0.85, lineHeight: 1.4, marginBottom: 16 }}>
              The robot is currently <strong>{state === 'executing_mission' ? 'executing a mission' : 'navigating to a goal'}</strong>.
              Disarming will cut motor torque mid-traverse — the active goal
              will fail and the robot will coast. Prefer <em>Cancel</em> in the
              top bar first, then disarm once stopped.
            </p>
            <div style={{ display: 'flex', gap: 8, justifyContent: 'flex-end' }}>
              <button onClick={() => setShowDisarmConfirm(false)}>Keep armed</button>
              <button onClick={() => { onMotorEnable(false); setShowDisarmConfirm(false) }}>
                Disarm anyway
              </button>
            </div>
          </div>
        </div>
      )}

      <div className="panel-section">
        <div className="section-title">Joystick</div>
        <Joystick
          enabled={actions.canTeleop && mode === 'teleop'}
          maxLinear={0.5}
          maxAngular={0.5}
          onMove={onCmdVel}
        />
      </div>

      {mode === 'nav' && (
        <div className="panel-hint">Click on map to send navigation goal</div>
      )}
    </div>
  )
}
