import { useCallback } from 'react'
import { Joystick } from '../Joystick'
import type { AllowedActions } from '../../api/types'

interface Props {
  actions: AllowedActions
  motorsArmed: boolean
  mode: string
  onCmdVel: (linear: number, angular: number) => void
  onMotorEnable: (active: boolean) => void
  onModeChange: (mode: string) => void
}

export function OperatePanel({ actions, motorsArmed, mode, onCmdVel, onMotorEnable, onModeChange }: Props) {
  const handleTeleop = useCallback(() => onModeChange('teleop'), [onModeChange])
  const handleNav = useCallback(() => onModeChange('nav'), [onModeChange])

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
          onClick={() => onMotorEnable(!motorsArmed)}
          disabled={!actions.canMotorEnable && !motorsArmed}
        >
          {motorsArmed ? 'Disarm Motors' : 'Arm Motors'}
        </button>
      </div>

      <div className="panel-section">
        <div className="section-title">Joystick</div>
        <Joystick
          enabled={actions.canTeleop && mode === 'teleop'}
          maxLinear={0.5}
          maxAngular={1.0}
          onMove={onCmdVel}
        />
      </div>

      {mode === 'nav' && (
        <div className="panel-hint">Click on map to send navigation goal</div>
      )}
    </div>
  )
}
