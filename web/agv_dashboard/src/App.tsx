import { useState, useCallback } from 'react'
import { useWebSocket } from './hooks/useWebSocket'
import type { ModeRail as ModeRailType, Waypoint, RobotState } from './api/types'

import { TopBar } from './components/TopBar'
import { ModeRail } from './components/ModeRail'
import { MapCanvas } from './components/MapCanvas'
import { EventLog } from './components/EventLog'

import { OperatePanel } from './components/panels/OperatePanel'
import { MappingPanel } from './components/panels/MappingPanel'
import { MissionsPanel } from './components/panels/MissionsPanel'
import { RecoveryPanel } from './components/panels/RecoveryPanel'

import './styles/global.css'

export default function App() {
  const { connected, status, path, scanPoints, mapData, accMapData, events, send } = useWebSocket()

  const [rail, setRail] = useState<ModeRailType>('operate')
  const [pendingWaypoints, setPendingWaypoints] = useState<Waypoint[]>([])
  const [capturingWaypoints, setCapturingWaypoints] = useState(false)

  // Backend-authoritative state (no more frontend derivation)
  const state: RobotState = connected ? (status?.robot_state || 'idle') : 'offline'
  const actions = status?.allowed_actions || {
    canTeleop: false, canStartMapping: false, canStopMapping: false,
    canSendGoal: false, canExecuteMission: false, canSaveMap: false,
    canLoadMap: false, canMotorEnable: false, canPauseMission: false, canCancelNav: false,
  }
  const mode = status?.mode || 'teleop'
  const pose = status?.pose || { x: 0, y: 0, theta: 0 }
  const navActive = status?.nav_state?.active || false

  // --- Actions ---
  const handleEStop = useCallback((active: boolean) => {
    send({ type: 'e_stop', active })
  }, [send])

  const handleNavCancel = useCallback(() => {
    send({ type: 'nav_cancel' })
  }, [send])

  const handleCmdVel = useCallback((linear: number, angular: number) => {
    send({ type: 'cmd_vel', linear, angular })
  }, [send])

  const handleMotorEnable = useCallback((active: boolean) => {
    send({ type: 'motor_enable', active })
  }, [send])

  const handleModeChange = useCallback((m: string) => {
    send({ type: 'mode', mode: m })
  }, [send])

  const handleRecording = useCallback((action: 'start' | 'stop') => {
    send({ type: 'recording', action })
  }, [send])

  const handleGoalClick = useCallback((x: number, y: number) => {
    if (capturingWaypoints) {
      setPendingWaypoints(wp => [...wp, { x, y, theta: 0 }])
    } else if (mode === 'nav') {
      send({ type: 'nav_goal', x, y, theta: 0 })
    }
  }, [capturingWaypoints, mode, send])

  // --- Render right panel ---
  function renderPanel() {
    switch (rail) {
      case 'operate':
        return (
          <OperatePanel
            actions={actions}
            motorsArmed={status?.motors_armed || false}
            mode={mode}
            onCmdVel={handleCmdVel}
            onMotorEnable={handleMotorEnable}
            onModeChange={handleModeChange}
          />
        )
      case 'map':
        return (
          <MappingPanel
            state={state}
            actions={actions}
            motorsArmed={status?.motors_armed || false}
            onModeChange={handleModeChange}
            onRecording={handleRecording}
            onCmdVel={handleCmdVel}
          />
        )
      case 'missions':
        return (
          <MissionsPanel
            actions={actions}
            navActive={navActive}
            missionProgress={status?.mission_progress || null}
            pendingWaypoints={pendingWaypoints}
            capturingWaypoints={capturingWaypoints}
            onStartCapture={() => { setCapturingWaypoints(true); handleModeChange('nav') }}
            onClearWaypoints={() => { setPendingWaypoints([]); setCapturingWaypoints(false) }}
          />
        )
      case 'recovery':
        return (
          <RecoveryPanel
            status={status}
            state={state}
            health={status?.health || {}}
            onEStop={handleEStop}
            onMotorEnable={handleMotorEnable}
            onNavCancel={handleNavCancel}
          />
        )
    }
  }

  return (
    <div className="app">
      <TopBar
        status={status}
        state={state}
        connected={connected}
        onEStop={handleEStop}
        onNavCancel={handleNavCancel}
      />

      <div className="body">
        <ModeRail active={rail} onChange={setRail} />

        <div className="map-area">
          <MapCanvas
            mapData={state === 'mapping' && accMapData ? accMapData : mapData}
            pose={pose}
            path={path}
            scanPoints={scanPoints}
            mode={capturingWaypoints ? 'nav' : mode}
            onGoalClick={handleGoalClick}
            waypoints={capturingWaypoints ? pendingWaypoints : undefined}
          />
        </div>

        <div className="right-panel">
          {renderPanel()}
        </div>
      </div>

      <EventLog entries={events} />
    </div>
  )
}
