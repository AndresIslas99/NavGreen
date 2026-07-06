import { useState, useCallback, useEffect } from 'react'
import { useWebSocket } from './hooks/useWebSocket'
import type { ModeRail as ModeRailType, Waypoint, RobotState } from './api/types'
import * as api from './api/client'

import { LoginPage } from './components/LoginPage'
import { TopBar } from './components/TopBar'
import { ModeRail } from './components/ModeRail'
import { MapView } from './components/MapView'
import { CameraFeed } from './components/CameraFeed'
import { EventLog } from './components/EventLog'

import { OperatePanel } from './components/panels/OperatePanel'
import { MappingPanel } from './components/panels/MappingPanel'
import { MissionsPanel } from './components/panels/MissionsPanel'
import { RecoveryPanel } from './components/panels/RecoveryPanel'
import { AnalyticsPanel } from './components/panels/AnalyticsPanel'
import { AprilTagsPanel } from './components/panels/AprilTagsPanel'
import { WaypointBatteryPanel } from './components/panels/WaypointBatteryPanel'
import { AprilTagAssignmentModal } from './components/AprilTagAssignmentModal'
import { ReplaySlider } from './components/ReplaySlider'
import { FleetOverlay } from './components/FleetOverlay'
import { useFleetSocket } from './hooks/useFleetSocket'

import './styles/global.css'

export default function App() {
  // Auth state
  const [authChecked, setAuthChecked] = useState(false)
  const [authRequired, setAuthRequired] = useState(false)
  const [loggedIn, setLoggedIn] = useState(false)
  const [username, setUsername] = useState('')
  const [userRole, setUserRole] = useState('')

  useEffect(() => {
    api.getAuthStatus().then(s => {
      setAuthRequired(s.enabled)
      if (!s.enabled) setLoggedIn(true)
      else if (api.getToken()) setLoggedIn(true) // has stored token
      setAuthChecked(true)
    }).catch(() => {
      setLoggedIn(true) // if endpoint fails, skip auth
      setAuthChecked(true)
    })
  }, [])

  // Session expiry: a 401 from any non-auth endpoint clears the token
  // (api/client.ts) and lands back on the login page without a page reload.
  useEffect(() => {
    api.setUnauthorizedHandler(() => {
      setAuthRequired(true)
      setLoggedIn(false)
      setUsername('')
      setUserRole('')
    })
    return () => api.setUnauthorizedHandler(null)
  }, [])

  const handleLogin = (user: string, role: string) => {
    setLoggedIn(true)
    setUsername(user)
    setUserRole(role)
  }

  const handleLogout = () => {
    api.setToken(null)
    setLoggedIn(false)
    setUsername('')
    setUserRole('')
  }

  if (!authChecked) return null
  if (authRequired && !loggedIn) return <LoginPage onLogin={handleLogin} />

  return <Dashboard username={username} userRole={userRole} onLogout={handleLogout} />
}

function Dashboard({ username, userRole, onLogout }: { username: string; userRole: string; onLogout: () => void }) {
  const { connected, status, path, scanPoints, mapData, accMapData, events, recordingResult, send, pendingApriltag, dismissPendingApriltag } = useWebSocket()
  const { fleetConnected, robots: fleetRobots, selectedRobot, selectRobot } = useFleetSocket()

  const [rail, setRail] = useState<ModeRailType>('operate')
  const [pendingWaypoints, setPendingWaypoints] = useState<Waypoint[]>([])
  const [capturingWaypoints, setCapturingWaypoints] = useState(false)
  const [ghostPose, setGhostPose] = useState<{ x: number; y: number; theta: number } | null>(null)

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
            recordingResult={recordingResult}
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
      case 'analytics':
        return <AnalyticsPanel />
      case 'apriltags':
        return <AprilTagsPanel />
      case 'waypoint_battery':
        return (
          <WaypointBatteryPanel
            status={status}
            onSendGoal={(x, y, theta) => send({ type: 'nav_goal', x, y, theta })}
            onCancel={handleNavCancel}
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
        username={username}
        userRole={userRole}
        onLogout={onLogout}
      />

      <div className="body">
        <ModeRail active={rail} onChange={setRail} />

        <div className={`map-area ${state === 'mapping' ? 'mapping-layout' : ''}`}>
          <div className="map-section">
          <MapView
            mapData={state === 'mapping' && accMapData ? accMapData : mapData}
            pose={pose}
            path={path}
            scanPoints={scanPoints}
            mode={capturingWaypoints ? 'nav' : mode}
            onGoalClick={handleGoalClick}
            waypoints={capturingWaypoints ? pendingWaypoints : undefined}
            fleetRobots={fleetRobots}
            selectedRobot={selectedRobot}
            ghostPose={rail === 'analytics' ? ghostPose : null}
            mappingCoverage={status?.mapping_coverage}
          />
          <FleetOverlay
            robots={fleetRobots}
            selectedRobot={selectedRobot}
            onSelectRobot={selectRobot}
            connected={fleetConnected}
          />
          {state !== 'mapping' && <CameraFeed visible={rail === 'operate' || rail === 'map'} expanded={false} />}
          <ReplaySlider
            visible={rail === 'analytics'}
            onGhostPose={setGhostPose}
          />
          </div>

          {/* Expanded camera panel during mapping */}
          {state === 'mapping' && (
            <div className="mapping-camera-panel">
              <CameraFeed visible={true} expanded={true} />
            </div>
          )}
        </div>

        <div className="right-panel">
          {renderPanel()}
        </div>
      </div>

      <EventLog entries={events} onClear={() => fetch(api.apiUrl('/api/events'), { method: 'DELETE' })} />

      {pendingApriltag !== null && (
        <AprilTagAssignmentModal
          hardwareId={pendingApriltag}
          onClose={dismissPendingApriltag}
        />
      )}
    </div>
  )
}
