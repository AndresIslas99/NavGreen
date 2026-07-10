/**
 * TopBar — slim 56px header.
 *
 * Composition (left → right):
 *   [Brand lockup]  [Map pill]  ┄ spacer ┄  [conn dot]  [user]  [E-STOP]
 *
 * The state / battery / localization indicators are NOT here — they live in
 * the dedicated HeroRow component below the topbar. The old metrics row
 * (vel / Hz / SLAM / pose / SAFETY / RailStatus) was demoted to the cockpit
 * panel's "Detalles técnicos" collapsible section so the topbar stays calm.
 */
import { useEffect, useRef, useState } from 'react'
import type { RobotStatus, RobotState } from '../api/types'
import { Pill, Button, StatusDot } from './ui'
import { MapIcon, WifiOff, Wifi, LogOut, User } from './ui/icons'

interface Props {
  status: RobotStatus | null
  state: RobotState
  connected: boolean
  onEStop: (active: boolean) => void
  onNavCancel: () => void
  username?: string
  userRole?: string
  onLogout?: () => void
}

// Ada Labs brand lockup — official logomark + wordmark. The PNG lives in
// /public; we prefix with import.meta.env.BASE_URL because Vite serves
// the dashboard under `/dashboard/` (see vite.config.ts → `base`),
// so a bare `/adalabs_logo_transparent.png` 404s in dev.
const ADALABS_LOGO_SRC = `${import.meta.env.BASE_URL}adalabs_logo_transparent.png`

function BrandLogo() {
  return (
    <img
      src={ADALABS_LOGO_SRC}
      alt="Ada Labs"
      className="topbar-brand__logo"
      draggable={false}
    />
  )
}

export function TopBar({
  status, state: _state, connected,
  onEStop, onNavCancel, username, userRole, onLogout,
}: Props) {
  const navActive = status?.nav_state?.active || false
  const mapName = status?.current_map_name ?? null
  const [showUserMenu, setShowUserMenu] = useState(false)
  const userMenuRef = useRef<HTMLDivElement>(null)

  // a11y: Escape closes; pointer outside closes. Only active while open.
  useEffect(() => {
    if (!showUserMenu) return
    const onKey = (e: KeyboardEvent) => { if (e.key === 'Escape') setShowUserMenu(false) }
    const onDown = (e: PointerEvent) => {
      if (userMenuRef.current && !userMenuRef.current.contains(e.target as Node)) {
        setShowUserMenu(false)
      }
    }
    window.addEventListener('keydown', onKey)
    window.addEventListener('pointerdown', onDown)
    return () => {
      window.removeEventListener('keydown', onKey)
      window.removeEventListener('pointerdown', onDown)
    }
  }, [showUserMenu])

  return (
    <header className="topbar">
      <div className="topbar-brand">
        <BrandLogo />
        <div className="topbar-brand__text">
          <span className="topbar-brand__name">AGV-01</span>
          <span className="topbar-brand__sub">Greenhouse 1</span>
        </div>
      </div>

      <Pill
        tone={mapName ? 'accent' : 'neutral'}
        size="md"
        leadingIcon={MapIcon}
      >
        {mapName ?? 'Mapeando…'}
      </Pill>

      <div className="topbar-spacer" />

      <span className="topbar-conn" title={connected ? 'Conectado al robot' : 'Sin conexión'}>
        {connected
          ? <StatusDot tone="ok" size="md" label="Conectado" />
          : <Wifi size={16} className="topbar-conn__icon" style={{ display: 'none' }} />}
        {!connected && <WifiOff size={16} className="topbar-conn__icon" />}
        <span className="topbar-conn__label">{connected ? 'Conectado' : 'Sin conexión'}</span>
      </span>

      {username && (
        <div className="topbar-user" ref={userMenuRef}>
          <button
            type="button"
            className="topbar-user__trigger"
            onClick={() => setShowUserMenu(v => !v)}
            aria-label={`Usuario ${username}`}
            aria-expanded={showUserMenu}
            aria-haspopup="menu"
          >
            <span className="topbar-user__avatar">
              <User size={14} />
            </span>
            <span className="topbar-user__name">{username}</span>
            <span className="topbar-user__role">{userRole}</span>
          </button>
          {showUserMenu && (
            <div className="topbar-user__menu" role="menu">
              {onLogout && (
                <Button
                  variant="ghost"
                  size="sm"
                  leadingIcon={LogOut}
                  onClick={() => { setShowUserMenu(false); onLogout() }}
                >
                  Cerrar sesión
                </Button>
              )}
            </div>
          )}
        </div>
      )}

      {navActive && (
        <Button variant="ghost" size="sm" onClick={onNavCancel}>
          Cancelar nav
        </Button>
      )}

      <button
        type="button"
        className={`topbar-estop ${status?.e_stop ? 'topbar-estop--engaged' : ''}`}
        onClick={() => onEStop(!status?.e_stop)}
        aria-label={status?.e_stop ? 'Liberar paro de emergencia' : 'Activar paro de emergencia'}
        title={status?.e_stop ? 'Click para liberar el paro' : 'Paro de emergencia'}
      >
        {status?.e_stop ? 'LIBERAR' : 'E-STOP'}
      </button>
    </header>
  )
}
