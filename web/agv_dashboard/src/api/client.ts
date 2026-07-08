import type { MapInfo, Mission, AuthStatus, AuthSession, TrafficZone, ZoneOccupancy, MissionRun } from './types'

// Sprint 1 Fase 1a: host-agnostic base URLs.
// VITE_API_BASE empty (default) preserves same-origin behavior — works when
// the backend serves the dashboard at /dashboard. When the dashboard runs on
// a different host (laptop x86 via nginx/caddy → Jetson backend), set
// VITE_API_BASE=http://<jetson-ip>:8090 and VITE_FLEET_BASE accordingly.
const API_BASE = (import.meta.env.VITE_API_BASE || '').replace(/\/+$/, '')
const FLEET_BASE_ENV = (import.meta.env.VITE_FLEET_BASE || '').replace(/\/+$/, '')

/** Prepend the API base to a path. Pass-through if base is empty. */
export function apiUrl(path: string): string {
  if (!path.startsWith('/')) path = '/' + path
  return API_BASE ? API_BASE + path : path
}

/** Compute a WebSocket URL for a given path, honoring VITE_API_BASE. */
export function wsUrl(path: string): string {
  if (!path.startsWith('/')) path = '/' + path
  if (API_BASE) {
    // http(s)://host:port → ws(s)://host:port
    return API_BASE.replace(/^http/, 'ws') + path
  }
  const proto = location.protocol === 'https:' ? 'wss' : 'ws'
  return `${proto}://${location.host}${path}`
}

/** Fleet manager origin (port 8092 by default; 8091 is agv_image_server). */
export function fleetBase(): string {
  if (FLEET_BASE_ENV) return FLEET_BASE_ENV
  if (API_BASE) {
    try {
      const u = new URL(API_BASE)
      return `${u.protocol}//${u.hostname}:8092`
    } catch { /* fall through */ }
  }
  return `${location.protocol}//${location.hostname}:8092`
}

/** WebSocket URL on the fleet manager origin, honoring VITE_FLEET_BASE. */
export function fleetWsUrl(path: string): string {
  if (!path.startsWith('/')) path = '/' + path
  // http(s)://host:port → ws(s)://host:port
  return fleetBase().replace(/^http/, 'ws') + path
}

// Auth token management
let authToken: string | null = localStorage.getItem('agv_token')

export function setToken(token: string | null) {
  authToken = token
  if (token) localStorage.setItem('agv_token', token)
  else localStorage.removeItem('agv_token')
}

export function getToken(): string | null { return authToken }

function authHeaders(): Record<string, string> {
  const h: Record<string, string> = { 'Content-Type': 'application/json' }
  if (authToken) h['Authorization'] = `Bearer ${authToken}`
  return h
}

// Invoked when a non-auth endpoint returns 401 (expired or revoked token).
// App registers a handler that routes to the login view via React state —
// never reload the page here, or a persistent 401 becomes a reload loop.
let unauthorizedHandler: (() => void) | null = null

export function setUnauthorizedHandler(handler: (() => void) | null) {
  unauthorizedHandler = handler
}

async function json<T>(url: string, init?: RequestInit): Promise<T> {
  const res = await fetch(apiUrl(url), {
    ...init,
    headers: { ...authHeaders(), ...(init?.headers || {}) },
  })
  // Auth endpoints are excluded: a 401 there means bad credentials, which
  // the login form reports inline rather than tearing down the session.
  if (res.status === 401 && !url.startsWith('/api/auth/')) {
    setToken(null)
    unauthorizedHandler?.()
    throw new Error(`Unauthorized: ${url}`)
  }
  return res.json()
}

function post(url: string, body: unknown) {
  return json(url, { method: 'POST', body: JSON.stringify(body) })
}

function put(url: string, body: unknown) {
  return json(url, { method: 'PUT', body: JSON.stringify(body) })
}

function del(url: string) {
  return json(url, { method: 'DELETE' })
}

// Auth
export const getAuthStatus = () => json<AuthStatus>('/api/auth/status')
export const login = (username: string, password: string) =>
  json<AuthSession>('/api/auth/login', { method: 'POST', body: JSON.stringify({ username, password }) })

// Status
export const getStatus = () => json('/api/status')

// Mode
export const getMode = () => json<{ mode: string }>('/api/mode')
export const setMode = (mode: string) => put('/api/mode', { mode })

// Maps
export const listMaps = () => json<MapInfo[]>('/api/maps')
export const saveMap = (name: string) => post('/api/maps/save', { name })
export const loadMap = (name: string) => post('/api/maps/load', { name })
export const mapImageUrl = (name: string) => apiUrl(`/api/maps/${name}/image`)

// Missions
export const listMissions = () => json<Mission[]>('/api/missions')
export const createMission = (m: Partial<Mission>) => post('/api/missions', m)
export const deleteMission = (id: string) => del(`/api/missions/${id}`)
export const executeMission = (id: string) => post(`/api/missions/${id}/execute`, {})
export const pauseMission = () => post('/api/missions/pause', {})
export const resumeMission = () => post('/api/missions/resume', {})

// Navigation
export const sendGoal = (x: number, y: number, theta = 0) =>
  post('/api/nav/goal', { x, y, theta })
export const cancelGoal = () => post('/api/nav/cancel', {})

// Accumulated map
export const clearAccMap = () => del('/api/acc_map')

// Events
export const clearEvents = () => del('/api/events')

// Analytics
export const getMissionRuns = (from?: number, to?: number) => {
  const params = new URLSearchParams()
  if (from) params.set('from', String(from))
  if (to) params.set('to', String(to))
  return json<MissionRun[]>(`/api/analytics/missions?${params}`)
}

// Traffic zones (fleet manager, port 8092 by default)
async function fleetJson<T>(url: string, init?: RequestInit): Promise<T> {
  const res = await fetch(fleetBase() + url, init)
  return res.json()
}

export const getTrafficZones = () => fleetJson<TrafficZone[]>('/api/traffic/zones')
export const createTrafficZone = (zone: TrafficZone) =>
  fleetJson('/api/traffic/zones', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(zone) })
export const deleteTrafficZone = (id: string) =>
  fleetJson(`/api/traffic/zones/${id}`, { method: 'DELETE' })
export const getTrafficOccupancy = () => fleetJson<ZoneOccupancy[]>('/api/traffic/occupancy')
