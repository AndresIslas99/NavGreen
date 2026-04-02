import type { MapInfo, Mission, AuthStatus, AuthSession, TrafficZone, ZoneOccupancy, MissionRun } from './types'

const BASE = ''  // same origin in production, proxied in dev
const FLEET_BASE = () => `${location.protocol}//${location.hostname}:8091`

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

async function json<T>(url: string, init?: RequestInit): Promise<T> {
  const res = await fetch(BASE + url, {
    ...init,
    headers: { ...authHeaders(), ...(init?.headers || {}) },
  })
  if (res.status === 401) {
    setToken(null)
    window.location.reload()
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
export const mapImageUrl = (name: string) => `${BASE}/api/maps/${name}/image`

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

// Traffic zones (fleet manager on port 8091)
async function fleetJson<T>(url: string, init?: RequestInit): Promise<T> {
  const res = await fetch(FLEET_BASE() + url, init)
  return res.json()
}

export const getTrafficZones = () => fleetJson<TrafficZone[]>('/api/traffic/zones')
export const createTrafficZone = (zone: TrafficZone) =>
  fleetJson('/api/traffic/zones', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(zone) })
export const deleteTrafficZone = (id: string) =>
  fleetJson(`/api/traffic/zones/${id}`, { method: 'DELETE' })
export const getTrafficOccupancy = () => fleetJson<ZoneOccupancy[]>('/api/traffic/occupancy')
