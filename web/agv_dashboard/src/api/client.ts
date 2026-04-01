import type { MapInfo, Mission } from './types'

const BASE = ''  // same origin in production, proxied in dev

async function json<T>(url: string, init?: RequestInit): Promise<T> {
  const res = await fetch(BASE + url, init)
  return res.json()
}

function post(url: string, body: unknown) {
  return json(url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  })
}

function put(url: string, body: unknown) {
  return json(url, {
    method: 'PUT',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  })
}

function del(url: string) {
  return json(url, { method: 'DELETE' })
}

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

// Navigation
export const sendGoal = (x: number, y: number, theta = 0) =>
  post('/api/nav/goal', { x, y, theta })
export const cancelGoal = () => post('/api/nav/cancel', {})
