/**
 * Unit tests for the API client's pure URL/host logic and token handling.
 *
 * client.ts reads VITE_API_BASE / VITE_FLEET_BASE and localStorage at module
 * load, so each scenario stubs the environment and globals first and then
 * imports a fresh copy of the module (vi.resetModules + dynamic import).
 * No DOM environment is needed: localStorage, location and fetch are stubbed.
 */

import { afterEach, describe, expect, it, vi } from 'vitest'

type ClientModule = typeof import('./client')

function memoryStorage(initial: Record<string, string> = {}) {
  const store = new Map(Object.entries(initial))
  return {
    getItem: (key: string) => store.get(key) ?? null,
    setItem: (key: string, value: string) => { store.set(key, value) },
    removeItem: (key: string) => { store.delete(key) },
    clear: () => { store.clear() },
  }
}

interface LoadOptions {
  apiBase?: string
  fleetBase?: string
  location?: { protocol?: string; host?: string; hostname?: string }
  storedToken?: string
}

async function loadClient(opts: LoadOptions = {}) {
  vi.resetModules()
  vi.stubEnv('VITE_API_BASE', opts.apiBase ?? '')
  vi.stubEnv('VITE_FLEET_BASE', opts.fleetBase ?? '')
  const storage = memoryStorage(
    opts.storedToken !== undefined ? { agv_token: opts.storedToken } : {},
  )
  vi.stubGlobal('localStorage', storage)
  vi.stubGlobal('location', {
    protocol: 'http:',
    host: 'jetson.local:8090',
    hostname: 'jetson.local',
    ...opts.location,
  })
  const client: ClientModule = await import('./client')
  return { client, storage }
}

/** Stub global fetch, recording every call's URL and init. */
function stubFetch(response: { status?: number; body?: unknown } = {}) {
  const calls: Array<{ url: string; init: RequestInit | undefined }> = []
  vi.stubGlobal('fetch', vi.fn(async (url: string, init?: RequestInit) => {
    calls.push({ url, init })
    return {
      status: response.status ?? 200,
      json: async () => response.body ?? {},
    }
  }))
  return calls
}

afterEach(() => {
  vi.unstubAllEnvs()
  vi.unstubAllGlobals()
})

describe('apiUrl', () => {
  it('passes the path through unchanged when VITE_API_BASE is empty (same-origin)', async () => {
    const { client } = await loadClient()
    expect(client.apiUrl('/api/status')).toBe('/api/status')
  })

  it('normalizes a missing leading slash with and without a base', async () => {
    const { client: sameOrigin } = await loadClient()
    expect(sameOrigin.apiUrl('api/maps')).toBe('/api/maps')

    const { client: offHost } = await loadClient({ apiBase: 'http://192.168.1.20:8090' })
    expect(offHost.apiUrl('api/maps')).toBe('http://192.168.1.20:8090/api/maps')
  })

  it('prepends VITE_API_BASE and strips its trailing slashes', async () => {
    const { client } = await loadClient({ apiBase: 'http://192.168.1.20:8090///' })
    expect(client.apiUrl('/api/status')).toBe('http://192.168.1.20:8090/api/status')
  })
})

describe('wsUrl', () => {
  it('converts an http(s) API base to the matching ws(s) scheme', async () => {
    const { client: plain } = await loadClient({ apiBase: 'http://192.168.1.20:8090' })
    expect(plain.wsUrl('/ws/control')).toBe('ws://192.168.1.20:8090/ws/control')

    const { client: tls } = await loadClient({ apiBase: 'https://192.168.1.20:8090' })
    expect(tls.wsUrl('/ws/control')).toBe('wss://192.168.1.20:8090/ws/control')
  })

  it('falls back to the page origin: wss on https pages, ws on http', async () => {
    const { client: secure } = await loadClient({
      location: { protocol: 'https:', host: 'ops.greenhouse.mx' },
    })
    expect(secure.wsUrl('/ws/control')).toBe('wss://ops.greenhouse.mx/ws/control')

    const { client: local } = await loadClient({
      location: { protocol: 'http:', host: 'jetson.local:8090' },
    })
    expect(local.wsUrl('ws/control')).toBe('ws://jetson.local:8090/ws/control')
  })
})

describe('fleetBase / fleetWsUrl', () => {
  it('prefers VITE_FLEET_BASE over everything, stripping trailing slashes', async () => {
    const { client } = await loadClient({
      fleetBase: 'http://10.0.0.9:8092/',
      apiBase: 'http://192.168.1.20:8090',
    })
    expect(client.fleetBase()).toBe('http://10.0.0.9:8092')
  })

  it('derives the fleet origin from VITE_API_BASE, pinning port 8092', async () => {
    const { client } = await loadClient({ apiBase: 'https://192.168.1.50:8090' })
    expect(client.fleetBase()).toBe('https://192.168.1.50:8092')
  })

  it('falls back to the page hostname:8092 when VITE_API_BASE is not a valid URL', async () => {
    const { client } = await loadClient({
      apiBase: 'not-a-url',
      location: { protocol: 'http:', hostname: 'jetson.local' },
    })
    expect(client.fleetBase()).toBe('http://jetson.local:8092')
  })

  it('fleetWsUrl builds a ws(s) URL on the fleet origin and adds the leading slash', async () => {
    const { client } = await loadClient({ fleetBase: 'https://10.0.0.9:8092' })
    expect(client.fleetWsUrl('ws/fleet')).toBe('wss://10.0.0.9:8092/ws/fleet')
  })
})

describe('token persistence', () => {
  it('round-trips the token through localStorage and clears it on null', async () => {
    const { client, storage } = await loadClient()
    expect(client.getToken()).toBeNull()

    client.setToken('tok-123')
    expect(client.getToken()).toBe('tok-123')
    expect(storage.getItem('agv_token')).toBe('tok-123')

    client.setToken(null)
    expect(client.getToken()).toBeNull()
    expect(storage.getItem('agv_token')).toBeNull()
  })

  it('restores a persisted token at module load', async () => {
    const { client } = await loadClient({ storedToken: 'persisted-tok' })
    expect(client.getToken()).toBe('persisted-tok')
  })
})

describe('request auth behavior', () => {
  it('a 401 from a non-auth endpoint clears the token, notifies the handler and rejects', async () => {
    const { client, storage } = await loadClient({ storedToken: 'expired-tok' })
    stubFetch({ status: 401 })
    const onUnauthorized = vi.fn()
    client.setUnauthorizedHandler(onUnauthorized)

    await expect(client.getStatus()).rejects.toThrow('Unauthorized: /api/status')
    expect(client.getToken()).toBeNull()
    expect(storage.getItem('agv_token')).toBeNull()
    expect(onUnauthorized).toHaveBeenCalledTimes(1)
  })

  it('a 401 from /api/auth/* does NOT tear down the session (login errors stay inline)', async () => {
    const { client } = await loadClient({ storedToken: 'valid-tok' })
    stubFetch({ status: 401, body: { error: 'bad credentials' } })
    const onUnauthorized = vi.fn()
    client.setUnauthorizedHandler(onUnauthorized)

    await expect(client.login('user', 'wrong')).resolves.toEqual({ error: 'bad credentials' })
    expect(client.getToken()).toBe('valid-tok')
    expect(onUnauthorized).not.toHaveBeenCalled()
  })

  it('attaches Authorization only when a token is set', async () => {
    const { client } = await loadClient({ storedToken: 'tok' })
    const authedCalls = stubFetch()
    await client.getStatus()
    const authedHeaders = authedCalls[0].init?.headers as Record<string, string>
    expect(authedHeaders['Authorization']).toBe('Bearer tok')
    expect(authedHeaders['Content-Type']).toBe('application/json')

    const { client: anon } = await loadClient()
    const anonCalls = stubFetch()
    await anon.getStatus()
    const anonHeaders = anonCalls[0].init?.headers as Record<string, string>
    expect('Authorization' in anonHeaders).toBe(false)
  })

  it('getMissionRuns serializes only the provided from/to bounds', async () => {
    const { client } = await loadClient()
    const calls = stubFetch({ body: [] })

    await client.getMissionRuns(100, 200)
    expect(calls[0].url).toBe('/api/analytics/missions?from=100&to=200')

    await client.getMissionRuns(100)
    expect(calls[1].url).toBe('/api/analytics/missions?from=100')
  })
})
