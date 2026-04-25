import { defineConfig, loadEnv } from 'vite'
import react from '@vitejs/plugin-react'

// Sprint 1 Fase 1a: support hosting the dashboard from a different origin than
// the backend. VITE_BASE_PATH controls the Vite public path (default
// '/dashboard/' for backward compat with the Express static mount); set to
// '/' when serving via nginx/caddy at the document root. The dev-server proxy
// still points at localhost:8090 by default; override with VITE_DEV_PROXY_TARGET
// if developing against a backend on a different host.
export default defineConfig(({ mode }) => {
  const env = loadEnv(mode, process.cwd(), '')
  const proxyTarget = env.VITE_DEV_PROXY_TARGET || 'http://localhost:8090'
  const wsProxyTarget = proxyTarget.replace(/^http/, 'ws')
  return {
    base: env.VITE_BASE_PATH || '/dashboard/',
    plugins: [react()],
    server: {
      host: '0.0.0.0',
      port: 5173,
      proxy: {
        '/api': proxyTarget,
        '/ws': { target: wsProxyTarget, ws: true },
      },
    },
  }
})
