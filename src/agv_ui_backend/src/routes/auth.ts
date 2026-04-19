import type { Express } from 'express';
import type { AppDeps } from '../app_deps';

export function register(app: Express, deps: AppDeps): void {
  const { authManager } = deps;

  app.get('/api/auth/status', (_req, res) => {
    res.json({ enabled: authManager.enabled });
  });

  app.post('/api/auth/login', (req, res) => {
    const { username, password } = req.body || {};
    if (!username || !password) return res.status(400).json({ error: 'Missing credentials' });
    const result = authManager.login(username, password);
    if (!result) return res.status(401).json({ error: 'Invalid credentials' });
    res.json(result);
  });

  app.get('/api/auth/me', authManager.requireAuth(), (req, res) => {
    res.json((req as any).user);
  });

  app.get('/api/auth/users', authManager.requireAuth('engineer'), (_req, res) => {
    res.json(authManager.listUsers());
  });

  app.post('/api/auth/users', authManager.requireAuth('engineer'), (req, res) => {
    const { username, password, role } = req.body || {};
    if (!username || !password || !role) return res.status(400).json({ error: 'Missing fields' });
    if (authManager.addUser(username, password, role)) {
      res.json({ success: true });
    } else {
      res.status(409).json({ error: 'User already exists' });
    }
  });

  app.delete('/api/auth/users/:username', authManager.requireAuth('engineer'), (req, res) => {
    if (authManager.removeUser(String(req.params.username))) {
      res.json({ success: true });
    } else {
      res.status(404).json({ error: 'User not found' });
    }
  });
}
