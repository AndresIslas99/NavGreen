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

  // Sprint A.5 / CRITICAL-11-C-01 — first-login password change route.
  // The caller proves identity by supplying the old password; no JWT
  // middleware needed. Used to clear the must_change_password flag set
  // for the auto-generated admin user on first boot.
  app.post('/api/auth/change-password', (req, res) => {
    const { username, old_password, new_password } = req.body || {};
    if (!username || !old_password || !new_password) {
      return res.status(400).json({ error: 'Missing fields' });
    }
    if (typeof new_password !== 'string' || new_password.length < 8) {
      return res.status(400).json({ error: 'New password must be at least 8 characters' });
    }
    const ok = authManager.changePassword(username, old_password, new_password);
    if (!ok) return res.status(401).json({ error: 'Invalid credentials' });
    res.json({ success: true });
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
