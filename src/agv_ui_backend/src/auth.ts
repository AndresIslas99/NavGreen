/**
 * Lightweight user management with JWT auth.
 *
 * Users stored in YAML config (not DB — 2-3 users on local network).
 * Roles: operator, engineer, viewer.
 * No OAuth/LDAP/SSO — local network only.
 */

import * as fs from 'fs';
import * as path from 'path';
import * as crypto from 'crypto';
import jwt from 'jsonwebtoken';
import { Request, Response, NextFunction } from 'express';

export type Role = 'viewer' | 'operator' | 'engineer';

export interface User {
  username: string;
  password_hash: string;
  role: Role;
}

export interface AuthConfig {
  enabled: boolean;
  jwt_secret: string;
  token_expiry: string;
  users: User[];
}

const ROLE_HIERARCHY: Record<Role, number> = {
  viewer: 0,
  operator: 1,
  engineer: 2,
};

/** Role-based permission for allowed actions */
export function filterActionsForRole(actions: Record<string, boolean>, role: Role): Record<string, boolean> {
  if (role === 'engineer') return actions;
  if (role === 'operator') return actions; // operators can do everything except config
  // Viewers can't do anything
  const filtered: Record<string, boolean> = {};
  for (const key of Object.keys(actions)) {
    filtered[key] = false;
  }
  return filtered;
}

function hashPassword(password: string): string {
  return crypto.createHash('sha256').update(password).digest('hex');
}

export class AuthManager {
  private config: AuthConfig;
  private configPath: string;

  constructor(dataDir: string) {
    this.configPath = path.join(dataDir, 'users.json');
    this.config = this.loadConfig();
  }

  private loadConfig(): AuthConfig {
    const defaultSecret = crypto.randomBytes(32).toString('hex');
    const defaults: AuthConfig = {
      enabled: false,
      jwt_secret: defaultSecret,
      token_expiry: '24h',
      users: [
        { username: 'engineer', password_hash: hashPassword('agv2026'), role: 'engineer' },
        { username: 'operator', password_hash: hashPassword('agv'), role: 'operator' },
      ],
    };

    try {
      if (fs.existsSync(this.configPath)) {
        const raw = JSON.parse(fs.readFileSync(this.configPath, 'utf-8'));
        return { ...defaults, ...raw };
      }
    } catch { /* ignore */ }

    // Write defaults on first run
    fs.writeFileSync(this.configPath, JSON.stringify(defaults, null, 2));
    return defaults;
  }

  get enabled(): boolean {
    return this.config.enabled;
  }

  /** Authenticate user, return JWT token or null */
  login(username: string, password: string): { token: string; role: Role; username: string } | null {
    const hash = hashPassword(password);
    const user = this.config.users.find(u => u.username === username && u.password_hash === hash);
    if (!user) return null;

    const token = jwt.sign(
      { username: user.username, role: user.role },
      this.config.jwt_secret,
      { expiresIn: this.config.token_expiry as jwt.SignOptions['expiresIn'] }
    );

    return { token, role: user.role, username: user.username };
  }

  /** Verify JWT token, return decoded payload or null */
  verify(token: string): { username: string; role: Role } | null {
    try {
      const decoded = jwt.verify(token, this.config.jwt_secret) as any;
      return { username: decoded.username, role: decoded.role };
    } catch {
      return null;
    }
  }

  /** Express middleware: require authentication */
  requireAuth(minRole: Role = 'viewer') {
    return (req: Request, res: Response, next: NextFunction) => {
      if (!this.config.enabled) return next();

      const authHeader = req.headers.authorization;
      if (!authHeader?.startsWith('Bearer ')) {
        return res.status(401).json({ error: 'Authentication required' });
      }

      const token = authHeader.slice(7);
      const user = this.verify(token);
      if (!user) {
        return res.status(401).json({ error: 'Invalid or expired token' });
      }

      if (ROLE_HIERARCHY[user.role] < ROLE_HIERARCHY[minRole]) {
        return res.status(403).json({ error: `Requires ${minRole} role` });
      }

      // Attach user to request
      (req as any).user = user;
      next();
    };
  }

  /** List users (without password hashes) */
  listUsers(): Array<{ username: string; role: Role }> {
    return this.config.users.map(u => ({ username: u.username, role: u.role }));
  }

  /** Add user */
  addUser(username: string, password: string, role: Role): boolean {
    if (this.config.users.find(u => u.username === username)) return false;
    this.config.users.push({ username, password_hash: hashPassword(password), role });
    this.saveConfig();
    return true;
  }

  /** Remove user */
  removeUser(username: string): boolean {
    const before = this.config.users.length;
    this.config.users = this.config.users.filter(u => u.username !== username);
    if (this.config.users.length !== before) {
      this.saveConfig();
      return true;
    }
    return false;
  }

  /** Enable/disable auth */
  setEnabled(enabled: boolean): void {
    this.config.enabled = enabled;
    this.saveConfig();
  }

  private saveConfig(): void {
    fs.writeFileSync(this.configPath, JSON.stringify(this.config, null, 2));
  }
}
