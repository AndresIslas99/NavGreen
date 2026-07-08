/**
 * Lightweight user management with JWT auth.
 *
 * Users stored in JSON config (not DB — 2-3 users on local network).
 * Roles: operator, engineer, viewer.
 * No OAuth/LDAP/SSO — local network only.
 *
 * No default accounts are shipped: create users with
 * `npm run adduser -- <username> <password> <role>` (writes to
 * $AGV_DATA_DIR/users.json), then set `"enabled": true` in that file.
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

// Unsalted SHA-256 digests of the default credentials seeded by earlier
// versions (engineer/agv2026, operator/agv). Kept only to detect and warn
// about users.json files that still carry these publicly-known passwords.
const LEGACY_DEFAULT_HASHES = new Set([
  '1e99803af2dbb6c3a1d4c23b21434e378f125cac3b174606554496517ba9eaac', // 'agv2026'
  'cce9d0b77372ab4671e4ddff3c40775e344039e39ee99fc37494d15e50576743', // 'agv'
]);

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

const SCRYPT_KEYLEN = 32;

export function hashPassword(password: string): string {
  const salt = crypto.randomBytes(16).toString('hex');
  const hash = crypto.scryptSync(password, salt, SCRYPT_KEYLEN).toString('hex');
  return `scrypt:${salt}:${hash}`;
}

function safeEqualHex(a: string, b: string): boolean {
  const ba = Buffer.from(a, 'hex');
  const bb = Buffer.from(b, 'hex');
  return ba.length === bb.length && crypto.timingSafeEqual(ba, bb);
}

function verifyPassword(password: string, stored: string): boolean {
  if (stored.startsWith('scrypt:')) {
    const [, salt, hash] = stored.split(':');
    if (!salt || !hash) return false;
    const candidate = crypto.scryptSync(password, salt, SCRYPT_KEYLEN).toString('hex');
    return safeEqualHex(candidate, hash);
  }
  // Legacy unsalted SHA-256 (users.json written by pre-0.1 versions).
  // Verified for backward compatibility; upgraded to scrypt on next login.
  const legacy = crypto.createHash('sha256').update(password).digest('hex');
  return /^[0-9a-f]{64}$/.test(stored) && safeEqualHex(legacy, stored);
}

export class AuthManager {
  private config: AuthConfig;
  private configPath: string;

  constructor(dataDir: string) {
    this.configPath = path.join(dataDir, 'users.json');
    this.config = this.loadConfig();
  }

  private loadConfig(): AuthConfig {
    const defaults: AuthConfig = {
      enabled: false,
      jwt_secret: crypto.randomBytes(32).toString('hex'),
      token_expiry: '24h',
      // No default users. Create them with `npm run adduser` — shipping
      // well-known credentials in a public repo defeats auth entirely.
      users: [],
    };

    try {
      if (fs.existsSync(this.configPath)) {
        const raw = JSON.parse(fs.readFileSync(this.configPath, 'utf-8'));
        return { ...defaults, ...raw };
      }
    } catch { /* ignore */ }

    // Write defaults on first run
    fs.writeFileSync(this.configPath, JSON.stringify(defaults, null, 2), { mode: 0o600 });
    return defaults;
  }

  get enabled(): boolean {
    return this.config.enabled;
  }

  /**
   * Startup security posture warnings — printed by index.ts so a weak
   * configuration is loud in the logs rather than silent.
   */
  securityWarnings(): string[] {
    const warnings: string[] = [];
    if (!this.config.enabled) {
      warnings.push(
        'Authentication is DISABLED — anyone who can reach this backend can command the robot. ' +
        `Enable it by setting "enabled": true in ${this.configPath} ` +
        '(create users first: npm run adduser -- <user> <pass> <role>).');
    } else if (this.config.users.length === 0) {
      warnings.push(
        `Authentication is enabled but ${this.configPath} has no users — all logins will fail. ` +
        'Create one with: npm run adduser -- <user> <pass> <role>.');
    }
    if (this.config.users.some(u => LEGACY_DEFAULT_HASHES.has(u.password_hash))) {
      warnings.push(
        `${this.configPath} still contains the publicly-known default credentials ` +
        'shipped by earlier versions. Change them: they are documented in the public repo.');
    }
    return warnings;
  }

  /** Authenticate user, return JWT token or null */
  login(username: string, password: string): { token: string; role: Role; username: string } | null {
    const user = this.config.users.find(u => u.username === username);
    if (!user || !verifyPassword(password, user.password_hash)) return null;

    // Transparent upgrade of legacy unsalted SHA-256 hashes to scrypt.
    if (!user.password_hash.startsWith('scrypt:')) {
      user.password_hash = hashPassword(password);
      this.saveConfig();
    }

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
    fs.writeFileSync(this.configPath, JSON.stringify(this.config, null, 2), { mode: 0o600 });
  }
}
