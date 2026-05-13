/**
 * Lightweight user management with JWT auth.
 *
 * Users stored in JSON config (not DB — 2-3 users on local network).
 * Roles: operator, engineer, viewer.
 * No OAuth/LDAP/SSO — local network only.
 *
 * Sprint A.5 (2026-05-13 audit, CRITICAL-11-C-01):
 *   - Defaults: enabled=true (was false). Production-safe default.
 *   - Hardcoded credentials removed. First boot generates a random
 *     16-character admin password and logs it once to stdout (which
 *     systemd captures into journalctl -u agv.service).
 *   - `must_change_password` flag carried in users.json; the frontend
 *     prompts on first login. The flag is informational — server does
 *     not (yet) refuse other operations until the password is changed.
 *   - Existing on-disk users.json is preserved unchanged: a dev Jetson
 *     that already has the legacy `engineer/agv2026` + `operator/agv`
 *     entries keeps them and the dev workflow stays intact. The fix
 *     prevents NEW installs from inheriting them.
 *
 * NOT in this commit (deferred to Sprint B / HIGH-11-C-02):
 *   - Salted KDF (bcrypt / argon2id / scrypt). Still SHA-256 unsalted.
 *   - TLS / HTTPS on the Express server.
 *   - Move JWT off the WebSocket URL query string.
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
  /** Sprint A.5: flag set on auto-generated admin so the frontend can
   *  prompt for a password change at first login. Server-side this is
   *  informational only — operations remain allowed until the change. */
  must_change_password?: boolean;
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

/** Role-based permission for allowed actions.
 *
 * Today the action set in state_machine.ts is purely operational
 * (teleop, mapping, mission execution, motor arm, nav cancel). Both
 * operator and engineer get the full set. Engineer-only privileges
 * (user management, calibration triggers when those exist) live at
 * the route level via `requireAuth('engineer')` — see routes/auth.ts
 * for the existing examples. When new config-mutating actions land,
 * they should be removed from operator's set HERE in addition to
 * being gated at the route. The previous comment claiming "operators
 * can do everything except config" was aspirational — there are no
 * config actions in the set today. (Sprint A.5 doc fix.)
 */
export function filterActionsForRole(actions: Record<string, boolean>, role: Role): Record<string, boolean> {
  if (role === 'engineer') return actions;
  if (role === 'operator') return actions;
  // Viewers can't command anything.
  const filtered: Record<string, boolean> = {};
  for (const key of Object.keys(actions)) {
    filtered[key] = false;
  }
  return filtered;
}

function hashPassword(password: string): string {
  // HIGH-11-C-02 (deferred to Sprint B): switch to salted KDF.
  return crypto.createHash('sha256').update(password).digest('hex');
}

/** Generate a memorable-but-unguessable random password.
 *  16 characters from a 62-char alphabet ≈ 95 bits of entropy. */
function generateAdminPassword(): string {
  const alphabet = 'ABCDEFGHJKLMNPQRSTUVWXYZabcdefghijkmnopqrstuvwxyz23456789';
  const bytes = crypto.randomBytes(16);
  let out = '';
  for (let i = 0; i < 16; i++) {
    out += alphabet[bytes[i] % alphabet.length];
  }
  return out;
}

export class AuthManager {
  private config: AuthConfig;
  private configPath: string;

  constructor(dataDir: string) {
    this.configPath = path.join(dataDir, 'users.json');
    this.config = this.loadConfig();
  }

  private loadConfig(): AuthConfig {
    // ── Existing file: preserve whatever is on disk ──
    if (fs.existsSync(this.configPath)) {
      try {
        const raw = JSON.parse(fs.readFileSync(this.configPath, 'utf-8'));
        // Minimal defaults for missing fields only — do NOT inject users.
        const cfg: AuthConfig = {
          enabled: raw.enabled ?? true,
          jwt_secret: raw.jwt_secret ?? crypto.randomBytes(32).toString('hex'),
          token_expiry: raw.token_expiry ?? '24h',
          users: Array.isArray(raw.users) ? raw.users : [],
        };
        // Persist if we filled in any missing field.
        if (cfg.jwt_secret !== raw.jwt_secret || cfg.enabled !== raw.enabled) {
          this.writeConfig(cfg);
        }
        return cfg;
      } catch (e) {
        // Malformed file: don't silently overwrite. Throw so the operator
        // notices instead of losing the existing user list.
        throw new Error(
          `Failed to parse ${this.configPath}: ${(e as Error).message}. ` +
          `Move it aside or restore from backup; the backend will not start ` +
          `with an unreadable users.json.`);
      }
    }

    // ── First boot: generate admin user with random password ──
    const adminPassword = generateAdminPassword();
    const cfg: AuthConfig = {
      enabled: true,
      jwt_secret: crypto.randomBytes(32).toString('hex'),
      token_expiry: '24h',
      users: [{
        username: 'admin',
        password_hash: hashPassword(adminPassword),
        role: 'engineer',
        must_change_password: true,
      }],
    };
    this.writeConfig(cfg);

    // Log the generated password ONCE. Goes to stdout → systemd journal.
    // Operator must record it before first login. The flag
    // must_change_password=true will prompt the dashboard to force a
    // change on first successful auth.
    console.log('');
    console.log('═══════════════════════════════════════════════════════════════');
    console.log('  agv_ui_backend: FIRST BOOT — admin credentials generated.');
    console.log('  username: admin');
    console.log(`  password: ${adminPassword}`);
    console.log('  Record this password NOW. It is logged only once.');
    console.log('  Change at first login via the dashboard prompt.');
    console.log('  File: ' + this.configPath);
    console.log('═══════════════════════════════════════════════════════════════');
    console.log('');

    return cfg;
  }

  get enabled(): boolean {
    return this.config.enabled;
  }

  /** Authenticate user, return JWT token + must_change_password flag, or null */
  login(username: string, password: string): { token: string; role: Role; username: string; must_change_password: boolean } | null {
    const hash = hashPassword(password);
    const user = this.config.users.find(u => u.username === username && u.password_hash === hash);
    if (!user) return null;

    const token = jwt.sign(
      { username: user.username, role: user.role },
      this.config.jwt_secret,
      { expiresIn: this.config.token_expiry as jwt.SignOptions['expiresIn'] }
    );

    return {
      token,
      role: user.role,
      username: user.username,
      must_change_password: user.must_change_password === true,
    };
  }

  /** Change a user's password. Verifies old password before updating.
   *  Clears the must_change_password flag on success. Returns true on
   *  success, false if username unknown or old password wrong. */
  changePassword(username: string, oldPassword: string, newPassword: string): boolean {
    if (!newPassword || newPassword.length < 8) return false;
    const oldHash = hashPassword(oldPassword);
    const user = this.config.users.find(u => u.username === username && u.password_hash === oldHash);
    if (!user) return false;
    user.password_hash = hashPassword(newPassword);
    user.must_change_password = false;
    this.saveConfig();
    return true;
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
    this.writeConfig(this.config);
  }

  /** Write a specific config to disk. Used by loadConfig() before
   *  `this.config` exists, and by saveConfig() for live updates. */
  private writeConfig(cfg: AuthConfig): void {
    fs.writeFileSync(this.configPath, JSON.stringify(cfg, null, 2));
  }
}
