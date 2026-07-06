#!/usr/bin/env node
/**
 * Bootstrap CLI: create a dashboard user in $AGV_DATA_DIR/users.json.
 *
 * Usage:
 *   npm run adduser -- <username> <password> [viewer|operator|engineer]
 *
 * The backend ships with no default accounts, so this is how the first
 * credential is created before setting "enabled": true in users.json.
 */

import { AuthManager, Role } from '../auth';

const DATA_DIR = process.env.AGV_DATA_DIR || '/tmp/agv_data';
const ROLES: Role[] = ['viewer', 'operator', 'engineer'];

function main(): number {
  const [, , username, password, roleArg] = process.argv;
  const role = (roleArg || 'operator') as Role;

  if (!username || !password) {
    console.error('Usage: npm run adduser -- <username> <password> [viewer|operator|engineer]');
    return 1;
  }
  if (!ROLES.includes(role)) {
    console.error(`Invalid role '${role}'. Valid roles: ${ROLES.join(', ')}`);
    return 1;
  }
  if (password.length < 8) {
    console.error('Password must be at least 8 characters.');
    return 1;
  }

  const auth = new AuthManager(DATA_DIR);
  if (!auth.addUser(username, password, role)) {
    console.error(`User '${username}' already exists in ${DATA_DIR}/users.json`);
    return 1;
  }
  console.log(`User '${username}' (${role}) added to ${DATA_DIR}/users.json`);
  if (!auth.enabled) {
    console.log('Note: auth is currently disabled. Set "enabled": true in users.json to enforce it.');
  }
  return 0;
}

process.exit(main());
