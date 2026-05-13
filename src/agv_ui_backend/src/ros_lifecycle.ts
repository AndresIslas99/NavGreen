/**
 * ROS lifecycle management — separates HTTP/WS server bootstrap from
 * rclnodejs initialization (Sub-fase 1.1.b).
 *
 * Why it exists
 * -------------
 * Before this module, `main()` did `await rclnodejs.init()` BEFORE
 * `server.listen()`. If rclnodejs failed (no DDS daemon, no
 * AGV_DOMAIN_ID, CAN bus down on a node the launch depends on...),
 * the HTTP server never came up. Operators reported on-site:
 *   "no se levantaba todo el sistema apenas se encendía el robot y
 *    ya no sabía ni cómo acceder a la UI porque tampoco se había
 *    levantado el servicio".
 *
 * After this module, the HTTP server ALWAYS starts. The dashboard
 * loads. The operator sees "ROS offline" with diagnostic detail.
 * Other systemd / network monitors keep working.
 *
 * The proxy + lifecycle pattern
 * ------------------------------
 * `RosBridgeProxy` implements `RosBridge` (from app_deps.ts).
 * Internally it holds a `RosBridge | null`. When null, every method
 * throws `RosOfflineError`. When a real impl is set, methods
 * delegate. Routes that call `deps.ros.X()` catch `RosOfflineError`
 * and return 503 with a clear reason.
 *
 * The lifecycle manager owns the connect→online→offline state
 * machine. On boot it tries `rclnodejs.init()` async; on success it
 * runs the caller-supplied `buildImpl(node)` to wire up publishers /
 * subscribers / action clients, then swaps the proxy's impl in.
 * On failure it backs off (1s, 2s, 4s, 8s, 16s, 30s cap) and retries.
 * Once online, a periodic health check pings the node graph; loss of
 * the daemon flips the status to offline and restarts the connect
 * loop.
 */

import * as rclnodejs from 'rclnodejs';
import type { RosBridge } from './app_deps';

export type RosStatus = 'connecting' | 'online' | 'offline' | 'degraded';

export class RosOfflineError extends Error {
  constructor(public method: string, public status: RosStatus = 'offline') {
    super(`ROS bridge is ${status}; cannot execute ${method}`);
    this.name = 'RosOfflineError';
  }
}

type StatusListener = (s: RosStatus, detail: string) => void;

/**
 * Proxy that implements RosBridge by delegating to an inner impl when
 * one exists. When the inner impl is null, methods throw
 * RosOfflineError. Stable reference — `deps.ros` points at this proxy
 * for the lifetime of the process; the inner impl is swapped on
 * connect/disconnect.
 */
export class RosBridgeProxy implements RosBridge {
  private impl: RosBridge | null = null;
  public status: RosStatus = 'offline';
  public detail: string = 'ROS not initialized yet';
  private listeners: StatusListener[] = [];

  /** Replace the inner impl + transition to online. */
  setImpl(impl: RosBridge): void {
    this.impl = impl;
    this.setStatus('online', 'ROS bridge active');
  }

  /** Drop the inner impl + transition to offline. */
  clearImpl(detail = 'ROS bridge lost'): void {
    this.impl = null;
    this.setStatus('offline', detail);
  }

  setStatus(s: RosStatus, detail: string): void {
    if (s === this.status && detail === this.detail) return;
    this.status = s;
    this.detail = detail;
    for (const fn of this.listeners) {
      try { fn(s, detail); } catch (e) { /* swallow listener errors */ }
    }
  }

  onChange(fn: StatusListener): () => void {
    this.listeners.push(fn);
    return () => { this.listeners = this.listeners.filter(x => x !== fn); };
  }

  private require(method: string): RosBridge {
    if (!this.impl) throw new RosOfflineError(method, this.status);
    return this.impl;
  }

  // ── RosBridge interface delegation ───────────────────────────────────────
  sendCmdVel(linear: number, angular: number): void {
    this.require('sendCmdVel').sendCmdVel(linear, angular);
  }
  sendNavGoal(x: number, y: number, theta: number) {
    return this.require('sendNavGoal').sendNavGoal(x, y, theta);
  }
  cancelNavGoal(): void {
    this.require('cancelNavGoal').cancelNavGoal();
  }
  sendEStop(active: boolean): void {
    this.require('sendEStop').sendEStop(active);
  }
  sendMotorEnable(active: boolean): void {
    this.require('sendMotorEnable').sendMotorEnable(active);
  }
  callTriggerService(client: any, name: string) {
    return this.require('callTriggerService').callTriggerService(client, name);
  }
  callRailApproach(req: any) {
    return this.require('callRailApproach').callRailApproach(req);
  }
  publishMapLoaded(name: string): void {
    this.require('publishMapLoaded').publishMapLoaded(name);
  }
  saveMap(name: string, mapDir: string, mapTopic: string) {
    return this.require('saveMap').saveMap(name, mapDir, mapTopic);
  }
  get startRecClient() { return this.impl?.startRecClient ?? null; }
  get stopRecClient() { return this.impl?.stopRecClient ?? null; }
  get loadMapClient() { return this.impl?.loadMapClient ?? null; }
}

/**
 * Build the inner impl by spinning up rclnodejs + a Node + all the
 * AGV publishers / subscribers / clients. Returns the implementation
 * AND the rclnodejs Node so the lifecycle manager can monitor health.
 *
 * The caller-supplied function is responsible for creating publishers,
 * subscribers, and action clients on the node, and returning a
 * RosBridge object whose methods reference those.
 */
export type BuildRosImpl = (node: rclnodejs.Node) => Promise<RosBridge>;

interface LifecycleOpts {
  /** First retry delay in ms. Defaults to 1000. */
  initialBackoffMs?: number;
  /** Max retry delay in ms. Defaults to 30000. */
  maxBackoffMs?: number;
  /** Periodic health-check interval in ms once online. Defaults to 3000. */
  healthCheckIntervalMs?: number;
  /** Health checks that fail consecutively count toward going offline.
   * After this many consecutive failures, status → offline. Defaults to 3. */
  healthFailuresBeforeOffline?: number;
}

/**
 * Owns the connect → online → (health degraded) → offline → reconnect
 * state machine. Call `start()` once after the HTTP server is
 * listening; the manager runs forever.
 */
export class RosLifecycleManager {
  private node: rclnodejs.Node | null = null;
  private healthTimer: NodeJS.Timeout | null = null;
  private healthFailures = 0;
  private stopping = false;

  constructor(
    private proxy: RosBridgeProxy,
    private buildImpl: BuildRosImpl,
    private opts: LifecycleOpts = {},
  ) {}

  async start(): Promise<void> {
    this.connect().catch(e => {
      // Should never escape — connect() has its own retry loop.
      console.error('[ros_lifecycle] connect() escaped:', e);
    });
  }

  async stop(): Promise<void> {
    this.stopping = true;
    if (this.healthTimer) clearInterval(this.healthTimer);
    if (this.node) {
      try { this.node.destroy(); } catch { /* ignore */ }
      this.node = null;
    }
    try { await rclnodejs.shutdown(); } catch { /* ignore */ }
  }

  /** Inner retry loop. Resolves only when the manager is told to stop. */
  private async connect(): Promise<void> {
    let backoff = this.opts.initialBackoffMs ?? 1000;
    const maxBackoff = this.opts.maxBackoffMs ?? 30000;

    while (!this.stopping) {
      this.proxy.setStatus('connecting', `Initializing rclnodejs (next retry in ${backoff}ms on failure)`);
      try {
        await rclnodejs.init();
        const node = new rclnodejs.Node('teleop_server');
        // Build the real impl. This sets up publishers, subscribers,
        // clients, and the spin. If it throws, the retry loop catches
        // it and tries again.
        const impl = await this.buildImpl(node);
        rclnodejs.spin(node);
        this.node = node;
        this.proxy.setImpl(impl);
        this.healthFailures = 0;
        backoff = this.opts.initialBackoffMs ?? 1000;  // reset for next disconnect
        this.startHealthCheck();
        // Stay here until health check kicks us out (return from this
        // method puts us back at the start of the loop).
        await this.waitForHealthFailure();
        // Coming back here means health check decided ROS is gone.
        // Tear down the current node and loop around to reconnect.
        if (this.healthTimer) clearInterval(this.healthTimer);
        this.healthTimer = null;
        if (this.node) {
          try { this.node.destroy(); } catch { /* ignore */ }
          this.node = null;
        }
        try { await rclnodejs.shutdown(); } catch { /* ignore */ }
        this.proxy.clearImpl(`Health check detected ROS down — reconnecting in ${backoff}ms`);
      } catch (e: any) {
        this.proxy.setStatus('offline', `Init failed: ${e?.message ?? String(e)}`);
        try { await rclnodejs.shutdown(); } catch { /* ignore */ }
      }
      if (this.stopping) return;
      await sleep(backoff);
      backoff = Math.min(backoff * 2, maxBackoff);
    }
  }

  private startHealthCheck(): void {
    const interval = this.opts.healthCheckIntervalMs ?? 3000;
    const limit = this.opts.healthFailuresBeforeOffline ?? 3;
    if (this.healthTimer) clearInterval(this.healthTimer);
    this.healthFailures = 0;
    this.healthResolver = null;
    this.healthTimer = setInterval(() => {
      if (!this.node) return;
      try {
        // Cheap: ask the node for the topic name list. If the
        // underlying middleware lost the daemon, this either throws or
        // returns nothing. rclnodejs's exact failure mode here varies
        // by Cyclone version — we just want a no-op call that touches
        // the participant.
        const names = (this.node as any).getTopicNamesAndTypes?.() ?? null;
        if (!names || names.length === 0) {
          this.healthFailures += 1;
        } else {
          this.healthFailures = 0;
        }
      } catch {
        this.healthFailures += 1;
      }
      if (this.healthFailures >= limit) {
        if (this.healthResolver) {
          const r = this.healthResolver;
          this.healthResolver = null;
          r();
        }
      }
    }, interval);
  }

  private healthResolver: (() => void) | null = null;
  private waitForHealthFailure(): Promise<void> {
    return new Promise(resolve => { this.healthResolver = resolve; });
  }
}

function sleep(ms: number): Promise<void> {
  return new Promise(r => setTimeout(r, ms));
}
