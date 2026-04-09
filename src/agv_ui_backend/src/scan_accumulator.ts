// Scan accumulation grid for live mapping visualization
// Probabilistic evidence grid: positive = occupied, negative = free, 0 = unknown

export class ScanAccumulator {
  private grid: Float32Array;
  private resolution: number;
  private size: number;
  private origin: number;
  changed = false;
  version = 0;
  pngBuffer: Buffer | null = null;

  constructor(resolution = 0.1, size = 500, origin = -25.0) {
    this.resolution = resolution;
    this.size = size;
    this.origin = origin;
    this.grid = new Float32Array(size * size);
  }

  get meta() {
    return {
      resolution: this.resolution,
      origin_x: this.origin,
      origin_y: this.origin,
      width: this.size,
      height: this.size,
    };
  }

  addScan(
    robotX: number, robotY: number, robotTheta: number,
    ranges: number[], angleMin: number, angleIncrement: number,
    rangeMin: number, rangeMax: number,
  ): Array<{ x: number; y: number }> {
    const cosT = Math.cos(robotTheta);
    const sinT = Math.sin(robotTheta);
    const res = this.resolution;
    const orig = this.origin;
    const sz = this.size;
    const grid = this.grid;
    const rx = Math.floor((robotX - orig) / res);
    const ry = Math.floor((robotY - orig) / res);
    const points: Array<{ x: number; y: number }> = [];

    let angle = angleMin;
    for (let i = 0; i < ranges.length; i++) {
      const r = ranges[i];
      if (r > rangeMin && r < rangeMax) {
        const lx = r * Math.cos(angle);
        const ly = r * Math.sin(angle);
        const mx = robotX + cosT * lx - sinT * ly;
        const my = robotY + sinT * lx + cosT * ly;
        points.push({ x: Math.round(mx * 1000) / 1000, y: Math.round(my * 1000) / 1000 });

        // Every 3rd ray for grid
        if (i % 3 === 0) {
          const gx = Math.floor((mx - orig) / res);
          const gy = Math.floor((my - orig) / res);

          // Endpoint: occupied evidence
          if (gx >= 0 && gx < sz && gy >= 0 && gy < sz) {
            const idx = gy * sz + gx;
            grid[idx] = Math.min(10.0, grid[idx] + 2.0);
          }

          // Free ray
          this.raycastFree(rx, ry, gx, gy);
        }
      }
      angle += angleIncrement;
    }

    this.changed = true;
    this.version++;
    return points;
  }

  private raycastFree(x0: number, y0: number, x1: number, y1: number): void {
    const sz = this.size;
    const grid = this.grid;
    const dx = Math.abs(x1 - x0);
    const dy = Math.abs(y1 - y0);
    const steps = Math.max(dx, dy);
    if (steps === 0) return;
    const step = Math.max(3, Math.floor(steps / 20));
    const sx = (x1 - x0) / steps;
    const sy = (y1 - y0) / steps;
    let cx = x0, cy = y0;
    for (let i = 0; i < steps - 1; i += step) {
      const gx = Math.floor(cx);
      const gy = Math.floor(cy);
      if (gx >= 0 && gx < sz && gy >= 0 && gy < sz) {
        const idx = gy * sz + gx;
        grid[idx] = Math.max(-5.0, grid[idx] - 0.5);
      }
      cx += sx * step;
      cy += sy * step;
    }
  }

  /** Fraction of explored area that has evidence (0.0–1.0) */
  get coveragePercent(): number {
    const grid = this.grid;
    let explored = 0;
    let total = 0;
    for (let i = 0; i < grid.length; i++) {
      const v = grid[i];
      if (v !== 0) { explored++; total++; }
      // Count cells within bounding box of explored area
    }
    // Use a simpler metric: % of non-zero cells vs total grid
    if (explored === 0) return 0;
    // Estimate useful area as 2x the explored area (accounts for unknown gaps)
    return Math.min(100, Math.round(explored / grid.length * 100 * 10) / 10);
  }

  async updatePng(): Promise<void> {
    if (!this.changed) return;
    this.changed = false;

    const sz = this.size;
    // RGBA: 4 channels for transparency support
    const pixels = Buffer.alloc(sz * sz * 4);
    const grid = this.grid;

    for (let i = 0; i < sz * sz; i++) {
      const v = grid[i];
      const p = i * 4;
      if (v === 0) {
        // Unknown — transparent (shows Leaflet grid background)
        pixels[p] = 0; pixels[p + 1] = 0; pixels[p + 2] = 0; pixels[p + 3] = 0;
      } else if (v < -0.5) {
        // Free space — white, semi-transparent
        pixels[p] = 240; pixels[p + 1] = 245; pixels[p + 2] = 240; pixels[p + 3] = 180;
      } else if (v > 1.5) {
        // Occupied — dark, opaque
        pixels[p] = 20; pixels[p + 1] = 20; pixels[p + 2] = 25; pixels[p + 3] = 240;
      } else if (v > 0) {
        // Partial occupied evidence — warm yellow
        const a = Math.min(200, Math.round(80 + v * 60));
        pixels[p] = 200; pixels[p + 1] = 170; pixels[p + 2] = 50; pixels[p + 3] = a;
      } else {
        // Partial free evidence — light with low alpha
        const a = Math.min(150, Math.round(50 + Math.abs(v) * 50));
        pixels[p] = 220; pixels[p + 1] = 230; pixels[p + 2] = 220; pixels[p + 3] = a;
      }
    }

    // Flip vertically (row-by-row, 4 bytes per pixel)
    const rowBytes = sz * 4;
    const flipped = Buffer.alloc(sz * sz * 4);
    for (let row = 0; row < sz; row++) {
      pixels.copy(flipped, (sz - 1 - row) * rowBytes, row * rowBytes, (row + 1) * rowBytes);
    }

    try {
      const sharp = require('sharp');
      this.pngBuffer = await sharp(flipped, { raw: { width: sz, height: sz, channels: 4 } })
        .png()
        .toBuffer();
    } catch {
      // sharp not available — skip PNG generation
    }
  }

  clear(): void {
    this.grid.fill(0);
    this.changed = true;
    this.version++;
    this.pngBuffer = null;
  }
}
