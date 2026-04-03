// Scan accumulation grid for live mapping visualization
// Probabilistic evidence grid: positive = occupied, negative = free, 0 = unknown

export class ScanAccumulator {
  private grid: Float32Array;
  private resolution: number;
  private size: number;
  private origin: number;
  changed = false;
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

  async updatePng(): Promise<void> {
    if (!this.changed) return;
    // Don't reset changed here — WS loop resets after broadcasting

    const sz = this.size;
    const pixels = Buffer.alloc(sz * sz);
    const grid = this.grid;

    for (let i = 0; i < sz * sz; i++) {
      const v = grid[i];
      if (v < -0.5) pixels[i] = 220;      // free
      else if (v > 1.5) pixels[i] = 25;    // occupied
      else if (v === 0) pixels[i] = 140;   // unknown
      else pixels[i] = Math.max(40, Math.min(210, Math.round(170 - v * 60))); // gradient
    }

    // Flip vertically
    const flipped = Buffer.alloc(sz * sz);
    for (let row = 0; row < sz; row++) {
      pixels.copy(flipped, (sz - 1 - row) * sz, row * sz, (row + 1) * sz);
    }

    try {
      const sharp = require('sharp');
      this.pngBuffer = await sharp(flipped, { raw: { width: sz, height: sz, channels: 1 } })
        .png()
        .toBuffer();
    } catch {
      // sharp not available — skip PNG generation
    }
  }

  clear(): void {
    this.grid.fill(0);
    this.changed = true;
    this.pngBuffer = null;
  }
}
