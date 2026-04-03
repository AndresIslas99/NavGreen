"""
Scan accumulator — probabilistic occupancy grid from laser scans.
Extracted from teleop_server.py for modularity.
"""

import io
import math
import numpy as np
from PIL import Image as PILImage


class ScanAccumulator:
    def __init__(self, resolution=0.1, size=500, origin=-25.0):
        self.resolution = resolution
        self.size = size
        self.origin = origin
        self.grid = np.zeros((size, size), dtype=np.float32)
        self.changed = False
        self.png_bytes = None
        self.meta = {
            'resolution': resolution,
            'origin_x': origin,
            'origin_y': origin,
            'width': size,
            'height': size,
        }

    def add_scan(self, px, py, pt, ranges, angle_min, angle_increment, range_min, range_max):
        """Process a laser scan and return world-frame scan points."""
        cos_t = math.cos(pt)
        sin_t = math.sin(pt)
        points = []
        res = self.resolution
        orig = self.origin
        sz = self.size
        grid = self.grid
        rx = int((px - orig) / res)
        ry = int((py - orig) / res)

        angle = angle_min
        ray_idx = 0
        for r in ranges:
            ray_idx += 1
            if range_min < r < range_max:
                lx = r * math.cos(angle)
                ly = r * math.sin(angle)
                mx = px + cos_t * lx - sin_t * ly
                my = py + sin_t * lx + cos_t * ly
                points.append({'x': round(mx, 3), 'y': round(my, 3)})

                if ray_idx % 3 == 0:
                    gx = int((mx - orig) / res)
                    gy = int((my - orig) / res)
                    if 0 <= gx < sz and 0 <= gy < sz:
                        grid[gy, gx] = min(10.0, grid[gy, gx] + 2.0)
                    self._raycast_free(rx, ry, gx, gy)

            angle += angle_increment

        self.changed = True
        return points

    def _raycast_free(self, x0, y0, x1, y1):
        """Mark cells along ray as free (decrease evidence)."""
        sz = self.size
        grid = self.grid
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        steps = max(dx, dy)
        if steps == 0:
            return
        step = max(3, steps // 20)
        sx = (x1 - x0) / steps
        sy = (y1 - y0) / steps
        cx, cy = float(x0), float(y0)
        for _ in range(0, steps - 1, step):
            gx, gy = int(cx), int(cy)
            if 0 <= gx < sz and 0 <= gy < sz:
                grid[gy, gx] = max(-5.0, grid[gy, gx] - 0.5)
            cx += sx * step
            cy += sy * step

    def update_png(self):
        """Convert grid to PNG (called periodically by timer)."""
        if not self.changed:
            return
        try:
            grid = self.grid
            img = np.full(grid.shape, 140, dtype=np.uint8)
            img[grid < -0.5] = 220
            img[grid > 1.5] = 25
            transition = (grid >= -0.5) & (grid <= 1.5) & (grid != 0.0)
            img[transition] = (170 - (grid[transition] * 60)).clip(40, 210).astype(np.uint8)
            img = np.flipud(img)
            pil_img = PILImage.fromarray(img, mode='L')
            buf = io.BytesIO()
            pil_img.save(buf, format='PNG', optimize=True)
            self.png_bytes = buf.getvalue()
        except Exception:
            pass

    def clear(self):
        """Reset the grid."""
        self.grid.fill(0.0)
        self.changed = True
        self.png_bytes = None
