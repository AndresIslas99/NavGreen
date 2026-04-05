#ifndef AGV_SCAN_MAPPER__GRID_MATH_HPP_
#define AGV_SCAN_MAPPER__GRID_MATH_HPP_

#include <cmath>
#include <vector>
#include <utility>

namespace agv_scan_mapper {

inline int world_to_grid(double w, double origin, double resolution)
{
  return static_cast<int>((w - origin) / resolution);
}

inline bool in_bounds(int x, int y, int width, int height)
{
  return x >= 0 && x < width && y >= 0 && y < height;
}

inline double log_odds_to_probability(double l)
{
  return 1.0 - 1.0 / (1.0 + std::exp(l));
}

/**
 * Bresenham line from (x0,y0) to (x1,y1), excluding endpoint.
 * Returns vector of (x,y) cells along the ray.
 */
inline std::vector<std::pair<int, int>> bresenham_line(
    int x0, int y0, int x1, int y1)
{
  std::vector<std::pair<int, int>> cells;
  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = x0 < x1 ? 1 : -1;
  int sy = y0 < y1 ? 1 : -1;
  int err = dx - dy;

  while (!(x0 == x1 && y0 == y1)) {
    cells.emplace_back(x0, y0);
    int e2 = 2 * err;
    if (e2 > -dy) { err -= dy; x0 += sx; }
    if (e2 < dx)  { err += dx; y0 += sy; }
  }
  return cells;
}

}  // namespace agv_scan_mapper

#endif  // AGV_SCAN_MAPPER__GRID_MATH_HPP_
