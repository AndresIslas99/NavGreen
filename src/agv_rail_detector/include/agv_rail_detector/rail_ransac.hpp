// RANSAC-based rail tube pair detector. Header-only, ROS-free.
//
// Algorithm summary:
//   Input: 2-D points from a BEV (bird's-eye-view) projection of the ZED
//          depth image, filtered to the ground-plane slice (z ≈ 0).
//   1. Sample pairs of points. Each pair defines a candidate line.
//   2. Count inliers within `inlier_tolerance` of the line.
//   3. Among candidate lines with enough inliers, find a SECOND parallel
//      line at `pair_spacing` ± `spacing_tolerance` that also has enough
//      inliers.
//   4. Return the best (primary, parallel) pair by total inlier count.
//
// Line parameterization: (a, b, c) in ax + by + c = 0 with a²+b² = 1.
// Output carries both lines + a confidence in [0, 1] based on the combined
// inlier fraction. Zero confidence means "no valid pair — fallback".

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <utility>
#include <vector>

namespace agv_rail_detector {

struct Point2D {
  double x;
  double y;
};

struct RansacParams {
  double pair_spacing_m = 0.45;       // Rail pair center-to-center.
  double spacing_tolerance_m = 0.03;  // Accept pair spacing 0.42-0.48 m.
  double inlier_tolerance_m = 0.02;   // Point-to-line distance threshold.
  int    min_inliers = 8;             // Per line.
  int    max_iterations = 200;        // RANSAC sample budget.
  double min_confidence = 0.3;        // Reject if combined inlier ratio < this.
  std::uint32_t seed = 0x5AFE5A11U;   // Deterministic RNG seed for tests.
};

// Line in 2D, normalised. a*x + b*y + c = 0 with a²+b² = 1.
struct Line2D {
  double a = 0.0;
  double b = 0.0;
  double c = 0.0;
};

struct RailDetection {
  Line2D line_left;         // Rail at y = y_aisle + 0.225 (+Y side).
  Line2D line_right;        // Rail at y = y_aisle − 0.225 (−Y side).
  double confidence = 0.0;  // 0..1
  int    inliers_left = 0;
  int    inliers_right = 0;
};

// Distance from a 2-D point to a normalised line.
inline double point_line_distance(const Line2D &l, const Point2D &p) {
  return std::abs(l.a * p.x + l.b * p.y + l.c);
}

// Construct a normalised line through two points. Returns std::nullopt if
// the points are too close to define a stable line.
inline std::optional<Line2D> line_from_two_points(const Point2D &p1,
                                                   const Point2D &p2) {
  const double dx = p2.x - p1.x;
  const double dy = p2.y - p1.y;
  const double len = std::hypot(dx, dy);
  if (len < 1e-6) return std::nullopt;
  // Normal is perpendicular to the direction vector.
  Line2D l;
  l.a = -dy / len;
  l.b =  dx / len;
  l.c = -(l.a * p1.x + l.b * p1.y);
  return l;
}

// Build the translated parallel line at `signed_distance` m offset from
// `l` in the +normal direction.
inline Line2D parallel_line(const Line2D &l, double signed_distance) {
  Line2D p = l;
  p.c = l.c - signed_distance;  // shift by distance along normal
  return p;
}

// Are two lines parallel within an angular tolerance (radians)?
inline bool lines_parallel(const Line2D &l1, const Line2D &l2,
                            double angle_tol_rad = 0.05) {
  // Since lines are normalised, the dot product of the normals gives
  // cos(angle). Parallel (or anti-parallel) lines produce |dot|≈1.
  const double dot = l1.a * l2.a + l1.b * l2.b;
  return std::abs(std::abs(dot) - 1.0) < (angle_tol_rad * angle_tol_rad);
}

// Signed perpendicular distance between two parallel normalised lines.
// Uses l2's normal as reference so a positive value means l2's origin is
// on the +normal side of l1.
inline double parallel_line_distance(const Line2D &l1, const Line2D &l2) {
  // If the two lines' normals are anti-parallel, flip l2's sign so the
  // distance makes sense.
  const double dot = l1.a * l2.a + l1.b * l2.b;
  const double sign = (dot >= 0.0) ? 1.0 : -1.0;
  return sign * l2.c - l1.c;
}

namespace detail {

// Tiny deterministic xorshift32 so tests don't depend on libc rng.
struct XorShift32 {
  std::uint32_t state;
  explicit XorShift32(std::uint32_t s) : state(s ? s : 0xDEADBEEFU) {}
  std::uint32_t next() {
    std::uint32_t x = state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    state = x;
    return x;
  }
  int next_index(int n) {
    if (n <= 0) return 0;
    return static_cast<int>(next() % static_cast<std::uint32_t>(n));
  }
};

inline int count_inliers(const Line2D &l, const std::vector<Point2D> &pts,
                          double tol) {
  int n = 0;
  for (const auto &p : pts) {
    if (point_line_distance(l, p) < tol) ++n;
  }
  return n;
}

}  // namespace detail

inline RailDetection detect_rails(const std::vector<Point2D> &pts,
                                   const RansacParams &p) {
  RailDetection best;
  if (pts.size() < static_cast<size_t>(p.min_inliers * 2)) {
    return best;  // confidence=0
  }

  detail::XorShift32 rng(p.seed);

  for (int iter = 0; iter < p.max_iterations; ++iter) {
    const int i = rng.next_index(static_cast<int>(pts.size()));
    const int j = rng.next_index(static_cast<int>(pts.size()));
    if (i == j) continue;
    auto line_opt = line_from_two_points(pts[i], pts[j]);
    if (!line_opt) continue;
    const Line2D &l1 = *line_opt;

    const int inl_1 = detail::count_inliers(l1, pts, p.inlier_tolerance_m);
    if (inl_1 < p.min_inliers) continue;

    // Try both parallel offsets (+ and − pair_spacing).
    for (double sign : {+1.0, -1.0}) {
      const Line2D l2 = parallel_line(l1, sign * p.pair_spacing_m);
      const int inl_2 = detail::count_inliers(l2, pts, p.inlier_tolerance_m);
      if (inl_2 < p.min_inliers) continue;
      // Score = total inliers. Prefer more inliers. Reject if either line
      // massively dominates (we want two real rails, not one fat cluster).
      if (std::min(inl_1, inl_2) < p.min_inliers) continue;
      const int total = inl_1 + inl_2;
      const int best_total = best.inliers_left + best.inliers_right;
      if (total > best_total) {
        // Order left/right by the +normal offset direction.
        if (sign > 0) {
          best.line_left  = l2;  // +spacing → +normal side → "left" (+Y)
          best.line_right = l1;
          best.inliers_left  = inl_2;
          best.inliers_right = inl_1;
        } else {
          best.line_left  = l1;
          best.line_right = l2;
          best.inliers_left  = inl_1;
          best.inliers_right = inl_2;
        }
      }
    }
  }

  // Confidence = fraction of points that are inliers of either rail.
  const int total = best.inliers_left + best.inliers_right;
  if (total == 0 || pts.empty()) {
    best.confidence = 0.0;
  } else {
    const double frac = static_cast<double>(total) /
                         static_cast<double>(pts.size());
    best.confidence = std::clamp(frac, 0.0, 1.0);
  }
  if (best.confidence < p.min_confidence) {
    best = RailDetection{};  // reset — below threshold is "no detection"
  }
  return best;
}

// Small helper used by tests (and by the node's BEV reprojector): build a
// synthetic rail pair centred at a given (x, y) and yaw angle, then
// sprinkle points along each rail.
inline std::vector<Point2D> synth_rail_points(double cx, double cy,
                                               double yaw, double length,
                                               double spacing,
                                               int samples_per_rail) {
  std::vector<Point2D> pts;
  const double cos_y = std::cos(yaw);
  const double sin_y = std::sin(yaw);
  // Along-rail step.
  for (int i = 0; i < samples_per_rail; ++i) {
    const double t = -length * 0.5 + length * (static_cast<double>(i) /
                                                std::max(1, samples_per_rail - 1));
    const double x_along = t;
    // +Y rail and −Y rail relative offsets.
    for (double s : {+spacing * 0.5, -spacing * 0.5}) {
      const double x_local = x_along;
      const double y_local = s;
      pts.push_back({cx + cos_y * x_local - sin_y * y_local,
                      cy + sin_y * x_local + cos_y * y_local});
    }
  }
  return pts;
}

}  // namespace agv_rail_detector
