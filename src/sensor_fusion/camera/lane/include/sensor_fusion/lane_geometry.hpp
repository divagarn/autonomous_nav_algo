/**
 * @file lane_geometry.hpp
 * @brief Small polyline / curve helpers shared by the lane estimator and the path tracker.
 */
#pragma once

#include <algorithm>
#include <vector>

#include <Eigen/Dense>

#include "sensor_fusion/lane_estimator.hpp"

namespace sensor_fusion
{

/**
 * @brief Closest point on a polyline.
 * @param[in]  line    polyline points
 * @param[in]  p       query point
 * @param[out] q       closest point
 * @return true if q lies inside the polyline (not clamped to its first or last point)
 */
inline bool closestOnPolyline(const std::vector<Eigen::Vector2d> & line, const Eigen::Vector2d & p, Eigen::Vector2d & q)
{
  double best = 1e18;
  bool inside = false;
  for (std::size_t i = 1; i < line.size(); ++i) {
    const Eigen::Vector2d ab = line[i] - line[i - 1];
    const double len2 = ab.squaredNorm();
    // projection parameter a = (p - a) . ab / |ab|^2, clamped to the segment
    const double a = len2 > 1e-12 ? std::clamp((p - line[i - 1]).dot(ab) / len2, 0.0, 1.0) : 0.0;
    const Eigen::Vector2d c = line[i - 1] + a * ab;
    const double d = (p - c).squaredNorm();
    if (d < best) {
      best = d;
      q = c;
      inside = !((i == 1 && a <= 0.0) || (i + 1 == line.size() && a >= 1.0));
    }
  }
  return inside;
}

/// Unit tangent of a polyline at point i (central difference).
inline Eigen::Vector2d tangent(const std::vector<Eigen::Vector2d> & line, std::size_t i)
{
  const std::size_t a = i > 0 ? i - 1 : i, b = i + 1 < line.size() ? i + 1 : i;
  const Eigen::Vector2d d = line[b] - line[a];
  return d.norm() > 1e-9 ? Eigen::Vector2d(d.normalized()) : Eigen::Vector2d::UnitX();
}

/**
 * @brief Closest point to p on a fitted boundary curve, extended up to `extend` metres before its first
 *        point (0.05 m steps), e.g. back to the vehicle.
 * @param[in]  b       fitted boundary
 * @param[in]  p       query point
 * @param[in]  extend  extension before s_min [m]
 * @param[out] q       closest point
 * @param[out] tg      unit tangent there
 */
inline void closestOnCurve(const LaneBoundary & b, const Eigen::Vector2d & p, double extend, Eigen::Vector2d & q,
  Eigen::Vector2d & tg)
{
  double best = 1e18, s_best = b.s_min;
  for (double s = b.s_min - extend; s <= b.s_max; s += 0.05) {
    const double d = (b.curveAt(s) - p).squaredNorm();
    if (d < best) {best = d; s_best = s;}
  }
  q = b.curveAt(s_best);
  tg = b.tangentAt(s_best);
}

/// Unit normal into the road of a road edge with unit tangent tg: left edge -> rotated -90 deg, right +90 deg.
inline Eigen::Vector2d inwardNormal(const Eigen::Vector2d & tg, bool left)
{
  return left ? Eigen::Vector2d(tg.y(), -tg.x()) : Eigen::Vector2d(-tg.y(), tg.x());
}

}  // namespace sensor_fusion
