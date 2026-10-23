// Local ground height from the ground points of one scan (base_link x / y, z = height above the
// flat reference plane): IDW-smoothed per-cell median where ground points are near, else a plane
// fitted to all ground points, else 0 (the old flat assumption). Port of GroundModel in
// buggy_pcd_lanelet_map/lidar_obstacle_detector.py (the Python is the reference).
#pragma once

#include <array>
#include <optional>
#include <vector>

#include "buggy_core/perception/obstacle_clustering.hpp"

namespace buggy_core
{
struct GroundModelParams
{
  double cell = 0.5, radius = 3.0, x0 = -4.0, y0 = -10.0, y1 = 10.0;
  size_t plane_min_points = 50;
  double plane_gate = 0.15, max_tilt_deg = 15.0;
};

class GroundModel
{
public:
  GroundModel(const std::vector<Point3> & ground_base, double length, const GroundModelParams & p = {});
  double height(double x, double y) const;
  bool has_plane() const { return plane_.has_value(); }
  double tilt_deg() const;
  size_t points() const { return n_points_; }

private:
  GroundModelParams p_;
  int nx_ = 0, ny_ = 0;
  std::vector<double> est_;                    // nx * ny, NaN where no ground is near
  std::optional<std::array<double, 3>> plane_;  // z = a x + b y + c
  size_t n_points_ = 0;
};
}  // namespace buggy_core
