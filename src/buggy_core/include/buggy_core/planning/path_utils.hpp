// Path helpers: resampling, curvature, steering demand and the steering-limited smoother.
// Port of buggy_pcd_lanelet_map/path_utils.py (no ROS types).
#pragma once

#include <array>
#include <vector>

namespace buggy_core
{
using Pt = std::array<double, 3>;
using Poly = std::vector<Pt>;

double dist3(const Pt & a, const Pt & b);
double length3(const Poly & p);

Poly resample(const Poly & pts, double step);
std::vector<double> curvatures(const Poly & pts);
double steer_deg_for_curvature(double kappa, double wheelbase);
double max_steer_deg(const Poly & pts, double wheelbase);

struct SmoothResult { Poly pts; double before, after; };
// Relax interior points toward their neighbours (endpoints pinned, within max_dev of the original)
// until the peak steering demand is within max_steer.
SmoothResult smooth(const Poly & pts, double wheelbase, double max_steer, double max_dev,
  double spacing = 0.5, int max_iter = 3000);
}  // namespace buggy_core
