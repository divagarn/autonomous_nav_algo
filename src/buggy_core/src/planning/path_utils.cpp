#include "buggy_core/planning/path_utils.hpp"

#include <algorithm>
#include <cmath>

namespace buggy_core
{
double dist3(const Pt & a, const Pt & b)
{
  return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
}

double length3(const Poly & p)
{
  double s = 0.0;
  for (size_t i = 1; i < p.size(); ++i) {s += dist3(p[i - 1], p[i]);}
  return s;
}

Poly resample(const Poly & pts, double step)
{
  std::vector<double> d{0.0};
  for (size_t i = 1; i < pts.size(); ++i) {d.push_back(d.back() + dist3(pts[i - 1], pts[i]));}
  const double total = d.back();
  if (total <= 0.0) {return Poly{pts[0], pts[0]};}
  const int n = std::max(2, static_cast<int>(std::nearbyint(total / step)) + 1);   // Python round(): half to even
  Poly out;
  size_t j = 0;
  for (int i = 0; i < n; ++i) {
    const double s = total * i / (n - 1);
    while (j + 2 < d.size() && d[j + 1] < s) {++j;}
    const double seg = d[j + 1] - d[j];
    const double t = seg <= 0 ? 0.0 : (s - d[j]) / seg;
    const Pt & a = pts[j];
    const Pt & b = pts[j + 1];
    out.push_back({a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * (b[2] - a[2])});
  }
  return out;
}

std::vector<double> curvatures(const Poly & pts)
{
  std::vector<double> k(pts.size(), 0.0);
  for (size_t i = 1; i + 1 < pts.size(); ++i) {
    const double ax = pts[i - 1][0], ay = pts[i - 1][1];
    const double bx = pts[i][0], by = pts[i][1];
    const double cx = pts[i + 1][0], cy = pts[i + 1][1];
    const double cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    const double den = std::hypot(bx - ax, by - ay) * std::hypot(cx - bx, cy - by) * std::hypot(cx - ax, cy - ay);
    k[i] = den < 1e-9 ? 0.0 : 2.0 * cross / den;
  }
  return k;
}

double steer_deg_for_curvature(double kappa, double wheelbase)
{
  return std::atan(wheelbase * kappa) * 180.0 / M_PI;
}

double max_steer_deg(const Poly & pts, double wheelbase)
{
  double m = 0.0;
  for (double k : curvatures(pts)) {m = std::max(m, std::abs(steer_deg_for_curvature(k, wheelbase)));}
  return m;
}

SmoothResult smooth(const Poly & pts, double wheelbase, double max_steer, double max_dev, double spacing, int max_iter)
{
  const Poly base = resample(pts, spacing);
  const double before = max_steer_deg(base, wheelbase);
  if (base.size() < 3 || before <= max_steer) {return {base, before, before};}
  Poly cur = base;
  for (int it = 0; it < max_iter; ++it) {
    Poly nw = cur;
    for (size_t i = 1; i + 1 < cur.size(); ++i) {
      for (int k = 0; k < 2; ++k) {
        nw[i][k] = cur[i][k] + 0.5 * (0.5 * (cur[i - 1][k] + cur[i + 1][k]) - cur[i][k]);
      }
      const double dx = nw[i][0] - base[i][0], dy = nw[i][1] - base[i][1];
      const double dist = std::hypot(dx, dy);
      if (dist > max_dev) {
        nw[i][0] = base[i][0] + dx * max_dev / dist;
        nw[i][1] = base[i][1] + dy * max_dev / dist;
      }
    }
    cur = std::move(nw);
    if (max_steer_deg(cur, wheelbase) <= max_steer) {break;}
  }
  return {cur, before, max_steer_deg(cur, wheelbase)};
}
}  // namespace buggy_core
