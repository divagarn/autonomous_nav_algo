#include "buggy_core/perception/ground_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace buggy_core
{
namespace
{
bool solve3(double A[3][3], double b[3], double x[3])
{
  for (int i = 0; i < 3; ++i) {
    int piv = i;
    for (int r = i + 1; r < 3; ++r) {if (std::abs(A[r][i]) > std::abs(A[piv][i])) {piv = r;}}
    if (std::abs(A[piv][i]) < 1e-12) {return false;}
    std::swap_ranges(A[i], A[i] + 3, A[piv]);
    std::swap(b[i], b[piv]);
    for (int r = i + 1; r < 3; ++r) {
      const double f = A[r][i] / A[i][i];
      for (int c = i; c < 3; ++c) {A[r][c] -= f * A[i][c];}
      b[r] -= f * b[i];
    }
  }
  for (int i = 2; i >= 0; --i) {
    double s = b[i];
    for (int c = i + 1; c < 3; ++c) {s -= A[i][c] * x[c];}
    x[i] = s / A[i][i];
  }
  return true;
}
}  // namespace

GroundModel::GroundModel(const std::vector<Point3> & ground_base, double length, const GroundModelParams & p)
: p_(p)
{
  nx_ = static_cast<int>(std::ceil((length + 10.0) / p_.cell));
  ny_ = static_cast<int>(std::ceil((p_.y1 - p_.y0) / p_.cell));
  std::vector<Point3> g;
  for (const auto & q : ground_base) {
    if (q.x >= p_.x0 && q.x < p_.x0 + nx_ * p_.cell && q.y >= p_.y0 && q.y < p_.y0 + ny_ * p_.cell) {g.push_back(q);}
  }
  n_points_ = g.size();
  // per-cell median
  std::vector<std::vector<double>> cells(static_cast<size_t>(nx_) * ny_);
  for (const auto & q : g) {
    const int ci = static_cast<int>(std::floor((q.x - p_.x0) / p_.cell));
    const int cj = static_cast<int>(std::floor((q.y - p_.y0) / p_.cell));
    cells[static_cast<size_t>(ci) * ny_ + cj].push_back(q.z);
  }
  std::vector<double> med(cells.size(), 0.0);
  std::vector<char> valid(cells.size(), 0);
  for (size_t i = 0; i < cells.size(); ++i) {
    auto & v = cells[i];
    if (v.empty()) {continue;}
    std::sort(v.begin(), v.end());
    const size_t m = v.size();
    med[i] = (m % 2) ? v[m / 2] : 0.5 * (v[m / 2 - 1] + v[m / 2]);
    valid[i] = 1;
  }
  // IDW over the cells within `radius`
  const int r = static_cast<int>(std::floor(p_.radius / p_.cell));
  std::vector<double> kernel((2 * r + 1) * (2 * r + 1), 0.0);
  for (int di = -r; di <= r; ++di) {
    for (int dj = -r; dj <= r; ++dj) {
      const double d = p_.cell * std::hypot(di, dj);
      if (d <= p_.radius) {kernel[(di + r) * (2 * r + 1) + (dj + r)] = 1.0 / std::pow(std::max(d, 0.25), 2);}
    }
  }
  est_.assign(cells.size(), std::numeric_limits<double>::quiet_NaN());
  for (int i = 0; i < nx_; ++i) {
    for (int j = 0; j < ny_; ++j) {
      double num = 0.0, den = 0.0;
      for (int di = -r; di <= r; ++di) {
        const int a = i + di;
        if (a < 0 || a >= nx_) {continue;}
        for (int dj = -r; dj <= r; ++dj) {
          const int b = j + dj;
          if (b < 0 || b >= ny_) {continue;}
          const size_t idx = static_cast<size_t>(a) * ny_ + b;
          if (!valid[idx]) {continue;}
          const double w = kernel[(di + r) * (2 * r + 1) + (dj + r)];
          num += w * med[idx];
          den += w;
        }
      }
      if (den > 0.0) {est_[static_cast<size_t>(i) * ny_ + j] = num / den;}
    }
  }
  // plane fit with outlier rejection
  if (g.size() >= p_.plane_min_points) {
    std::vector<char> use(g.size(), 1);
    std::optional<std::array<double, 3>> coef;
    for (int it = 0; it < 3; ++it) {
      double A[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, b[3] = {0, 0, 0};
      size_t cnt = 0;
      for (size_t k = 0; k < g.size(); ++k) {
        if (!use[k]) {continue;}
        const double v[3] = {g[k].x, g[k].y, 1.0};
        for (int a = 0; a < 3; ++a) {
          for (int c = 0; c < 3; ++c) {A[a][c] += v[a] * v[c];}
          b[a] += v[a] * g[k].z;
        }
        ++cnt;
      }
      double x[3];
      if (cnt < 3 || !solve3(A, b, x)) {coef.reset(); break;}
      coef = std::array<double, 3>{x[0], x[1], x[2]};
      for (size_t k = 0; k < g.size(); ++k) {
        use[k] = std::abs(g[k].x * x[0] + g[k].y * x[1] + x[2] - g[k].z) < p_.plane_gate;
      }
    }
    const size_t used = static_cast<size_t>(std::count(use.begin(), use.end(), 1));
    if (coef && used >= p_.plane_min_points &&
      std::atan(std::hypot((*coef)[0], (*coef)[1])) * 180.0 / M_PI <= p_.max_tilt_deg)
    {
      plane_ = coef;
    }
  }
}

double GroundModel::tilt_deg() const
{
  return plane_ ? std::atan(std::hypot((*plane_)[0], (*plane_)[1])) * 180.0 / M_PI : 0.0;
}

double GroundModel::height(double x, double y) const
{
  const double fallback = plane_ ? x * (*plane_)[0] + y * (*plane_)[1] + (*plane_)[2] : 0.0;
  const double ci = std::floor((x - p_.x0) / p_.cell), cj = std::floor((y - p_.y0) / p_.cell);
  if (!(ci >= 0 && cj >= 0 && ci < nx_ && cj < ny_)) {return fallback;}
  const double e = est_[static_cast<size_t>(ci) * ny_ + static_cast<size_t>(cj)];
  return std::isnan(e) ? fallback : e;
}
}  // namespace buggy_core
