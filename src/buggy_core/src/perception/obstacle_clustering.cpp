#include "buggy_core/perception/obstacle_clustering.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace buggy_core
{

Polyline corridor_ahead(
  const std::vector<std::array<double, 2>> & route_map, double rx, double ry, double yaw,
  double length)
{
  auto straight = [&]() {
      Polyline s;
      for (double x = 0.0; x < length; x += 0.5) {
        s.push_back({x, 0.0});
      }
      return s;
    };
  if (route_map.size() < 2) {
    return straight();
  }
  const double c = std::cos(-yaw), s = std::sin(-yaw);
  Polyline b;
  b.reserve(route_map.size());
  for (const auto & p : route_map) {
    const double dx = p[0] - rx, dy = p[1] - ry;
    b.push_back({dx * c - dy * s, dx * s + dy * c});
  }
  size_t i0 = 0;
  double best = std::numeric_limits<double>::max();
  for (size_t i = 0; i < b.size(); ++i) {
    const double d = std::hypot(b[i][0], b[i][1]);
    if (d < best) {
      best = d;
      i0 = i;
    }
  }
  Polyline out;
  double cum = 0.0;
  for (size_t i = i0; i < b.size(); ++i) {
    if (i > i0) {
      cum += std::hypot(b[i][0] - b[i - 1][0], b[i][1] - b[i - 1][1]);
    }
    if (cum > length) {
      break;
    }
    out.push_back(b[i]);
  }
  return out.size() < 2 ? straight() : out;
}

std::vector<Point3> to_base_frame(
  const std::vector<Point3> & sensor_points, const DetectorParams & prm)
{
  std::vector<Point3> out;
  out.reserve(sensor_points.size() / 4);
  for (const auto & p : sensor_points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    const double bx = p.x + prm.sensor_x, by = p.y, bz = p.z + prm.sensor_z;
    if (bz > prm.min_height && bz < prm.max_height && bx > 0.0 && bx < prm.length + 3.0) {
      out.push_back({bx, by, bz});
    }
  }
  return out;
}

std::vector<Cluster> cluster_grid(const std::vector<Point3> & pts, double cell, int min_points)
{
  std::vector<Cluster> out;
  if (pts.empty()) {
    return out;
  }
  double minx = pts[0].x, miny = pts[0].y;
  for (const auto & p : pts) {
    minx = std::min(minx, p.x);
    miny = std::min(miny, p.y);
  }
  std::vector<int> ix(pts.size()), iy(pts.size());
  int nx = 0, ny = 0;
  for (size_t i = 0; i < pts.size(); ++i) {
    ix[i] = static_cast<int>(std::floor((pts[i].x - minx) / cell));
    iy[i] = static_cast<int>(std::floor((pts[i].y - miny) / cell));
    nx = std::max(nx, ix[i] + 1);
    ny = std::max(ny, iy[i] + 1);
  }
  // dense occupancy grid, labelled in raster order (the order scipy.ndimage.label uses)
  std::vector<int> label(static_cast<size_t>(nx) * ny, 0);
  std::vector<char> occ(static_cast<size_t>(nx) * ny, 0);
  for (size_t i = 0; i < pts.size(); ++i) {
    occ[static_cast<size_t>(ix[i]) * ny + iy[i]] = 1;
  }
  int n_labels = 0;
  std::vector<std::pair<int, int>> stack;
  for (int a = 0; a < nx; ++a) {
    for (int b = 0; b < ny; ++b) {
      const size_t idx = static_cast<size_t>(a) * ny + b;
      if (!occ[idx] || label[idx]) {
        continue;
      }
      ++n_labels;
      label[idx] = n_labels;
      stack.assign(1, {a, b});
      while (!stack.empty()) {
        const auto [ca, cb] = stack.back();
        stack.pop_back();
        for (int da = -1; da <= 1; ++da) {
          for (int db = -1; db <= 1; ++db) {
            const int na = ca + da, nb = cb + db;
            if (na < 0 || nb < 0 || na >= nx || nb >= ny) {
              continue;
            }
            const size_t nidx = static_cast<size_t>(na) * ny + nb;
            if (occ[nidx] && !label[nidx]) {
              label[nidx] = n_labels;
              stack.push_back({na, nb});
            }
          }
        }
      }
    }
  }
  struct Acc
  {
    double x0, x1, y0, y1, hmax, hmin;
    int n;
  };
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<Acc> acc(n_labels, {inf, -inf, inf, -inf, -inf, inf, 0});
  for (size_t i = 0; i < pts.size(); ++i) {
    auto & a = acc[label[static_cast<size_t>(ix[i]) * ny + iy[i]] - 1];
    a.x0 = std::min(a.x0, pts[i].x);
    a.x1 = std::max(a.x1, pts[i].x);
    a.y0 = std::min(a.y0, pts[i].y);
    a.y1 = std::max(a.y1, pts[i].y);
    a.hmax = std::max(a.hmax, pts[i].z);
    a.hmin = std::min(a.hmin, pts[i].z);
    ++a.n;
  }
  for (const auto & a : acc) {
    if (a.n < min_points) {
      continue;
    }
    out.push_back(
      {(a.x0 + a.x1) / 2, (a.y0 + a.y1) / 2, std::max(a.x1 - a.x0, 0.4),
        std::max(a.y1 - a.y0, 0.4), a.hmax, a.n, a.x0, a.hmin});
  }
  return out;
}

std::vector<Cluster> detect_clusters(
  const std::vector<Point3> & base_points, const Polyline & corridor, const DetectorParams & prm)
{
  if (static_cast<int>(base_points.size()) < prm.min_points) {
    return {};
  }
  const double hw2 = prm.half_width * prm.half_width;
  std::vector<Point3> inside;
  inside.reserve(base_points.size() / 4);
  for (const auto & p : base_points) {
    for (const auto & c : corridor) {
      const double dx = p.x - c[0], dy = p.y - c[1];
      if (dx * dx + dy * dy < hw2) {
        inside.push_back(p);
        break;
      }
    }
  }
  if (static_cast<int>(inside.size()) < prm.min_points) {
    return {};
  }
  return cluster_grid(inside, prm.cell_size, prm.min_points);
}

}  // namespace buggy_core
