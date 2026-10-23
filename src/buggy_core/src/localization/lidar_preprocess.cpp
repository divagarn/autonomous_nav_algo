#include "buggy_core/localization/lidar_preprocess.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>

namespace buggy_core
{
std::vector<float> voxel_centroids(const std::vector<float> & pts, float leaf)
{
  const size_t m = pts.size() / 3;
  std::vector<int64_t> ijk(3 * m);
  int64_t mn[3] = {INT64_MAX, INT64_MAX, INT64_MAX}, mx[3] = {INT64_MIN, INT64_MIN, INT64_MIN};
  for (size_t i = 0; i < m; ++i) {
    for (int c = 0; c < 3; ++c) {
      const float q = pts[3 * i + c] / leaf;                      // float32 division like numpy
      const int64_t v = static_cast<int64_t>(std::floor(q));
      ijk[3 * i + c] = v;
      mn[c] = std::min(mn[c], v);
      mx[c] = std::max(mx[c], v);
    }
  }
  int64_t dims[3];
  for (int c = 0; c < 3; ++c) {dims[c] = mx[c] - mn[c] + 1;}
  std::vector<int64_t> key(m);
  for (size_t i = 0; i < m; ++i) {
    key[i] = ((ijk[3 * i] - mn[0]) * dims[1] + (ijk[3 * i + 1] - mn[1])) * dims[2] + (ijk[3 * i + 2] - mn[2]);
  }
  std::vector<size_t> order(m);
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) { return key[a] < key[b]; });
  std::vector<float> out;
  size_t i = 0;
  while (i < m) {
    size_t j = i;
    double s[3] = {0, 0, 0};
    while (j < m && key[order[j]] == key[order[i]]) {
      for (int c = 0; c < 3; ++c) {s[c] += static_cast<double>(pts[3 * order[j] + c]);}
      ++j;
    }
    const double cnt = static_cast<double>(j - i);
    for (int c = 0; c < 3; ++c) {out.push_back(static_cast<float>(s[c] / cnt));}
    i = j;
  }
  return out;
}

bool preprocess_cloud(const float * xyz, size_t n, const PreprocessParams & p, std::vector<float> & out)
{
  std::vector<float> pts;
  pts.reserve(n * 3);
  for (size_t i = 0; i < n; ++i) {
    const float x = xyz[3 * i], y = xyz[3 * i + 1], z = xyz[3 * i + 2];
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {continue;}
    const float r = std::sqrt(x * x + y * y + z * z);
    if (!(r > p.min_range && r < p.max_range)) {continue;}
    const bool inside = x > p.x_min && x < p.x_max && y > p.y_min && y < p.y_max && z > p.z_min && z < p.z_max;
    if (inside) {continue;}
    pts.push_back(x); pts.push_back(y); pts.push_back(z);
  }
  if (pts.size() / 3 < p.min_points) {return false;}
  out = voxel_centroids(pts, p.voxel);
  return true;
}
}  // namespace buggy_core
