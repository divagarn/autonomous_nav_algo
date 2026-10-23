#include "buggy_core/perception/ground_filter.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <numeric>

namespace buggy_core
{
namespace
{
constexpr float kTwoPi = static_cast<float>(2.0 * M_PI);
}

bool segment_ground_mask(const float * xyz, size_t n, const GroundParams & p, std::vector<uint8_t> & mask)
{
  mask.assign(n, 0);
  // pre-filter: range, height (float32 like numpy)
  std::vector<size_t> keep;
  keep.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    const float x = xyz[3 * i], y = xyz[3 * i + 1], z = xyz[3 * i + 2];
    const float rad = std::sqrt(x * x + y * y);
    if (rad < p.max_distance && z > p.min_height && z < p.max_height) {keep.push_back(i);}
  }
  const size_t m = keep.size();
  if (m < p.min_points) {return false;}

  std::vector<float> r(m);
  std::vector<int64_t> key(m);
  const float seg_w = static_cast<float>(2.0 * M_PI / p.num_segments);
  for (size_t k = 0; k < m; ++k) {
    const float x = xyz[3 * keep[k]], y = xyz[3 * keep[k] + 1];
    r[k] = std::sqrt(x * x + y * y);
    float ang = std::atan2(y, x);
    // python-style modulo 2*pi
    float md = std::fmod(ang, kTwoPi);
    if (md != 0.0f && md < 0.0f) {md += kTwoPi;}
    const int64_t seg = std::min<int64_t>(static_cast<int64_t>(md / seg_w), p.num_segments - 1);
    const int64_t bin = std::max<int64_t>(static_cast<int64_t>(r[k] / static_cast<float>(p.bin_size)), 0);
    key[k] = seg * 100000 + bin;
  }

  // cells = unique keys, ascending (np.unique)
  std::vector<int64_t> u(key);
  std::sort(u.begin(), u.end());
  u.erase(std::unique(u.begin(), u.end()), u.end());
  const size_t ng = u.size();
  std::vector<size_t> inv(m);
  for (size_t k = 0; k < m; ++k) {
    inv[k] = static_cast<size_t>(std::lower_bound(u.begin(), u.end(), key[k]) - u.begin());
  }
  std::vector<double> proto_z(ng, std::numeric_limits<double>::infinity()), proto_r(ng, 0.0);
  std::vector<int64_t> cnt(ng, 0);
  for (size_t k = 0; k < m; ++k) {
    const double z = xyz[3 * keep[k] + 2];
    proto_z[inv[k]] = std::min(proto_z[inv[k]], z);
    proto_r[inv[k]] += r[k];
    cnt[inv[k]] += 1;
  }
  for (size_t c = 0; c < ng; ++c) {proto_r[c] /= static_cast<double>(cnt[c]);}
  std::vector<int64_t> proto_seg(ng);
  for (size_t c = 0; c < ng; ++c) {proto_seg[c] = u[c] / 100000;}

  std::vector<size_t> order(ng);
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        if (proto_seg[a] != proto_seg[b]) {return proto_seg[a] < proto_seg[b];}
        return proto_r[a] < proto_r[b];
      });

  const double max_slope = std::tan(p.max_slope_deg * M_PI / 180.0);
  std::vector<uint8_t> cell_ground(ng, 0);
  std::vector<double> a_r(ng, 0.0), a_z(ng, 0.0), e_r(ng, 0.0), e_z(ng, 0.0);
  int64_t prev_seg = -1;
  double prev_r = 0.0, prev_z = 0.0;
  bool have_prev = false;
  for (size_t i = 0; i < ng; ++i) {
    const size_t idx = order[i];
    const int64_t seg = proto_seg[idx];
    if (seg != prev_seg) {have_prev = false;}
    const double rr = proto_r[idx], zz = proto_z[idx];
    bool accept;
    if (!have_prev) {
      accept = p.seed_z_min <= zz && zz <= p.seed_z_max;
    } else {
      const double dr = std::max(rr - prev_r, 1e-3);
      accept = std::abs((zz - prev_z) / dr) <= max_slope && std::abs(zz - prev_z) <= p.max_height_jump;
    }
    if (accept) {
      a_r[idx] = have_prev ? prev_r : rr;
      a_z[idx] = have_prev ? prev_z : zz;
      e_r[idx] = rr;
      e_z[idx] = zz;
      cell_ground[idx] = 1;
      prev_r = rr; prev_z = zz; have_prev = true;
    }
    prev_seg = seg;
  }

  for (size_t k = 0; k < m; ++k) {
    const size_t c = inv[k];
    const double slope = e_r[c] > a_r[c] ? (e_z[c] - a_z[c]) / std::max(e_r[c] - a_r[c], 1e-3) : 0.0;
    const double expected = a_z[c] + slope * (static_cast<double>(r[k]) - a_r[c]);
    const double signed_d = static_cast<double>(xyz[3 * keep[k] + 2]) - expected;
    const float ra = r[k];
    const float nb = 0.08f * std::max(1.0f - ra / 3.0f, 0.0f);
    const float adapt = std::min(p.base_thresh + p.distance_scale * ra + nb, p.max_adaptive);
    if (cell_ground[c] && std::abs(signed_d) < static_cast<double>(adapt) &&
      signed_d < p.ground_max_above && signed_d > -p.ground_max_below)
    {
      mask[keep[k]] = 1;
    }
  }
  return true;
}

void self_hit_keep(const uint8_t * data, size_t n, size_t stride, const SelfHitParams & p,
  std::vector<uint8_t> & keep)
{
  keep.assign(n, 1);
  for (size_t i = 0; i < n; ++i) {
    float v[3];
    std::memcpy(v, data + i * stride, sizeof(v));
    const float x = v[0], y = v[1], z = v[2];
    const bool too_close = std::sqrt(x * x + y * y + z * z) < p.min_range;
    const bool in_box = x > p.x_min && x < p.x_max && y > p.y_min && y < p.y_max && z > p.z_min && z < p.z_max;
    const float bx = x + p.sensor_offset_x, by = y + p.sensor_offset_y;
    const bool in_rect = std::abs(bx) < p.half_length && std::abs(by) < p.half_width;
    if (too_close || in_box || in_rect) {keep[i] = 0;}
  }
}

void colour_ground(const float * xyz, const std::vector<size_t> & idx, std::vector<uint32_t> & rgb)
{
  rgb.resize(idx.size());
  float mx = 1e-6f;
  std::vector<float> d(idx.size());
  for (size_t k = 0; k < idx.size(); ++k) {
    const float x = xyz[3 * idx[k]], y = xyz[3 * idx[k] + 1];
    d[k] = std::sqrt(x * x + y * y);
    mx = std::max(mx, d[k]);
  }
  for (size_t k = 0; k < idx.size(); ++k) {
    const uint32_t g = static_cast<uint32_t>(255.0f * (0.5f + 0.5f * (1.0f - d[k] / mx)));
    rgb[k] = g << 8;
  }
}

void colour_non_ground(const float * xyz, const std::vector<size_t> & idx, std::vector<uint32_t> & rgb)
{
  rgb.resize(idx.size());
  float hmn = std::numeric_limits<float>::infinity(), hmx = -std::numeric_limits<float>::infinity();
  for (size_t k : idx) {hmn = std::min(hmn, xyz[3 * k + 2]); hmx = std::max(hmx, xyz[3 * k + 2]);}
  const float rng = std::max(hmx - hmn, 0.01f);
  for (size_t k = 0; k < idx.size(); ++k) {
    const float nr = (xyz[3 * idx[k] + 2] - hmn) / rng;
    const uint32_t r = static_cast<uint32_t>(255.0f * (1.0f - nr));
    const uint32_t g = static_cast<uint32_t>(255.0f * std::abs(nr - 0.5f) * 2.0f);
    const uint32_t b = static_cast<uint32_t>(255.0f * nr);
    rgb[k] = (r << 16) | (g << 8) | b;
  }
}
}  // namespace buggy_core
