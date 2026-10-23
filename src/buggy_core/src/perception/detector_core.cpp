#include "buggy_core/perception/detector_core.hpp"

#include <cmath>
#include <memory>

namespace buggy_core
{
DetectorCore::Result DetectorCore::process(const std::vector<Point3> & nonground,
  const std::vector<Point3> * ground, const Polyline & corridor)
{
  Result r;
  // float32 like the numpy arrays of the Python reference
  const float sx = static_cast<float>(p_.sensor_x), sz = static_cast<float>(p_.sensor_z);
  std::unique_ptr<GroundModel> model;
  if (g_.aware && ground != nullptr) {
    std::vector<Point3> gb;
    gb.reserve(ground->size());
    for (const auto & q : *ground) {
      gb.push_back({static_cast<double>(static_cast<float>(q.x) + sx), q.y, static_cast<double>(static_cast<float>(q.z) + sz)});
    }
    model = std::make_unique<GroundModel>(gb, p_.length);
    r.used_ground_model = true;
    r.plane_tilt_deg = model->tilt_deg();
  }
  const double min_h = model ? g_.min_height : p_.min_height;
  std::vector<Point3> base;
  base.reserve(nonground.size() / 4);
  for (const auto & p : nonground) {
    const float x = static_cast<float>(p.x), y = static_cast<float>(p.y), z = static_cast<float>(p.z);
    const float bx = x + sx, by = y, bzf = z + sz;
    if (!std::isfinite(bx) || !std::isfinite(by) || !std::isfinite(bzf)) {continue;}
    double bz = bzf;
    if (model) {bz = static_cast<double>(bzf) - model->height(bx, by);}
    if (bz > min_h && bz < p_.max_height && bx > 0.0f && bx < static_cast<float>(p_.length + 3.0)) {
      base.push_back({bx, by, bz});
    }
  }
  r.clusters = detect_clusters(base, corridor, p_);
  if (g_.reject_thin_low && model) {
    std::vector<Cluster> kept;
    for (const auto & c : r.clusters) {
      if (!((c.h - c.hmin) < g_.thin_extent && c.h < g_.low_height)) {kept.push_back(c);}
    }
    rejected_ += static_cast<int>(r.clusters.size() - kept.size());
    r.clusters = std::move(kept);
  }
  for (const auto & c : r.clusters) {
    for (const auto & q : prev_) {
      if (std::hypot(c.cx - q[0], c.cy - q[1]) < 0.8) {r.confirmed.push_back(c); break;}
    }
  }
  prev_.clear();
  for (const auto & c : r.clusters) {prev_.push_back({c.cx, c.cy});}
  return r;
}
}  // namespace buggy_core
