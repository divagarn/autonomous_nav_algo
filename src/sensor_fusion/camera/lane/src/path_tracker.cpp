#include "sensor_fusion/path_tracker.hpp"

#include <algorithm>
#include <cmath>

#include "sensor_fusion/lane_geometry.hpp"

namespace sensor_fusion
{

namespace
{
/// Angle wrapped to [-pi, pi).
double wrap(double a) {return std::remainder(a, 2.0 * M_PI);}

/// Left unit normal of a polyline at point i.
Eigen::Vector2d leftNormal(const std::vector<Eigen::Vector2d> & line, std::size_t i)
{
  const Eigen::Vector2d tg = tangent(line, i);
  return {-tg.y(), tg.x()};
}
}  // namespace

// Predict (ego motion, resample from the vehicle), correct with the edges, shape; lane = path +- half width.
const TrackedPath & PathTracker::update(const LaneEstimate & lane, const Eigen::Vector2d & start, double start_yaw,
  const Eigen::Matrix2d & R, const Eigen::Vector2d & t, double ego_curvature)
{
  // p_now = R p_prev + t: the points stay on the ground
  for (auto & p : pts_) {p = R * p + t;}
  if (!pts_.empty()) {unmeasured_ += (R * last_start_ + t - start).norm();}
  last_start_ = start;
  if (unmeasured_ > cfg_.hold_distance) {   // no road edges for too long: restart from the ego curvature
    pts_.clear();
    offset_ = -1;
    reference_ = -1;
    unmeasured_ = 0;
  }
  // a new path (start, after hold_distance, or still far from the edges long after the gate opened)
  // is snapped onto the edges at once; a tracked one is corrected gently
  if (gated_ >= 2 * cfg_.gate_updates || lost_ >= 2 * cfg_.gate_updates) {pts_.clear(); gated_ = 0; lost_ = 0;}
  const bool fresh = pts_.empty();
  std::vector<Eigen::Vector2d> p = predict(start, start_yaw, ego_curvature);
  const int measured = correct(p, lane, start, start_yaw, fresh);
  // edges seen but no point of the path can use them: the path has lost them
  lost_ = measured == 0 && (lane.road_left.valid || lane.road_right.valid) ? lost_ + 1 : 0;
  if (measured > 0) {unmeasured_ = 0;}
  shape(p, start, start_yaw);
  pts_ = p;

  out_ = TrackedPath{};
  out_.valid = true;
  out_.centre = p;
  const double half = laneHalfWidth();
  for (std::size_t i = 0; i < p.size(); ++i) {
    const Eigen::Vector2d n = leftNormal(p, i);
    out_.left.push_back(p[i] + half * n);
    out_.right.push_back(p[i] - half * n);
  }
  out_.measured = measured;
  out_.unmeasured_m = unmeasured_;
  out_.reference = reference_;
  out_.offset = offset_;
  return out_;
}

// Old path from the projection of the vehicle point on, resampled by arc length; beyond its end the
// last heading turns with the curvature of the last steps (or ego_curvature), limited and fading.
std::vector<Eigen::Vector2d> PathTracker::predict(const Eigen::Vector2d & start, double start_yaw,
  double ego_curvature) const
{
  const double kmax = std::tan(cfg_.max_steer_deg * M_PI / 180.0) / cfg_.wheelbase;
  std::vector<Eigen::Vector2d> line{start};
  if (pts_.size() >= 2) {
    // first old segment ahead of the vehicle: projection parameter of start on segment j
    std::size_t j = pts_.size();
    for (std::size_t i = 1; i < pts_.size(); ++i) {
      const Eigen::Vector2d ab = pts_[i] - pts_[i - 1];
      if ((start - pts_[i - 1]).dot(ab) < ab.squaredNorm()) {j = i; break;}
    }
    for (std::size_t i = j; i < pts_.size(); ++i) {line.push_back(pts_[i]);}
  }
  // resample every spacing along the line
  const int n = std::max(2, cfg_.points);
  const double ds = cfg_.spacing;
  std::vector<Eigen::Vector2d> out{start};
  std::size_t seg = 1;   // segment line[seg - 1] -> line[seg] holding out.back() ...
  double on = 0;         // ... at this distance from its start
  while (static_cast<int>(out.size()) < n && seg < line.size()) {
    double remaining = ds;
    bool placed = false;
    for (; seg < line.size(); ++seg, on = 0) {
      const Eigen::Vector2d sa = line[seg - 1], sd = line[seg] - sa;
      const double sl = sd.norm();
      if (sl > 1e-9 && on + remaining <= sl) {
        on += remaining;
        out.push_back(sa + sd * (on / sl));
        placed = true;
        break;
      }
      remaining -= std::max(0.0, sl - on);
    }
    if (!placed) {break;}
  }
  // extension: heading of the last step, curvature of the last steps (else ego), both limited
  double heading = start_yaw, k = std::clamp(ego_curvature, -kmax, kmax);
  if (out.size() >= 2) {
    const Eigen::Vector2d d = out.back() - out[out.size() - 2];
    heading = std::atan2(d.y(), d.x());
  }
  if (out.size() >= 3) {
    const Eigen::Vector2d d0 = out[out.size() - 2] - out[out.size() - 3];
    k = std::clamp(wrap(heading - std::atan2(d0.y(), d0.x())) / ds, -kmax, kmax);
  }
  while (static_cast<int>(out.size()) < n) {
    // first step from the vehicle: chord heading = yaw + k ds / 2; the curvature fades towards straight
    heading += out.size() == 1 ? 0.5 * k * ds : k * ds;
    k *= std::pow(cfg_.extend_decay, ds);
    out.push_back(out.back() + ds * Eigen::Vector2d(std::cos(heading), std::sin(heading)));
  }
  return out;
}

// Target of point i: offset from the reference edge along its inward normal (lane_half_width from the
// other edge); error along the path normal, gain / max_step / gate.
int PathTracker::correct(std::vector<Eigen::Vector2d> & p, const LaneEstimate & lane, const Eigen::Vector2d & start,
  double start_yaw, bool snap)
{
  const LaneBoundary * edges[2] = {&lane.road_left, &lane.road_right};
  const double half = laneHalfWidth();
  // distance of the vehicle point inside the road from each edge (edge curve extended back to the vehicle)
  double dist[2] = {-1e9, -1e9};
  for (int k = 0; k < 2; ++k) {
    if (!edges[k]->valid) {continue;}
    Eigen::Vector2d q, tg;
    closestOnCurve(*edges[k], start, 25.0, q, tg);
    dist[k] = inwardNormal(tg, k == 0).dot(start - q);
  }
  int ref = reference_ >= 0 && edges[reference_]->valid ? reference_ : -1;
  if (ref < 0) {   // nearer valid edge
    for (int k = 0; k < 2; ++k) {
      if (edges[k]->valid && (ref < 0 || dist[k] < dist[ref])) {ref = k;}
    }
  }
  if (ref < 0) {return 0;}
  if (ref != reference_) {
    // new reference edge: keep the path where it is if the width is known, else start at the vehicle
    offset_ = offset_ > 0 && reference_ >= 0 && lane.road_width > 0 ? lane.road_width - offset_ : dist[ref];
    reference_ = ref;
  } else {
    offset_ += cfg_.offset_rate * (dist[ref] - offset_);   // follows the vehicle slowly
  }
  offset_ = std::max(offset_, half);
  const LaneBoundary & a = *edges[ref];
  const LaneBoundary & b = *edges[1 - ref];
  int measured = 0, gated = 0;
  double max_err = 0;
  const bool open = gated_ >= cfg_.gate_updates;   // persistent large errors: gate open until they are small
  const double max_cross = std::sin(cfg_.max_assoc_deg * M_PI / 180.0);
  const Eigen::Vector2d side(-std::sin(start_yaw), std::cos(start_yaw));   // vehicle left axis
  for (std::size_t i = 1; i < p.size(); ++i) {
    Eigen::Vector2d q, tg;
    if (!closestOnPolyline(a.points, p[i], q)) {continue;}   // edge not seen at this point
    closestOnCurve(a, p[i], 0.0, q, tg);
    // association: the edge must run along the vehicle's direction (|sin angle| = |tg . left axis|),
    // not across it (a curb across a junction ahead)
    if (std::abs(tg.dot(side)) > max_cross) {continue;}
    const Eigen::Vector2d n = leftNormal(p, i);
    const Eigen::Vector2d n_in = inwardNormal(tg, ref == 0);
    Eigen::Vector2d target = q + offset_ * n_in;
    Eigen::Vector2d q2;
    if (b.valid && closestOnPolyline(b.points, target, q2)) {
      // keep lane_half_width from the other edge too: w = road width along the normal
      const double w = n_in.dot(q2 - q);
      if (w > 0 && offset_ > w - half) {target = q + (w < 2 * half ? 0.5 * w : w - half) * n_in;}
    }
    const double e = n.dot(target - p[i]);
    max_err = std::max(max_err, std::abs(e));
    if (snap) {   // new path: straight onto the target
      p[i] += n * e;
      ++measured;
      continue;
    }
    if (std::abs(e) > cfg_.gate && !open) {++gated; continue;}
    p[i] += n * std::clamp(cfg_.gain * e, -cfg_.max_step, cfg_.max_step);
    ++measured;
  }
  if (snap) {
    gated_ = 0;
  } else if (open) {
    // closes once the path is back near the edges; still far off after as long again: restart
    gated_ = max_err < 0.5 * cfg_.gate ? 0 : gated_ + 1;
  } else {
    gated_ = gated > measured ? gated_ + 1 : 0;        // mostly gated: count towards opening
  }
  return measured;
}

// Neighbour-mean smoothing (2 passes), then the points rebuilt from the vehicle every spacing with the
// heading change limited: first step |dyaw| <= kmax ds / 2 (chord of an arc tangent to the vehicle
// heading), then |dyaw| <= kmax ds per step; heading within max_heading of the vehicle heading.
void PathTracker::shape(std::vector<Eigen::Vector2d> & p, const Eigen::Vector2d & start, double start_yaw) const
{
  for (int pass = 0; pass < 2; ++pass) {
    for (std::size_t i = 1; i + 1 < p.size(); ++i) {
      p[i] += cfg_.smoothing * (0.5 * (p[i - 1] + p[i + 1]) - p[i]);
    }
  }
  const double kmax = std::tan(cfg_.max_steer_deg * M_PI / 180.0) / cfg_.wheelbase, ds = cfg_.spacing;
  std::vector<Eigen::Vector2d> q{start};
  const double max_heading = cfg_.max_heading_deg * M_PI / 180.0;
  double prev = start_yaw;
  for (std::size_t i = 1; i < p.size(); ++i) {
    const Eigen::Vector2d d = p[i] - p[i - 1];
    const double limit = (i == 1 ? 0.5 : 1.0) * kmax * ds;
    prev += std::clamp(wrap(std::atan2(d.y(), d.x()) - prev), -limit, limit);
    prev = start_yaw + std::clamp(wrap(prev - start_yaw), -max_heading, max_heading);
    q.push_back(q.back() + ds * Eigen::Vector2d(std::cos(prev), std::sin(prev)));
  }
  p = q;
}

}  // namespace sensor_fusion
