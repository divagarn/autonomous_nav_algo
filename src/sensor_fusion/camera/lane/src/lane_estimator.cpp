#include "sensor_fusion/lane_estimator.hpp"
#include "sensor_fusion/lane_geometry.hpp"

#include <algorithm>
#include <cmath>

namespace sensor_fusion
{

namespace
{
/// Road surface for the edge search: road or a lane marking on it.
bool drivable(uint8_t c) {return c == LaneEstimator::kRoad || c == LaneEstimator::kLane;}

/**
 * @brief Last drivable pixel of the run around the seed in one direction: non-road gaps up to max_gap
 *        are crossed, longer ones too if road (> max_gap wide) follows within max_occluder of the run;
 *        pixels of an occluder box (occ != 0) neither extend nor end the run.
 * @param[in]  row   mask row
 * @param[in]  occ   occluder flags of the row (nullptr: none)
 * @param[in]  w     row width [px]
 * @param[in]  seed  drivable start column
 * @param[in]  dir   -1 left, +1 right in the image
 * @param[out] last  last drivable column
 * @return true if the run ends inside the image at a visible edge; false at the image border or when
 *         the end lies behind an occluder box (the edge is hidden)
 */
bool runEnd(const uint8_t * row, const uint8_t * occ, int w, int seed, int dir, int max_gap, int max_occluder,
  int & last)
{
  last = seed;
  int gap = 0, u = seed + dir;
  bool hidden = false;   // occluder pixels since the last road pixel
  for (; u >= 0 && u < w; u += dir) {
    if (drivable(row[u])) {
      last = u;
      gap = 0;
      hidden = false;
      continue;
    }
    if (occ && occ[u]) {
      hidden = true;
      continue;
    }
    if (++gap <= max_gap) {continue;}
    // longer gap: an object on the road if road (> max_gap wide) follows within max_occluder
    int k = u + dir, run = 0;
    for (; k >= 0 && k < w && std::abs(k - last) <= max_occluder; k += dir) {
      run = drivable(row[k]) ? run + 1 : 0;
      if (run > max_gap) {break;}
    }
    if (run <= max_gap) {return !hidden;}   // no road behind: the edge (unless behind an object)
    u = k;
    last = k;
    gap = 0;
    hidden = false;
  }
  return false;   // the road leaves the image
}

/// Edge point of one row with the range of the row (for the continuity check).
struct RowPoint
{
  Eigen::Vector2d p;
  double range;
};

/**
 * @brief Splits the edge points of one mask (nearest row first) where successive points are further
 *        apart than their range difference + max_jump, or where the edge runs more than max_angle
 *        sideways from the forward direction (the road ends ahead / turns away), and drops pieces of
 *        fewer than 3 points (an object at the edge for a row or two); the robust fit sorts out the rest.
 */
std::vector<Eigen::Vector2d> edgePieces(const std::vector<RowPoint> & pts, double max_jump, double max_angle)
{
  const double tan_max = std::tan(max_angle);
  std::vector<std::vector<Eigen::Vector2d>> pieces;
  for (std::size_t i = 0; i < pts.size(); ++i) {
    bool split = i == 0;
    if (!split) {
      const Eigen::Vector2d d = pts[i].p - pts[i - 1].p;
      // too far apart, or |dy| > tan(max_angle) dx (sideways or backwards)
      split = d.norm() > std::abs(pts[i].range - pts[i - 1].range) + max_jump || std::abs(d.y()) > tan_max * d.x();
    }
    if (split) {pieces.emplace_back();}
    pieces.back().push_back(pts[i].p);
  }
  std::vector<Eigen::Vector2d> out;
  for (const auto & piece : pieces) {
    if (piece.size() >= 3) {out.insert(out.end(), piece.begin(), piece.end());}
  }
  return out;
}

}  // namespace

void LaneEstimator::reset()
{
  est_ = LaneEstimate{};
  for (auto & p : pts_) {p.clear();}
  rows_.clear();
}

// Old points into the new frame, new boundary points of the mask, refit, road width.
const LaneEstimate & LaneEstimator::update(
  Time stamp, const uint8_t * mask, int mask_w, int mask_h, int image_w, int image_h,
  const fusion_common::CameraModel & cam, double ground_z, const Eigen::Matrix2d & R, const Eigen::Vector2d & t,
  const std::vector<Eigen::Vector4d> & occluders)
{
  if (est_.stamp >= 0 && stamp < est_.stamp - 1.0) {reset();}   // bag restarted
  for (auto & list : pts_) {
    // p_now = R p_prev + t; drop points older than keep_s
    for (auto & q : list) {q.p = R * q.p + t;}
    list.erase(std::remove_if(list.begin(), list.end(),
      [&](const Point & q) {return q.t < stamp - cfg_.keep_s;}), list.end());
  }
  const double sx = static_cast<double>(image_w) / mask_w, sy = static_cast<double>(image_h) / mask_h;
  std::vector<uint8_t> occ;   // occluder box flags per mask pixel
  if (!occluders.empty()) {
    occ.assign(static_cast<std::size_t>(mask_w) * mask_h, 0);
    for (const auto & b : occluders) {
      // image px -> mask px, clipped
      const int u0 = std::max(0, static_cast<int>(b(0) / sx)), u1 = std::min(mask_w - 1, static_cast<int>(b(2) / sx));
      const int v0 = std::max(0, static_cast<int>(b(1) / sy)), v1 = std::min(mask_h - 1, static_cast<int>(b(3) / sy));
      for (int v = v0; v <= v1; ++v) {
        std::fill_n(occ.begin() + static_cast<std::ptrdiff_t>(v) * mask_w + u0, std::max(0, u1 - u0 + 1), 1);
      }
    }
  }
  const Eigen::Vector3d o = cam.cameraPositionInRadar();
  // ground point of mask pixel (um, vm): ray o + s d meets z = ground_z at s = (ground_z - o_z) / d_z
  auto ground = [&](double um, double vm, Eigen::Vector2d & g) {
      const Eigen::Vector2d uv((um + 0.5) * sx - 0.5, (vm + 0.5) * sy - 0.5);
      const Eigen::Vector3d d = cam.pixelToRadarRay(uv);
      if (d.z() > -1e-6) {return false;}
      const double s = (ground_z - o.z()) / d.z();
      if (s <= 0) {return false;}
      g = (o + s * d).head<2>();
      return true;
    };
  const int step = std::max(1, cfg_.row_step);
  std::vector<RowPoint> edge[2];    // road left / right of this mask, nearest row first
  rows_.clear();
  int seed = -1;
  for (int vm = mask_h - 1; vm >= 0; vm -= step) {
    Eigen::Vector2d gc;
    if (!ground(0.5 * mask_w - 0.5, vm, gc)) {break;}   // above the horizon (all rows above too)
    if (gc.x() < cfg_.min_range) {continue;}
    if (gc.x() > cfg_.max_range) {break;}
    const uint8_t * row = mask + static_cast<std::size_t>(vm) * mask_w;
    const uint8_t * occ_row = occ.empty() ? nullptr : occ.data() + static_cast<std::size_t>(vm) * mask_w;
    if (seed < 0) {   // nearest row: vehicle centre line (y = 0) at this range -> mask column
      const auto uv = cam.projectRadarPoint({gc.x(), 0.0, ground_z}, 1e4);
      if (!uv) {continue;}
      seed = std::clamp(static_cast<int>(std::lround((uv->x() + 0.5) / sx - 0.5)), 0, mask_w - 1);
    }
    if (!drivable(row[seed])) {   // object or gap at the seed: nearest road within max_occluder_px
      int found = -1;
      for (int d = 1; d <= cfg_.max_occluder_px && found < 0; ++d) {
        for (const int c : {seed - d, seed + d}) {
          if (c >= 0 && c < mask_w && drivable(row[c])) {found = c; break;}
        }
      }
      if (found < 0) {break;}   // no road ahead
      seed = found;
    }
    int end[2];
    bool seen[2];
    for (int k = 0; k < 2; ++k) {   // k 0: image left = vehicle left (y > 0)
      seen[k] = runEnd(row, occ_row, mask_w, seed, k == 0 ? -1 : 1, cfg_.max_gap_px, cfg_.max_occluder_px, end[k]);
      Eigen::Vector2d g;
      if (seen[k] && ground(end[k] + (k == 0 ? -0.5 : 0.5), vm, g) && std::abs(g.y()) <= cfg_.max_lateral) {
        edge[k].push_back({g, gc.x()});
      }
    }
    rows_.push_back({vm, seen[0] ? end[0] : -1, seen[1] ? end[1] : -1});
    // lane marking: first marking run outwards from the seed
    for (int k = 0; k < 2; ++k) {
      const int dir = k == 0 ? -1 : 1;
      for (int u = seed + dir; u >= 0 && u < mask_w; u += dir) {
        if (row[u] != kLane) {continue;}
        int e = u;
        while (e + dir >= 0 && e + dir < mask_w && row[e + dir] == kLane) {e += dir;}
        if (std::abs(e - u) + 1 >= cfg_.min_run_px) {
          Eigen::Vector2d g;
          if (ground(0.5 * (u + e), vm, g) && std::abs(g.y()) <= cfg_.max_lateral) {pts_[k].push_back({g, stamp});}
          break;
        }
        u = e;
      }
    }
    seed = (end[0] + end[1]) / 2;   // next row: the middle of this run (follows curves)
  }
  for (int k = 0; k < 2; ++k) {
    for (const auto & p : edgePieces(edge[k], cfg_.max_edge_jump, cfg_.max_edge_angle * M_PI / 180.0)) {pts_[2 + k].push_back({p, stamp});}
  }
  est_.stamp = stamp;
  LaneBoundary * fitted[] = {&est_.lane_left, &est_.lane_right, &est_.road_left, &est_.road_right};
  for (int k = 0; k < 4; ++k) {
    *fitted[k] = fit(pts_[k]);
    truncateSideways(*fitted[k]);
  }
  roadWidth();
  return est_;
}

// From the vehicle outwards up to the first step with |dy| > tan(max_edge_angle) dx (sideways or backwards).
void LaneEstimator::truncateSideways(LaneBoundary & b) const
{
  if (!b.valid) {return;}
  const double tan_max = std::tan(cfg_.max_edge_angle * M_PI / 180.0);
  std::size_t n = 1;
  double length = 0;
  for (; n < b.points.size(); ++n) {
    const Eigen::Vector2d d = b.points[n] - b.points[n - 1];
    if (std::abs(d.y()) > tan_max * d.x()) {break;}
    length += d.norm();
  }
  b.points.resize(n);
  if (length < cfg_.min_span) {b = LaneBoundary{};}
}

// Each road-left point paired with the closest point of the right edge (inside it): median distance.
void LaneEstimator::roadWidth()
{
  const auto & L = est_.road_left;
  const auto & Rt = est_.road_right;
  est_.road_width = -1;
  if (!L.valid || !Rt.valid) {return;}
  std::vector<double> widths;
  for (const auto & p : L.points) {
    Eigen::Vector2d q;
    if (!closestOnPolyline(Rt.points, p, q)) {continue;}
    const double w = (q - p).norm();
    if (w >= cfg_.min_width && w <= cfg_.max_width) {widths.push_back(w);}
  }
  if (widths.empty()) {return;}
  std::nth_element(widths.begin(), widths.begin() + widths.size() / 2, widths.end());
  est_.road_width = widths[widths.size() / 2];
}

// Least squares v = c0 + c1 s (+ c2 s^2) in the principal frame of the points (s along, v across),
// points beyond 3 outlier then 1 outlier dropped and refitted.
LaneBoundary LaneEstimator::fit(const std::vector<Point> & pts) const
{
  const int n = static_cast<int>(pts.size());
  if (n < cfg_.min_points) {return {};}
  std::vector<char> use(n, 1);
  Eigen::Vector3d c = Eigen::Vector3d::Zero();
  Eigen::Vector2d mean = Eigen::Vector2d::Zero(), u = Eigen::Vector2d::UnitX(), nv = Eigen::Vector2d::UnitY();
  for (const double tol : {3.0 * cfg_.outlier, cfg_.outlier, cfg_.outlier}) {
    int m = 0;
    mean.setZero();
    for (int i = 0; i < n; ++i) {
      if (use[i]) {mean += pts[i].p; ++m;}
    }
    if (m < cfg_.min_points) {return {};}
    mean /= m;
    // principal direction: eigenvector of the largest eigenvalue of the scatter matrix
    Eigen::Matrix2d S = Eigen::Matrix2d::Zero();
    double near = 1e18;
    Eigen::Vector2d p_near = mean;
    for (int i = 0; i < n; ++i) {
      if (!use[i]) {continue;}
      const Eigen::Vector2d d = pts[i].p - mean;
      S += d * d.transpose();
      if (pts[i].p.norm() < near) {near = pts[i].p.norm(); p_near = pts[i].p;}
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(S);
    u = es.eigenvectors().col(1);
    if (u.dot(p_near - mean) > 0) {u = -u;}   // s grows away from the vehicle
    nv = Eigen::Vector2d(-u.y(), u.x());
    double s0 = 1e18, s1 = -1e18;
    for (int i = 0; i < n; ++i) {
      if (!use[i]) {continue;}
      const double s = u.dot(pts[i].p - mean);
      s0 = std::min(s0, s);
      s1 = std::max(s1, s);
    }
    if (s1 - s0 < cfg_.min_span) {return {};}
    const int k = s1 - s0 >= cfg_.quadratic_span ? 3 : 2;
    Eigen::MatrixXd A(m, k);
    Eigen::VectorXd y(m);
    for (int i = 0, r = 0; i < n; ++i) {
      if (!use[i]) {continue;}
      const Eigen::Vector2d d = pts[i].p - mean;
      const double s = u.dot(d);
      A(r, 0) = 1.0;
      A(r, 1) = s;
      if (k == 3) {A(r, 2) = s * s;}
      y(r++) = nv.dot(d);
    }
    const Eigen::VectorXd sol = A.colPivHouseholderQr().solve(y);
    c = Eigen::Vector3d(sol(0), sol(1), k == 3 ? sol(2) : 0.0);
    // |v_i - (c0 + c1 s_i + c2 s_i^2)| > tol -> dropped
    for (int i = 0; i < n; ++i) {
      const Eigen::Vector2d d = pts[i].p - mean;
      const double s = u.dot(d);
      use[i] = std::abs(nv.dot(d) - (c(0) + (c(1) + c(2) * s) * s)) <= tol;
    }
  }
  LaneBoundary b;
  double s0 = 1e18, s1 = -1e18;
  for (int i = 0; i < n; ++i) {
    if (!use[i]) {continue;}
    const double s = u.dot(pts[i].p - mean);
    s0 = std::min(s0, s);
    s1 = std::max(s1, s);
    ++b.support;
  }
  if (b.support < cfg_.min_points || s1 - s0 < cfg_.min_span) {return {};}
  b.valid = true;
  b.origin = mean;
  b.dir = u;
  b.coef = c;
  b.s_min = s0;
  b.s_max = s1;
  // p(s) = mean + s u + v(s) n
  for (double s = s0; s <= s1 + 1e-9; s += cfg_.step) {b.points.push_back(b.curveAt(s));}
  return b;
}

}  // namespace sensor_fusion
