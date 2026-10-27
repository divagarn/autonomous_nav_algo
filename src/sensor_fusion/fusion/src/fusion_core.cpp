#include "sensor_fusion/fusion_core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "fusion_common/hungarian.hpp"
#include "fusion_common/object_classes.hpp"

namespace sensor_fusion
{

FusionCore::FusionCore(const FusionConfig & cfg)
: cfg_(cfg), ego_(cfg.fusion_frame_mount), path_(cfg.path, cfg.fusion_frame_mount)
{
}

// Odometry: yaw rate always, speed only if ego_speed_source is odom.
void FusionCore::addEgo(const EgoSample & s)
{
  last_yaw_rate_ = s.yaw_rate;
  path_.addOdom(s.speed, s.yaw_rate);
  if (cfg_.ego_speed_source == "radar_doppler") {
    // keep odometry yaw rate; speed is filled per radar cycle
    EgoSample y = s;
    y.speed = ego_.empty() ? s.speed : ego_.at(s.stamp).speed;
    ego_.add(y);
    return;
  }
  ego_.add(s);
}

// Clears all state.
void FusionCore::reset()
{
  tracks_.clear();
  last_stamp_ = -1;
}

// Tracker output: its tracks are the radar tracks.
std::size_t FusionCore::processRadarTracks(
  Time stamp, const std::vector<ExternalRadarTrack> & ext, double ego_speed, const std::string & tracker)
{
  if (last_stamp_ >= 0 && stamp < last_stamp_ - 1.0) {reset();}   // bag restarted
  if (last_stamp_ >= 0 && stamp < last_stamp_) {return 0;}        // out of order, drop
  if (cfg_.ego_speed_source == "radar_doppler") {
    // no raw targets here: the tracker's ego speed stands in for the Doppler estimate
    const EgoSample odo = ego_.at(stamp);
    ego_.add(EgoSample{stamp, ego_speed, ego_.empty() ? last_yaw_rate_ : odo.yaw_rate});
  }
  const EgoSample e = ego_.at(stamp);
  last_ego_speed_ = e.speed;
  last_stamp_ = stamp;
  tracker_ = tracker;
  const auto & xc = cfg_.external;

  const bool ekf_flag = xc.motion_source == "ekf_flag";
  std::map<uint32_t, std::size_t> by_ext;
  for (std::size_t i = 0; i < tracks_.size(); ++i) {by_ext[tracks_[i].ext_id] = i;}
  std::vector<Track> next;
  next.reserve(ext.size());
  for (const auto & r : ext) {
    if (r.mirror_ghost && xc.drop_mirror_ghosts) {continue;}
    if (r.ground_reflection && xc.drop_ground_reflections) {continue;}
    Track tr;
    const auto it = by_ext.find(r.id);
    const bool born = it == by_ext.end();
    if (!born) {
      tr = std::move(tracks_[it->second]);   // keeps id, camera association, votes, motion score
    } else {
      tr.id = next_id_++;
      tr.ext_id = r.id;
    }
    if (r.full_state) {
      tr.x << r.px, r.py, r.vx, r.vy;
    } else {
      // over-ground velocity: a static point seen from the moving sensor has
      // v_rel = -(v_e, 0) - w x p  ->  v_ground = v_rel + (v_e - w p_y, w p_x)
      tr.x << r.px, r.py, r.vx + e.speed - e.yaw_rate * r.py, r.vy + e.yaw_rate * r.px;
    }
    tr.P = r.P;
    tr.yaw_rate = r.yaw_rate;
    tr.mode_probs = r.mode_probs;
    const bool detected = born || r.hits > tr.hits;   // a new detection in this cycle (not a prediction)
    tr.hits = r.hits;
    tr.age = r.age;
    tr.confirmed = r.confirmed;
    tr.z = r.pz;
    tr.existence = r.existence;
    tr.radar_updated = r.radar_updated;
    tr.ghost = r.ghost;
    tr.turning = r.turning;
    if (r.full_state) {
      tr.heading = r.heading;
      tr.moving = r.moving;
      tr.ground = r.ground;
      // final-list membership with hysteresis: enter at output_min_existence, leave below
      // output_keep_existence (weak targets such as pedestrians miss single scans)
      if (!tr.listed && tr.existence >= cfg_.output_min_existence) {tr.listed = true;}
      if (tr.listed && tr.existence < cfg_.output_keep_existence) {tr.listed = false;}
    } else {
      // height above ground EMA (EKF tracker bookkeeping: alpha = min(1 / (hits + 1), 0.3),
      // seeded at birth); its own ground rule needs a weak return, see external.ground_*
      const double h = r.pz + cfg_.camera.radar_height_above_ground;
      tr.z_ema = born ? h : tr.z_ema + std::min(1.0 / (r.hits + 1.0), 0.3) * (h - tr.z_ema);
      if (detected) {pushGroundHeight(tr.z_hist, h, xc.ground_window);}
      tr.ground = xc.ground_check && groundByHeight(tr.z_hist, xc.ground_z_threshold, xc.ground_min_samples,
        xc.ground_quantile);
      tr.listed = true;    // the EKF tracker exports only confirmed tracks with poe >= 0.6
      const Eigen::Vector2d v = tr.x.segment<2>(2);
      if (v.norm() > xc.moving_speed_threshold) {tr.heading = std::atan2(v.y(), v.x());}
      if (ekf_flag) {
        tr.moving = r.dynamic;
      } else {
        // motion state: EMA of "over-ground speed >= moving_speed_threshold and significant", hysteresis
        const double speed = v.norm();
        const Eigen::Matrix2d Pv = tr.P.block<2, 2>(2, 2);
        const double s_speed = speed > 1e-6 ? std::sqrt(std::max(0.0, v.dot(Pv * v))) / speed : 0.0;
        const bool evidence = speed >= xc.moving_speed_threshold && speed >= xc.moving_min_sigma * s_speed;
        tr.motion_score += xc.motion_alpha * ((evidence ? 1.0 : 0.0) - tr.motion_score);
        if (!tr.moving && tr.motion_score > xc.motion_enter) {tr.moving = true;}
        if (tr.moving && tr.motion_score < xc.motion_exit) {tr.moving = false;}
      }
    }
    next.push_back(std::move(tr));
  }
  tracks_ = std::move(next);
  output_time_ = stamp;
  return tracks_.size();
}

// ------------------------------------------------------------------ camera cycle

// Cached camera model per detector image size.
void FusionCore::updateCameraModel(int width, int height)
{
  if (!have_camera_model_ || width <= 0 || height <= 0) {return;}
  if (cam_size_ == std::make_pair(width, height)) {return;}
  const auto & cc = cfg_.camera;
  camera_ = fusion_common::cameraForImage(base_camera_, width, height, cc.rectified, cc.rectify_balance,
    cc.rectify_fov_scale);
  cam_size_ = {width, height};
}

// Ground range of a pixel (ray to the ground plane), nullopt above the horizon.
std::optional<double> FusionCore::groundRange(double u, double v) const
{
  const Eigen::Vector3d d = camera_.pixelToRadarRay({u, v});
  const Eigen::Vector3d o = camera_.cameraPositionInRadar();
  if (d.z() >= -1e-6) {return std::nullopt;}
  // ray o + s d hits the ground plane z = -h:  s = (-h - o_z) / d_z
  const double s = (-cfg_.camera.radar_height_above_ground - o.z()) / d.z();
  if (s <= 0) {return std::nullopt;}
  return (o + s * d).head<2>().norm();
}

// Bearing (radar frame) of pixel (u, v) at horizontal range r from the radar (parallax exact).
std::optional<double> FusionCore::bearingAtRange(double u, double v, double r) const
{
  const Eigen::Vector3d d3 = camera_.pixelToRadarRay({u, v});
  const Eigen::Vector2d o = camera_.cameraPositionInRadar().head<2>();
  if (d3.head<2>().norm() < 1e-6) {return std::nullopt;}
  const Eigen::Vector2d e = d3.head<2>().normalized();
  // |o + s e| = r  ->  s = -o.e + sqrt((o.e)^2 - |o|^2 + r^2)
  const double oe = o.dot(e), disc = oe * oe - o.squaredNorm() + r * r;
  if (disc < 0) {return std::nullopt;}
  const double s = -oe + std::sqrt(disc);
  if (s <= 0) {return std::nullopt;}
  const Eigen::Vector2d p = o + s * e;
  return std::atan2(p.y(), p.x());
}

// Camera cycle: gating, bounded cheap JPDA, one-to-one assignment, bearing Bayes, class votes.
std::size_t FusionCore::processCamera(const CameraFrame & f)
{
  const auto & cc = cfg_.camera;
  if (last_stamp_ >= 0 && f.stamp < last_stamp_ - 1.0) {reset();}   // bag restarted
  if (!cc.enabled || !have_camera_model_) {return 0;}
  updateCameraModel(f.width, f.height);
  output_time_ = f.stamp;

  // confirmed radar tracks at the frame time
  std::vector<int> ti;
  for (std::size_t i = 0; i < tracks_.size(); ++i) {
    if (tracks_[i].confirmed) {ti.push_back(static_cast<int>(i));}
  }
  std::vector<const CameraDetection *> boxes;
  for (const auto & b : f.detections) {
    if (b.score < cc.min_detection_score) {continue;}
    if (std::find(cc.ignore_classes.begin(), cc.ignore_classes.end(), b.label) != cc.ignore_classes.end()) {continue;}
    boxes.push_back(&b);
  }
  const int nb = static_cast<int>(boxes.size()), nt = static_cast<int>(ti.size());
  if (nb == 0 || nt == 0) {return 0;}

  const double W = cam_size_.first, H = cam_size_.second, px = W / 1920.0;
  const double fx = camera_.calib().fx;
  const double hg = -cc.radar_height_above_ground;
  std::vector<double> r(nt), th_r(nt), s_r(nt), u(nt, NAN), v_ground(nt, NAN), v_top(nt, NAN);
  for (int k = 0; k < nt; ++k) {
    const auto & tr = tracks_[ti[k]];
    const Snapshot sn = snapshot(tr, f.stamp);   // track at the frame time (tracker unchanged)
    const Eigen::Vector2d p = sn.p;
    r[k] = std::max(0.1, p.norm());
    th_r[k] = std::atan2(p.y(), p.x());
    // bearing std: J = d atan2(y, x) / d(x, y) = (-y, x) / r^2,  var = J P J^T
    const Eigen::Vector2d j(-p.y() / (r[k] * r[k]), p.x() / (r[k] * r[k]));
    const double var = j.dot(sn.P * j);
    const double smin = cc.sigma_radar_min_deg * M_PI / 180.0;
    s_r[k] = std::sqrt(std::max(var, smin * smin));
    const auto a = camera_.projectRadarPoint({p.x(), p.y(), tr.z}, 1e4);
    const auto g = camera_.projectRadarPoint({p.x(), p.y(), hg}, 1e4);
    const auto t = camera_.projectRadarPoint({p.x(), p.y(), hg + cc.object_height}, 1e4);
    if (a && g && t) {u[k] = a->x(); v_ground[k] = g->y(); v_top[k] = t->y();}
  }

  // gates + cost per (box, track)
  const double BLOCK = 1e6;
  Eigen::MatrixXd cost = Eigen::MatrixXd::Constant(nb, nt, BLOCK);
  Eigen::MatrixXd th_c = Eigen::MatrixXd::Constant(nb, nt, NAN);
  std::vector<double> s_c(nb, 0.0);
  const double m = cc.margin_px * px;
  for (int i = 0; i < nb; ++i) {
    const auto & d = *boxes[i];
    const bool bottom_cut = d.y_max >= H - cc.border_px * px;
    const auto rg = bottom_cut ? std::nullopt : groundRange(0.5 * (d.x_min + d.x_max), d.y_max);
    std::vector<char> g(nt, 0);
    double rmin = 1e9;
    for (int k = 0; k < nt; ++k) {
      if (std::isnan(u[k])) {continue;}
      // a track at ground level (kerb, drain cover) is no camera object
      if (r[k] < cc.min_track_height_range &&
        tracks_[ti[k]].z + cc.radar_height_above_ground < cc.min_track_height) {continue;}
      // gate: projected track column inside the box, [ground, ground + object_height] overlaps the box rows
      bool ok = u[k] >= d.x_min - m && u[k] <= d.x_max + m && v_top[k] <= d.y_max + m && v_ground[k] >= d.y_min - m;
      if (ok && rg) {
        // range gate: ratio_min <= r_radar / r_ground(box bottom) <= ratio_max
        const double ratio = r[k] / *rg;
        ok = ratio >= cc.range_ratio_min && ratio <= cc.range_ratio_max;
      }
      if (ok) {g[k] = 1; rmin = std::min(rmin, r[k]);}
    }
    // camera bearing std = (sigma_px + sigma_frac * box_width) / fx
    s_c[i] = (cc.sigma_px * px + cc.sigma_frac * (d.x_max - d.x_min)) / fx;
    for (int k = 0; k < nt; ++k) {
      if (!g[k]) {continue;}
      const auto tc = bearingAtRange(0.5 * (d.x_min + d.x_max), 0.5 * (d.y_min + d.y_max), r[k]);
      if (!tc) {continue;}
      th_c(i, k) = *tc;
      const double dth = std::remainder(th_r[k] - *tc, 2 * M_PI);
      const double se = cc.sigma_extent / r[k];
      // d_b^2 = dtheta^2 / (s_r^2 + s_c^2 + (sigma_extent / r)^2)
      const double db = dth * dth / (s_r[k] * s_r[k] + s_c[i] * s_c[i] + se * se);
      // cost = d_b^2 + ((r - r_min) / range_pref)^2
      if (db < cc.gate_d2) {cost(i, k) = db + std::pow((r[k] - rmin) / cc.range_pref, 2);}
    }
  }
  // bounded cheap JPDA (best few tracks per box), then one-to-one on -log(probability)
  Eigen::MatrixXd lik = Eigen::MatrixXd::Zero(nb, nt);
  for (int i = 0; i < nb; ++i) {
    std::vector<int> idx;
    for (int k = 0; k < nt; ++k) {if (cost(i, k) < BLOCK) {idx.push_back(k);}}
    std::sort(idx.begin(), idx.end(), [&](int a, int b) {return cost(i, a) < cost(i, b);});
    if (static_cast<int>(idx.size()) > cc.jpda_max_candidates) {idx.resize(cc.jpda_max_candidates);}
    // likelihood = exp(-cost / 2)
    for (int k : idx) {lik(i, k) = std::exp(-0.5 * std::min(cost(i, k), 50.0));}
  }
  const Eigen::MatrixXd beta = cheapJpda(lik, cc.jpda_pd, cc.jpda_clutter);
  Eigen::MatrixXd nl = Eigen::MatrixXd::Constant(nt, nb, BLOCK);   // rows = tracks
  for (int i = 0; i < nb; ++i) {
    for (int k = 0; k < nt; ++k) {
      // assignment cost = -ln(beta)
      if (beta(i, k) > 1e-6) {nl(k, i) = -std::log(beta(i, k));}
    }
  }
  const auto assign = fusion_common::hungarian(nl, BLOCK / 10);

  std::size_t fused = 0;
  for (int k = 0; k < nt; ++k) {
    if (assign[k] < 0) {continue;}
    const int i = assign[k];
    auto & tr = tracks_[ti[k]];
    const auto & d = *boxes[i];
    // Gaussian Bayes on the bearing, radar keeps at least radar_min_weight
    // s_c' = sqrt(s_c^2 + (sigma_extent / r)^2)
    const double sc = std::hypot(s_c[i], cc.sigma_extent / r[k]);
    const double dth = std::remainder(th_c(i, k) - th_r[k], 2 * M_PI);
    double delta = 0.0;
    // consistency: |dtheta| <= k * sqrt(s_r^2 + s_c'^2)
    if (std::abs(dth) <= cc.consistency_sigma * std::hypot(s_r[k], sc)) {
      const double wr = 1.0 / (s_r[k] * s_r[k]), wc = 1.0 / (sc * sc);
      // radar weight a = max(w_r / (w_r + w_c), radar_min_weight), w = 1 / sigma^2;  delta = (1 - a) dtheta
      const double a = std::max(wr / (wr + wc), cc.radar_min_weight);
      delta = (1.0 - a) * dth;
    }
    tr.bearing_delta = delta;
    if (classStale(tr, f.stamp)) {tr.votes.clear();}   // old label gone: start voting afresh
    tr.camera_time = f.stamp;
    tr.camera_track_id = boxes[i]->track_id;
    tr.assoc_prob = beta(i, k);
    // class: temporal filter, votes decay per associated frame
    // votes_c <- decay * votes_c, then +1 for the detected class
    for (auto & kv : tr.votes) {kv.second *= cc.class_vote_decay;}
    tr.votes[fusion_common::canonicalClass(d.label)] += 1.0;
    ++fused;
  }
  return fused;
}

// Fitzgerald (1985): G = pd * lik, B = clutter (1 - pd).
// beta_tm = G_tm / (S_t + S_m - G_tm + B), S_t = row sum, S_m = column sum; rows renormalised to <= 1.
Eigen::MatrixXd cheapJpda(const Eigen::MatrixXd & lik, double pd, double clutter)
{
  const Eigen::MatrixXd G = pd * lik;
  const Eigen::VectorXd St = G.rowwise().sum();
  const Eigen::RowVectorXd Sm = G.colwise().sum();
  const double B = clutter * (1.0 - pd) + 1e-12;
  Eigen::MatrixXd beta = Eigen::MatrixXd::Zero(G.rows(), G.cols());
  for (int t = 0; t < G.rows(); ++t) {
    for (int m = 0; m < G.cols(); ++m) {
      if (G(t, m) > 0) {beta(t, m) = G(t, m) / (St[t] + Sm[m] - G(t, m) + B);}
    }
    const double s = beta.row(t).sum();
    if (s > 1.0) {beta.row(t) /= s;}
  }
  return beta;
}

// ------------------------------------------------------------------ output

// |t_output - t_camera| <= hold_s
bool FusionCore::cameraRecent(const Track & t) const
{
  return t.camera_time >= 0 && std::abs(output_time_ - t.camera_time) <= cfg_.camera.hold_s;
}

// Constant-velocity prediction of position and covariance, then ego-frame transform.
FusionCore::Snapshot FusionCore::snapshot(const Track & t, Time time) const
{
  Snapshot s;
  const auto & x = t.x;
  const auto & P = t.P;
  const double dt = last_stamp_ >= 0 ? time - last_stamp_ : 0.0;
  // p(t) = p + dt v,  P_p(t) = P_p + dt^2 P_v
  s.p = x.head<2>() + dt * x.segment<2>(2);
  s.v = x.segment<2>(2);
  s.P = P.topLeftCorner<2, 2>() + dt * dt * P.block<2, 2>(2, 2);
  if (std::abs(dt) > 1e-4 && !ego_.empty()) {   // into the sensor frame at `time`
    Eigen::Matrix2d R;
    Eigen::Vector2d tr;
    ego_.frameTransform(last_stamp_, time, R, tr);
    // p' = R p + t, v' = R v, P' = R P R^T
    s.p = R * s.p + tr;
    s.v = R * s.v;
    s.P = R * s.P * R.transpose();
  }
  return s;
}

// Radar position at the output time, rotated by the last camera bearing correction while it
// is recent.
Eigen::Vector2d FusionCore::outputPosition(const Track & t) const
{
  const Eigen::Vector2d p = snapshot(t, output_time_).p;
  if (!cameraRecent(t) || t.bearing_delta == 0.0) {return p;}
  // rotate p by the bearing correction delta: p' = Rot(delta) p
  const double c = std::cos(t.bearing_delta), s = std::sin(t.bearing_delta);
  return {c * p.x() - s * p.y(), s * p.x() + c * p.y()};
}

// Object centre: the radar sees the face towards the sensor, so the centre lies half
// the class extent further along the line of sight, at half the class height.
Eigen::Vector3d FusionCore::classCentre(const Track & t, const Eigen::Vector2d & p) const
{
  const auto & ci = fusion_common::classInfo(bestClass(t, nullptr));
  const double r = std::max(0.5, p.norm());
  const double rel = t.heading - std::atan2(p.y(), p.x());
  // extent along the line of sight = |L cos(rel)| + |W sin(rel)|,  centre = p + u * extent / 2
  const double extent = std::abs(ci.length * std::cos(rel)) + std::abs(ci.width * std::sin(rel));
  const Eigen::Vector2d c = p + p / r * 0.5 * extent;
  return {c.x(), c.y(), -cfg_.camera.radar_height_above_ground + 0.5 * ci.height};
}

// Argmax of the votes; confidence = best / sum.
bool FusionCore::classStale(const Track & t, Time at) const
{
  const double keep = cfg_.camera.class_keep_s;
  return keep > 0 && (t.camera_time < 0 || at - t.camera_time > keep);
}

std::string FusionCore::bestClass(const Track & t, double * conf) const
{
  if (conf) {*conf = 0.0;}
  std::string best = "unknown";
  if (classStale(t, output_time_)) {return best;}   // no camera association for class_keep_s
  double bv = 0, sum = 0;
  for (const auto & [k, v] : t.votes) {
    sum += v;
    if (v > bv) {bv = v; best = k;}
  }
  if (conf && sum > 0) {*conf = bv / sum;}
  return best;
}

// 2D box of the projected class-sized 3D box at the fused position.
bool FusionCore::constructBox(const Track & t, const Eigen::Vector2d & pos, double box[4]) const
{
  if (cam_size_.first == 0) {return false;}
  const auto & ci = fusion_common::classInfo(bestClass(t, nullptr));
  const Eigen::Vector3d c = classCentre(t, pos);
  const double ch = std::cos(t.heading), sh = std::sin(t.heading);
  double x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
  int n = 0;
  for (int i = 0; i < 8; ++i) {
    const double l = (i & 1 ? 0.5 : -0.5) * ci.length, w = (i & 2 ? 0.5 : -0.5) * ci.width;
    const double z = (i & 4 ? 0.5 : -0.5) * ci.height;
    // corner = centre + Rot(heading) (+-L/2, +-W/2) + (0, 0, +-H/2)
    const auto uv = camera_.projectRadarPoint(
      Eigen::Vector3d(c.x() + l * ch - w * sh, c.y() + l * sh + w * ch, c.z() + z), 1e4);
    if (!uv) {continue;}
    x0 = std::min(x0, uv->x()); y0 = std::min(y0, uv->y());
    x1 = std::max(x1, uv->x()); y1 = std::max(y1, uv->y());
    ++n;
  }
  if (n < 4) {return false;}
  box[0] = std::clamp(x0, 0.0, static_cast<double>(cam_size_.first - 1));
  box[1] = std::clamp(y0, 0.0, static_cast<double>(cam_size_.second - 1));
  box[2] = std::clamp(x1, 0.0, static_cast<double>(cam_size_.first - 1));
  box[3] = std::clamp(y1, 0.0, static_cast<double>(cam_size_.second - 1));
  return box[2] - box[0] >= 2 && box[3] - box[1] >= 2;
}

// Fills every output field of one track.
FusedTrack FusionCore::toOutput(const Track & t) const
{
  FusedTrack o;
  const auto & x = t.x;
  o.id = t.id;
  o.tracker = tracker_;
  o.ekf_track_id = tracker_ == "ekf" ? static_cast<int64_t>(t.ext_id) : -1;
  o.pos = outputPosition(t);
  o.vel = snapshot(t, output_time_).v;
  o.pos_cov = t.P.topLeftCorner<2, 2>();
  o.yaw_rate = t.yaw_rate;           // EKF tracker: no turn-rate estimate (0)
  o.z = t.z;
  o.height = t.z + cfg_.camera.radar_height_above_ground;
  o.model_probs = t.mode_probs;      // EKF tracker: no IMM modes (empty)
  o.confirmed = t.confirmed;
  o.ghost = t.ghost;
  // filtered state (hysteresis), see processRadarTracks
  o.motion_state = !t.moving ? "stationary" : t.turning ? "turning" : "moving";
  o.is_moving = o.motion_state != "stationary";
  o.heading = t.heading;

  // Relative range rate uses over-ground object velocity minus the radar mount's
  // over-ground velocity. Positive closing_speed means the measured separation is
  // shrinking. This remains meaningful for a stationary obstacle while ego moves.
  const Eigen::Vector2d ego_vel = ego_.empty() ? Eigen::Vector2d::Zero() :
    ego_.sensorVelocity(output_time_);
  const double range = std::max(1e-3, o.pos.norm());
  const Eigen::Vector2d los = o.pos / range;
  o.closing_speed = -(o.vel - ego_vel).dot(los);
  o.time_to_collision = o.closing_speed > cfg_.relative_motion.radial_speed_threshold ?
    range / o.closing_speed : -1.0;

  const auto & rm = cfg_.relative_motion;
  const double ego_speed = ego_vel.norm();
  const double object_speed = o.vel.norm();
  const double same_direction_cos = std::cos(
    rm.same_direction_max_angle * M_PI / 180.0);
  const bool same_direction = o.is_moving && ego_speed >= rm.min_ego_speed &&
    object_speed >= rm.min_object_speed &&
    o.vel.dot(ego_vel) / (object_speed * ego_speed) >= same_direction_cos;
  if (same_direction) {
    o.relative_motion_state = "same_direction";
  } else if (o.closing_speed > rm.radial_speed_threshold) {
    o.relative_motion_state = "approaching";
  } else if (o.closing_speed < -rm.radial_speed_threshold) {
    o.relative_motion_state = "moving_away";
  } else if (!o.is_moving) {
    o.relative_motion_state = "stationary";
  } else {
    o.relative_motion_state = "crossing";
  }

  // predicted_pos: where the object will be over ground, in the CURRENT radar frame, so it is
  // transformed to the output frame exactly like pos (stationary tracks stay where they are).
  // p_pred = p + h v_over_ground
  o.prediction_horizon = std::max(0.0, rm.prediction_horizon);
  o.predicted_pos = o.is_moving ? Eigen::Vector2d(o.pos + o.prediction_horizon * o.vel) : o.pos;
  // relative to the ego at the horizon (ego moves too): what the corridor test below needs
  Eigen::Vector2d rel_future = snapshot(t, output_time_ + o.prediction_horizon).p;
  if (cameraRecent(t) && t.bearing_delta != 0.0) {
    const double c = std::cos(t.bearing_delta), s = std::sin(t.bearing_delta);
    rel_future = Eigen::Vector2d(c * rel_future.x() - s * rel_future.y(), s * rel_future.x() + c * rel_future.y());
  }
  o.age = static_cast<uint32_t>(t.age);
  o.hits = static_cast<uint32_t>(t.hits);
  o.existence = t.existence;
  o.radar_updated = t.radar_updated;
  // camera
  o.camera_associated = cameraRecent(t);
  o.status = o.camera_associated ? "radar+camera" : "radar";
  o.camera_track_id = o.camera_associated ? t.camera_track_id : 0;
  o.assoc_prob = o.camera_associated ? t.assoc_prob : 0.0;
  o.bearing_radar = std::atan2(x[1], x[0]);
  o.bearing_fused = std::atan2(o.pos.y(), o.pos.x());
  o.class_name = bestClass(t, &o.class_confidence);
  if (!t.votes.empty() && !classStale(t, output_time_)) {
    double sum = 0;
    for (const auto & kv : t.votes) {sum += kv.second;}
    for (const auto & c : fusion_common::fusedClasses()) {
      const auto it = t.votes.find(c);
      o.class_probs.push_back(it == t.votes.end() ? 0.0 : it->second / sum);
    }
  }
  const double half_w = 0.5 * (o.class_name == "unknown" ? 1.0 : fusion_common::classInfo(o.class_name).width);
  const PathCheck pc = path_.check(o.pos, half_w);
  o.in_path = pc.in_path;
  o.path_distance = pc.distance;
  o.path_lateral = pc.lateral;
  const PathCheck predicted_pc = path_.check(rel_future, half_w);
  o.predicted_in_path = predicted_pc.in_path;
  o.predicted_path_distance = predicted_pc.distance;
  o.predicted_path_lateral = predicted_pc.lateral;
  o.time_to_stop_zone = timeToStopZone(t, half_w);
  o.in_stop_zone = o.time_to_stop_zone == 0.0;
  // image (detector image): fused object centre, radar-only projection, class box
  if (have_camera_model_ && cam_size_.first > 0) {
    const auto uv = camera_.projectRadarPoint(classCentre(t, o.pos), 0.0);
    const auto pr = camera_.projectRadarPoint(classCentre(t, x.head<2>()), 1e4);
    if (uv) {
      o.in_image = true;
      o.u = uv->x();
      o.v = uv->y();
      if (pr) {o.proj_u = pr->x(); o.proj_v = pr->y();}
      o.has_bbox = constructBox(t, o.pos, o.bbox);
    }
  }
  return o;
}

// Every confirmed track.
std::vector<FusedTrack> FusionCore::confirmedTracks() const
{
  std::vector<FusedTrack> out;
  for (const auto & t : tracks_) {
    if (t.confirmed) {out.push_back(toOutput(t));}
  }
  return out;
}

// ------------------------------------------------------------------ stop zone

// L = min_distance + |v_ego| * time_headway
double FusionCore::stopZoneLength() const
{
  const double v = ego_.empty() ? 0.0 : std::abs(ego_.at(output_time_).speed);
  return cfg_.stop_zone.min_distance + v * cfg_.stop_zone.time_headway;
}

// Corridor from the radar to L ahead of it (path arc lengths are measured from base_link).
std::vector<Eigen::Vector2d> FusionCore::stopZone() const
{
  const double s0 = std::max(0.0, cfg_.fusion_frame_mount.x);
  return path_.section(s0, s0 + stopZoneLength());
}

// Sensor frame at last_stamp_ -> sensor frame at output_time_ + k step, ego holding its last
// speed / yaw rate beyond the newest odometry sample (EgoMotion::at holds the last sample).
const FusionCore::FutureFrames & FusionCore::futureFrames() const
{
  if (future_.from == last_stamp_ && future_.to == output_time_) {return future_;}
  const auto & sz = cfg_.stop_zone;
  const int n = static_cast<int>(std::floor(std::max(0.0, sz.horizon) / std::max(0.01, sz.step))) + 1;
  future_ = FutureFrames{last_stamp_, output_time_, {}, {}, {}};
  for (int k = 0; k < n; ++k) {
    const double time = output_time_ + k * sz.step;
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
    Eigen::Vector2d tr = Eigen::Vector2d::Zero();
    if (!ego_.empty() && std::abs(time - last_stamp_) > 1e-4) {ego_.frameTransform(last_stamp_, time, R, tr);}
    future_.dt.push_back(time - last_stamp_);
    future_.R.push_back(R);
    future_.t.push_back(tr);
  }
  return future_;
}

// p_k = R_k (p + dt_k v) + t_k in the ego frame at step k, inside the zone if it is in the
// corridor between the radar (s0) and s0 + L. Stationary tracks use v = 0 (their relative
// motion is the ego's), so velocity noise on walls does not create entries.
double FusionCore::timeToStopZone(const Track & t, double half_w) const
{
  const auto & ff = futureFrames();
  const auto & x = t.x;
  const Eigen::Vector2d v = t.moving ? Eigen::Vector2d(x.segment<2>(2)) : Eigen::Vector2d::Zero();
  const double s0 = std::max(0.0, cfg_.fusion_frame_mount.x);
  const double s1 = s0 + stopZoneLength();
  const bool rotate = cameraRecent(t) && t.bearing_delta != 0.0;
  const double c = std::cos(t.bearing_delta), s = std::sin(t.bearing_delta);
  for (std::size_t k = 0; k < ff.R.size(); ++k) {
    Eigen::Vector2d p = ff.R[k] * (x.head<2>() + ff.dt[k] * v) + ff.t[k];
    if (rotate) {p = Eigen::Vector2d(c * p.x() - s * p.y(), s * p.x() + c * p.y());}
    const PathCheck pc = path_.check(p, half_w);
    if (pc.in_path && pc.distance >= s0 && pc.distance <= s1) {return k * cfg_.stop_zone.step;}
  }
  return -1.0;
}

// Final list filters: ghost, existence, in front, range, high static radar-only clutter.
std::vector<FusedTrack> FusionCore::objects() const
{
  std::vector<FusedTrack> out;
  for (const auto & t : tracks_) {
    if (!t.confirmed) {continue;}
    FusedTrack o = toOutput(t);
    if (t.ghost || !t.listed) {continue;}   // existence hysteresis, see processRadarTracks
    // ground / overdrivable (drain cover, grating, speed breaker): stationary and its height near the
    // ground, unless the camera has a box on it (a person / vehicle / obstacle is no ground)
    if (t.ground && !t.moving && !o.camera_associated) {continue;}
    if (o.pos.x() <= 0.0 || o.range() > cfg_.output_max_range) {continue;}   // in front, within range
    if (!o.is_moving && !o.camera_associated && o.height > cfg_.output_max_static_height) {continue;}
    out.push_back(std::move(o));
  }
  std::sort(out.begin(), out.end(), [](const FusedTrack & a, const FusedTrack & b) {return a.range() < b.range();});
  return out;
}

}  // namespace sensor_fusion
