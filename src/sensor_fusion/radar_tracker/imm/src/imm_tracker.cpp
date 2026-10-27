#include "sensor_fusion/imm_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "fusion_common/hungarian.hpp"

namespace sensor_fusion
{

// Builds the IMM model set and normalises the transition matrix and initial mode probabilities.
ImmTracker::ImmTracker(const ImmTrackerConfig & cfg)
: cfg_(cfg), radar_pre_(cfg.radar), ego_(cfg.mount)
{
  const auto & tc = cfg_.tracker;
  const int M = static_cast<int>(tc.model_types.size());
  if (M == 0) {throw std::invalid_argument("at least one IMM model required");}
  for (int i = 0; i < M; ++i) {
    std::string type = tc.model_types[i];
    std::transform(type.begin(), type.end(), type.begin(), ::tolower);
    const double q = type == "ct" ? tc.accel_std_ct : type == "st" ? tc.pos_std_st : tc.accel_std_cv;
    imm_cfg_.models.push_back(makeMotionModel(type, q, tc.yaw_accel_std));
    if (type == "st") {idx_st_ = i;}
    if (type == "ct") {idx_ct_ = i;}
  }
  if (static_cast<int>(tc.transition.size()) != M * M) {
    throw std::invalid_argument("imm transition matrix must have models^2 entries");
  }
  imm_cfg_.transition = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
    tc.transition.data(), M, M);
  // pi_ij <- pi_ij / sum_j pi_ij
  for (int i = 0; i < M; ++i) {imm_cfg_.transition.row(i) /= imm_cfg_.transition.row(i).sum();}
  imm_cfg_.initial_mode_probs = tc.initial_mode_probs;
  imm_cfg_.initial_mode_probs.resize(M, 1.0 / M);
  double s = 0;
  for (double v : imm_cfg_.initial_mode_probs) {s += v;}
  for (double & v : imm_cfg_.initial_mode_probs) {v /= s;}
}

// Odometry: yaw rate always, speed only if ego_speed_source is odom.
void ImmTracker::addEgo(const EgoSample & s)
{
  last_yaw_rate_ = s.yaw_rate;
  if (cfg_.ego_speed_source == "radar_doppler") {
    // keep odometry yaw rate; speed is filled per radar cycle
    EgoSample y = s;
    y.speed = ego_.empty() ? s.speed : ego_.at(s.stamp).speed;
    ego_.add(y);
    return;
  }
  ego_.add(s);
}

// Clears all tracks.
void ImmTracker::reset()
{
  tracks_.clear();
  last_stamp_ = -1;
}

// Ego speed (radar Doppler or odom), preprocessing, then the IMM cycle.
std::vector<ExternalRadarTrack> ImmTracker::process(
  Time stamp, const std::vector<RadarDetection> & dets, double * ego_speed)
{
  if (cfg_.ego_speed_source == "radar_doppler") {
    double v = 0;
    const EgoSample odo = ego_.at(stamp);
    if (RadarPreprocessor::estimateSensorSpeed(dets, v)) {
      // sensor speed -> base speed (radar mounted with yaw mount.yaw, ignore lever arm)
      // v_base = v_sensor / cos(mount_yaw)
      v /= std::max(0.3, std::cos(cfg_.mount.yaw));
    } else {
      v = odo.speed;
    }
    ego_.add(EgoSample{stamp, v, ego_.empty() ? last_yaw_rate_ : odo.yaw_rate});
  }
  if (ego_speed) {*ego_speed = ego_.at(stamp).speed;}
  auto meas = radar_pre_.process(dets, stamp, &ego_);
  runCycle(stamp, std::move(meas));
  std::vector<ExternalRadarTrack> out;
  out.reserve(tracks_.size());
  for (const auto & t : tracks_) {out.push_back(toOutput(t));}
  return out;
}


// Prediction of every track to the new scan time in the new sensor frame.
void ImmTracker::predictTo(Time stamp)
{
  if (last_stamp_ < 0) {last_stamp_ = stamp; return;}
  const double dt = stamp - last_stamp_;
  if (dt <= 0) {return;}
  Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
  Eigen::Vector2d t = Eigen::Vector2d::Zero();
  if (!ego_.empty()) {ego_.frameTransform(last_stamp_, stamp, R, t);}
  for (auto & tr : tracks_) {
    tr.filter->predict(dt);
    tr.filter->transform(R, t);   // express in the sensor frame at `stamp`
  }
  last_stamp_ = stamp;
}

// Initial state at the cluster, velocity along the line of sight from Doppler.
std::unique_ptr<ImmFilter> ImmTracker::initFilter(const Measurement & m) const
{
  const auto & tc = cfg_.tracker;
  StateVec x = StateVec::Zero();
  StateMat P = StateMat::Zero();
  x.head<2>() = m.pos;
  P.topLeftCorner<2, 2>() = m.pos_cov;
  const double r = std::max(0.5, m.pos.norm());
  const Eigen::Vector2d u = m.pos / r;           // radial unit vector
  const Eigen::Vector2d n(-u.y(), u.x());        // tangential
  // P_v = s_t^2 n n^T + s_r^2 u u^T  (tangential velocity unknown, radial from Doppler)
  Eigen::Matrix2d Pv = tc.init_vel_std_tangential * tc.init_vel_std_tangential * n * n.transpose();
  if (m.has_doppler) {
    // v0 = doppler * u
    x.segment<2>(2) = m.doppler * u;
    Pv += (tc.init_vel_std_radial * tc.init_vel_std_radial + m.doppler_var) * u * u.transpose();
  } else {
    Pv += tc.init_vel_std_tangential * tc.init_vel_std_tangential * u * u.transpose();
  }
  P.block<2, 2>(2, 2) = Pv;
  P(4, 4) = tc.init_yaw_rate_std * tc.init_yaw_rate_std;
  return std::make_unique<ImmFilter>(&imm_cfg_, x, P);
}


// ------------------------------------------------------------------ radar cycle

// IMM tracker cycle of one radar scan.
std::size_t ImmTracker::runCycle(Time stamp, std::vector<Measurement> meas)
{
  if (last_stamp_ >= 0 && stamp < last_stamp_ - 1.0) {reset();}   // bag restarted
  if (last_stamp_ >= 0 && stamp < last_stamp_) {return 0;}        // out of order, drop
  predictTo(stamp);
  last_meas_ = meas;
  for (auto & tr : tracks_) {tr.radar_updated = false;}

  // --- association (global nearest neighbour, Mahalanobis gating incl. Doppler) ---
  const int nt = static_cast<int>(tracks_.size());
  const int nm = static_cast<int>(meas.size());
  Eigen::MatrixXd d2(nt, nm), cost = Eigen::MatrixXd::Constant(nt, nm, 1e9);
  const bool ekf_gate = cfg_.tracker.gating == "ekf";
  const Eigen::Vector2d vs = ego_.empty() ? Eigen::Vector2d::Zero() : ego_.sensorVelocity(stamp);
  for (int i = 0; i < nt; ++i) {
    for (int j = 0; j < nm; ++j) {
      if (ekf_gate) {
        // The EKF tracker's AdaptiveGating score S (pass S <= 1) is the assignment cost
        double score = 1e9;
        d2(i, j) = ekfGate(tracks_[i], meas[j], vs, score) ? score : 1e9;
        if (d2(i, j) <= 1.0) {cost(i, j) = score;}
        continue;
      }
      double logdet = 0;
      d2(i, j) = tracks_[i].filter->gatingDistance(meas[j], &logdet);
      // cost = d^2 + ln|S| (negative log-likelihood up to a constant), gated by chi2
      if (d2(i, j) < cfg_.tracker.gate_chi2) {cost(i, j) = d2(i, j) + logdet;}
    }
  }
  const auto assign = fusion_common::hungarian(cost, 1e8);
  const auto & tc = cfg_.tracker;
  std::vector<char> used(nm, 0), updated(nt, 0);
  for (int i = 0; i < nt; ++i) {
    auto & tr = tracks_[i];
    tr.age++;
    const bool hit = assign[i] >= 0;
    tr.hit_history.push_back(hit);
    while (static_cast<int>(tr.hit_history.size()) > cfg_.tracker.confirm_window) {
      tr.hit_history.pop_front();
    }
    if (!hit) {
      tr.missed++;
      // miss: pe <- pe * miss_factor
      tr.existence *= tc.existence_miss_factor;
      continue;
    }
    // hit: pe <- pe + gain * (1 - pe)
    tr.existence += tc.existence_hit_gain * (1.0 - tr.existence);
    const auto & m = meas[assign[i]];
    used[assign[i]] = 1;
    updated[i] = 1;
    tr.filter->update(m);
    updateEkfStats(tr, m, false);
    tr.hits++;
    tr.missed = 0;
    // height EMA: z <- 0.8 z + 0.2 z_meas
    tr.z = tr.hits == 1 ? m.z : 0.8 * tr.z + 0.2 * m.z;
    tr.radar_updated = true;
  }
  // --- new tracks; a second cluster of an object that just updated a track is absorbed ---
  for (int j = 0; j < nm; ++j) {
    if (used[j]) {continue;}
    bool absorbed = false;
    for (int i = 0; i < nt && !absorbed; ++i) {
      absorbed = updated[i] && d2(i, j) < (ekf_gate ? 1.0 : cfg_.tracker.absorb_gate);
    }
    if (absorbed) {continue;}
    Track tr;
    tr.id = next_id_++;
    tr.filter = initFilter(meas[j]);
    tr.hits = 1;
    tr.age = 1;
    tr.hit_history.push_back(true);
    tr.z = meas[j].z;
    tr.existence = tc.existence_init;
    tr.radar_updated = true;
    updateEkfStats(tr, meas[j], true);
    tracks_.push_back(std::move(tr));
  }
  manageTracks();
  // motion state: EMA of the per-scan evidence with hysteresis (no stationary/moving flicker)
  for (auto & tr : tracks_) {
    // s <- s + alpha * (evidence - s), moving if s > enter, stationary if s < exit
    tr.motion_score += tc.motion_alpha * ((movingEvidence(tr) ? 1.0 : 0.0) - tr.motion_score);
    if (!tr.moving && tr.motion_score > tc.motion_enter) {tr.moving = true;}
    if (tr.moving && tr.motion_score < tc.motion_exit) {tr.moving = false;}
  }
  if (tc.ghost_check) {
    checkGhosts(stamp);   // double bounce (sets ghost)
  } else {
    for (auto & tr : tracks_) {tr.ghost = false;}
  }
  if (tc.mirror_check) {checkMirrors(stamp);}
  for (auto & tr : tracks_) {
    if (tr.mirror) {tr.ghost = true;}   // The EKF tracker's N-fold mirror ghosts leave the output like double bounces
  }
  return meas.size();
}

// Instantaneous "moving" evidence: fast enough, significant, and not explained by the
// stationary model.
bool ImmTracker::movingEvidence(const Track & t) const
{
  const auto & tc = cfg_.tracker;
  const Eigen::Vector2d v = t.filter->x().segment<2>(2);
  const double speed = v.norm();
  const Eigen::Matrix2d Pv = t.filter->P().block<2, 2>(2, 2);
  // sigma_speed = sqrt(v^T P_v v) / |v|  (speed std along the velocity direction)
  const double s_speed = speed > 1e-6 ? std::sqrt(std::max(0.0, v.dot(Pv * v))) / speed : 0.0;
  const double p_st = idx_st_ >= 0 ? t.filter->modeProbs()[idx_st_] : 0.0;
  return speed >= tc.moving_speed_threshold && p_st <= tc.stationary_prob && speed >= tc.moving_min_sigma * s_speed;
}

// ------------------------------------------------------------------ mechanisms ported from the EKF tracker
// (radar_tracker/ekf: gating_module.cpp AdaptiveGating, mirror_detector.cpp, ekf_tracker.cpp)

// AdaptiveGating::evaluate on our state. The EKF tracker's track velocity is relative to the sensor (its EKF
// has no ego compensation): here v_rel = v_over_ground - v_sensor, so the gate is the same.
bool ImmTracker::ekfGate(
  const Track & t, const Measurement & m, const Eigen::Vector2d & vs, double & score) const
{
  score = 1e9;
  const auto & x = t.filter->x();
  const auto & P = t.filter->P();
  const Eigen::Vector2d p = x.head<2>();
  const Eigen::Vector2d v_ground = x.segment<2>(2);
  const Eigen::Vector2d v_rel = v_ground - vs;
  const double r = p.norm() + 1e-6;
  const Eigen::Vector2d u = p / r;
  // 1. motion class from the ego-compensated radial speed of the track (track bearing)
  const double abs_rel_speed = std::abs(v_ground.dot(u));
  const bool dynamic = abs_rel_speed > 0.3;
  // 2. gate factors: fast-dynamic 25/16, slow-dynamic 9/9, static 4/4
  double gx = 4.0, gy = 4.0;
  if (dynamic) {
    gx = abs_rel_speed > 5.0 ? 25.0 : 9.0;
    gy = abs_rel_speed > 5.0 ? 16.0 : 9.0;
  }
  // 3. object-size-adaptive gate: G = factor + size
  const double G_long = gx + std::max(t.length, 0.5);
  const double G_lat = gy + std::max(t.width, 0.5);
  // 4. diagonal variance floor
  const double var_x = std::max(P(0, 0), 0.2), var_y = std::max(P(1, 1), 0.2);
  // 5-7. residual rotated into the (relative) motion direction, bearing when not moving
  const Eigen::Vector2d d = m.pos - p;
  const double th = v_rel.squaredNorm() > 1e-4 ? std::atan2(v_rel.y(), v_rel.x()) : std::atan2(p.y(), p.x());
  const double c = std::cos(th), s = std::sin(th);
  const double d_long = c * d.x() + s * d.y();
  const double d_lat = -s * d.x() + c * d.y();
  // 8. variance in the rotated frame (cross-covariance ignored), floor 0.2
  const double var_long = std::max(var_x * c * c + var_y * s * s, 0.2);
  const double var_lat = std::max(var_x * s * s + var_y * c * c, 0.2);
  // 9. elliptical gate
  const double S = d_long * d_long / (var_long * G_long) + d_lat * d_lat / (var_lat * G_lat);
  if (S > 1.0) {return false;}
  // 10. radial consistency: predicted vs measured relative radial speed (Doppler only). Not on a
  // track's first association: the IMM's combined birth velocity is pulled towards 0 by the
  // stationary model (the EKF tracker's single CV EKF starts at the Doppler velocity), so a fast new track
  // would fail the 3 m/s gate on its second scan and never be confirmed.
  if (m.has_doppler && t.hits >= 2) {
    const double rm = std::max(1e-6, m.pos.norm());
    const double measured_rel = m.doppler - vs.dot(m.pos / rm);   // over-ground -> relative
    if (std::abs(v_rel.dot(u) - measured_rel) > cfg_.tracker.ekf_radial_gate) {return false;}
  }
  score = S;
  return true;
}

// ekf_tracker.cpp bookkeeping: extent and ground statistics, alpha = min(1 / (hits + 1), 0.3);
// seeded from the first cluster at birth; ground height = median of the last detections.
void ImmTracker::updateEkfStats(Track & t, const Measurement & m, bool first) const
{
  const auto & tc = cfg_.tracker;
  const double h = m.z + cfg_.radar.radar_height;   // height above ground
  if (first) {
    t.length = std::clamp(m.size_x, 0.2, 10.0);
    t.width = std::clamp(m.size_y, 0.2, 5.0);
    t.z_ema = h;
    t.rcs_ema = m.rcs_mean;
    t.rcs_var = 0.0;
  } else {
    const double alpha = std::min(1.0 / (t.hits + 1.0), 0.3);
    t.length = (1.0 - alpha) * t.length + alpha * std::clamp(m.size_x, 0.5, 10.0);
    t.width = (1.0 - alpha) * t.width + alpha * std::clamp(m.size_y, 0.5, 5.0);
    t.z_ema = (1.0 - alpha) * t.z_ema + alpha * h;
    // mean += a d,  var = (1 - a)(var + a d^2)
    const double delta = m.rcs_mean - t.rcs_ema;
    t.rcs_ema += alpha * delta;
    t.rcs_var = (1.0 - alpha) * (t.rcs_var + alpha * delta * delta);
  }
  pushGroundHeight(t.z_hist, h, tc.ground_window);
  t.ground = tc.ground_check && groundByHeight(t.z_hist, tc.ground_z_threshold, tc.ground_min_samples,
    tc.ground_quantile) &&
    t.rcs_ema < tc.ground_rcs_max;
}

// MirrorDetector::classify on our tracks: nearest stationary reflector in the lane; a track at
// N x its range (+-range_tol), same lateral position (+-lateral_tol) and N x its relative radial
// speed (+-speed_tol, magnitudes) is its N-fold ghost. Flags persist mirror_forget_frames and are
// frozen while the ego vehicle stands. the EKF tracker's reflector test reads vabsx/vabsy, which the EKF
// tracker never sets (always 0): here the real over-ground speed is used.
void ImmTracker::checkMirrors(Time stamp)
{
  const auto & tc = cfg_.tracker;
  const Eigen::Vector2d vs = ego_.empty() ? Eigen::Vector2d::Zero() : ego_.sensorVelocity(stamp);
  const double ego_speed = ego_.empty() ? 0.0 : ego_.at(stamp).speed;
  if (std::abs(ego_speed) < tc.mirror_ego_stationary) {return;}   // freeze
  const int n = static_cast<int>(tracks_.size());
  auto range = [&](int i) {return tracks_[i].filter->x().head<2>().norm();};
  auto rel_radial = [&](int i) {
      const Eigen::Vector2d p = tracks_[i].filter->x().head<2>();
      return (tracks_[i].filter->x().segment<2>(2) - vs).dot(p / std::max(1e-6, p.norm()));
    };
  int ref = -1;
  for (int i = 0; i < n; ++i) {
    const auto & t = tracks_[i];
    if (!t.confirmed || t.hits < tc.mirror_min_hits || t.mirror) {continue;}
    if (t.filter->x().segment<2>(2).norm() > tc.mirror_stationary_speed) {continue;}
    if (std::abs(t.filter->x()(1)) > tc.mirror_lane_half_width) {continue;}
    if (ref < 0 || range(i) < range(ref)) {ref = i;}
  }
  auto forget = [&](Track & t) {
      if (!t.mirror) {return;}
      if (++t.mirror_miss > tc.mirror_forget_frames) {
        t.mirror = false;
        t.mirror_source = 0;
        t.mirror_order = 0;
      }
    };
  if (ref < 0) {
    for (auto & t : tracks_) {forget(t);}
    return;
  }
  const double r_ref = range(ref), y_ref = tracks_[ref].filter->x()(1);
  const double vr_ref = std::abs(rel_radial(ref));
  for (int g = 0; g < n; ++g) {
    if (g == ref) {continue;}
    auto & t = tracks_[g];
    bool matched = false;
    for (const auto N : tc.mirror_fold_orders) {
      const double Nf = static_cast<double>(N);
      if (std::abs(range(g) - Nf * r_ref) > tc.mirror_range_tol) {continue;}
      if (std::abs(t.filter->x()(1) - y_ref) > tc.mirror_lateral_tol) {continue;}
      if (std::abs(std::abs(rel_radial(g)) - Nf * vr_ref) > tc.mirror_speed_tol) {continue;}
      if (tc.mirror_require_doppler_separation &&
        std::abs(std::abs(rel_radial(g)) - vr_ref) <= tc.mirror_speed_tol) {continue;}
      t.mirror = true;
      t.mirror_source = tracks_[ref].id;
      t.mirror_order = static_cast<int>(N);
      t.mirror_miss = 0;
      matched = true;
      break;
    }
    if (!matched) {forget(t);}
  }
}

// Double bounce (radar -> object -> ego -> object -> radar): a ghost appears on the bearing
// of a real object at twice its range with twice its relative radial speed.
void ImmTracker::checkGhosts(Time stamp)
{
  const auto & tc = cfg_.tracker;
  const Eigen::Vector2d vs = ego_.empty() ? Eigen::Vector2d::Zero() : ego_.sensorVelocity(stamp);
  const int n = static_cast<int>(tracks_.size());
  std::vector<double> r(n), az(n), vr(n);
  for (int i = 0; i < n; ++i) {
    const Eigen::Vector2d p = tracks_[i].filter->x().head<2>();
    r[i] = std::max(0.1, p.norm());
    az[i] = std::atan2(p.y(), p.x());
    // relative radial speed vr = (v_obj - v_sensor) . p / |p|
    vr[i] = (tracks_[i].filter->x().segment<2>(2) - vs).dot(p / r[i]);   // relative radial speed
  }
  const double az_tol = tc.ghost_az_tol_deg * M_PI / 180.0;
  for (int g = 0; g < n; ++g) {
    bool candidate = false;
    for (int k = 0; k < n && !candidate; ++k) {
      if (k == g || !tracks_[k].confirmed || tracks_[k].ghost || r[k] >= r[g]) {continue;}
      // ghost if |az_g - az_k| < tol, |r_g - 2 r_k| < tol r_g, |vr_g - 2 vr_k| < tol and
      // |vr_g - vr_k| > tol (the Doppler must tell 2x from 1x: at low relative speed it cannot,
      // and any object at twice the range of another one would be flagged)
      candidate = std::abs(std::remainder(az[g] - az[k], 2 * M_PI)) < az_tol &&
        std::abs(r[g] - 2.0 * r[k]) < tc.ghost_range_tol * r[g] &&
        std::abs(vr[g] - 2.0 * vr[k]) < tc.ghost_doppler_tol &&
        std::abs(vr[g] - vr[k]) > tc.ghost_doppler_tol;
    }
    auto & t = tracks_[g];
    t.ghost_count = candidate ? t.ghost_count + 1 : std::max(0, t.ghost_count - 1);
    t.ghost = t.ghost_count >= tc.ghost_min_count;
  }
}

// Confirmation (M of N), heading, deletion, duplicate merge.
void ImmTracker::manageTracks()
{
  const auto & tc = cfg_.tracker;
  for (auto & tr : tracks_) {
    if (!tr.confirmed) {
      const int h = static_cast<int>(std::count(tr.hit_history.begin(), tr.hit_history.end(), true));
      if (h >= tc.confirm_hits) {tr.confirmed = true;}
    }
    const Eigen::Vector2d v = tr.filter->x().segment<2>(2);
    if (v.norm() > tc.moving_speed_threshold) {tr.heading = std::atan2(v.y(), v.x());}
  }
  // delete
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const Track & tr) {
      const Eigen::Matrix2d Pp = tr.filter->P().topLeftCorner<2, 2>();
      const double pos_std = std::sqrt(std::max(Pp(0, 0), Pp(1, 1)));
      const bool window_failed = !tr.confirmed &&
      static_cast<int>(tr.hit_history.size()) >= tc.confirm_window;
      return tr.missed > (tr.confirmed ? tc.max_missed : tc.tentative_max_missed) ||
             pos_std > tc.max_pos_std || window_failed || tr.filter->x().head<2>().norm() > tc.max_range;
    }), tracks_.end());
  // merge duplicate confirmed tracks (keep the older one)
  std::sort(tracks_.begin(), tracks_.end(), [](const Track & a, const Track & b) {return a.id < b.id;});
  std::vector<char> dead(tracks_.size(), 0);
  for (std::size_t i = 0; i < tracks_.size(); ++i) {
    if (dead[i] || !tracks_[i].confirmed) {continue;}
    for (std::size_t j = i + 1; j < tracks_.size(); ++j) {
      if (dead[j]) {continue;}
      const Eigen::Vector2d d = tracks_[i].filter->x().head<2>() - tracks_[j].filter->x().head<2>();
      const Eigen::Vector2d dv = tracks_[i].filter->x().segment<2>(2) - tracks_[j].filter->x().segment<2>(2);
      if (d.norm() < tc.merge_distance && dv.norm() < 1.0) {dead[j] = 1;}
    }
  }
  std::vector<Track> keep;
  for (std::size_t i = 0; i < tracks_.size(); ++i) {
    if (!dead[i]) {keep.push_back(std::move(tracks_[i]));}
  }
  tracks_ = std::move(keep);
}

// Full state and bookkeeping of a track for the fusion (velocity over ground).
ExternalRadarTrack ImmTracker::toOutput(const Track & t) const
{
  ExternalRadarTrack o;
  const auto & x = t.filter->x();
  o.id = t.id;
  o.full_state = true;
  o.px = x[0];
  o.py = x[1];
  o.pz = t.z;
  o.vx = x[2];
  o.vy = x[3];
  o.yaw_rate = x[4];
  o.P = t.filter->P().topLeftCorner<4, 4>();
  o.mode_probs = t.filter->modeProbs();
  o.existence = t.existence;
  o.hits = t.hits;
  o.age = t.age;
  o.confirmed = t.confirmed;
  o.heading = t.heading;
  o.radar_updated = t.radar_updated;
  o.moving = t.moving;
  // turning: moving with the coordinated-turn model dominant and a significant turn rate
  const auto & mu = o.mode_probs;
  const double p_ct = idx_ct_ >= 0 && idx_ct_ < static_cast<int>(mu.size()) ? mu[idx_ct_] : 0.0;
  o.turning = t.moving && !mu.empty() && p_ct >= *std::max_element(mu.begin(), mu.end()) &&
    std::abs(o.yaw_rate) > cfg_.tracker.turn_rate_threshold;
  o.ghost = t.ghost;
  o.ground = t.ground;
  return o;
}

}  // namespace sensor_fusion
