/**
 * @file imm_tracker.hpp
 * @brief IMM radar tracker (radar.tracker imm). Pure C++ (no ROS).
 *
 * Radar scan: ego speed (radar Doppler or odometry) -> ground filter -> DBSCAN -> IMM tracks
 * (stationary / CV / coordinated turn) with the EKF tracker's gating, mirror-ghost and ground
 * mechanisms. Output: every track with its full state and bookkeeping (motion state, ghost,
 * ground), the input of FusionCore::processRadarTracks.
 */
#pragma once

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "sensor_fusion/ego_motion.hpp"
#include "sensor_fusion/imm_filter.hpp"
#include "sensor_fusion/preprocessors.hpp"

namespace sensor_fusion
{

/// Radar tracker parameters (imm.* and tracker.* in sensor_fusion.yaml).
struct TrackerConfig
{
  // IMM: st = stationary, cv = constant velocity, ct = coordinated turn
  std::vector<std::string> model_types{"st", "cv", "ct"};
  double pos_std_st{0.1};          // stationary position random walk [m/sqrt(s)]
  double accel_std_cv{1.5};
  double accel_std_ct{1.0};
  double yaw_accel_std{0.5};
  std::vector<double> transition{   // row-major MxM
    0.95, 0.04, 0.01,
    0.03, 0.92, 0.05,
    0.01, 0.07, 0.92};
  std::vector<double> initial_mode_probs{0.5, 0.4, 0.1};
  // motion state reported per track: stationary | moving | turning
  double stationary_prob{0.6};     // P(st) above this -> stationary
  double moving_min_sigma{2.0};    // moving only if speed > this many std of the speed estimate
  // motion state temporal filter: EMA of the per-scan "moving" evidence + hysteresis
  double motion_alpha{0.3};
  double motion_enter{0.7};        // stationary -> moving above this
  double motion_exit{0.3};         // moving -> stationary below this
  // existence probability: hit pe += hit_gain (1 - pe), miss pe *= miss_factor
  double existence_init{0.3};
  double existence_hit_gain{0.3};
  double existence_miss_factor{0.7};
  // double-bounce ghost: a track at ~2x the range and ~2x the relative radial speed of a
  // closer track on the same bearing (radar -> object -> ego -> object -> radar)
  bool ghost_check{true};
  double ghost_az_tol_deg{3.0};
  double ghost_range_tol{0.1};     // relative tolerance on r_ghost = 2 r_real
  double ghost_doppler_tol{1.0};   // [m/s] on vr_ghost = 2 vr_real
  int ghost_min_count{3};          // consecutive scans before a track is flagged
  double turn_rate_threshold{0.1}; // |omega| [rad/s] with P(ct) dominant -> turning
  // init
  double init_vel_std_radial{0.7};
  double init_vel_std_tangential{4.0};
  double init_yaw_rate_std{0.3};
  // association
  double gate_chi2{13.8};          // 99.9% for 2 dof
  double absorb_gate{9.0};         // unmatched cluster this close to an updated track: no new track
  // lifecycle (radar scans)
  int confirm_hits{3};
  int confirm_window{5};
  int tentative_max_missed{1};
  int max_missed{5};
  double max_pos_std{4.0};         // delete if position std exceeds this
  double max_range{80.0};
  double moving_speed_threshold{0.6};
  double merge_distance{0.8};      // merge confirmed tracks closer than this
  // --- mechanisms ported from the EKF tracker (radar_tracker/ekf, unchanged there) ---
  // association gate: "chi2" = Mahalanobis on position + Doppler below gate_chi2 |
  // "ekf" = AdaptiveGating: ellipse along the track's relative motion, size by motion class
  // (static 4, slow 9, fast 25/16) + object length/width, variance floor 0.2, and radial
  // consistency |predicted - measured relative radial speed| <= ekf_radial_gate
  std::string gating{"ekf"};
  double ekf_radial_gate{3.0};
  // MirrorDetector: N-fold range/velocity ghosts of the nearest stationary reflector
  bool mirror_check{true};
  double mirror_stationary_speed{1.5};   // reflector: over-ground speed below this [m/s]
  int mirror_min_hits{5};
  double mirror_lane_half_width{4.5};    // reflector |y| [m]
  std::vector<int64_t> mirror_fold_orders{2, 3, 4, 5, 6};
  double mirror_range_tol{2.5};          // |r_ghost - N r_ref| [m]
  double mirror_lateral_tol{2.0};        // |y_ghost - y_ref| [m]
  double mirror_speed_tol{1.0};          // ||vr_ghost| - N |vr_ref|| [m/s], relative radial speeds
  // not in the EKF tracker: flag only when the Doppler tells the ghost from a real object (||vr_ghost| - |vr_ref||
  // > speed_tol). At buggy speeds N |vr_ref| is within speed_tol of |vr_ref| and every real object at
  // ~N x the reflector range in its lane would be dropped (same rule as the double-bounce check).
  bool mirror_require_doppler_separation{true};
  int mirror_forget_frames{8};
  double mirror_ego_stationary{0.3};     // below this ego speed: freeze the flags [m/s]
  // ground / overdrivable (from the EKF tracker, both radar.tracker modes): a STATIONARY track
  // whose height above ground stays < ground_z_threshold and mean-RCS EMA < ground_rcs_max
  // is kept out of the object list (still tracked). The EKF tracker uses 0.12 m / -5 dBsm; on
  // campus the false stops come from strong flat metal (drain covers, gratings), so the RCS
  // condition is off by default (ground_rcs_max 100) and the height is 0.30 m (people: >= 0.4 m)
  bool ground_check{true};
  double ground_z_threshold{0.30};
  double ground_rcs_max{100.0};
  // the height is a low quantile of the last ground_window detections (an EMA follows single high / low
  // returns of a flat reflector such as a speed breaker over the threshold); a stationary track
  // counts as ground until ground_min_samples heights are known
  int ground_window{15};
  int ground_min_samples{8};
  double ground_quantile{0.5};          // quantile of the heights compared with ground_z_threshold
};

/// Complete IMM tracker configuration.
struct ImmTrackerConfig
{
  TrackerConfig tracker;
  RadarPreprocessorConfig radar;
  // ego speed source: "odom" (vehicle odometry) or "radar_doppler" (estimated
  // from static radar targets; yaw rate still from odometry)
  std::string ego_speed_source{"radar_doppler"};
  MountPose2D mount;                // radar pose in vehicle base frame (ego motion)
};

/// Clustering + IMM radar tracker.
class ImmTracker
{
public:
  /**
   * @param[in] cfg  tracker configuration
   * @throws std::invalid_argument if the IMM set-up is inconsistent
   */
  explicit ImmTracker(const ImmTrackerConfig & cfg);

  /**
   * @brief Adds a vehicle odometry sample (ego motion).
   * @param[in] s  speed / yaw-rate sample
   */
  void addEgo(const EgoSample & s);
  /**
   * @brief Radar cycle: ego speed, preprocessing, prediction, association, update, track management.
   *
   * One cycle per scan. Scans must not go back in time (older scans are dropped, a jump back of
   * more than 1 s restarts the tracker).
   * @param[in]  stamp      scan time [s]
   * @param[in]  dets       raw targets of the scan
   * @param[out] ego_speed  if not null, ego speed at the scan [m/s] (the Doppler estimate in
   *                        radar_doppler mode: FusionCore::processRadarTracks takes it)
   * @return all tracks (tentative ones with confirmed = false), radar frame, over-ground velocity
   */
  std::vector<ExternalRadarTrack> process(Time stamp, const std::vector<RadarDetection> & dets,
    double * ego_speed = nullptr);
  /// @brief Clusters of the last radar cycle.
  const std::vector<Measurement> & lastMeasurements() const {return last_meas_;}
  /// @brief Deletes all tracks.
  void reset();

private:
  /// Internal radar track.
  struct Track
  {
    uint32_t id;
    std::unique_ptr<ImmFilter> filter;
    int hits{0};
    int age{0};
    int missed{0};                       // consecutive radar scans without an update
    std::deque<bool> hit_history;
    bool confirmed{false};
    double z{0};
    double heading{0};
    bool radar_updated{false};
    double existence{0};
    double motion_score{0};              // EMA of the moving evidence
    bool moving{false};
    int ghost_count{0};
    bool ghost{false};
    // EKF-tracker mechanisms
    double length{0.5}, width{0.5};      // smoothed object extent (gating ekf)
    double z_ema{0}, rcs_ema{0}, rcs_var{0};   // height above ground / mean RCS EMAs (ground_check)
    std::deque<double> z_hist;           // heights above ground of the last detections (ground_check)
    bool ground{false};
    bool mirror{false};                  // MirrorDetector ghost
    uint32_t mirror_source{0};
    int mirror_order{0}, mirror_miss{0};
  };

  /**
   * @brief IMM radar cycle: predict, GNN association, update, births, management,
   *        motion state filter and ghost check.
   * @param[in] stamp   measurement time [s]
   * @param[in] meas    radar clusters
   * @return number of measurements
   */
  std::size_t runCycle(Time stamp, std::vector<Measurement> meas);
  /**
   * @brief Predicts all filters to stamp and compensates the ego motion.
   * @param[in] stamp  target time [s]
   */
  void predictTo(Time stamp);
  /**
   * @brief New filter from an unassociated cluster (velocity from Doppler along the line of sight).
   * @param[in] m  measurement
   * @return filter
   */
  std::unique_ptr<ImmFilter> initFilter(const Measurement & m) const;
  /// @brief M/N confirmation, heading, deletion and merging of duplicate tracks.
  void manageTracks();
  /**
   * @brief Per-scan moving evidence: speed above threshold, significant and not stationary-model.
   * @param[in] t  track
   */
  bool movingEvidence(const Track & t) const;
  /**
   * @brief Flags double-bounce multipath ghosts (2x range, 2x radial speed on the bearing of a
   *        closer track) after ghost_min_count consecutive scans.
   * @param[in] stamp  scan time [s]
   */
  void checkGhosts(Time stamp);
  /**
   * @brief the EKF tracker's AdaptiveGating for one track / cluster pair.
   * @param[in]  t, m   predicted track and cluster
   * @param[in]  vs     sensor velocity over ground (relative velocity = v - vs)
   * @param[out] score  S = d_long^2/(var_long G_long) + d_lat^2/(var_lat G_lat), lower = better
   * @return true if inside the gate (S <= 1 and radial speeds consistent)
   */
  bool ekfGate(const Track & t, const Measurement & m, const Eigen::Vector2d & vs, double & score) const;
  /// @brief the EKF tracker's MirrorDetector on the track list (sets mirror flags).
  void checkMirrors(Time stamp);
  /// @brief the EKF tracker's size / height / RCS bookkeeping of a track hit by cluster m (first = birth).
  void updateEkfStats(Track & t, const Measurement & m, bool first) const;
  /**
   * @brief Track -> tracker output (full state, motion state, ghost and ground flags).
   * @param[in] t  track
   */
  ExternalRadarTrack toOutput(const Track & t) const;

  ImmTrackerConfig cfg_;
  ImmConfig imm_cfg_;
  RadarPreprocessor radar_pre_;
  EgoMotion ego_;
  std::vector<Track> tracks_;
  int idx_st_{-1}, idx_ct_{-1};
  std::vector<Measurement> last_meas_;
  uint32_t next_id_{1};
  Time last_stamp_{-1};                  // time of the filter states (last radar scan)
  double last_yaw_rate_{0};
};

}  // namespace sensor_fusion
