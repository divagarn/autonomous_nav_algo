/**
 * @file types.hpp
 * @brief Plain data types of the fusion core (no ROS dependency).
 */
#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cstdint>
#include <deque>
#include <map>
#include <string>
#include <vector>

namespace sensor_fusion
{

using Time = double;  ///< time stamp [s]

/// Source sensor of a measurement.
enum class SensorType : uint8_t { RADAR = 0, CAMERA = 2 };

/// One raw radar target (radar frame: x forward, y left, z up).
struct RadarDetection
{
  double x{0}, y{0}, z{0};
  double radial_speed{0};   // relative radial speed as reported by the sensor [m/s]
  double rcs{0}, snr{0}, power{0};
  double range{0}, azimuth{0}, elevation{0};
};

/// Track of a radar tracker (EKF tracker, IMM tracker, a platform's own): input of FusionCore::processRadarTracks.
struct ExternalRadarTrack
{
  uint32_t id{0};
  double px{0}, py{0}, pz{0};   // radar frame [m]
  double vx{0}, vy{0};          // relative to the sensor [m/s] (EKF tracker), over ground if full_state
  Eigen::Matrix4d P{Eigen::Matrix4d::Identity()};   // [px, py, vx, vy]
  double existence{0};          // probability of existence
  int hits{0}, age{0};
  bool dynamic{false};          // EkfTracker is_dynamic: |relative velocity| > 0.5 m/s
  bool mirror_ghost{false};     // MirrorDetector
  bool ground_reflection{false};
  // full_state: the tracker keeps the fusion's bookkeeping itself (IMM tracker): over-ground
  // velocity and the fields below are taken as they are. Otherwise (EKF tracker) the fusion
  // derives motion state and ground flag and lists every track.
  bool full_state{false};
  bool confirmed{true};
  double yaw_rate{0};           // [rad/s]
  std::vector<double> mode_probs;
  double heading{0};            // [rad]
  bool moving{false}, turning{false};
  bool ghost{false};            // double bounce / mirror ghost (never in the final list)
  bool ground{false};           // ground level, overdrivable
  bool radar_updated{true};     // measured in this scan
};

/// Position measurement fed to the IMM tracker (one radar cluster): position + over-ground Doppler.
struct Measurement
{
  SensorType sensor{SensorType::RADAR};
  Time stamp{0};
  Eigen::Vector2d pos{Eigen::Vector2d::Zero()};      // x, y in fusion frame
  Eigen::Matrix2d pos_cov{Eigen::Matrix2d::Identity()};
  bool has_doppler{false};
  double doppler{0};          // over-ground radial speed (ego compensated) [m/s]
  double doppler_var{0.25};
  double z{0};                // mean height of the cluster (informative only)
  double extent{0};           // cluster extent [m]
  double size_x{0}, size_y{0};  // cluster extent along x / y [m] (gating ekf)
  int num_points{1};
  double rcs{0};              // max RCS of the points [dBsm]
  double rcs_mean{0};         // mean RCS of the points [dBsm] (ground_check)
};

/// 2D object from the camera detector / tracker (pixels of the detector image).
struct CameraDetection
{
  uint32_t track_id{0};        // 0 from the detector; set by the 2D tracker
  int class_id{-1};
  std::string label;          // detector label (e.g. COCO "person")
  float score{0};
  float x_min{0}, y_min{0}, x_max{0}, y_max{0};
};

/// All detections of one camera image.
struct CameraFrame
{
  Time stamp{0};
  int width{0}, height{0};
  std::vector<CameraDetection> detections;
};

/// Vehicle motion sample (from /odom or wheel speed + steering).
struct EgoSample
{
  Time stamp{0};
  double speed{0};      // [m/s] forward
  double yaw_rate{0};   // [rad/s]
};

/// Output object of the fusion (fusion frame = radar frame).
struct FusedTrack
{
  uint32_t id{0};
  std::string tracker{"imm"};          // radar tracking: "ekf" (EKF tracker tracks) | "imm"
  int64_t ekf_track_id{-1};            // EKF tracker id (tracker ekf), -1 for imm
  Eigen::Vector2d pos{Eigen::Vector2d::Zero()};
  Eigen::Vector2d vel{Eigen::Vector2d::Zero()};
  Eigen::Matrix2d pos_cov{Eigen::Matrix2d::Identity()};
  double z{0};
  double yaw_rate{0};
  double heading{0};
  std::vector<double> model_probs;
  double existence{0};
  bool confirmed{false};
  bool ghost{false};                   // double-bounce multipath ghost (never in the final list)
  bool is_moving{false};
  std::string motion_state{"stationary"};   // stationary | moving | turning
  std::string relative_motion_state{"unknown"};
  double closing_speed{0};                  // signed LOS closing speed, positive = closing
  double time_to_collision{-1};             // [s], -1 when not closing
  Eigen::Vector2d predicted_pos{Eigen::Vector2d::Zero()};
  double prediction_horizon{0};
  // ego path
  bool in_path{false};
  double path_distance{0}, path_lateral{0};
  bool predicted_in_path{false};
  bool in_stop_zone{false};                 // inside the stop zone now
  double time_to_stop_zone{-1};             // [s] predicted entry into the stop zone, 0 = inside, -1 = not within horizon
  double predicted_path_distance{0}, predicted_path_lateral{0};
  uint32_t age{0}, hits{0};
  bool radar_updated{false}, camera_associated{false};
  uint32_t camera_track_id{0};
  std::string class_name{"unknown"};
  double class_confidence{0};
  std::vector<double> class_probs;     // vote shares, order = fusion_common::fusedClasses()
  // "radar" | "radar+camera" (camera associated within the hold time)
  std::string status{"radar"};
  double assoc_prob{0};                // JPDA probability of the camera association
  double bearing_radar{0}, bearing_fused{0};   // [rad]
  double height{0};                    // mean measured height above ground [m]
  // image plane (detector image): fused object centre, radar-only projection
  bool in_image{false};
  double u{0}, v{0};
  double proj_u{0}, proj_v{0};
  // box built from the fused position and the class dimensions
  bool has_bbox{false};
  double bbox[4]{0, 0, 0, 0};          // x_min, y_min, x_max, y_max
  /// @brief Range from the radar [m]: sqrt(x^2 + y^2).
  double range() const {return pos.norm();}
};

/**
 * @brief Adds a detection height to a track's history (ground test), keeping the last `window`.
 * @param[in,out] hist    heights above ground [m], oldest first
 * @param[in]     h       height of the new detection [m]
 * @param[in]     window  history length
 */
inline void pushGroundHeight(std::deque<double> & hist, double h, int window)
{
  hist.push_back(h);
  while (static_cast<int>(hist.size()) > std::max(1, window)) {hist.pop_front();}
}

/**
 * @brief Ground / overdrivable test of a track (used for stationary tracks only): a low quantile of
 *        the recent detection heights below `threshold`. A flat reflector (speed breaker, drain cover)
 *        has single returns far above and below its true height (~40 % above 0.3 m on bag_record_3);
 *        an EMA follows them over the threshold, a low quantile does not, and people stay far above
 *        it (their returns are 0.2 - 1.0 m). With fewer than `min_samples` heights the track counts
 *        as ground (height not known yet): a new stationary track is listed only after its height has
 *        been measured, which delays a real stationary obstacle by `min_samples` detections.
 * @param[in] hist         recent detection heights above ground [m]
 * @param[in] threshold    ground height threshold [m]
 * @param[in] min_samples  detections needed for a height
 * @param[in] quantile     quantile of the heights compared with the threshold (0.5 = median)
 * @return true if ground / not yet known
 */
inline bool groundByHeight(const std::deque<double> & hist, double threshold, int min_samples, double quantile)
{
  if (static_cast<int>(hist.size()) < std::max(1, min_samples)) {return true;}
  std::vector<double> v(hist.begin(), hist.end());
  // k = floor(q (n - 1)): the q-quantile without interpolation
  const auto k = static_cast<std::size_t>(std::clamp(quantile, 0.0, 1.0) * (v.size() - 1));
  std::nth_element(v.begin(), v.begin() + k, v.end());
  return v[k] < threshold;
}

}  // namespace sensor_fusion
