/**
 * @file preprocessors.hpp
 * @brief Sensor preprocessors: raw sensor data -> generic tracker Measurements.
 *
 * To add a sensor, implement a new preprocessor producing Measurement objects;
 * the tracker itself is sensor agnostic.
 */
#pragma once

#include <vector>

#include "sensor_fusion/ego_motion.hpp"
#include "sensor_fusion/types.hpp"

namespace sensor_fusion
{

/// Radar filter, ground removal, DBSCAN and noise parameters (radar.* in sensor_fusion.yaml).
struct RadarPreprocessorConfig
{
  double min_range{1.0};
  double max_range{80.0};
  double min_snr{0.0};
  double min_rcs{-100.0};
  double max_abs_azimuth{1.0};    // rad
  double z_min{-3.0}, z_max{4.0}; // hard limits in the radar frame
  // Ground / overhead removal. Radar elevation is noisy, so every height test allows
  // range * elevation_std of slack. Heights are above the ground plane.
  bool ground_filter{true};
  double radar_height{0.5};       // radar above ground [m]
  double elevation_std{0.05};     // rad
  double below_ground_margin{0.15}; // drop returns clearly below the ground (multipath, road)
  double ground_band{0.25};       // static + weak returns lower than this are ground clutter
  double ground_max_rcs{0.0};     // dBsm
  double static_speed{0.3};       // |over-ground Doppler| below this = static [m/s]
  double overhead_height{3.5};    // drop returns above this (trees, signs, ceilings)
  // DBSCAN: neighbours within eps_xy + eps_rel * range and eps_doppler (over-ground Doppler)
  double cluster_eps_xy{1.0};     // m
  double cluster_eps_rel{0.02};   // azimuth spread grows with range
  double cluster_eps_doppler{1.5};// m/s
  int cluster_min_points{1};      // core point: >= this many points in its neighbourhood (incl. itself);
                                  // points not reachable from a core point are noise and dropped
  // measurement noise
  double range_std{0.25};
  double azimuth_std{0.02};       // rad
  double doppler_std{0.3};
  double glint_std{0.4};          // [m] reflection point wander on extended objects (isotropic)
  bool use_doppler{true};
};

/// Radar scan -> filtered, ego-compensated, clustered measurements.
class RadarPreprocessor
{
public:
  /// @param[in] c  radar configuration
  explicit RadarPreprocessor(const RadarPreprocessorConfig & c) : c_(c) {}
  /**
   * @brief Filters, ego-compensates Doppler and clusters one radar scan.
   * @param[in] dets   raw targets of the scan
   * @param[in] stamp  scan time [s]
   * @param[in] ego    ego motion (may be null: Doppler is then not compensated)
   * @return one measurement per DBSCAN cluster (noise points dropped)
   */
  std::vector<Measurement> process(
    const std::vector<RadarDetection> & dets, Time stamp, const EgoMotion * ego) const;

  /**
   * @brief Over-ground radial speed of a single detection.
   * @param[in] d         detection (azimuth, elevation, radial speed)
   * @param[in] v_sensor  sensor over-ground velocity in the sensor frame [m/s]
   * @return vr + v_sensor . u (u = unit line of sight) [m/s]
   */
  static double groundDoppler(const RadarDetection & d, const Eigen::Vector2d & v_sensor);

  /**
   * @brief Ground / overhead test of one detection.
   * @param[in] d   detection
   * @param[in] vg  over-ground Doppler of the detection [m/s]
   * @return true if the detection is road clutter, below ground (multipath) or overhead
   */
  bool isGroundOrOverhead(const RadarDetection & d, double vg) const;

  /**
   * @brief Estimates the sensor forward speed from the Doppler of static targets
   *        (RANSAC on vr = -v * cos(az)). Used for ego speed only, not for ground removal.
   * @param[in]  dets              raw targets
   * @param[out] speed             estimated forward speed [m/s]
   * @param[in]  inlier_tol        inlier tolerance on vr [m/s]
   * @param[in]  min_inlier_ratio  minimum inlier share
   * @param[in]  min_inliers       minimum inlier count
   * @return false if there is no consistent majority
   */
  static bool estimateSensorSpeed(
    const std::vector<RadarDetection> & dets, double & speed, double inlier_tol = 0.15,
    double min_inlier_ratio = 0.4, int min_inliers = 4);

private:
  RadarPreprocessorConfig c_;
};

}  // namespace sensor_fusion
