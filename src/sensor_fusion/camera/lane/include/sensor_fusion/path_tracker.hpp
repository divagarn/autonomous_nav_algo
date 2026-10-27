/**
 * @file path_tracker.hpp
 * @brief Driving path of a fixed length from the vehicle, kept as tracked ground points and corrected
 *        gently by the road edges of each segmentation mask. Pure C++ (no ROS).
 *
 * State: `points` path points every `spacing` metres along the path, starting at the vehicle reference
 * point (base_link) in the vehicle heading, stored in the working frame (radar frame) and moved with
 * the ego motion every update, so they stay fixed on the ground between masks.
 *
 * Per update:
 *   1. predict  : old points moved with the ego motion; the part the vehicle has passed is dropped, the
 *                 path is resampled from the vehicle and extended at the far end with its last curvature
 *                 (with no old path: the ego curvature, e.g. steering), fading to straight (extend_decay).
 *   2. measure  : target of each point = `offset` from the reference road edge (the nearer edge; kept
 *                 until it is lost), at least lane_half_width from both edges. `offset` starts at the
 *                 vehicle's distance to that edge and follows it only slowly (offset_rate), so the path
 *                 runs parallel to the road from where the vehicle drives.
 *                 Only edge parts running along the vehicle (within max_assoc_deg) count, not a curb
 *                 across a junction ahead.
 *   3. correct  : each point moves along its normal by gain * error, at most max_step per update; errors
 *                 above `gate` are ignored unless they persist (mostly gated for gate_updates updates):
 *                 then the gate stays open until all errors are below gate / 2; still far off after
 *                 gate_updates more updates, or edges seen but none usable for 2 x gate_updates updates,
 *                 the path restarts. A new path snaps onto the edges.
 *   4. shape    : smoothed, then rebuilt from the vehicle with the heading change per step limited to
 *                 the buggy's turning (curvature <= tan(max_steer) / wheelbase) and the heading within
 *                 max_heading_deg of the vehicle heading.
 * Without road edges for hold_distance of driving the path restarts from the ego curvature.
 * The lane (corridor) is the path +- lane_half_width = vehicle_width / 2 + lane_margin.
 */
#pragma once

#include <vector>

#include <Eigen/Dense>

#include "sensor_fusion/lane_estimator.hpp"

namespace sensor_fusion
{

/// Path tracker parameters (lane.path.* in sensor_fusion.yaml).
struct PathTrackerConfig
{
  int points{21};               // path points incl. the vehicle point (21 = 0..20 m)
  double spacing{1.0};          // [m] between the points along the path
  double wheelbase{1.6};        // [m]
  double max_steer_deg{20.0};   // [deg] road-wheel steering limit -> curvature limit
  double vehicle_width{1.35};   // [m]
  double lane_margin{0.5};      // [m] each side: lane half width = vehicle_width / 2 + lane_margin
  double gain{0.2};             // fraction of the error corrected per update (10 Hz: ~0.5 s)
  double max_step{0.15};        // [m] largest correction of a point per update
  double gate{1.0};             // [m] larger errors ignored ...
  int gate_updates{5};          // ... unless they persist for this many updates
  double offset_rate{0.05};     // fraction per update by which the edge offset follows the vehicle
  double hold_distance{15.0};   // [m] driven without road edges before the path restarts
  double smoothing{0.5};        // weight of the neighbour mean per smoothing pass (2 passes)
  double max_heading_deg{60.0}; // [deg] path heading relative to the vehicle heading (no hooks)
  double extend_decay{0.8};     // extension beyond the known path: curvature x this per metre (-> straight)
  double max_assoc_deg{50.0};   // [deg] an edge corrects a point only if it runs within this of the vehicle heading
};

/// Path and lane of one update, working frame.
struct TrackedPath
{
  bool valid{false};
  std::vector<Eigen::Vector2d> centre;        // from the vehicle point outwards, every spacing
  std::vector<Eigen::Vector2d> left, right;   // lane borders: centre +- lane half width
  int measured{0};                            // points corrected by road edges in this update
  double unmeasured_m{0};                     // [m] driven since the last road-edge correction
  int reference{-1};                          // reference edge: 0 left, 1 right, -1 none
  double offset{-1};                          // [m] distance of the path from the reference edge
};

/// Tracked driving path from the road edges.
class PathTracker
{
public:
  /// @param[in] cfg  configuration
  explicit PathTracker(const PathTrackerConfig & cfg = {}) : cfg_(cfg) {}

  /**
   * @brief One update with the road edges of a mask.
   * @param[in] lane       road edges (working frame at this update; invalid edges are fine)
   * @param[in] start      vehicle reference point (base_link) in the working frame
   * @param[in] start_yaw  vehicle heading in the working frame [rad]
   * @param[in] R, t       working frame at the previous update -> now (p_now = R p_prev + t)
   * @param[in] ego_curvature  current path curvature of the vehicle [1/m] (+ left), for a new path
   * @return path in the working frame
   */
  const TrackedPath & update(const LaneEstimate & lane, const Eigen::Vector2d & start, double start_yaw,
    const Eigen::Matrix2d & R, const Eigen::Vector2d & t, double ego_curvature);
  /// @brief Last path.
  const TrackedPath & path() const {return out_;}
  /// @brief Lane half width [m].
  double laneHalfWidth() const {return 0.5 * cfg_.vehicle_width + cfg_.lane_margin;}
  /// @brief Forgets the path.
  void reset() {pts_.clear(); out_ = TrackedPath{}; offset_ = -1; reference_ = -1; gated_ = 0; lost_ = 0; unmeasured_ = 0;}

private:
  /**
   * @brief Path from the vehicle: old points from the vehicle's projection on, resampled every spacing,
   *        extended with the last curvature.
   */
  std::vector<Eigen::Vector2d> predict(const Eigen::Vector2d & start, double start_yaw, double ego_curvature) const;
  /**
   * @brief Moves the points towards the road-edge targets.
   * @param[in] snap  a new path: points set onto their targets (no gain, step limit or gate)
   * @return number of points corrected
   */
  int correct(std::vector<Eigen::Vector2d> & p, const LaneEstimate & lane, const Eigen::Vector2d & start,
    double start_yaw, bool snap);
  /// @brief Smoothing + rebuild from the vehicle with the curvature limit.
  void shape(std::vector<Eigen::Vector2d> & p, const Eigen::Vector2d & start, double start_yaw) const;

  PathTrackerConfig cfg_;
  std::vector<Eigen::Vector2d> pts_;   // tracked path, working frame of the last update
  double offset_{-1};                  // distance of the path from the reference edge [m]
  int reference_{-1};                  // 0 left, 1 right
  int gated_{0};                       // successive updates mostly gated (>= gate_updates: gate open)
  int lost_{0};                        // successive updates with edges but no point corrected
  double unmeasured_{0};               // [m] driven since the last correction
  Eigen::Vector2d last_start_{Eigen::Vector2d::Zero()};
  TrackedPath out_;
};

}  // namespace sensor_fusion
