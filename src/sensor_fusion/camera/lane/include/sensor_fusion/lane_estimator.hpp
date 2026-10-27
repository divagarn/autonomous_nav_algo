/**
 * @file lane_estimator.hpp
 * @brief Lane / road boundaries on the ground from a camera segmentation mask (the driving path from
 *        them: path_tracker.hpp).
 *        Pure C++ (no ROS).
 *
 * Per mask, every row_step-th row between min_range and max_range ahead is searched from a seed
 * column outwards: the drivable run (road or marking) around the seed, crossing non-road gaps up to
 * max_gap_px and objects on the road up to max_occluder_px, ends at the road edges (pixels in detector
 * boxes of objects on the road, e.g. people, are unknown: an end behind one is a hidden edge, so a
 * person never becomes an edge); an end at the
 * image border is no edge (the road leaves the image). The seed starts on the vehicle centre line
 * (radar frame y = 0) in the nearest row and follows the middle of the run upwards, so the search
 * follows the road through curves. The nearest lane-marking run on each side of the seed is the lane
 * boundary. Pixels are intersected with the ground plane (flat ground, camera calibration).
 *
 * Each boundary keeps the points of the last keep_s (moved with the ego motion) and is fitted in
 * its own direction (principal axis of the points): v = c0 + c1 s + c2 s^2, robust, so curves that
 * turn sideways stay one curve; a fitted curve ends where it turns more than max_edge_angle away from
 * the forward direction.
 */
#pragma once

#include <cstdint>
#include <limits>
#include <vector>

#include <Eigen/Dense>

#include "fusion_common/camera_model.hpp"
#include "sensor_fusion/types.hpp"

namespace sensor_fusion
{

/// Lane estimator parameters (lane.* in sensor_fusion.yaml).
struct LaneEstimatorConfig
{
  double min_range{3.0};      // [m] ahead of the radar; nearer rows (hood, bumper) are skipped
  double max_range{25.0};     // [m] flat-ground error grows with range
  int row_step{4};            // every n-th mask row
  double max_lateral{10.0};   // [m] boundary points further sideways are dropped
  int min_run_px{2};          // lane-marking runs shorter than this (mask px) are noise
  int max_gap_px{6};          // road edge search crosses non-road gaps up to this (mask px)
  int max_occluder_px{48};    // ... and longer ones with road behind within this (objects on the road)
  double max_edge_jump{1.5};  // [m] edge points of successive rows further apart split the boundary
  double max_edge_angle{60.0};  // [deg] edge parts running further sideways (road ending / turning away) dropped
  double keep_s{0.6};         // [s] points of older masks kept (moved with the ego motion)
  int min_points{6};          // per boundary for a fit
  double min_span{4.0};       // [m] extent of the points along the boundary for a fit
  double quadratic_span{5.0};   // [m] shorter extent: straight line (c2 = 0)
  double outlier{0.5};        // [m] residual rejection of the robust fit
  double step{1.0};           // [m] spacing of the output points
  double min_width{2.0};      // [m] road width range accepted for pairing the edges
  double max_width{15.0};
};

/// One boundary or path on the ground, radar frame.
struct LaneBoundary
{
  bool valid{false};
  int support{0};                       // points used by the fit
  std::vector<Eigen::Vector2d> points;  // every step along the curve, from the vehicle outwards
  // fitted curve p(s) = origin + s dir + (c0 + c1 s + c2 s^2) normal, normal = dir rotated +90 deg;
  // points cover s_min..s_max
  Eigen::Vector2d origin{Eigen::Vector2d::Zero()}, dir{Eigen::Vector2d::UnitX()};
  Eigen::Vector3d coef{Eigen::Vector3d::Zero()};
  double s_min{0}, s_max{0};
  /// @brief Point of the fitted curve at s (also outside s_min..s_max: extrapolated).
  Eigen::Vector2d curveAt(double s) const
  {
    return origin + s * dir + (coef(0) + (coef(1) + coef(2) * s) * s) * Eigen::Vector2d(-dir.y(), dir.x());
  }
  /// @brief Unit tangent of the fitted curve at s.
  Eigen::Vector2d tangentAt(double s) const
  {
    return (dir + (coef(1) + 2 * coef(2) * s) * Eigen::Vector2d(-dir.y(), dir.x())).normalized();
  }
  /**
   * @brief Lateral offset y at x, interpolated between the points (first crossing from the vehicle).
   * @param[in] x  distance ahead [m]
   * @return y [m]; NaN if the curve does not reach x
   */
  double at(double x) const
  {
    for (std::size_t i = 1; i < points.size(); ++i) {
      const auto & a = points[i - 1];
      const auto & b = points[i];
      if ((a.x() - x) * (b.x() - x) > 0 || a.x() == b.x()) {continue;}
      return a.y() + (b.y() - a.y()) * (x - a.x()) / (b.x() - a.x());
    }
    return std::numeric_limits<double>::quiet_NaN();
  }
};

/// Boundaries of one mask.
struct LaneEstimate
{
  Time stamp{-1};                     // mask stamp (frame of the points)
  LaneBoundary lane_left, lane_right; // lane markings left (y > 0) / right of the vehicle
  LaneBoundary road_left, road_right; // edges of the road surface around the vehicle
  double road_width{-1};              // [m] median width of this mask (both edges seen), -1 unknown
};

/// Road edges and lane markings from successive segmentation masks.
class LaneEstimator
{
public:
  /// @param[in] cfg  configuration
  explicit LaneEstimator(const LaneEstimatorConfig & cfg = {}) : cfg_(cfg) {}

  /// Mask class values.
  static constexpr uint8_t kBackground = 0, kLane = 1, kRoad = 2;

  /**
   * @brief Adds one mask and refits the boundaries.
   * @param[in] stamp     mask (image) stamp [s]
   * @param[in] mask      class per pixel, row-major, mask_w x mask_h (0 background, 1 lane, 2 road)
   * @param[in] mask_w, mask_h    mask size [px]
   * @param[in] image_w, image_h  size of the image the mask belongs to [px]
   * @param[in] cam       camera model of that image (fusion_common::cameraForImage)
   * @param[in] ground_z  ground height in the radar frame [m] (= -radar height above ground)
   * @param[in] R, t      radar frame at the previous update -> radar frame at stamp (ego motion:
   *                      p_now = R p_prev + t); identity / zero if unknown
   * @param[in] occluders boxes (x_min, y_min, x_max, y_max) [image px] of objects on the road in the
   *                      same image (people, two-wheelers): their pixels neither extend nor end the road
   * @return boundaries in the radar frame at stamp
   */
  const LaneEstimate & update(Time stamp, const uint8_t * mask, int mask_w, int mask_h, int image_w, int image_h,
    const fusion_common::CameraModel & cam, double ground_z, const Eigen::Matrix2d & R, const Eigen::Vector2d & t,
    const std::vector<Eigen::Vector4d> & occluders = {});
  /// @brief Last estimate.
  const LaneEstimate & estimate() const {return est_;}
  /// @brief Mask rows searched in the last update, with the run ends found (for display).
  struct Row
  {
    int v{0};                 // mask row
    int left{-1}, right{-1};  // road run ends [mask px]; -1: at the image border (no edge)
  };
  /// @brief Rows of the last update (display / debugging).
  const std::vector<Row> & rows() const {return rows_;}
  /// @brief Forgets all points (e.g. bag restart).
  void reset();

private:
  /// Ground point with its mask stamp.
  struct Point
  {
    Eigen::Vector2d p;
    Time t;
  };
  /**
   * @brief Robust polynomial fit of one boundary along its principal direction.
   * @param[in] pts  points of the boundary
   * @return boundary (valid if enough points over enough extent)
   */
  LaneBoundary fit(const std::vector<Point> & pts) const;
  /**
   * @brief Ends a fitted curve where it turns more than max_edge_angle away from the forward direction.
   * @param[in,out] b  boundary; invalid if less than min_span remains
   */
  void truncateSideways(LaneBoundary & b) const;
  /// @brief Median distance between the fitted road edges (est_.road_width, -1 if not both seen).
  void roadWidth();

  LaneEstimatorConfig cfg_;
  std::vector<Point> pts_[4];   // lane left, lane right, road left, road right
  std::vector<Row> rows_;
  LaneEstimate est_;
};

}  // namespace sensor_fusion
