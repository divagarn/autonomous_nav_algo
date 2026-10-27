/**
 * @file sensor_fusion_node.cpp
 * @brief ROS 2 node: radar tracks (EKF tracker node or the IMM tracker, radar.tracker) fused with camera objects.
 *
 * Radar scans and camera frames each run a cycle of FusionCore on arrival
 * (reorder_delay 0 by default; > 0 holds messages to process them in stamp order).
 *
 * Compile-time stages (CMake options, a stage that is OFF is not in the binary):
 *   SF_WITH_IMM        radar.tracker imm: the IMM tracker runs here on the raw scans
 *   SF_WITH_BYTETRACK  ByteTrack on the detector boxes; without it the camera objects must
 *                      carry track ids already (camera tracker upstream)
 *   SF_WITH_LANE       lane / road boundaries from the camera segmentation (lane.topic)
 */
// Subscribes
//   radar_topic   sensor_msgs/PointCloud2  (radar.tracker imm, tracked here by ImmTracker; smartmicro UMRR: x y z radial_speed power rcs noise snr azimuth_angle elevation_angle range)
//   radar.ekf.tracks_topic perception_msgs/RadarTrackArray (radar.tracker ekf; EKF tracker node)
//   odom_topic    nav_msgs/Odometry        (vehicle speed + yaw rate)
//   steering_topic std_msgs/Float32        (steering angle for the ego path)
//   camera_objects_topic perception_msgs/CameraObjectArray (raw detector boxes, rectified image)
//   lane.topic    perception_msgs/SegmentationMask (lane / road classes of the rectified image, SF_WITH_LANE)
// Publishes after every camera cycle (trigger "camera", 30 Hz); radar cycles only update the
// tracks unless the camera is silent for output.camera_timeout (trigger "radar")
//   ~/objects        perception_msgs/FusedObjectArray   final list: confirmed, < output.max_range,
//                                                       in_path flag, ego path corridor
//   ~/tracks         perception_msgs/FusedObjectArray   all confirmed tracks (debug)
//   ~/camera_tracks  perception_msgs/CameraObjectArray  2D box tracks of ByteTrack (camera cycles)
//   ~/lane_path      nav_msgs/Path  tracked driving path (lane.path.*) in the base frame, every output
//                                   cycle (empty: no path), SF_WITH_LANE
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "fusion_common/camera_model.hpp"
#include "fusion_common/object_classes.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "perception_msgs/msg/camera_object_array.hpp"
#include "perception_msgs/msg/fused_object_array.hpp"
#include "perception_msgs/msg/radar_track_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_fusion/fusion_core.hpp"
#ifdef SF_WITH_BYTETRACK
#include "sensor_fusion/byte_tracker.hpp"
#endif
#ifdef SF_WITH_IMM
#include "sensor_fusion/imm_tracker.hpp"
#endif
#ifdef SF_WITH_LANE
#include "nav_msgs/msg/path.hpp"
#include "perception_msgs/msg/segmentation_mask.hpp"
#include "sensor_fusion/lane_estimator.hpp"
#include "sensor_fusion/path_tracker.hpp"
#endif
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace sensor_fusion
{

namespace
{
/**
 * @brief ROS time to seconds.
 * @param[in] t  ROS time
 * @return sec + 1e-9 nanosec
 */
double toSec(const builtin_interfaces::msg::Time & t) {return t.sec + 1e-9 * t.nanosec;}

/**
 * @brief Checks whether a PointCloud2 has a field.
 * @param[in] m     point cloud
 * @param[in] name  field name
 * @return true if present
 */
bool hasField(const sensor_msgs::msg::PointCloud2 & m, const std::string & name)
{
  for (const auto & f : m.fields) {
    if (f.name == name) {return true;}
  }
  return false;
}
}  // namespace

/// ROS wrapper around FusionCore: parameters, subscriptions, cycle scheduling, publishing.
class SensorFusionNode : public rclcpp::Node
{
public:
  /**
   * @brief Declares all parameters, loads the calibration, creates subscriptions and publishers.
   * @param[in] opt  node options
   */
  explicit SensorFusionNode(const rclcpp::NodeOptions & opt = rclcpp::NodeOptions())
  : Node("sensor_fusion", opt)
  {
    FusionConfig c;
    // --- topics ---
    const auto radar_topic = declare_parameter<std::string>("radar_topic", "/smart_radar/can_targets_0");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/odom");
    const auto cam_topic = declare_parameter<std::string>("camera_objects_topic", "/object_detection/objects");
    const auto steering_topic = declare_parameter<std::string>("steering_topic", "/steering_angle");
    // radar tracking: "ekf" (default) = tracks of the radar_tracker node (the EKF tracker),
    // "imm" = the IMM tracker (clustering + IMM) in this node on the raw radar_topic scans
    radar_tracker_ = declare_parameter<std::string>("radar.tracker", "ekf");
    const auto ekf_topic = declare_parameter<std::string>("radar.ekf.tracks_topic", "/radar_tracker/tracks");
    c.external.drop_mirror_ghosts = declare_parameter<bool>("radar.ekf.drop_mirror_ghosts", c.external.drop_mirror_ghosts);
    c.external.drop_ground_reflections =
      declare_parameter<bool>("radar.ekf.drop_ground_reflections", c.external.drop_ground_reflections);
    c.external.motion_source = declare_parameter<std::string>("radar.ekf.motion_source", c.external.motion_source);
    if (c.external.motion_source != "fusion" && c.external.motion_source != "ekf_flag") {
      throw std::invalid_argument("radar.ekf.motion_source must be 'fusion' or 'ekf_flag'");
    }
    if (radar_tracker_ != "ekf" && radar_tracker_ != "imm") {
      throw std::invalid_argument("radar.tracker must be 'ekf' or 'imm', got '" + radar_tracker_ + "'");
    }
#ifndef SF_WITH_IMM
    if (radar_tracker_ == "imm") {
      throw std::invalid_argument("radar.tracker imm: this build has no IMM tracker (SF_WITH_IMM=OFF)");
    }
#endif
    // one radar height for ground removal, camera projection and path
    const double radar_height = declare_parameter<double>("radar_height_above_ground", 0.5);
    output_frame_ = declare_parameter<std::string>("output_frame", "");   // "" = radar msg frame
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    output_tf_tolerance_ = declare_parameter<double>("output.tf_tolerance", 0.1);
    reorder_delay_ = declare_parameter<double>("reorder_delay", 0.0);   // 0 = process on arrival
    timing_log_s_ = declare_parameter<double>("timing_log_s", 10.0);
    output_trigger_ = declare_parameter<std::string>("output.trigger", "camera");
    camera_timeout_ = declare_parameter<double>("output.camera_timeout", 0.2);
    // --- calibration (calibration app output) ---
    const auto intr_file = declare_parameter<std::string>("camera.intrinsics_file", "");
    const auto extr_file = declare_parameter<std::string>("camera.extrinsics_file", "");
    // --- ego motion ---
    c.ego_speed_source = declare_parameter<std::string>("ego_motion.speed_source", c.ego_speed_source);
    c.fusion_frame_mount.x = declare_parameter<double>("ego_motion.radar_mount_x", 0.0);
    c.fusion_frame_mount.y = declare_parameter<double>("ego_motion.radar_mount_y", 0.0);
    c.fusion_frame_mount.yaw = declare_parameter<double>("ego_motion.radar_mount_yaw", 0.0);
#ifdef SF_WITH_IMM
    ImmTrackerConfig ic;   // radar.tracker imm
    // --- radar preprocessing ---
    auto & r = ic.radar;
    r.min_range = declare_parameter<double>("radar.min_range", r.min_range);
    r.max_range = declare_parameter<double>("radar.max_range", r.max_range);
    r.min_snr = declare_parameter<double>("radar.min_snr", r.min_snr);
    r.min_rcs = declare_parameter<double>("radar.min_rcs", r.min_rcs);
    r.max_abs_azimuth = declare_parameter<double>("radar.max_abs_azimuth", r.max_abs_azimuth);
    r.z_min = declare_parameter<double>("radar.z_min", r.z_min);
    r.z_max = declare_parameter<double>("radar.z_max", r.z_max);
    r.cluster_eps_xy = declare_parameter<double>("radar.cluster_eps_xy", r.cluster_eps_xy);
    r.cluster_eps_rel = declare_parameter<double>("radar.cluster_eps_rel", r.cluster_eps_rel);
    r.cluster_eps_doppler = declare_parameter<double>("radar.cluster_eps_doppler", r.cluster_eps_doppler);
    r.cluster_min_points = declare_parameter<int>("radar.cluster_min_points", r.cluster_min_points);
    r.range_std = declare_parameter<double>("radar.range_std", r.range_std);
    r.azimuth_std = declare_parameter<double>("radar.azimuth_std", r.azimuth_std);
    r.doppler_std = declare_parameter<double>("radar.doppler_std", r.doppler_std);
    r.glint_std = declare_parameter<double>("radar.glint_std", r.glint_std);
    r.use_doppler = declare_parameter<bool>("radar.use_doppler", r.use_doppler);
    r.radar_height = radar_height;
    r.ground_filter = declare_parameter<bool>("radar.ground_filter", r.ground_filter);
    r.elevation_std = declare_parameter<double>("radar.elevation_std", r.elevation_std);
    r.below_ground_margin = declare_parameter<double>("radar.below_ground_margin", r.below_ground_margin);
    r.ground_band = declare_parameter<double>("radar.ground_band", r.ground_band);
    r.ground_max_rcs = declare_parameter<double>("radar.ground_max_rcs", r.ground_max_rcs);
    r.static_speed = declare_parameter<double>("radar.static_speed", r.static_speed);
    r.overhead_height = declare_parameter<double>("radar.overhead_height", r.overhead_height);
    // --- IMM tracker ---
    auto & t = ic.tracker;
    t.model_types = declare_parameter<std::vector<std::string>>("imm.models", t.model_types);
    t.pos_std_st = declare_parameter<double>("imm.pos_std_st", t.pos_std_st);
    t.accel_std_cv = declare_parameter<double>("imm.accel_std_cv", t.accel_std_cv);
    t.accel_std_ct = declare_parameter<double>("imm.accel_std_ct", t.accel_std_ct);
    t.yaw_accel_std = declare_parameter<double>("imm.yaw_accel_std", t.yaw_accel_std);
    t.transition = declare_parameter<std::vector<double>>("imm.transition", t.transition);
    t.initial_mode_probs = declare_parameter<std::vector<double>>("imm.initial_mode_probs", t.initial_mode_probs);
    t.stationary_prob = declare_parameter<double>("imm.stationary_prob", t.stationary_prob);
    t.moving_min_sigma = declare_parameter<double>("imm.moving_min_sigma", t.moving_min_sigma);
    t.motion_alpha = declare_parameter<double>("imm.motion_alpha", t.motion_alpha);
    t.motion_enter = declare_parameter<double>("imm.motion_enter", t.motion_enter);
    t.motion_exit = declare_parameter<double>("imm.motion_exit", t.motion_exit);
    t.existence_init = declare_parameter<double>("tracker.existence_init", t.existence_init);
    t.existence_hit_gain = declare_parameter<double>("tracker.existence_hit_gain", t.existence_hit_gain);
    t.existence_miss_factor = declare_parameter<double>("tracker.existence_miss_factor", t.existence_miss_factor);
    t.ghost_check = declare_parameter<bool>("tracker.ghost_check", t.ghost_check);
    t.ghost_az_tol_deg = declare_parameter<double>("tracker.ghost_az_tol_deg", t.ghost_az_tol_deg);
    t.ghost_range_tol = declare_parameter<double>("tracker.ghost_range_tol", t.ghost_range_tol);
    t.ghost_doppler_tol = declare_parameter<double>("tracker.ghost_doppler_tol", t.ghost_doppler_tol);
    t.ghost_min_count = declare_parameter<int>("tracker.ghost_min_count", t.ghost_min_count);
    t.turn_rate_threshold = declare_parameter<double>("imm.turn_rate_threshold", t.turn_rate_threshold);
    t.init_vel_std_radial = declare_parameter<double>("tracker.init_vel_std_radial", t.init_vel_std_radial);
    t.init_vel_std_tangential = declare_parameter<double>("tracker.init_vel_std_tangential", t.init_vel_std_tangential);
    t.gate_chi2 = declare_parameter<double>("tracker.gate_chi2", t.gate_chi2);
    t.confirm_hits = declare_parameter<int>("tracker.confirm_hits", t.confirm_hits);
    t.confirm_window = declare_parameter<int>("tracker.confirm_window", t.confirm_window);
    t.absorb_gate = declare_parameter<double>("tracker.absorb_gate", t.absorb_gate);
    t.tentative_max_missed = declare_parameter<int>("tracker.tentative_max_missed", t.tentative_max_missed);
    t.max_missed = declare_parameter<int>("tracker.max_missed", t.max_missed);
    t.max_pos_std = declare_parameter<double>("tracker.max_pos_std", t.max_pos_std);
    t.max_range = declare_parameter<double>("tracker.max_range", t.max_range);
    t.moving_speed_threshold = declare_parameter<double>("tracker.moving_speed_threshold", t.moving_speed_threshold);
    t.merge_distance = declare_parameter<double>("tracker.merge_distance", t.merge_distance);
    // mechanisms ported from the EKF tracker (AEB pipeline)
    t.gating = declare_parameter<std::string>("tracker.gating", t.gating);
    if (t.gating != "chi2" && t.gating != "ekf") {
      throw std::invalid_argument("tracker.gating must be 'chi2' or 'ekf'");
    }
    t.ekf_radial_gate = declare_parameter<double>("tracker.ekf_radial_gate", t.ekf_radial_gate);
    t.mirror_check = declare_parameter<bool>("tracker.mirror_check", t.mirror_check);
    t.mirror_stationary_speed = declare_parameter<double>("tracker.mirror.stationary_speed", t.mirror_stationary_speed);
    t.mirror_min_hits = declare_parameter<int>("tracker.mirror.min_hits", t.mirror_min_hits);
    t.mirror_lane_half_width = declare_parameter<double>("tracker.mirror.lane_half_width", t.mirror_lane_half_width);
    t.mirror_fold_orders = declare_parameter<std::vector<int64_t>>("tracker.mirror.fold_orders", t.mirror_fold_orders);
    t.mirror_range_tol = declare_parameter<double>("tracker.mirror.range_tol", t.mirror_range_tol);
    t.mirror_lateral_tol = declare_parameter<double>("tracker.mirror.lateral_tol", t.mirror_lateral_tol);
    t.mirror_speed_tol = declare_parameter<double>("tracker.mirror.speed_tol", t.mirror_speed_tol);
    t.mirror_forget_frames = declare_parameter<int>("tracker.mirror.forget_frames", t.mirror_forget_frames);
    t.mirror_require_doppler_separation =
      declare_parameter<bool>("tracker.mirror.require_doppler_separation", t.mirror_require_doppler_separation);
    t.mirror_ego_stationary = declare_parameter<double>("tracker.mirror.ego_stationary", t.mirror_ego_stationary);
    t.ground_check = declare_parameter<bool>("tracker.ground_check", t.ground_check);
    t.ground_z_threshold = declare_parameter<double>("tracker.ground_z_threshold", t.ground_z_threshold);
    t.ground_rcs_max = declare_parameter<double>("tracker.ground_rcs_max", t.ground_rcs_max);
    t.ground_window = declare_parameter<int>("tracker.ground_window", t.ground_window);
    t.ground_min_samples = declare_parameter<int>("tracker.ground_min_samples", t.ground_min_samples);
    t.ground_quantile = declare_parameter<double>("tracker.ground_quantile", t.ground_quantile);
    ic.ego_speed_source = c.ego_speed_source;
    ic.mount = c.fusion_frame_mount;
    model_names_ = t.model_types;
#endif
    // EKF tracker tracks: the fusion derives motion state and ground flag (the IMM tracker's parameters)
    auto & x = c.external;
    x.moving_speed_threshold = param<double>("tracker.moving_speed_threshold", x.moving_speed_threshold);
    x.moving_min_sigma = param<double>("imm.moving_min_sigma", x.moving_min_sigma);
    x.motion_alpha = param<double>("imm.motion_alpha", x.motion_alpha);
    x.motion_enter = param<double>("imm.motion_enter", x.motion_enter);
    x.motion_exit = param<double>("imm.motion_exit", x.motion_exit);
    x.ground_check = param<bool>("tracker.ground_check", x.ground_check);
    x.ground_z_threshold = param<double>("tracker.ground_z_threshold", x.ground_z_threshold);
    x.ground_window = param<int>("tracker.ground_window", x.ground_window);
    x.ground_min_samples = param<int>("tracker.ground_min_samples", x.ground_min_samples);
    x.ground_quantile = param<double>("tracker.ground_quantile", x.ground_quantile);
    // --- detector score thresholds: YOLOX scores much lower than YOLOv8 on our camera (yolox_scores.*) ---
    detector_model_type_ = declare_parameter<std::string>("detector_model_type", "yolov8");
    // camera_tracker.* / camera.* threshold, or yolox_scores.<name> for a YOLOX detector
    auto score = [this](const std::string & param, const std::string & yolox_name, double def) {
        const double v = declare_parameter<double>(param, def);
        const double y = declare_parameter<double>("yolox_scores." + yolox_name, v);
        return detector_model_type_ == "yolox" ? y : v;
      };
    // --- camera fusion ---
    auto & cc = c.camera;
    cc.enabled = declare_parameter<bool>("camera.enabled", cc.enabled);
    cc.rectified = declare_parameter<bool>("camera.rectified", cc.rectified);
    cc.rectify_balance = declare_parameter<double>("camera.rectify_balance", cc.rectify_balance);
    cc.rectify_fov_scale = declare_parameter<double>("camera.rectify_fov_scale", cc.rectify_fov_scale);
    cc.radar_height_above_ground = radar_height;
    cc.min_detection_score = score("camera.min_detection_score", "min_detection_score", cc.min_detection_score);
    cc.margin_px = declare_parameter<double>("camera.margin_px", cc.margin_px);
    cc.object_height = declare_parameter<double>("camera.object_height", cc.object_height);
    cc.range_ratio_min = declare_parameter<double>("camera.range_ratio_min", cc.range_ratio_min);
    cc.range_ratio_max = declare_parameter<double>("camera.range_ratio_max", cc.range_ratio_max);
    cc.border_px = declare_parameter<double>("camera.border_px", cc.border_px);
    cc.gate_d2 = declare_parameter<double>("camera.gate_d2", cc.gate_d2);
    cc.range_pref = declare_parameter<double>("camera.range_pref", cc.range_pref);
    cc.jpda_max_candidates = declare_parameter<int>("camera.jpda_max_candidates", cc.jpda_max_candidates);
    cc.jpda_pd = declare_parameter<double>("camera.jpda_pd", cc.jpda_pd);
    cc.jpda_clutter = declare_parameter<double>("camera.jpda_clutter", cc.jpda_clutter);
    cc.sigma_px = declare_parameter<double>("camera.sigma_px", cc.sigma_px);
    cc.sigma_frac = declare_parameter<double>("camera.sigma_frac", cc.sigma_frac);
    cc.sigma_extent = declare_parameter<double>("camera.sigma_extent", cc.sigma_extent);
    cc.sigma_radar_min_deg = declare_parameter<double>("camera.sigma_radar_min_deg", cc.sigma_radar_min_deg);
    cc.radar_min_weight = declare_parameter<double>("camera.radar_min_weight", cc.radar_min_weight);
    cc.consistency_sigma = declare_parameter<double>("camera.consistency_sigma", cc.consistency_sigma);
    cc.hold_s = declare_parameter<double>("camera.hold_s", cc.hold_s);
    cc.class_vote_decay = declare_parameter<double>("camera.class_vote_decay", cc.class_vote_decay);
    cc.class_keep_s = declare_parameter<double>("camera.class_keep_s", cc.class_keep_s);
    cc.min_track_height = declare_parameter<double>("camera.min_track_height", cc.min_track_height);
    cc.min_track_height_range = declare_parameter<double>("camera.min_track_height_range", cc.min_track_height_range);
    cc.ignore_classes = declare_parameter<std::vector<std::string>>("camera.ignore_classes", cc.ignore_classes);
#ifdef SF_WITH_LANE
    // --- lane / road boundaries (camera segmentation) ---
    const bool lane_enabled = declare_parameter<bool>("lane.enabled", true);
    const auto lane_topic = declare_parameter<std::string>("lane.topic", "/object_detection/lane_mask");
    lane_max_age_ = declare_parameter<double>("lane.max_age", 0.5);
    LaneEstimatorConfig lec;
    lec.min_range = declare_parameter<double>("lane.min_range", lec.min_range);
    lec.max_range = declare_parameter<double>("lane.max_range", lec.max_range);
    lec.row_step = declare_parameter<int>("lane.row_step", lec.row_step);
    lec.max_lateral = declare_parameter<double>("lane.max_lateral", lec.max_lateral);
    lec.min_run_px = declare_parameter<int>("lane.min_run_px", lec.min_run_px);
    lec.max_gap_px = declare_parameter<int>("lane.max_gap_px", lec.max_gap_px);
    lec.max_occluder_px = declare_parameter<int>("lane.max_occluder_px", lec.max_occluder_px);
    lec.keep_s = declare_parameter<double>("lane.keep_s", lec.keep_s);
    lec.min_points = declare_parameter<int>("lane.min_points", lec.min_points);
    lec.min_span = declare_parameter<double>("lane.min_span", lec.min_span);
    lec.quadratic_span = declare_parameter<double>("lane.quadratic_span", lec.quadratic_span);
    lec.outlier = declare_parameter<double>("lane.outlier", lec.outlier);
    lec.step = declare_parameter<double>("lane.step", lec.step);
    lec.max_edge_jump = declare_parameter<double>("lane.max_edge_jump", lec.max_edge_jump);
    lec.max_edge_angle = declare_parameter<double>("lane.max_edge_angle", lec.max_edge_angle);
    lec.min_width = declare_parameter<double>("lane.min_width", lec.min_width);
    lec.max_width = declare_parameter<double>("lane.max_width", lec.max_width);
    PathTrackerConfig ptc;
    ptc.points = declare_parameter<int>("lane.path.points", ptc.points);
    ptc.spacing = declare_parameter<double>("lane.path.spacing", ptc.spacing);
    ptc.wheelbase = declare_parameter<double>("lane.path.wheelbase", ptc.wheelbase);
    ptc.max_steer_deg = declare_parameter<double>("lane.path.max_steer_deg", ptc.max_steer_deg);
    ptc.vehicle_width = declare_parameter<double>("lane.path.vehicle_width", ptc.vehicle_width);
    ptc.lane_margin = declare_parameter<double>("lane.path.lane_margin", ptc.lane_margin);
    ptc.gain = declare_parameter<double>("lane.path.gain", ptc.gain);
    ptc.max_step = declare_parameter<double>("lane.path.max_step", ptc.max_step);
    ptc.gate = declare_parameter<double>("lane.path.gate", ptc.gate);
    ptc.gate_updates = declare_parameter<int>("lane.path.gate_updates", ptc.gate_updates);
    ptc.offset_rate = declare_parameter<double>("lane.path.offset_rate", ptc.offset_rate);
    ptc.hold_distance = declare_parameter<double>("lane.path.hold_distance", ptc.hold_distance);
    ptc.smoothing = declare_parameter<double>("lane.path.smoothing", ptc.smoothing);
    path_tracker_ = PathTracker(ptc);
    for (const auto & c : declare_parameter<std::vector<std::string>>("lane.occluder_classes",
      {"person", "pedestrian", "partial pedestrian", "bicycle", "bicyclist", "motorcycle", "motorcyclist", "animal"}))
    {
      lane_occluder_classes_.insert(c);
    }
    lane_est_ = LaneEstimator(lec);
    lane_rectified_ = cc.rectified;
    lane_balance_ = cc.rectify_balance;
    lane_fov_scale_ = cc.rectify_fov_scale;
#endif
#ifdef SF_WITH_BYTETRACK
    // --- 2D box tracker (ByteTrack) ---
    ByteTrackerConfig btc;
    auto & bt = btc;
    bt.high_thresh = static_cast<float>(score("camera_tracker.high_thresh", "high_thresh", bt.high_thresh));
    bt.low_thresh = static_cast<float>(score("camera_tracker.low_thresh", "low_thresh", bt.low_thresh));
    bt.new_track_thresh = static_cast<float>(score("camera_tracker.new_track_thresh", "new_track_thresh",
      bt.new_track_thresh));
    bt.match_iou = static_cast<float>(declare_parameter<double>("camera_tracker.match_iou", bt.match_iou));
    bt.low_match_iou = static_cast<float>(declare_parameter<double>("camera_tracker.low_match_iou", bt.low_match_iou));
    bt.min_hits = declare_parameter<int>("camera_tracker.min_hits", bt.min_hits);
    bt.max_lost_s = declare_parameter<double>("camera_tracker.max_lost_s", bt.max_lost_s);
    bt.class_consistent = declare_parameter<bool>("camera_tracker.class_consistent", bt.class_consistent);
    RCLCPP_INFO(get_logger(), "camera scores for detector %s: ByteTrack high %.2f low %.2f new track %.2f, "
      "fusion min score %.2f", detector_model_type_.c_str(), bt.high_thresh, bt.low_thresh, bt.new_track_thresh,
      cc.min_detection_score);
#else
    RCLCPP_INFO(get_logger(), "camera scores for detector %s: fusion min score %.2f", detector_model_type_.c_str(),
      cc.min_detection_score);
#endif
    // --- ego path ---
    auto & pa = c.path;
    pa.enabled = declare_parameter<bool>("path.enabled", pa.enabled);
    pa.curvature_source = declare_parameter<std::string>("path.curvature_source", pa.curvature_source);
    pa.wheelbase = declare_parameter<double>("path.wheelbase", pa.wheelbase);
    pa.steering_ratio = declare_parameter<double>("path.steering_ratio", pa.steering_ratio);
    pa.steering_in_degrees = declare_parameter<bool>("path.steering_in_degrees", pa.steering_in_degrees);
    pa.steering_offset = declare_parameter<double>("path.steering_offset", pa.steering_offset);
    pa.steering_sign = declare_parameter<double>("path.steering_sign", pa.steering_sign);
    pa.half_width = declare_parameter<double>("path.half_width", pa.half_width);
    pa.length = declare_parameter<double>("path.length", pa.length);
    pa.min_speed_odom = declare_parameter<double>("path.min_speed_odom", pa.min_speed_odom);
    pa.max_curvature = declare_parameter<double>("path.max_curvature", pa.max_curvature);
    pa.max_heading_change = declare_parameter<double>("path.max_heading_change", pa.max_heading_change);
    pa.smoothing = declare_parameter<double>("path.smoothing", pa.smoothing);
    // --- relative motion and future position ---
    auto & rm = c.relative_motion;
    rm.prediction_horizon = declare_parameter<double>(
      "relative_motion.prediction_horizon", rm.prediction_horizon);
    rm.radial_speed_threshold = declare_parameter<double>(
      "relative_motion.radial_speed_threshold", rm.radial_speed_threshold);
    rm.min_ego_speed = declare_parameter<double>("relative_motion.min_ego_speed", rm.min_ego_speed);
    rm.min_object_speed = declare_parameter<double>(
      "relative_motion.min_object_speed", rm.min_object_speed);
    rm.same_direction_max_angle = declare_parameter<double>(
      "relative_motion.same_direction_max_angle", rm.same_direction_max_angle);
    // --- stop zone ---
    auto & sz = c.stop_zone;
    sz.horizon = declare_parameter<double>("stop_zone.horizon", sz.horizon);
    sz.step = declare_parameter<double>("stop_zone.step", sz.step);
    sz.min_distance = declare_parameter<double>("stop_zone.min_distance", sz.min_distance);
    sz.time_headway = declare_parameter<double>("stop_zone.time_headway", sz.time_headway);
    // --- output ---
    c.output_max_range = declare_parameter<double>("output.max_range", c.output_max_range);
    c.output_min_existence = declare_parameter<double>("output.min_existence", c.output_min_existence);
    c.output_keep_existence = declare_parameter<double>("output.keep_existence", c.output_keep_existence);
    c.output_max_static_height = declare_parameter<double>("output.max_static_height", c.output_max_static_height);
    radar_height_ = radar_height;
    radar_mount_x_ = c.fusion_frame_mount.x;
    radar_mount_y_ = c.fusion_frame_mount.y;
    radar_mount_yaw_ = c.fusion_frame_mount.yaw;

    if (!output_frame_.empty()) {
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    core_ = std::make_unique<FusionCore>(c);
#ifdef SF_WITH_IMM
    if (radar_tracker_ == "imm") {imm_ = std::make_unique<ImmTracker>(ic);}
#endif
#ifdef SF_WITH_BYTETRACK
    box_tracker_ = std::make_unique<ByteTracker>(btc);
#endif
    if (!intr_file.empty() && !extr_file.empty()) {
      try {
        const auto cam = fusion_common::CameraModel::load(intr_file, extr_file);
        core_->setCamera(cam);
        base_camera_ = cam;
        const Eigen::Vector3d cp = cam.cameraPositionInRadar();
        RCLCPP_INFO(get_logger(), "camera calibration %s + %s (camera at x=%.2f y=%.2f z=%.2f m in radar frame, %s)",
          intr_file.c_str(), extr_file.c_str(), cp.x(), cp.y(), cp.z(), cc.rectified ? "rectified images" : "raw images");
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "failed to load camera calibration: %s (camera fusion off)", e.what());
      }
    } else {
      RCLCPP_WARN(get_logger(), "camera.intrinsics_file / extrinsics_file not set: camera fusion disabled");
    }

    const auto sensor_qos = rclcpp::SensorDataQoS();
    if (radar_tracker_ == "ekf") {
      ekf_sub_ = create_subscription<perception_msgs::msg::RadarTrackArray>(
        ekf_topic, rclcpp::QoS(10), [this](perception_msgs::msg::RadarTrackArray::ConstSharedPtr m) {onEkfTracks(*m);});
    } else {
      radar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        radar_topic, sensor_qos, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {onRadar(*m);});
    }
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(50), [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        const EgoSample s{toSec(m->header.stamp), m->twist.twist.linear.x, m->twist.twist.angular.z};
        core_->addEgo(s);
#ifdef SF_WITH_IMM
        if (imm_) {imm_->addEgo(s);}
#endif
      });
    steer_sub_ = create_subscription<std_msgs::msg::Float32>(
      steering_topic, rclcpp::QoS(50), [this](std_msgs::msg::Float32::ConstSharedPtr m) {core_->addSteering(m->data);});
#ifdef SF_WITH_LANE
    if (lane_enabled && base_camera_.valid()) {
      lane_sub_ = create_subscription<perception_msgs::msg::SegmentationMask>(
        lane_topic, rclcpp::QoS(5), [this](perception_msgs::msg::SegmentationMask::ConstSharedPtr m) {onLane(*m);});
      lane_path_pub_ = create_publisher<nav_msgs::msg::Path>("~/lane_path", 10);
    }
#endif
    cam_sub_ = create_subscription<perception_msgs::msg::CameraObjectArray>(
      cam_topic, rclcpp::QoS(10), [this](perception_msgs::msg::CameraObjectArray::ConstSharedPtr m) {onCamera(*m);});
    objects_pub_ = create_publisher<perception_msgs::msg::FusedObjectArray>("~/objects", 20);
    tracks_pub_ = create_publisher<perception_msgs::msg::FusedObjectArray>("~/tracks", 20);
    cam_tracks_pub_ = create_publisher<perception_msgs::msg::CameraObjectArray>("~/camera_tracks", 20);
    RCLCPP_INFO(get_logger(), "sensor_fusion up: radar tracker=%s (%s) odom=%s camera=%s ego_speed=%s output_frame=%s",
      radar_tracker_.c_str(), radar_tracker_ == "ekf" ? ekf_topic.c_str() : radar_topic.c_str(), odom_topic.c_str(), cam_topic.c_str(), c.ego_speed_source.c_str(),
      output_frame_.empty() ? "<radar>" : output_frame_.c_str());
  }

private:
  /// Pending cycle (radar scan or camera frame) in the stamp-ordered queue.
  struct Pending
  {
    builtin_interfaces::msg::Time stamp;
    bool radar{true};
    std::vector<RadarDetection> dets;
    bool external{false};                    // radar cycle from the EKF tracker
    std::vector<ExternalRadarTrack> ext;
    double ext_ego_speed{0};
    CameraFrame frame;
  };

  /**
   * @brief Queues a cycle and runs every cycle older than newest - reorder_delay.
   * @param[in] p  pending cycle (moved)
   */
  void enqueue(Pending && p)
  {
    const double t = toSec(p.stamp);
    if (t < newest_ - 2.0) {   // bag restarted: run what is queued, start over
      flush(1e18);
      newest_ = t;
    }
    newest_ = std::max(newest_, t);
    queue_.emplace(t, std::move(p));
    flush(newest_ - reorder_delay_);
  }

  /**
   * @brief Runs queued cycles up to a time; decides which cycle publishes (camera = output clock,
   *        radar only after camera_timeout) and records the cycle time.
   * @param[in] until  run cycles with stamp <= until [s]
   */
  void flush(double until)
  {
    while (!queue_.empty() && queue_.begin()->first <= until) {
      Pending p = std::move(queue_.begin()->second);
      queue_.erase(queue_.begin());
      const auto t0 = std::chrono::steady_clock::now();
      // output clock = camera (fastest sensor); radar cycles only update the tracks,
      // unless the camera is silent for camera_timeout (then the radar publishes)
      if (p.radar) {
        const double t = toSec(p.stamp);
        if (p.external) {
          core_->processRadarTracks(t, p.ext, p.ext_ego_speed, "ekf");
        } else {
#ifdef SF_WITH_IMM
          double ego_speed = 0;
          const auto tracks = imm_->process(t, p.dets, &ego_speed);
          core_->processRadarTracks(t, tracks, ego_speed, "imm");
#endif
        }
        if (output_trigger_ == "all" || !core_->hasCameraModel() || t - last_camera_stamp_ > camera_timeout_) {
          publish(p.stamp, "radar");
        }
      } else {
#ifdef SF_WITH_BYTETRACK
        core_->processCamera(trackBoxes(*box_tracker_, p.frame, &camera_tracks_));
        publishCameraTracks(p);
#else
        core_->processCamera(p.frame);   // track ids from the camera tracker upstream
#endif
        last_camera_stamp_ = p.frame.stamp;
        if (core_->hasCameraModel()) {publish(p.stamp, "camera");}
      }
      (p.radar ? radar_ms_ : camera_ms_).push_back(
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
    }
    logTiming();
  }

  /// @brief Logs n / mean / p50 / p95 / max cycle time (fusion + publishing) every timing_log_s of wall time.
  void logTiming()
  {
    const auto now = std::chrono::steady_clock::now();
    if (timing_log_s_ <= 0 || now - last_timing_log_ < std::chrono::duration<double>(timing_log_s_)) {return;}
    auto stats = [](std::vector<double> & v) {
        if (v.empty()) {return std::string("-");}
        std::sort(v.begin(), v.end());
        double sum = 0;
        for (double x : v) {sum += x;}
        const std::string s = fmtStats(v.size(), sum / v.size(), v[v.size() / 2], v[v.size() * 95 / 100], v.back());
        v.clear();
        return s;
      };
    RCLCPP_INFO(get_logger(), "cycle time [ms]  radar: %s | camera: %s | tracks %zu",
      stats(radar_ms_).c_str(), stats(camera_ms_).c_str(), core_->confirmedTracks().size());
    last_timing_log_ = now;
  }

  /**
   * @brief Formats cycle-time statistics.
   * @param[in] n, mean, p50, p95, max  statistics [ms]
   * @return log text
   */
  static std::string fmtStats(std::size_t n, double mean, double p50, double p95, double max)
  {
    char b[128];
    std::snprintf(b, sizeof(b), "n %zu mean %.2f p50 %.2f p95 %.2f max %.2f", n, mean, p50, p95, max);
    return b;
  }

  /**
   * @brief Detector boxes -> camera cycle.
   * @param[in] m  detector output of one rectified image
   */
  void onCamera(const perception_msgs::msg::CameraObjectArray & m)
  {
    Pending p;
    p.stamp = m.header.stamp;
    p.radar = false;
    camera_frame_ = m.header.frame_id;
    p.frame.stamp = toSec(m.header.stamp);
    p.frame.width = static_cast<int>(m.image_width);
    p.frame.height = static_cast<int>(m.image_height);
    for (const auto & o : m.objects) {
      p.frame.detections.push_back(
        CameraDetection{o.track_id, o.class_id, o.class_name, o.score, o.x_min, o.y_min, o.x_max, o.y_max});
    }
#ifdef SF_WITH_LANE
    if (lane_sub_) {   // boxes of objects on the road for the mask of the same image (last ~1 s)
      std::vector<Eigen::Vector4d> boxes;
      for (const auto & o : m.objects) {
        if (lane_occluder_classes_.count(o.class_name)) {boxes.emplace_back(o.x_min, o.y_min, o.x_max, o.y_max);}
      }
      lane_occluders_.emplace_back(p.frame.stamp, std::move(boxes));
      if (lane_occluders_.size() > 30) {lane_occluders_.pop_front();}
    }
#endif
    enqueue(std::move(p));
  }

#ifdef SF_WITH_LANE
  /**
   * @brief Segmentation mask -> lane / road boundaries (radar frame at the mask stamp).
   * @param[in] m  lane / road classes of one rectified image
   */
  void onLane(const perception_msgs::msg::SegmentationMask & m)
  {
    if (m.mask.encoding != "mono8" || m.mask.data.size() < static_cast<std::size_t>(m.mask.width) * m.mask.height ||
      m.image_width == 0 || m.image_height == 0)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "lane mask: expected mono8 with the image size");
      return;
    }
    const double t = toSec(m.header.stamp);
    if (lane_cam_size_ != std::make_pair(static_cast<int>(m.image_width), static_cast<int>(m.image_height))) {
      lane_cam_ = fusion_common::cameraForImage(base_camera_, m.image_width, m.image_height, lane_rectified_,
        lane_balance_, lane_fov_scale_);
      lane_cam_size_ = {static_cast<int>(m.image_width), static_cast<int>(m.image_height)};
    }
    // radar frame at the previous mask -> radar frame now (ego motion)
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
    Eigen::Vector2d tr = Eigen::Vector2d::Zero();
    const double prev = lane_est_.estimate().stamp;
    if (prev >= 0 && t >= prev && t - prev < 2.0) {core_->egoFrameTransform(prev, t, R, tr);}
    const auto * data = m.mask.step == m.mask.width ? m.mask.data.data() : nullptr;
    std::vector<uint8_t> packed;
    if (!data) {   // rows with padding: pack them
      packed.resize(static_cast<std::size_t>(m.mask.width) * m.mask.height);
      for (uint32_t v = 0; v < m.mask.height; ++v) {
        std::copy_n(m.mask.data.data() + v * m.mask.step, m.mask.width, packed.data() + v * m.mask.width);
      }
      data = packed.data();
    }
    // detector boxes of the same image (stamps equal; 20 ms tolerance)
    const std::vector<Eigen::Vector4d> * occluders = nullptr;
    for (const auto & [stamp, boxes] : lane_occluders_) {
      if (std::abs(stamp - t) < 0.02) {occluders = &boxes;}
    }
    const auto & e = lane_est_.update(t, data, static_cast<int>(m.mask.width), static_cast<int>(m.mask.height),
      static_cast<int>(m.image_width), static_cast<int>(m.image_height), lane_cam_, -radar_height_, R, tr,
      occluders ? *occluders : std::vector<Eigen::Vector4d>{});
    // tracked path from base_link: its origin and heading in the radar frame (p_radar = rb^T (p_base - tb))
    Eigen::Matrix3d rb;
    Eigen::Vector3d tb;
    radarToBase(rb, tb);
    const Eigen::Vector3d base = rb.transpose() * -tb;
    const Eigen::Vector3d fwd = rb.transpose() * Eigen::Vector3d::UnitX();
    path_tracker_.update(e, base.head<2>(), std::atan2(fwd.y(), fwd.x()), R, tr, core_->path().curvature());
  }

  /**
   * @brief Lane / road boundaries at the output stamp (ego motion since the mask) into the message,
   *        radar frame (transformOutput moves them to the output frame) and base frame copies.
   * @param[in]     stamp  output stamp
   * @param[in,out] msg    output message
   */
  void addLane(const builtin_interfaces::msg::Time & stamp, perception_msgs::msg::FusedObjectArray & msg)
  {
    const auto & e = lane_est_.estimate();
    const double t = toSec(stamp);
    msg.lane_age = -1.0f;
    msg.road_width = -1.0f;
    msg.base_frame_id = base_frame_;
    Eigen::Matrix3d rb;
    Eigen::Vector3d tb;
    radarToBase(rb, tb);
    const Eigen::Quaterniond qb(rb);
    msg.radar_pose_base.position.x = tb.x();
    msg.radar_pose_base.position.y = tb.y();
    msg.radar_pose_base.position.z = tb.z();
    msg.radar_pose_base.orientation.x = qb.x();
    msg.radar_pose_base.orientation.y = qb.y();
    msg.radar_pose_base.orientation.z = qb.z();
    msg.radar_pose_base.orientation.w = qb.w();
    nav_msgs::msg::Path path;   // empty: no path
    path.header.stamp = stamp;
    path.header.frame_id = base_frame_;
    if (e.stamp < 0 || std::abs(t - e.stamp) > lane_max_age_) {
      if (lane_path_pub_) {lane_path_pub_->publish(path);}
      return;
    }
    msg.lane_age = static_cast<float>(t - e.stamp);
    msg.road_width = static_cast<float>(e.road_width);
    const auto & tp = path_tracker_.path();
    msg.lane_path_unmeasured = static_cast<float>(tp.unmeasured_m);
    Eigen::Matrix2d R;
    Eigen::Vector2d tr;
    core_->egoFrameTransform(e.stamp, t, R, tr);   // radar frame at the mask -> at the output stamp
    static const std::vector<Eigen::Vector2d> none;
    auto pts = [](const LaneBoundary & b) -> const std::vector<Eigen::Vector2d> & {return b.valid ? b.points : none;};
    const std::pair<const std::vector<Eigen::Vector2d> *, std::pair<std::vector<geometry_msgs::msg::Point> *,
      std::vector<geometry_msgs::msg::Point> *>> sets[] = {
      {&pts(e.lane_left), {&msg.lane_left, &msg.lane_left_base}},
      {&pts(e.lane_right), {&msg.lane_right, &msg.lane_right_base}},
      {&pts(e.road_left), {&msg.road_left, &msg.road_left_base}},
      {&pts(e.road_right), {&msg.road_right, &msg.road_right_base}},
      {&tp.centre, {&msg.lane_path, &msg.lane_path_base}},
      {&tp.left, {&msg.lane_path_left, &msg.lane_path_left_base}},
      {&tp.right, {&msg.lane_path_right, &msg.lane_path_right_base}}};
    for (const auto & [line, out] : sets) {
      for (const auto & p0 : *line) {
        const Eigen::Vector2d p = R * p0 + tr;
        geometry_msgs::msg::Point q;
        q.x = p.x();
        q.y = p.y();
        q.z = -radar_height_;
        out.first->push_back(q);
        const Eigen::Vector3d pb = rb * Eigen::Vector3d(q.x, q.y, q.z) + tb;
        geometry_msgs::msg::Point qb;
        qb.x = pb.x();
        qb.y = pb.y();
        qb.z = pb.z();
        out.second->push_back(qb);
      }
    }
    // planner path: base-frame points, heading along the path (yaw of the step to the next point)
    const auto & centre = msg.lane_path_base;
    for (std::size_t i = 0; i < centre.size(); ++i) {
      const std::size_t a = i + 1 < centre.size() ? i : i - 1, b = a + 1;
      const double yaw = centre.size() > 1 ? std::atan2(centre[b].y - centre[a].y, centre[b].x - centre[a].x) : 0.0;
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position = centre[i];
      ps.pose.orientation.z = std::sin(0.5 * yaw);
      ps.pose.orientation.w = std::cos(0.5 * yaw);
      path.poses.push_back(ps);
    }
    if (lane_path_pub_) {lane_path_pub_->publish(path);}
  }

  /**
   * @brief Radar -> base frame: TF if it knows the radar, else the configured mount.
   * @param[out] rotation, translation  p_base = rotation p_radar + translation
   */
  void radarToBase(Eigen::Matrix3d & rotation, Eigen::Vector3d & translation)
  {
    rotation = Eigen::AngleAxisd(radar_mount_yaw_, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    translation = Eigen::Vector3d(radar_mount_x_, radar_mount_y_, radar_height_);
    if (!tf_buffer_ || frame_ == base_frame_) {
      if (frame_ == base_frame_) {rotation.setIdentity(); translation.setZero();}
      return;
    }
    try {
      if (!tf_buffer_->canTransform(base_frame_, frame_, tf2::TimePointZero)) {return;}
      const auto tf = tf_buffer_->lookupTransform(base_frame_, frame_, tf2::TimePointZero);
      const auto & q = tf.transform.rotation;
      rotation = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
      translation = Eigen::Vector3d(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
    } catch (const tf2::TransformException &) {
      // configured mount
    }
  }
#endif

  /**
   * @brief Tracks of the EKF tracker (radar_tracker node) -> radar cycle.
   * @param[in] m  tracks of one radar scan, radar frame
   */
  void onEkfTracks(const perception_msgs::msg::RadarTrackArray & m)
  {
    Pending p;
    p.stamp = m.header.stamp;
    p.external = true;
    p.ext_ego_speed = m.ego_speed;
    p.ext.reserve(m.tracks.size());
    for (const auto & t : m.tracks) {
      ExternalRadarTrack e;
      e.id = t.id;
      e.px = t.px; e.py = t.py; e.pz = t.pz;
      e.vx = t.vx; e.vy = t.vy;
      for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {e.P(i, j) = t.covariance[i * 4 + j];}
      }
      e.existence = t.poe;
      e.hits = t.hits;
      e.age = t.age;
      e.dynamic = t.is_dynamic;
      e.mirror_ghost = t.is_mirror_ghost;
      e.ground_reflection = t.is_ground_reflection;
      p.ext.push_back(e);
    }
    frame_ = m.header.frame_id;
    enqueue(std::move(p));
  }

  /**
   * @brief Radar point cloud (smartmicro fields) -> radar cycle.
   * @param[in] m  radar scan; must contain x, y, z, radial_speed (rcs, snr, range optional)
   */
  void onRadar(const sensor_msgs::msg::PointCloud2 & m)
  {
    if (!hasField(m, "radial_speed")) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "radar cloud has no radial_speed field");
      return;
    }
    const std::size_t n = static_cast<std::size_t>(m.width) * m.height;
    std::vector<RadarDetection> dets(n);
    sensor_msgs::PointCloud2ConstIterator<float> x(m, "x"), y(m, "y"), z(m, "z"), vr(m, "radial_speed");
    const bool has_rcs = hasField(m, "rcs"), has_snr = hasField(m, "snr"), has_range = hasField(m, "range");
    for (std::size_t i = 0; i < n; ++i, ++x, ++y, ++z, ++vr) {
      dets[i].x = *x; dets[i].y = *y; dets[i].z = *z; dets[i].radial_speed = *vr;
    }
    if (has_rcs) {sensor_msgs::PointCloud2ConstIterator<float> it(m, "rcs"); for (std::size_t i = 0; i < n; ++i, ++it) {dets[i].rcs = *it;}}
    if (has_snr) {sensor_msgs::PointCloud2ConstIterator<float> it(m, "snr"); for (std::size_t i = 0; i < n; ++i, ++it) {dets[i].snr = *it;}}
    if (has_range) {sensor_msgs::PointCloud2ConstIterator<float> it(m, "range"); for (std::size_t i = 0; i < n; ++i, ++it) {dets[i].range = *it;}}
    frame_ = m.header.frame_id;
    Pending p;
    p.stamp = m.header.stamp;
    p.dets = std::move(dets);
    enqueue(std::move(p));
  }

  /**
   * @brief Fusion output object -> ROS message.
   * @param[in] t  fused track
   * @return message (class dimensions from classInfo)
   */
  perception_msgs::msg::FusedObject toMsg(const FusedTrack & t) const
  {
    perception_msgs::msg::FusedObject o;
    o.id = t.id;
    o.class_name = t.class_name;
    o.class_confidence = static_cast<float>(t.class_confidence);
    o.position.x = t.pos.x();
    o.position.y = t.pos.y();
    o.position.z = t.z;
    o.velocity.x = t.vel.x();
    o.velocity.y = t.vel.y();
    o.yaw_rate = static_cast<float>(t.yaw_rate);
    o.heading = static_cast<float>(t.heading);
    const auto & ci = fusion_common::classInfo(t.class_name);
    o.dimensions.x = ci.length;
    o.dimensions.y = ci.width;
    o.dimensions.z = ci.height;
    o.range = static_cast<float>(t.range());
    o.azimuth = static_cast<float>(std::atan2(t.pos.y(), t.pos.x()));
    o.position_covariance = {static_cast<float>(t.pos_cov(0, 0)), static_cast<float>(t.pos_cov(0, 1)),
      static_cast<float>(t.pos_cov(1, 0)), static_cast<float>(t.pos_cov(1, 1))};
    for (double p : t.model_probs) {o.model_probabilities.push_back(static_cast<float>(p));}
    o.existence_probability = static_cast<float>(t.existence);
    o.is_moving = t.is_moving;
    o.ghost = t.ghost;
    o.motion_state = t.motion_state;
    o.relative_motion_state = t.relative_motion_state;
    o.closing_speed = static_cast<float>(t.closing_speed);
    o.time_to_collision = static_cast<float>(t.time_to_collision);
    o.predicted_position.x = t.predicted_pos.x();
    o.predicted_position.y = t.predicted_pos.y();
    o.predicted_position.z = t.z;
    o.prediction_horizon = static_cast<float>(t.prediction_horizon);
    o.in_path = t.in_path;
    o.path_distance = static_cast<float>(t.path_distance);
    o.path_lateral = static_cast<float>(t.path_lateral);
    o.predicted_in_path = t.predicted_in_path;
    o.predicted_path_distance = static_cast<float>(t.predicted_path_distance);
    o.predicted_path_lateral = static_cast<float>(t.predicted_path_lateral);
    // radar-frame copies (transformOutput only changes position / velocity / predicted_position)
    o.position_radar.x = t.pos.x();
    o.position_radar.y = t.pos.y();
    o.position_radar.z = t.z;
    o.velocity_radar.x = t.vel.x();
    o.velocity_radar.y = t.vel.y();
    o.predicted_position_radar.x = t.predicted_pos.x();
    o.predicted_position_radar.y = t.predicted_pos.y();
    o.predicted_position_radar.z = t.z;
    o.in_stop_zone = t.in_stop_zone;
    o.time_to_stop_zone = static_cast<float>(t.time_to_stop_zone);
    o.age = t.age;
    o.hits = t.hits;
    o.radar_updated = t.radar_updated;
    o.camera_associated = t.camera_associated;
    o.camera_track_id = t.camera_track_id;
    o.tracker = t.tracker;
    o.ekf_track_id = static_cast<int32_t>(t.ekf_track_id);
    o.in_image = t.in_image;
    o.image_u = static_cast<float>(t.u);
    o.image_v = static_cast<float>(t.v);
    o.projected_u = static_cast<float>(t.proj_u);
    o.projected_v = static_cast<float>(t.proj_v);
    o.status = t.status;
    o.assoc_probability = static_cast<float>(t.assoc_prob);
    o.bearing_radar = static_cast<float>(t.bearing_radar);
    o.bearing_fused = static_cast<float>(t.bearing_fused);
    o.height = static_cast<float>(t.height);
    o.has_bbox = t.has_bbox;
    o.bbox_x_min = static_cast<float>(t.bbox[0]);
    o.bbox_y_min = static_cast<float>(t.bbox[1]);
    o.bbox_x_max = static_cast<float>(t.bbox[2]);
    o.bbox_y_max = static_cast<float>(t.bbox[3]);
    if (!t.class_probs.empty()) {
      o.class_names = fusion_common::fusedClasses();
      for (double p : t.class_probs) {o.class_probabilities.push_back(static_cast<float>(p));}
    }
    return o;
  }

  /**
   * @brief Resolves the rigid transform from the internal radar frame (at the output stamp) to the
   *        configured output frame, without ever waiting for TF.
   *
   * The radar mount comes from TF (base -> umrr) when it is connected, else from configuration;
   * localization supplies output <- base. If TF already covers the output stamp it is used as is.
   * Otherwise (live: wheel odometry TF at ~10 Hz is usually older than the newest radar / camera
   * cycle) the newest TF is used and the gap to the output stamp is bridged with the fusion's own
   * ego motion (odometry speed / yaw rate). Waiting for TF here blocked the single fusion thread
   * up to 50 ms per output and delayed / dropped outputs. A TF older than output_tf_tolerance is an
   * output failure: publishing radar coordinates with a map header would be unsafe.
   */
  bool outputTransform(const builtin_interfaces::msg::Time & stamp,
    Eigen::Matrix3d & rotation, Eigen::Vector3d & translation)
  {
    rotation.setIdentity();
    translation.setZero();
    if (output_frame_.empty() || output_frame_ == frame_) {return true;}
    // no output frame yet (map: localisation not initialised) - tf2 would print "Invalid frame ID"
    // on every canTransform
    if (!tf_buffer_->_frameExists(output_frame_)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "not publishing fused objects: no '%s' frame yet (waiting for localisation / initial pose)",
        output_frame_.c_str());
      return false;
    }
    try {
      const rclcpp::Time time(stamp);
      // source frame: the radar itself when TF knows it (sim, live base_link -> umrr), else base_link
      const bool radar_in_tf = tf_buffer_->_frameExists(frame_) &&
        tf_buffer_->canTransform(output_frame_, frame_, tf2::TimePointZero);
      const std::string & src = radar_in_tf ? frame_ : base_frame_;
      if (!radar_in_tf) {
        RCLCPP_INFO_ONCE(get_logger(),
          "TF frame '%s' is not connected to '%s'; using configured %s -> radar mount "
          "(x=%.3f y=%.3f z=%.3f yaw=%.3f) for map output.",
          frame_.c_str(), output_frame_.c_str(), base_frame_.c_str(), radar_mount_x_,
          radar_mount_y_, radar_height_, radar_mount_yaw_);
      }
      geometry_msgs::msg::TransformStamped tf;
      bool exact = tf_buffer_->canTransform(output_frame_, src, time, rclcpp::Duration::from_seconds(0.0));
      if (exact) {
        tf = tf_buffer_->lookupTransform(output_frame_, src, time);
      } else {
        tf = tf_buffer_->lookupTransform(output_frame_, src, tf2::TimePointZero);   // newest, no wait
        const double age = toSec(stamp) - toSec(tf.header.stamp);
        if (std::abs(age) > output_tf_tolerance_) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "not publishing fused objects: newest %s -> %s TF is %.3f s from the output stamp "
            "(limit %.3f s)", src.c_str(), output_frame_.c_str(), age, output_tf_tolerance_);
          return false;
        }
      }
      // output <- src at the TF time
      const auto & q = tf.transform.rotation;
      Eigen::Quaterniond q_out_src(q.w, q.x, q.y, q.z);
      q_out_src.normalize();
      Eigen::Matrix3d r = q_out_src.toRotationMatrix();
      const auto & p = tf.transform.translation;
      Eigen::Vector3d t(p.x, p.y, p.z);
      if (!radar_in_tf) {   // output <- base <- radar (configured mount)
        t += r * Eigen::Vector3d(radar_mount_x_, radar_mount_y_, radar_height_);
        r = r * Eigen::AngleAxisd(radar_mount_yaw_, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      }
      if (!exact) {
        // radar frame at the output stamp -> radar frame at the TF time, from the ego motion:
        // p_tf = R_e p_out + t_e, so output <- radar(stamp) = (r R_e, r t_e + t)
        Eigen::Matrix2d re;
        Eigen::Vector2d te;
        core_->egoFrameTransform(toSec(stamp), toSec(tf.header.stamp), re, te);
        Eigen::Matrix3d re3 = Eigen::Matrix3d::Identity();
        re3.topLeftCorner<2, 2>() = re;
        t = r * Eigen::Vector3d(te.x(), te.y(), 0.0) + t;
        r = r * re3;
      }
      rotation = r;
      translation = t;
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "not publishing fused objects: cannot transform %s -> %s at output stamp: %s",
        frame_.c_str(), output_frame_.c_str(), e.what());
      return false;
    }
  }

  /// @brief Applies a radar-to-output rigid transform to all Cartesian fields in a result.
  void transformOutput(perception_msgs::msg::FusedObjectArray & msg,
    const Eigen::Matrix3d & rotation, const Eigen::Vector3d & translation) const
  {
    const Eigen::Matrix2d a = rotation.topLeftCorner<2, 2>();
    for (auto & o : msg.objects) {
      const Eigen::Vector3d p = rotation * Eigen::Vector3d(o.position.x, o.position.y, o.position.z) + translation;
      const Eigen::Vector3d v = rotation * Eigen::Vector3d(o.velocity.x, o.velocity.y, o.velocity.z);
      o.position.x = p.x(); o.position.y = p.y(); o.position.z = p.z();
      const Eigen::Vector3d pp = rotation * Eigen::Vector3d(
        o.predicted_position.x, o.predicted_position.y, o.predicted_position.z) + translation;
      o.predicted_position.x = pp.x();
      o.predicted_position.y = pp.y();
      o.predicted_position.z = pp.z();
      o.velocity.x = v.x(); o.velocity.y = v.y(); o.velocity.z = v.z();
      const Eigen::Vector3d h = rotation * Eigen::Vector3d(std::cos(o.heading), std::sin(o.heading), 0.0);
      o.heading = static_cast<float>(std::atan2(h.y(), h.x()));
      Eigen::Matrix2d covariance;
      covariance << o.position_covariance[0], o.position_covariance[1],
        o.position_covariance[2], o.position_covariance[3];
      covariance = a * covariance * a.transpose();
      o.position_covariance = {static_cast<float>(covariance(0, 0)), static_cast<float>(covariance(0, 1)),
        static_cast<float>(covariance(1, 0)), static_cast<float>(covariance(1, 1))};
    }
    for (auto * border : {&msg.path_left, &msg.path_right, &msg.stop_zone, &msg.lane_left, &msg.lane_right,
        &msg.road_left, &msg.road_right, &msg.lane_path, &msg.lane_path_left, &msg.lane_path_right})
    {
      for (auto & point : *border) {
        const Eigen::Vector3d p = rotation * Eigen::Vector3d(point.x, point.y, point.z) + translation;
        point.x = p.x(); point.y = p.y(); point.z = p.z();
      }
    }
    msg.header.frame_id = output_frame_.empty() ? frame_ : output_frame_;
  }

  /**
   * @brief Publishes the final list with the ego path, and all confirmed tracks if subscribed.
   * @param[in] stamp    output stamp (stamp of the triggering cycle)
   * @param[in] trigger  "camera" or "radar"
   */
  void publish(const builtin_interfaces::msg::Time & stamp, const std::string & trigger)
  {
    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;
    if (!outputTransform(stamp, rotation, translation)) {return;}
    perception_msgs::msg::FusedObjectArray all, obj;
    all.header.stamp = obj.header.stamp = stamp;
    all.trigger = obj.trigger = trigger;
    all.radar_frame_id = obj.radar_frame_id = frame_;
    all.radar_tracker = obj.radar_tracker = radar_tracker_;
    for (const auto & t : core_->objects()) {obj.objects.push_back(toMsg(t));}
    const auto & path = core_->path();
    obj.path_curvature = static_cast<float>(path.curvature());
    obj.path_source = path.source();
    if (path.config().enabled) {
      for (int side : {1, -1}) {
        for (const auto & p : path.border(side)) {
          geometry_msgs::msg::Point q;
          q.x = p.x();
          q.y = p.y();
          q.z = -radar_height_;
          (side > 0 ? obj.path_left : obj.path_right).push_back(q);
        }
      }
    }
    obj.stop_zone_length = static_cast<float>(core_->stopZoneLength());
    for (const auto & p : core_->stopZone()) {
      geometry_msgs::msg::Point q;
      q.x = p.x();
      q.y = p.y();
      q.z = -radar_height_;
      obj.stop_zone.push_back(q);
    }
#ifdef SF_WITH_LANE
    addLane(stamp, obj);
#else
    obj.lane_age = -1.0f;
    obj.road_width = -1.0f;
#endif
    transformOutput(obj, rotation, translation);
    objects_pub_->publish(obj);
    if (tracks_pub_->get_subscription_count() > 0) {
      for (const auto & t : core_->confirmedTracks()) {all.objects.push_back(toMsg(t));}
      transformOutput(all, rotation, translation);
      tracks_pub_->publish(all);
    }
  }

#ifdef SF_WITH_BYTETRACK
  /**
   * @brief Publishes the 2D box tracks of a camera cycle.
   * @param[in] p  camera cycle (stamp, image size)
   */
  void publishCameraTracks(const Pending & p)
  {
    perception_msgs::msg::CameraObjectArray out;
    out.header.stamp = p.stamp;
    out.header.frame_id = camera_frame_;
    out.source = "sensor_fusion/byte_tracker";
    out.image_width = p.frame.width;
    out.image_height = p.frame.height;
    for (const auto & t : camera_tracks_) {
      perception_msgs::msg::CameraObject o;
      o.track_id = t.track_id;
      o.class_id = t.det.class_id;
      o.class_name = t.det.class_name;
      o.score = t.det.score;
      o.x_min = t.det.x1; o.y_min = t.det.y1; o.x_max = t.det.x2; o.y_max = t.det.y2;
      o.vu = t.vu; o.vv = t.vv;
      o.age = t.age;
      out.objects.push_back(o);
    }
    cam_tracks_pub_->publish(out);
  }
#endif

  /**
   * @brief Declares a parameter, or returns its value if it is declared already (one yaml
   *        value used by two configurations).
   * @param[in] name  parameter name
   * @param[in] def   default value
   * @return value
   */
  template<class T>
  T param(const std::string & name, const T & def)
  {
    return has_parameter(name) ? get_parameter(name).get_value<T>() : declare_parameter<T>(name, def);
  }

  std::unique_ptr<FusionCore> core_;
#ifdef SF_WITH_IMM
  std::unique_ptr<ImmTracker> imm_;          // radar.tracker imm
#endif
#ifdef SF_WITH_BYTETRACK
  std::unique_ptr<ByteTracker> box_tracker_;
  std::vector<TrackedBox> camera_tracks_;    // 2D tracks of the last camera cycle
#endif
  std::vector<std::string> model_names_;
  std::string output_frame_, base_frame_, frame_{"umrr"};
  double reorder_delay_{0.1}, newest_{-1e18}, radar_height_{0.5};
  double output_tf_tolerance_{0.1};
  double radar_mount_x_{0.0}, radar_mount_y_{0.0}, radar_mount_yaw_{0.0};
  std::multimap<double, Pending> queue_;
  double timing_log_s_{10.0};
  std::string output_trigger_{"camera"};
  double camera_timeout_{0.2}, last_camera_stamp_{-1e18};
  std::vector<double> radar_ms_, camera_ms_;
  std::chrono::steady_clock::time_point last_timing_log_{std::chrono::steady_clock::now()};
  rclcpp::Subscription<perception_msgs::msg::RadarTrackArray>::SharedPtr ekf_sub_;
  std::string radar_tracker_{"ekf"};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr radar_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<perception_msgs::msg::CameraObjectArray>::SharedPtr cam_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steer_sub_;
  rclcpp::Publisher<perception_msgs::msg::FusedObjectArray>::SharedPtr objects_pub_, tracks_pub_;
  rclcpp::Publisher<perception_msgs::msg::CameraObjectArray>::SharedPtr cam_tracks_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string camera_frame_{"camera"};
  std::string detector_model_type_{"yolov8"};   // object_detection model: picks the score thresholds
  fusion_common::CameraModel base_camera_;   // calibrated camera (raw lens)
#ifdef SF_WITH_LANE
  rclcpp::Subscription<perception_msgs::msg::SegmentationMask>::SharedPtr lane_sub_;
  LaneEstimator lane_est_;
  PathTracker path_tracker_;                 // tracked driving path from base_link (radar frame)
  fusion_common::CameraModel lane_cam_;      // model of the masked (rectified) image
  std::pair<int, int> lane_cam_size_{0, 0};
  bool lane_rectified_{true};
  double lane_balance_{0.0}, lane_fov_scale_{1.0}, lane_max_age_{0.5};
  std::set<std::string> lane_occluder_classes_;   // detector labels of objects on the road (no road edge)
  std::deque<std::pair<double, std::vector<Eigen::Vector4d>>> lane_occluders_;   // stamp, boxes
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr lane_path_pub_;
#endif
};

}  // namespace sensor_fusion

/// Node entry point.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sensor_fusion::SensorFusionNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
