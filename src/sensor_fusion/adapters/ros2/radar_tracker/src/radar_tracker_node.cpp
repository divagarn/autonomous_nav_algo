/**
 * @file radar_tracker_node.cpp
 * @brief ROS 2 host for the EKF radar tracker (package ekf_tracker: aeb/ is a byte-identical copy of
 *        ti-processor-sdk .../app_radarsocket_pipeline: aeb_wrapper.cpp + radarprocessing/).
 *
 * Per radar scan, the same steps as main.c's AEB branch (aeb_view, front radar 0):
 *   1. transform_to_vehicle(): rotate by mount_yaw_deg, add mount (x, y, z)
 *   2. azimuth bias correction (rotation by azimuth_bias_rad)
 *   3. radar_filters: keep points with noise < noise_max (96)
 *   4. RadarData with x y z range radial_speed power noise rcs snr (azimuth/elevation 0, as mois_arr)
 *   5. aeb_process_frame_mois(points, ego_speed [m/s, >= 0], dt [s, clamped 0.01..0.5])
 * There is no speed gate (main.c's aeb_view_speed_kmph): the tracker runs at every speed.
 *
 * Output /radar_tracker/tracks (perception_msgs/RadarTrackArray): the tracks aeb_process_frame_mois
 * exports (confirmed, poe >= 0.6) with their full EkfTracker state; mirror ghosts and ground
 * reflections are flagged, not removed - the AEB pipeline only keeps them out of the MIO.
 *
 * Topics
 *   in   radar_topic  sensor_msgs/PointCloud2  smartmicro UMRR fields (x y z radial_speed power rcs noise snr ...)
 *   in   odom_topic   nav_msgs/Odometry        ego speed = |twist.linear.x|
 *   out  ~/tracks     perception_msgs/RadarTrackArray (frame: radar frame, or frame_id when set)
 *   out  ~/markers    visualization_msgs/MarkerArray
 */
#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "perception_msgs/msg/radar_track_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "radar_tracker/ekf_aeb_bridge.hpp"

namespace radar_tracker
{

namespace
{
bool hasField(const sensor_msgs::msg::PointCloud2 & m, const std::string & name)
{
  return std::any_of(m.fields.begin(), m.fields.end(), [&](const auto & f) {return f.name == name;});
}
}  // namespace

class RadarTrackerNode : public rclcpp::Node
{
public:
  RadarTrackerNode()
  : Node("radar_tracker")
  {
    const auto radar_topic = declare_parameter<std::string>("radar_topic", "/smart_radar/can_targets_0");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/odom");
    frame_id_ = declare_parameter<std::string>("frame_id", "");
    mount_[0] = declare_parameter<double>("mount_x", 0.0);
    mount_[1] = declare_parameter<double>("mount_y", 0.0);
    mount_[2] = declare_parameter<double>("mount_z", 0.5);
    mount_yaw_deg_ = static_cast<int>(declare_parameter<int>("mount_yaw_deg", 0));
    azimuth_bias_rad_ = declare_parameter<double>("azimuth_bias_rad", 0.0);
    radar_filters_ = declare_parameter<bool>("radar_filters", true);
    noise_max_ = declare_parameter<double>("noise_max", 96.0);
    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.5);

    if (aeb_init() != 0) {
      throw std::runtime_error("aeb_init() failed");
    }
    tracks_pub_ = create_publisher<perception_msgs::msg::RadarTrackArray>("~/tracks", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/markers", 10);
    radar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      radar_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {onRadar(*m);});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 20, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        ego_speed_ = static_cast<float>(std::fabs(m->twist.twist.linear.x));
        last_odom_ = now();
      });
    RCLCPP_INFO(get_logger(),
      "EKF radar tracker (AEB pipeline): %s + %s -> ~/tracks | mount (%.2f, %.2f, %.2f) yaw %d deg, "
      "azimuth bias %.4f rad, radar_filters %s (noise < %.0f), no speed gate",
      radar_topic.c_str(), odom_topic.c_str(), mount_[0], mount_[1], mount_[2], mount_yaw_deg_,
      azimuth_bias_rad_, radar_filters_ ? "on" : "off", noise_max_);
  }

  ~RadarTrackerNode() override {aeb_shutdown();}

private:
  void onRadar(const sensor_msgs::msg::PointCloud2 & m)
  {
    if (!hasField(m, "radial_speed")) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "radar cloud has no radial_speed field");
      return;
    }
    const rclcpp::Time stamp(m.header.stamp, get_clock()->get_clock_type());

    // dt from the scan stamps (main.c: playback stamps / live wall clock), clamped like main.c
    float dt_s = 0.1f;
    if (last_stamp_.nanoseconds() > 0) {
      const double d = (stamp - last_stamp_).seconds();
      if (d < -1.0) {   // bag restarted: start a fresh tracker
        aeb_shutdown();
        aeb_init();
      } else {
        dt_s = std::clamp(static_cast<float>(d), 0.01f, 0.5f);
      }
    }
    last_stamp_ = stamp;

    float ego_speed = ego_speed_;
    if (last_odom_.nanoseconds() == 0 || (now() - last_odom_).seconds() > odom_timeout_) {
      ego_speed = 0.0f;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no fresh odometry: ego speed 0");
    }

    // --- main.c AEB branch: mois_arr ---
    const std::size_t n = static_cast<std::size_t>(m.width) * m.height;
    std::vector<RadarData> mois;
    mois.reserve(std::min<std::size_t>(n, MAX_RADAR_POINTS));
    const bool has_power = hasField(m, "power"), has_noise = hasField(m, "noise");
    const bool has_rcs = hasField(m, "rcs"), has_snr = hasField(m, "snr"), has_range = hasField(m, "range");
    sensor_msgs::PointCloud2ConstIterator<float> ix(m, "x"), iy(m, "y"), iz(m, "z"), ivr(m, "radial_speed");
    auto field = [&](bool has, const char * name) {
        std::vector<float> v(n, 0.0f);
        if (has) {
          sensor_msgs::PointCloud2ConstIterator<float> it(m, name);
          for (std::size_t i = 0; i < n; ++i, ++it) {v[i] = *it;}
        }
        return v;
      };
    const auto power = field(has_power, "power"), noise = field(has_noise, "noise");
    const auto rcs = field(has_rcs, "rcs"), snr = field(has_snr, "snr"), range = field(has_range, "range");
    const float yaw = static_cast<float>(mount_yaw_deg_) * static_cast<float>(M_PI) / 180.0f;
    const float cy = std::cos(yaw), sy = std::sin(yaw);
    const float cb = std::cos(static_cast<float>(azimuth_bias_rad_));
    const float sb = std::sin(static_cast<float>(azimuth_bias_rad_));
    std::size_t filtered = 0;
    for (std::size_t i = 0; i < n; ++i, ++ix, ++iy, ++iz, ++ivr) {
      // transform_to_vehicle()
      float x = cy * *ix - sy * *iy + static_cast<float>(mount_[0]);
      float y = sy * *ix + cy * *iy + static_cast<float>(mount_[1]);
      const float z = *iz + static_cast<float>(mount_[2]);
      // azimuth bias correction (radar 0, AEB view)
      const float xc = x * cb - y * sb;
      const float yc = x * sb + y * cb;
      x = xc;
      y = yc;
      // radar_filters, AEB view: noise < 96 only
      if (radar_filters_ && !(noise[i] < noise_max_)) {++filtered; continue;}
      if (mois.size() >= MAX_RADAR_POINTS) {break;}
      RadarData d;
      std::memset(&d, 0, sizeof(d));   // main.c memsets mois_arr: azimuth/elevation stay 0
      d.x = x;
      d.y = y;
      d.z = z;
      d.range = range[i];
      d.radial_speed = *ivr;
      d.power = power[i];
      d.noise = noise[i];
      d.rcs = rcs[i];
      d.snr = snr[i];
      d.radar_id = 0;
      d.association_only = false;
      mois.push_back(d);
    }

    Top5Objects top5;
    std::vector<RadarData> out(MAX_RADAR_POINTS);
    const int nt = aeb_process_frame_mois(mois.data(), static_cast<int>(mois.size()), ego_speed, dt_s,
        &top5, out.data(), MAX_RADAR_POINTS);
    if (nt < 0) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "aeb_process_frame_mois failed (%d)", nt);
      return;
    }
    publish(m, ego_speed, dt_s, top5.frame_number, nt);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "scan: %zu points (%zu noise-filtered) -> %d exported tracks, MIO %ld, ego %.2f m/s, dt %.3f s",
      n, filtered, nt, static_cast<long>(aebMioId()), ego_speed, dt_s);
  }

  void publish(const sensor_msgs::msg::PointCloud2 & scan, float ego_speed, float dt_s,
    uint32_t frame_number, int exported)
  {
    perception_msgs::msg::RadarTrackArray arr;
    arr.header = scan.header;
    if (!frame_id_.empty()) {arr.header.frame_id = frame_id_;}
    arr.ego_speed = ego_speed;
    arr.dt = dt_s;
    arr.frame_number = frame_number;
    const int64_t mio = aebMioId();
    arr.mio_id = static_cast<int32_t>(mio);
    // same selection and order as aeb_process_frame_mois' out_tracks (PART 2)
    for (const RadarTrack & t : aebLastTracks()) {
      if (!t.confirmed || t.poe < 0.6f) {continue;}
      if (static_cast<int>(arr.tracks.size()) >= exported) {break;}
      perception_msgs::msg::RadarTrack r;
      r.id = t.id;
      r.px = t.px; r.py = t.py;
      r.pz = t.pz - static_cast<float>(mount_[2]);   // tracker z includes mount_z: back to the radar frame
      r.vx = t.vx; r.vy = t.vy;
      r.yaw = std::atan2(t.vy, t.vx);
      r.length = t.length; r.width = t.width;
      r.poe = t.poe;
      r.hits = t.hits; r.age = t.age; r.missed_frames = t.missed_frames;
      r.confirmed = t.confirmed;
      r.is_dynamic = t.is_dynamic;
      r.is_mirror_ghost = t.is_mirror_ghost;
      r.is_stationary_reflector = t.is_stationary_reflector;
      r.is_ground_reflection = t.is_ground_reflection;
      r.is_mio = (mio >= 0 && static_cast<int64_t>(t.id) == mio);
      r.radial_speed = t.radial_speed;
      r.avg_speed = t.avg_speed;
      r.avg_rcs = t.avg_rcs;
      r.avg_snr = t.avg_snr;
      r.n_pts = t.n_pts;
      for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {r.covariance[i * 4 + j] = t.P(i, j);}
      }
      arr.tracks.push_back(r);
    }
    tracks_pub_->publish(arr);
    publishMarkers(arr);
  }

  void publishMarkers(const perception_msgs::msg::RadarTrackArray & arr)
  {
    visualization_msgs::msg::MarkerArray ma;
    visualization_msgs::msg::Marker clear;
    clear.header = arr.header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(clear);
    int k = 0;
    for (const auto & t : arr.tracks) {
      visualization_msgs::msg::Marker b;
      b.header = arr.header;
      b.ns = "ekf_tracks";
      b.id = k++;
      b.type = visualization_msgs::msg::Marker::CUBE;
      b.pose.position.x = t.px;
      b.pose.position.y = t.py;
      b.pose.position.z = 0.5;
      b.pose.orientation.w = 1.0;
      b.scale.x = std::max(0.3f, t.length);
      b.scale.y = std::max(0.3f, t.width);
      b.scale.z = 1.0;
      // MIO red, ghost grey, ground brown, others cyan
      b.color.a = 0.6f;
      if (t.is_mio) {b.color.r = 1.0f;} else if (t.is_mirror_ghost) {b.color.r = b.color.g = b.color.b = 0.5f;} else if (
        t.is_ground_reflection) {b.color.r = 0.6f; b.color.g = 0.4f; b.color.b = 0.2f;} else {b.color.g = 0.8f; b.color.b = 1.0f;}
      ma.markers.push_back(b);
      visualization_msgs::msg::Marker txt = b;
      txt.id = k++;
      txt.ns = "ekf_track_ids";
      txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      txt.pose.position.z = 1.4;
      txt.scale.x = txt.scale.y = 0.0;
      txt.scale.z = 0.4;
      txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0f;
      char s[96];
      std::snprintf(s, sizeof(s), "R%u%s%s%s", t.id, t.is_mio ? " MIO" : "",
        t.is_mirror_ghost ? " ghost" : "", t.is_ground_reflection ? " ground" : "");
      txt.text = s;
      ma.markers.push_back(txt);
    }
    marker_pub_->publish(ma);
  }

  std::string frame_id_;
  double mount_[3]{0, 0, 0};
  int mount_yaw_deg_{0};
  double azimuth_bias_rad_{0};
  bool radar_filters_{true};
  double noise_max_{96.0};
  double odom_timeout_{0.5};
  float ego_speed_{0.0f};
  rclcpp::Time last_odom_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<perception_msgs::msg::RadarTrackArray>::SharedPtr tracks_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr radar_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

}  // namespace radar_tracker

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<radar_tracker::RadarTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
