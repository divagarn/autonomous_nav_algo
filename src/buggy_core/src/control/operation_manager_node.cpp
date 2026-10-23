// C++ version of buggy_pcd_lanelet_map/operation_manager.py for the LIVE vehicle (live:=true: real
// buggy, Gazebo, bag replay). Same topics, parameters and behaviour; the simulated-vehicle branch
// (live:=false, RViz-only driving) stays in the Python node.
//
// All decisions live in control/operation_core.cpp (no ROS types). This file only connects topics,
// TF, the planning display (stop / slow zones) and the timers.
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <ground_segmentation_msgs/msg/steer_speed.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "buggy_core/control/operation_core.hpp"

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

class OperationManager : public rclcpp::Node
{
public:
  OperationManager()
  : Node("operation_manager"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_, this, false)
  {
    buggy_core::CoreParams p;
    p.limit_kmh = declare_parameter<double>("velocity_limit_kmh", 5.0);
    p.wheelbase = declare_parameter<double>("wheelbase", 1.6);
    p.front_offset = declare_parameter<double>("front_offset", 2.2);
    p.detect_range = declare_parameter<double>("detect_range", 30.0);
    front_offset_ = p.front_offset;
    p.max_steer = declare_parameter<double>("max_steering_deg", 20.0);
    p.lookahead = declare_parameter<double>("lookahead_distance", 4.0);
    p.steer_rate = declare_parameter<double>("max_steering_rate_deg_s", 40.0);
    declare_parameter<bool>("publish_real_topics", false);     // sim-only, accepted and ignored
    declare_parameter<bool>("live", true);
    p.drive_enabled = declare_parameter<bool>("drive_enabled", false);
    p.base_at_center = declare_parameter<bool>("base_link_at_center", false);
    p.safe_half_width = declare_parameter<double>("corridor_half_width", 1.25);
    p.max_cte = declare_parameter<double>("max_cross_track", 2.5);
    p.loc_grace = declare_parameter<double>("localization_grace_sec", 1.0);
    p.react = declare_parameter<double>("stop_react_sec", 1.0);
    p.decel = declare_parameter<double>("stop_decel", 0.8);
    p.stop_margin = declare_parameter<double>("stop_margin", 2.5);
    p.slow_zone = declare_parameter<double>("slow_zone", 6.0);
    p.goal_margin = declare_parameter<double>("goal_margin", 0.5);
    const std::string calib = declare_parameter<std::string>("brake_calibration", default_calibration());
    p.require_detector = declare_parameter<bool>("require_obstacle_detector", true);
    p.radar_enabled = declare_parameter<bool>("radar_enabled", false);
    p.camera_enabled = declare_parameter<bool>("camera_enabled", false);
    p.avoid_enabled = declare_parameter<bool>("avoid_enabled", false);
    p.avoid_after = declare_parameter<double>("avoid_after_sec", 5.0);
    p.avoid_hw = declare_parameter<double>("avoid_corridor_half_width", 1.05);
    p.avoid_retry = declare_parameter<double>("avoid_retry_sec", 5.0);
    p.avoid_clear_sec = declare_parameter<double>("avoid_clear_sec", 2.0);
    p.avoid_stop_margin = declare_parameter<double>("avoid_stop_margin", 7.5);
    p.avoid_stop_early = declare_parameter<bool>("avoid_stop_early", false);
    p.avoid_probe = declare_parameter<bool>("avoid_probe", true);
    p.avoid_max_tries = static_cast<int>(declare_parameter<int>("avoid_max_tries", 6));
    p.memory_sec = declare_parameter<double>("obstacle_memory_sec", 4.0);
    p.memory_max_sec = declare_parameter<double>("obstacle_memory_max_sec", 25.0);
    p.creep_sec = declare_parameter<double>("lost_obstacle_creep_sec", 10.0);
    p.memory_min_hits = static_cast<int>(declare_parameter<int>("obstacle_memory_min_hits", 5));
    zone_hw_ = declare_parameter<double>("zone_half_width", 1.5);
    base_at_center_ = p.base_at_center;
    wheelbase_ = p.wheelbase;

    buggy_core::Hooks h;
    h.log = [this](buggy_core::Level l, const std::string & s) {
        switch (l) {
          case buggy_core::Level::Info: RCLCPP_INFO(get_logger(), "%s", s.c_str()); break;
          case buggy_core::Level::Warn: RCLCPP_WARN(get_logger(), "%s", s.c_str()); break;
          case buggy_core::Level::Error: RCLCPP_ERROR(get_logger(), "%s", s.c_str()); break;
        }
      };
    h.avoid_request = [this](const std::string & j) {
        std_msgs::msg::String m; m.data = j; avoid_pub_->publish(m);
      };
    h.detour_clear = [this]() {
        MarkerArray a; Marker m; m.action = Marker::DELETEALL; a.markers.push_back(m);
        detour_pub_->publish(a);
      };
    h.detour_show = [this](const std::vector<std::array<double, 3>> & seg, const std::vector<bool> & inside) {
        Marker m;
        m.header.frame_id = "map"; m.header.stamp = now();
        m.ns = "detour"; m.id = 0; m.type = Marker::LINE_STRIP; m.action = Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.35;
        m.color.r = 0.0; m.color.g = 0.9; m.color.b = 1.0; m.color.a = 1.0;
        std::vector<std::array<double, 3>> outs;
        for (size_t i = 0; i < seg.size(); ++i) {
          geometry_msgs::msg::Point pt; pt.x = seg[i][0]; pt.y = seg[i][1]; pt.z = seg[i][2] + 0.15;
          m.points.push_back(pt);
          std_msgs::msg::ColorRGBA c;       // cyan = inside the lane, magenta = outside the lane
          c.a = 1.0f;
          if (inside[i]) {c.r = 0.0f; c.g = 0.9f; c.b = 1.0f;} else {c.r = 1.0f; c.g = 0.2f; c.b = 0.9f; outs.push_back(seg[i]);}
          m.colors.push_back(c);
        }
        MarkerArray a; a.markers.push_back(m);
        if (!outs.empty()) {
          Marker t;
          t.header = m.header;
          t.ns = "detour_label"; t.id = 1; t.type = Marker::TEXT_VIEW_FACING; t.action = Marker::ADD;
          const auto & mid = outs[outs.size() / 2];
          t.pose.position.x = mid[0]; t.pose.position.y = mid[1]; t.pose.position.z = mid[2] + 1.6;
          t.pose.orientation.w = 1.0;
          t.scale.z = 0.6;
          t.color.r = 1.0f; t.color.g = 0.2f; t.color.b = 0.9f; t.color.a = 1.0f;
          t.text = "DETOUR OUTSIDE THE LANE";
          a.markers.push_back(t);
        }
        detour_pub_->publish(a);
      };
    h.pose = [this]() { return live_pose(); };
    core_ = std::make_unique<buggy_core::OperationCore>(p, h);
    core_->load_calibration(calib);

    using std_msgs::msg::String;
    using Objs = autoware_perception_msgs::msg::PredictedObjects;
    sub_req_ = create_subscription<String>("/system/request", 10,
        [this](String::ConstSharedPtr m) { core_->on_request(m->data); });
    sub_path_ = create_subscription<nav_msgs::msg::Path>("/planning/route_path", 10,
        [this](nav_msgs::msg::Path::ConstSharedPtr m) { on_path(*m); });
    sub_loc_ = create_subscription<String>("/localization/state", 10,
        [this](String::ConstSharedPtr m) { core_->on_loc(m->data, mono()); });
    sub_wheel_ = create_subscription<ground_segmentation_msgs::msg::SteerSpeed>("/wheel_uart", 10,
        [this](ground_segmentation_msgs::msg::SteerSpeed::ConstSharedPtr m) {
          core_->on_wheel(m->steering, m->speed);
        });
    sub_uart_ = create_subscription<String>("/uart/status", 10,
        [this](String::ConstSharedPtr m) { core_->on_uart(m->data, mono()); });
    sub_dummy_ = create_subscription<Objs>("/perception/object_recognition/objects", 10,
        [this](Objs::ConstSharedPtr m) { core_->on_dummy(parse_objects(*m)); });
    sub_lidar_ = create_subscription<Objs>("/perception/lidar_objects", 10,
        [this](Objs::ConstSharedPtr m) { core_->on_lidar(parse_objects(*m), mono()); });
    sub_radar_ = create_subscription<Objs>("/perception/radar_objects", 10,
        [this](Objs::ConstSharedPtr m) { core_->on_radar(parse_objects(*m), mono()); });
    sub_names_ = create_subscription<String>("/perception/object_names", 10,
        [this](String::ConstSharedPtr m) { core_->on_names(m->data, mono()); });
    sub_calib_ = create_subscription<std_msgs::msg::Bool>("/calibration/active", 10,
        [this](std_msgs::msg::Bool::ConstSharedPtr m) { core_->on_calib(m->data, mono()); });
    sub_avoid_ = create_subscription<String>("/planning/avoid_result", 10,
        [this](String::ConstSharedPtr m) { core_->on_avoid_result(m->data, mono()); });

    avoid_pub_ = create_publisher<String>("/planning/avoid_request", 5);
    detour_pub_ = create_publisher<MarkerArray>("/planning/detour_marker", 1);
    steer_pub_ = create_publisher<std_msgs::msg::Float32>("/steering_angle", 1);
    red_pub_ = create_publisher<std_msgs::msg::Bool>("/final_red_detected", 1);
    yellow_pub_ = create_publisher<std_msgs::msg::Bool>("/final_yellow_detected", 1);
    state_pub_ = create_publisher<String>("/system/state", 10);
    zone_pub_ = create_publisher<MarkerArray>("/planning/stop_zone_marker", 1);

    // node clock timers (sim time on replay / Gazebo), like rclpy's create_timer
    zone_timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(200), [this]() { publish_zones(); });
    step_timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(50), [this]() { step(); });
    state_timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(100), [this]() {
        std_msgs::msg::String m; m.data = core_->state_string(mono()); state_pub_->publish(m);
      });
  }

private:
  static std::string default_calibration()
  {
    const char * d = std::getenv("BUGGY_DATA_DIR");
    std::string dir = d ? d : "~/autoware_data";
    if (!dir.empty() && dir[0] == '~') {
      const char * home = std::getenv("HOME");
      dir = std::string(home ? home : "") + dir.substr(1);
    }
    return dir + "/buggy_brake_calibration.yaml";
  }

  double mono() { return get_clock()->now().nanoseconds() / 1e9; }

  void on_path(const nav_msgs::msg::Path & m)
  {
    std::vector<buggy_core::PathPt> pts;
    for (const auto & p : m.poses) {
      pts.push_back({p.pose.position.x, p.pose.position.y, p.pose.position.z,
          p.pose.orientation.z, p.pose.orientation.w});
    }
    core_->on_path(pts);
  }

  static std::vector<buggy_core::Obj> parse_objects(const autoware_perception_msgs::msg::PredictedObjects & m)
  {
    std::vector<buggy_core::Obj> out;
    for (const auto & o : m.objects) {
      const auto & p = o.kinematics.initial_pose_with_covariance.pose;
      const double yaw = 2.0 * std::atan2(p.orientation.z, p.orientation.w);
      const int label = o.classification.empty() ? 0 : o.classification[0].label;
      out.push_back({p.position.x, p.position.y, yaw, o.shape.dimensions.x, o.shape.dimensions.y, label});
    }
    return out;
  }

  // (rear-axle x, y, yaw, z) from map->base_link if fresh (< 0.5 s)
  std::optional<buggy_core::Pose> live_pose()
  {
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
    const rclcpp::Time stamp(t.header.stamp, get_clock()->get_clock_type());
    if ((get_clock()->now() - stamp).seconds() > 0.5) {return std::nullopt;}
    const auto & q = t.transform.rotation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    double x = t.transform.translation.x, y = t.transform.translation.y;
    if (base_at_center_) {
      x -= wheelbase_ / 2.0 * std::cos(yaw);
      y -= wheelbase_ / 2.0 * std::sin(yaw);
    }
    return buggy_core::Pose{x, y, yaw, t.transform.translation.z};
  }

  void step()
  {
    const auto cmd = core_->step_live(mono());
    if (!cmd) {return;}
    std_msgs::msg::Float32 s; s.data = static_cast<float>(cmd->steer);
    std_msgs::msg::Bool r; r.data = cmd->red;
    std_msgs::msg::Bool y; y.data = cmd->yellow;
    steer_pub_->publish(s);
    red_pub_->publish(r);
    yellow_pub_->publish(y);
  }

  // ---- planning display (stop / slow zones) --------------------------------------------------
  Marker ribbon_marker(double a, double b, double half_w, double z, const std::array<double, 4> & rgba,
    int mid, const std::string & ns, const builtin_interfaces::msg::Time & stamp, bool & ok)
  {
    const auto & path = core_->path();
    const auto & seg = core_->seg();
    Marker m;
    a = std::max(a, 0.0);
    b = std::min(b, seg.back());
    ok = b - a >= 0.05;
    if (!ok) {return m;}
    std::vector<double> samples{a};
    for (double t : seg) {if (a < t && t < b) {samples.push_back(t);}}
    samples.push_back(b);
    std::vector<std::array<std::array<double, 3>, 2>> pts;
    size_t i = 0;
    for (double sv : samples) {
      while (i + 2 < seg.size() && seg[i + 1] < sv) {++i;}
      const double span = seg[i + 1] - seg[i];
      const double f = span <= 0 ? 0.0 : (sv - seg[i]) / span;
      const auto & p0 = path[i];
      const auto & p1 = path[i + 1];
      const double x = p0.x + f * (p1.x - p0.x), y = p0.y + f * (p1.y - p0.y);
      const double zc = p0.z + f * (p1.z - p0.z);
      const double tx = p1.x - p0.x, ty = p1.y - p0.y;
      double n = std::hypot(tx, ty);
      if (n == 0.0) {n = 1.0;}
      const double nx = -ty / n, ny = tx / n;
      pts.push_back({{{x + nx * half_w, y + ny * half_w, zc + z}, {x - nx * half_w, y - ny * half_w, zc + z}}});
    }
    m.header.frame_id = "map"; m.header.stamp = stamp;
    m.ns = ns; m.id = mid; m.type = Marker::TRIANGLE_LIST; m.action = Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 1.0;
    m.color.r = rgba[0]; m.color.g = rgba[1]; m.color.b = rgba[2]; m.color.a = rgba[3];
    m.lifetime.sec = 0; m.lifetime.nanosec = 500000000;
    for (size_t k = 0; k + 1 < pts.size(); ++k) {
      const auto & l0 = pts[k][0]; const auto & r0 = pts[k][1];
      const auto & l1 = pts[k + 1][0]; const auto & r1 = pts[k + 1][1];
      for (const auto * q : {&l0, &r0, &l1, &r0, &r1, &l1}) {
        geometry_msgs::msg::Point pt; pt.x = (*q)[0]; pt.y = (*q)[1]; pt.z = (*q)[2];
        m.points.push_back(pt);
      }
    }
    return m;
  }

  void zone_markers(MarkerArray & out, double line, const std::string & label,
    const builtin_interfaces::msg::Time & stamp, int mid, double buffer_m)
  {
    const auto & path = core_->path();
    const auto & seg = core_->seg();
    const auto & p = core_->params();
    const double preset = p.limit_kmh <= 3.0 ? 2.0 : 4.0;
    const double cruise = preset;
    const double s_stop = core_->stopping_distance(std::max(core_->v(), cruise / 3.6));
    const double s_slow = core_->stopping_distance(preset / 3.6) + p.slow_zone;
    const double sb = seg[core_->idx()] + front_offset_;
    struct Z { double a, b; std::array<double, 4> rgba; const char * ns; double z; };
    const Z zones[3] = {
      {line - s_slow, line - s_stop, {1.0, 0.62, 0.0, 0.45}, "slow_zone", 0.07},
      {line - s_stop, line, {0.10, 0.35, 1.0, 0.60}, "stop_zone", 0.08},
      {line, line + buffer_m, {0.85, 0.10, 0.10, 0.40}, "keep_clear", 0.07}};
    for (const auto & z : zones) {
      bool ok = false;
      Marker r = ribbon_marker(std::max(z.a, sb), z.b, zone_hw_, z.z, z.rgba, mid, z.ns, stamp, ok);
      if (ok) {out.markers.push_back(r); ++mid;}
    }
    if (line > sb - 0.2 && line <= seg.back()) {
      size_t j = 0;
      double best = 1e300;
      for (size_t i = 0; i < seg.size(); ++i) {
        const double d = std::abs(seg[i] - line);
        if (d < best) {best = d; j = i;}
      }
      const size_t k = std::min(j, path.size() - 2);
      const double yaw = std::atan2(path[k + 1].y - path[k].y, path[k + 1].x - path[k].x);
      Marker w;
      w.header.frame_id = "map"; w.header.stamp = stamp;
      w.ns = "stop_wall"; w.id = mid; w.type = Marker::CUBE; w.action = Marker::ADD;
      w.pose.position.x = path[j].x; w.pose.position.y = path[j].y; w.pose.position.z = path[j].z + 0.6;
      w.pose.orientation.z = std::sin(yaw / 2); w.pose.orientation.w = std::cos(yaw / 2);
      w.scale.x = 0.12; w.scale.y = 2 * zone_hw_; w.scale.z = 1.2;
      w.color.r = 1.0; w.color.g = 0.1; w.color.b = 0.1; w.color.a = 0.85;
      w.lifetime.sec = 0; w.lifetime.nanosec = 500000000;
      out.markers.push_back(w);
      Marker t;
      t.header = w.header;
      t.ns = "stop_label"; t.id = mid + 1; t.type = Marker::TEXT_VIEW_FACING; t.action = Marker::ADD;
      t.pose.position.x = w.pose.position.x; t.pose.position.y = w.pose.position.y;
      t.pose.position.z = w.pose.position.z + 1.1;
      t.pose.orientation.w = 1.0;
      t.scale.z = 0.6;
      t.color.r = t.color.g = t.color.b = t.color.a = 1.0;
      char buf[256];
      std::snprintf(buf, sizeof(buf), "%s %.1f m", label.c_str(), std::max(line - sb, 0.0));
      t.text = buf;
      t.lifetime.sec = 0; t.lifetime.nanosec = 500000000;
      out.markers.push_back(t);
    }
  }

  void publish_zones()
  {
    if (!core_->path_ready()) {return;}
    core_->match();     // live: idx follows the last known rear-axle pose, like the Python node
    const auto stamp = now();
    MarkerArray arr;
    const auto obs = core_->obstacle_ahead(mono());
    const double sb = core_->seg()[core_->idx()] + front_offset_;
    int mid = 0;
    if (obs) {
      zone_markers(arr, sb + obs->dist - core_->params().stop_margin,
        "STOP: " + buggy_core::OperationCore::obs_name(*obs), stamp, mid, core_->params().stop_margin);
      mid += 10;
    }
    zone_markers(arr, core_->seg().back() - core_->params().goal_margin + front_offset_, "GOAL", stamp, mid, 0.0);
    zone_pub_->publish(arr);
  }

  std::unique_ptr<buggy_core::OperationCore> core_;
  double zone_hw_ = 1.5, wheelbase_ = 1.6, front_offset_ = 2.2;
  bool base_at_center_ = false;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_req_, sub_loc_, sub_uart_, sub_names_, sub_avoid_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<ground_segmentation_msgs::msg::SteerSpeed>::SharedPtr sub_wheel_;
  rclcpp::Subscription<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr sub_dummy_, sub_lidar_, sub_radar_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_calib_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr avoid_pub_, state_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr detour_pub_, zone_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steer_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr red_pub_, yellow_pub_;
  rclcpp::TimerBase::SharedPtr zone_timer_, step_timer_, state_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OperationManager>());
  rclcpp::shutdown();
  return 0;
}
