// C++ version of buggy_pcd_lanelet_map/route_planner.py: RViz goal -> smoothed lane path, HOME and
// loop requests. Same topics, QoS, parameters and status texts; the routing itself is in
// planning/route_core.cpp.
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "buggy_core/planning/route_core.hpp"

using buggy_core::Poly;
using buggy_core::Pt;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace
{
std::string fmt(const char * f, ...)
{
  char buf[512];
  va_list a;
  va_start(a, f);
  std::vsnprintf(buf, sizeof(buf), f, a);
  va_end(a);
  return buf;
}

std::vector<std::string> dedupe(const std::vector<std::string> & v)
{
  std::vector<std::string> out;
  std::set<std::string> seen;
  for (const auto & s : v) {if (seen.insert(s).second) {out.push_back(s);}}
  return out;
}
}  // namespace

class RoutePlanner : public rclcpp::Node
{
public:
  RoutePlanner()
  : Node("route_planner"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_, this, false)
  {
    frame_ = declare_parameter<std::string>("map_frame", "map");
    buggy_core::RouteParams rp;
    rp.wheelbase = declare_parameter<double>("wheelbase", 1.6);
    rp.max_steer_deg = declare_parameter<double>("max_steering_deg", 20.0);
    rp.steering_margin = declare_parameter<double>("steering_margin", 0.9);
    rp.smoothing_max_deviation = declare_parameter<double>("smoothing_max_deviation", 0.6);
    const std::string path = declare_parameter<std::string>("lanelet2_map_path", "");
    rp.link_gap = declare_parameter<double>("link_gap", 4.0);
    rp.link_angle_deg = declare_parameter<double>("link_angle_deg", 60.0);
    rp.home_reach = declare_parameter<double>("home_reach", 2.5);
    core_ = std::make_unique<buggy_core::RouteCore>(buggy_core::load_lanelets(path), rp);
    RCLCPP_INFO(get_logger(), "%zu lanelets loaded", core_->lanelets().size());
    if (!core_->linked().empty()) {
      std::string pairs;
      for (const auto & p : core_->linked()) {pairs += (pairs.empty() ? "" : ", ") + std::string("('") + p.first + "', '" + p.second + "')";}
      RCLCPP_INFO(get_logger(), "linked %zu lane end(s) to a lane start by position (closed loop / unshared nodes): [%s]",
        core_->linked().size(), pairs.c_str());
    }
    rclcpp::QoS qos(1);
    qos.transient_local().reliable();
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/planning/route_path", qos);
    marker_pub_ = create_publisher<MarkerArray>("/planning/route_marker", qos);
    status_pub_ = create_publisher<std_msgs::msg::String>("/planning/route_status", qos);
    status("No route (use 2D Goal Pose)");
    home_pub_ = create_publisher<MarkerArray>("/planning/home_marker", qos);
    using std_msgs::msg::String;
    sub_req_ = create_subscription<String>("/system/request", 10, [this](String::ConstSharedPtr m) { on_request(m->data); });
    sub_clear_ = create_subscription<std_msgs::msg::Empty>("/planning/clear_route", 10,
        [this](std_msgs::msg::Empty::ConstSharedPtr) { clear(); });
    sub_init_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10,
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m) { on_init(*m); });
    sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) { on_goal(*m); });
    sub_goal2_ = create_subscription<geometry_msgs::msg::PoseStamped>("/planning/mission_planning/goal", 10,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) { on_goal(*m); });
    RCLCPP_INFO(get_logger(), "waiting for goal (RViz: 2D Goal Pose)");
  }

private:
  struct Pose2 { double x, y, yaw; };

  void status(const std::string & t)
  {
    std_msgs::msg::String m; m.data = t; status_pub_->publish(m);
  }

  std_msgs::msg::Header hdr()
  {
    std_msgs::msg::Header h; h.frame_id = frame_; h.stamp = now(); return h;
  }

  std::optional<Pose2> current_pose()
  {
    try {
      const auto t = tf_buffer_.lookupTransform(frame_, "base_link", tf2::TimePointZero);
      const auto & q = t.transform.rotation;
      return Pose2{t.transform.translation.x, t.transform.translation.y,
          std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))};
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }

  std::optional<std::pair<double, double>> current_xy()
  {
    if (const auto p = current_pose()) {return std::make_pair(p->x, p->y);}
    return start_pose_;
  }

  void on_init(const geometry_msgs::msg::PoseWithCovarianceStamped & m)
  {
    start_pose_ = std::make_pair(m.pose.pose.position.x, m.pose.pose.position.y);
    clear();
    clear_home();                 // a new initial pose starts a new session
  }

  static double length(const Poly & p) { return buggy_core::length3(p); }

  // ---- HOME -----------------------------------------------------------------------------
  geometry_msgs::msg::Pose home_pose_msg() const
  {
    geometry_msgs::msg::Pose p;
    p.position.x = home_->x; p.position.y = home_->y;
    p.orientation.z = std::sin(home_->yaw / 2); p.orientation.w = std::cos(home_->yaw / 2);
    return p;
  }

  bool set_home()
  {
    const auto cur = current_pose();
    if (!cur) {status("Give an initial pose first"); return false;}
    home_ = cur;
    Marker m;
    m.header = hdr();
    m.ns = "home"; m.id = 0; m.type = Marker::CYLINDER; m.action = Marker::ADD;
    m.pose = home_pose_msg();
    m.pose.position.z = 0.05;
    m.scale.x = m.scale.y = 2 * core_->params().home_reach;
    m.scale.z = 0.1;
    m.color.r = 0.1f; m.color.g = 0.9f; m.color.b = 0.3f; m.color.a = 0.55f;
    Marker t;
    t.header = m.header;
    t.ns = "home_label"; t.id = 1; t.type = Marker::TEXT_VIEW_FACING; t.action = Marker::ADD;
    t.pose.position.x = cur->x; t.pose.position.y = cur->y; t.pose.position.z = 1.5;
    t.pose.orientation.w = 1.0;
    t.scale.z = 0.8;
    t.color.r = t.color.g = t.color.b = t.color.a = 1.0;
    t.text = "HOME";
    MarkerArray a; a.markers = {m, t};
    home_pub_->publish(a);
    RCLCPP_INFO(get_logger(), "HOME set at (%.1f, %.1f)", cur->x, cur->y);
    return true;
  }

  void clear_home()
  {
    if (home_) {
      home_.reset();
      Marker m; m.action = Marker::DELETEALL;
      MarkerArray a; a.markers = {m};
      home_pub_->publish(a);
    }
  }

  void on_request(const std::string & data)
  {
    const auto c = data.find(':');
    const std::string key = data.substr(0, c);
    const std::string val = c == std::string::npos ? "" : data.substr(c + 1);
    if (key == "mode" && val == "auto" && !home_) {
      set_home();                                  // HOME = where AUTO was first engaged
    } else if (key == "home" && val == "set") {
      set_home();
    } else if (key == "home" && val == "clear") {
      clear_home();
      status("HOME cleared");
    } else if (key == "home" && val == "go") {
      go_home();
    } else if (key == "loop") {
      char * end = nullptr;
      const double d = std::strtod(val.c_str(), &end);
      if (end == val.c_str()) {status("loop: bad lap count \"" + val + "\""); return;}
      go_loop(std::max(1, static_cast<int>(d)));
    }
  }

  void go_home()
  {
    const auto cur = current_pose();
    if (!cur || !home_) {status("No HOME yet: press Set home (or engage AUTO once) first"); return;}
    if (std::hypot(cur->x - home_->x, cur->y - home_->y) < core_->params().home_reach) {status("Already at HOME"); return;}
    const auto res = core_->plan(cur->x, cur->y, home_->x, home_->y);
    if (!res) {status("No route home"); return;}
    const Poly raw = core_->chain_points(res->chain, res->si, res->gi);
    const auto sm = core_->smooth_pts(raw);
    publish(dedupe(res->chain), sm.pts, home_pose_msg(), raw);
    status(fmt("Back to HOME: %.0f m, peak steer %.0f -> %.0f deg", length(sm.pts), sm.before, sm.after));
  }

  void go_loop(int laps)
  {
    const auto cur = current_pose();
    if (!cur) {status("Give an initial pose first"); return;}
    if (!home_ && !set_home()) {return;}
    const bool at_home = std::hypot(cur->x - home_->x, cur->y - home_->y) < core_->params().home_reach;
    std::vector<Poly> pieces;
    Poly raw_all;
    std::vector<std::string> chains;
    double before = 0.0, after = 0.0;
    int more = laps;
    if (!at_home) {
      const auto res = core_->plan(cur->x, cur->y, home_->x, home_->y);
      if (!res) {status("No route back to HOME"); return;}
      const Poly raw = core_->chain_points(res->chain, res->si, res->gi);
      const auto sm = core_->smooth_pts(raw);
      pieces.push_back(sm.pts);
      raw_all.insert(raw_all.end(), raw.begin(), raw.end());
      chains.insert(chains.end(), res->chain.begin(), res->chain.end());
      before = std::max(before, sm.before); after = std::max(after, sm.after);
      more = laps - 1;
    }
    if (more > 0) {
      const auto lap = core_->lap_points(home_->x, home_->y);
      if (!lap) {status("Not a closed loop: the lanes do not lead back to HOME"); return;}
      const auto sm = core_->smooth_pts(lap->pts);
      for (int i = 0; i < more; ++i) {
        pieces.push_back(sm.pts);
        raw_all.insert(raw_all.end(), lap->pts.begin(), lap->pts.end());
        chains.insert(chains.end(), lap->chain.begin(), lap->chain.end());
      }
      before = std::max(before, sm.before); after = std::max(after, sm.after);
    }
    Poly pts;
    for (const auto & piece : pieces) {
      if (pts.empty() || buggy_core::dist3(pts.back(), piece.front()) > 0.3) {
        pts.insert(pts.end(), piece.begin(), piece.end());
      } else {
        pts.insert(pts.end(), piece.begin() + 1, piece.end());
      }
    }
    if (pts.size() < 2) {status("Already at HOME"); return;}
    publish(dedupe(chains), pts, home_pose_msg(), raw_all);
    status(fmt("Loop x%d: %.0f m, back to HOME at the end; peak steer %.0f -> %.0f deg", laps, length(pts), before, after));
    RCLCPP_INFO(get_logger(), "loop route: %d lap(s), %.0f m", laps, length(pts));
  }

  void clear()
  {
    status("No route (use 2D Goal Pose)");
    nav_msgs::msg::Path p; p.header = hdr();
    path_pub_->publish(p);
    Marker m; m.action = Marker::DELETEALL;
    MarkerArray a; a.markers = {m};
    marker_pub_->publish(a);
  }

  void on_goal(const geometry_msgs::msg::PoseStamped & msg)
  {
    const auto start = current_xy();
    if (!start) {
      status("Give an initial pose first");
      RCLCPP_WARN(get_logger(), "no start pose yet: give an initial pose first");
      return;
    }
    const double gx = msg.pose.position.x, gy = msg.pose.position.y;
    const auto res = core_->plan(start->first, start->second, gx, gy);
    if (!res) {
      status("No route to goal");
      RCLCPP_WARN(get_logger(), "no route to goal");
      return;
    }
    const auto & chain = res->chain;
    const Poly pts = core_->chain_points(chain, res->si, res->gi);
    const auto sm = core_->smooth_pts(pts);
    publish(dedupe(chain), sm.pts, msg.pose, pts);
    const double len = length(sm.pts);
    const double limit = core_->params().steer_limit();
    std::string info = fmt("%zu lanelets, %.1f m, peak steer %.0f -> %.0f deg (limit %.0f)", chain.size(), len, sm.before, sm.after, limit);
    const double lane_d = buggy_core::project(core_->nearest(gx, gy), gx, gy).second;
    if (lane_d > 6.0) {info += fmt("  [goal is %.0f m from the lane: snapped to it]", lane_d);}
    if (chain.size() > 1 && chain.front() == chain.back()) {info += "  [goal is behind: around the loop]";}
    status(info + (sm.after <= limit + 0.5 ? "" : "  [TOO TIGHT]"));
    RCLCPP_INFO(get_logger(), "route: %s", info.c_str());
  }

  void publish(const std::vector<std::string> & chain, const Poly & pts, const geometry_msgs::msg::Pose & goal_pose, const Poly & raw_pts)
  {
    const auto h = hdr();
    nav_msgs::msg::Path path;
    path.header = h;
    for (size_t i = 0; i < pts.size(); ++i) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = h;
      ps.pose.position.x = pts[i][0]; ps.pose.position.y = pts[i][1]; ps.pose.position.z = pts[i][2];
      const Pt & a = i + 1 < pts.size() ? pts[i] : pts[std::max<size_t>(i, 1) - 1];
      const Pt & b = i + 1 < pts.size() ? pts[i + 1] : pts[i];
      const double yaw = std::atan2(b[1] - a[1], b[0] - a[0]);
      ps.pose.orientation.z = std::sin(yaw / 2); ps.pose.orientation.w = std::cos(yaw / 2);
      path.poses.push_back(ps);
    }
    path_pub_->publish(path);

    MarkerArray arr;
    const auto marker = [&](int id, const char * ns, int type) {
        Marker m;
        m.header = h; m.ns = ns; m.id = id; m.type = type; m.action = Marker::ADD;
        m.pose.orientation.w = 1.0;
        return m;
      };
    const auto point = [](const Pt & p, double dz) {
        geometry_msgs::msg::Point q; q.x = p[0]; q.y = p[1]; q.z = p[2] + dz; return q;
      };
    Marker fill = marker(0, "route_lanelets", Marker::TRIANGLE_LIST);
    fill.scale.x = fill.scale.y = fill.scale.z = 1.0;
    fill.color.g = 1.0; fill.color.a = 0.35;
    for (const auto & lid : chain) {
      const auto & l = core_->lanelet(lid);
      for (size_t i = 0; i + 1 < l.left.size(); ++i) {
        for (const Pt * p : {&l.left[i], &l.right[i], &l.left[i + 1], &l.right[i], &l.right[i + 1], &l.left[i + 1]}) {
          fill.points.push_back(point(*p, 0.05));
        }
      }
    }
    arr.markers.push_back(fill);
    Marker line = marker(1, "route_path", Marker::LINE_STRIP);
    line.scale.x = 0.3;
    line.color.r = 1.0; line.color.g = 0.9; line.color.b = 0.0; line.color.a = 1.0;
    for (const auto & p : pts) {line.points.push_back(point(p, 0.1));}
    arr.markers.push_back(line);
    Marker raw = marker(3, "route_path_raw", Marker::LINE_STRIP);
    raw.scale.x = 0.08;
    raw.color.r = raw.color.g = raw.color.b = 0.6; raw.color.a = 0.8;
    for (const auto & p : raw_pts) {raw.points.push_back(point(p, 0.08));}
    arr.markers.push_back(raw);
    Marker goal = marker(2, "goal", Marker::ARROW);
    goal.pose = goal_pose;
    goal.scale.x = 2.0; goal.scale.y = 0.4; goal.scale.z = 0.4;
    goal.color.r = 1.0; goal.color.a = 1.0;
    arr.markers.push_back(goal);
    marker_pub_->publish(arr);
  }

  std::string frame_;
  std::unique_ptr<buggy_core::RouteCore> core_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::optional<std::pair<double, double>> start_pose_;
  std::optional<Pose2> home_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_, home_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_req_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_clear_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_init_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_, sub_goal2_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoutePlanner>());
  rclcpp::shutdown();
  return 0;
}
