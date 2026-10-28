// Mission manager: drives a list of named stops (home, A, B ...) one after the other and waits
// wait_sec at each. All logic is in control/mission_core.cpp; this file connects the topics.
//
//   in : /system/state (operation manager)  /planning/route_path  /mission/request
//        (start | start:N | stop | resume)  TF map->base_link
//   out: /goal_pose (the next stop, to the route planner)  /system/request ("mode:auto" - the operator's
//        AUTO button)  /mission/state  /mission/markers (RViz)
// The stops come from <map>/mission.yaml: the launch file turns it into the stop_* parameters.
// Works with the Python and the C++ route planner / operation manager (only topics are used).
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "buggy_core/control/mission_core.hpp"
#include "buggy_core/control/operation_core.hpp"      // parse_kv

using std_msgs::msg::String;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

class MissionManager : public rclcpp::Node
{
public:
  MissionManager()
  : Node("mission_manager"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_, this, false)
  {
    const auto names = declare_parameter<std::vector<std::string>>("stop_names", std::vector<std::string>{});
    const auto xs = declare_parameter<std::vector<double>>("stop_x", std::vector<double>{});
    const auto ys = declare_parameter<std::vector<double>>("stop_y", std::vector<double>{});
    const auto waits = declare_parameter<std::vector<double>>("stop_wait_sec", std::vector<double>{});
    buggy_core::MissionParams p;
    p.loop = declare_parameter<bool>("loop", true);
    p.auto_restart = declare_parameter<bool>("auto_restart", true);
    p.plan_timeout = declare_parameter<double>("plan_timeout_sec", 5.0);
    p.abort_stop_sec = declare_parameter<double>("abort_stop_sec", 2.0);
    p.auto_grace = declare_parameter<double>("auto_grace_sec", 3.0);
    p.at_stop_radius = declare_parameter<double>("at_stop_radius", 4.0);
    p.start_as_home = declare_parameter<bool>("start_as_home", false);
    autostart_ = declare_parameter<bool>("autostart", false);
    const size_t n = std::min({names.size(), xs.size(), ys.size(), waits.size()});
    if (n != names.size() || n != xs.size() || n != ys.size() || n != waits.size()) {
      RCLCPP_ERROR(get_logger(), "stop_names / stop_x / stop_y / stop_wait_sec differ in length: using the first %zu", n);
    }
    for (size_t i = 0; i < n; ++i) {p.stops.push_back({names[i], xs[i], ys[i], waits[i]});}
    core_ = std::make_unique<buggy_core::MissionCore>(p);

    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    req_pub_ = create_publisher<String>("/system/request", 10);
    state_pub_ = create_publisher<String>("/mission/state", 10);
    marker_pub_ = create_publisher<MarkerArray>("/mission/markers", 1);
    sub_req_ = create_subscription<String>("/mission/request", 10,
        [this](String::ConstSharedPtr m) {core_->on_request(m->data);});
    sub_state_ = create_subscription<String>("/system/state", 10,
        [this](String::ConstSharedPtr m) {
          const auto kv = buggy_core::parse_kv(m->data);
          auto get = [&kv](const char * k) {const auto it = kv.find(k); return it == kv.end() ? std::string() : it->second;};
          mode_ = get("mode"); routing_ = get("routing"); msg_ = get("msg");
          state_time_ = now_s();
        });
    sub_path_ = create_subscription<nav_msgs::msg::Path>("/planning/route_path", 10,
        [this](nav_msgs::msg::Path::ConstSharedPtr) {new_path_ = true;});
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {step();});
    marker_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {publish_markers();});
    RCLCPP_INFO(get_logger(), "mission with %zu stops (loop %s): %s", n, p.loop ? "on" : "off",
      autostart_ ? "starts by itself" : "send 'start' to /mission/request (./buggy mission start)");
  }

private:
  double now_s() {return get_clock()->now().seconds();}

  std::optional<std::pair<double, double>> pose()
  {
    try {
      const auto t = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
      return std::make_pair(t.transform.translation.x, t.transform.translation.y);
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }

  void step()
  {
    const double now = now_s();
    buggy_core::MissionInput in;
    in.now = now;
    in.mode = mode_; in.routing = routing_; in.msg = msg_;
    in.state_fresh = state_time_ > 0.0 && now - state_time_ < 2.0;
    in.new_path = new_path_;
    new_path_ = false;
    in.pose = pose();
    if (autostart_ && !autostarted_ && in.state_fresh && in.pose && core_->phase() == buggy_core::MissionPhase::Idle) {
      autostarted_ = true;
      core_->on_request("start");
    }
    const auto a = core_->step(in);
    if (a.send_goal) {
      geometry_msgs::msg::PoseStamped g;
      g.header.frame_id = "map";
      g.header.stamp = get_clock()->now();
      g.pose.position.x = a.send_goal->x;
      g.pose.position.y = a.send_goal->y;
      g.pose.orientation.w = 1.0;
      goal_pub_->publish(g);
      RCLCPP_INFO(get_logger(), "goal: %s (%.1f, %.1f)", a.send_goal->name.c_str(), a.send_goal->x, a.send_goal->y);
    }
    if (a.request_auto) {send_request("mode:auto");}
    if (a.request_stop) {send_request("mode:stop");}
    const auto text = core_->state_string(now);
    String s;
    s.data = text;
    state_pub_->publish(s);
    if (core_->message() != last_msg_) {
      last_msg_ = core_->message();
      RCLCPP_INFO(get_logger(), "%s", text.c_str());
    }
  }

  void send_request(const std::string & r)
  {
    String m;
    m.data = r;
    req_pub_->publish(m);
  }

  void publish_markers()
  {
    MarkerArray arr;
    Marker clear;
    clear.action = Marker::DELETEALL;
    arr.markers.push_back(clear);
    const auto & stops = core_->stops();
    const auto phase = core_->phase();
    const bool active = phase != buggy_core::MissionPhase::Idle && phase != buggy_core::MissionPhase::Done &&
      phase != buggy_core::MissionPhase::Aborted;
    for (size_t i = 0; i < stops.size(); ++i) {
      const bool target = active && i == core_->target_index();
      Marker m;
      m.header.frame_id = "map";
      m.header.stamp = get_clock()->now();
      m.ns = "mission_stop"; m.id = static_cast<int>(i);
      m.type = Marker::CYLINDER; m.action = Marker::ADD;
      m.pose.position.x = stops[i].x; m.pose.position.y = stops[i].y; m.pose.position.z = 0.1;
      m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = 1.6; m.scale.z = 0.2;
      m.color.r = target ? 0.1f : 1.0f; m.color.g = target ? 0.9f : 0.8f; m.color.b = 0.1f; m.color.a = 0.85f;
      arr.markers.push_back(m);
      Marker t = m;
      t.ns = "mission_label"; t.type = Marker::TEXT_VIEW_FACING;
      t.pose.position.z = 2.0;
      t.scale.x = t.scale.y = 0.0; t.scale.z = 0.9;
      t.color.r = t.color.g = t.color.b = 1.0f; t.color.a = 1.0f;
      char b[96];
      std::snprintf(b, sizeof(b), "%zu: %s (%.0f s)", i + 1, stops[i].name.c_str(), stops[i].wait_sec);
      t.text = b;
      arr.markers.push_back(t);
    }
    marker_pub_->publish(arr);
  }

  std::unique_ptr<buggy_core::MissionCore> core_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string mode_, routing_, msg_, last_msg_;
  double state_time_ = 0.0;
  bool new_path_ = false, autostart_ = false, autostarted_ = false;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<String>::SharedPtr req_pub_, state_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<String>::SharedPtr sub_req_, sub_state_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::TimerBase::SharedPtr timer_, marker_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionManager>());
  rclcpp::shutdown();
  return 0;
}
