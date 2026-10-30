// Automatic start pose from GNSS: /chcnav/devpvt -> map pose -> /initialpose (see localization/gnss_init_core.hpp).
// Needs a georeferenced map (map_projector_info.yaml LocalCartesian) and a GNSS with RTK fixed + dual-antenna heading.
#include <cstdio>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <msg_interfaces/msg/hcinspvatzcb.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "buggy_core/control/operation_core.hpp"      // parse_kv
#include "buggy_core/localization/gnss_init_core.hpp"

using geometry_msgs::msg::PoseWithCovarianceStamped;

class GnssPoseInitializer : public rclcpp::Node
{
public:
  GnssPoseInitializer()
  : Node("gnss_pose_initializer")
  {
    buggy_core::GnssInitParams p;
    const auto states = declare_parameter<std::vector<int64_t>>("accepted_states", std::vector<int64_t>{4});
    p.accepted_states.assign(states.begin(), states.end());
    p.max_std_m = declare_parameter<double>("max_std_m", 0.3);
    p.stable_sec = declare_parameter<double>("stable_sec", 3.0);
    p.fix_timeout = declare_parameter<double>("fix_timeout_sec", 1.0);
    p.retry_sec = declare_parameter<double>("retry_sec", 20.0);
    p.max_attempts = static_cast<int>(declare_parameter<int>("max_attempts", 3));
    p.reinit_when_lost = declare_parameter<bool>("reinit_when_lost", true);
    p.lost_sec = declare_parameter<double>("lost_sec", 5.0);
    p.lever_x = declare_parameter<double>("gnss_to_rear_axle_x", 0.0);
    p.lever_y = declare_parameter<double>("gnss_to_rear_axle_y", 0.0);
    p.heading_offset_deg = declare_parameter<double>("heading_offset_deg", 0.0);
    const auto proj_path = declare_parameter<std::string>("map_projector_info_path", "");
    buggy_core::Projector proj;
    try {
      proj = buggy_core::Projector::load(proj_path);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(get_logger(), "%s", e.what());
      throw;
    }
    core_ = std::make_unique<buggy_core::GnssInitCore>(p, proj);
    core_->set_enabled(declare_parameter<bool>("enabled", true));
    if (!core_->usable()) {RCLCPP_WARN(get_logger(), "%s", core_->status().c_str());}

    init_pub_ = create_publisher<PoseWithCovarianceStamped>("/initialpose", 1);
    status_pub_ = create_publisher<std_msgs::msg::String>("/gnss_init/state", 1);
    sub_fix_ = create_subscription<msg_interfaces::msg::Hcinspvatzcb>("/chcnav/devpvt", rclcpp::SensorDataQoS(),
        [this](msg_interfaces::msg::Hcinspvatzcb::ConstSharedPtr m) {
          buggy_core::GnssFix f;
          f.lat = m->latitude; f.lon = m->longitude;
          f.heading_cw_deg = m->heading2;
          f.state = m->stat[1];
          f.std_m = std::max(m->position_stdev[0], m->position_stdev[1]);
          f.time = now_s();
          core_->on_fix(f);
        });
    // "enable" | "disable" | "toggle" (the RViz panel button)
    sub_req_ = create_subscription<std_msgs::msg::String>("/gnss_init/request", 10,
        [this](std_msgs::msg::String::ConstSharedPtr m) {
          if (m->data == "enable") {core_->set_enabled(true);}
          else if (m->data == "disable") {core_->set_enabled(false);}
          else if (m->data == "toggle") {core_->set_enabled(!core_->enabled());}
          RCLCPP_WARN(get_logger(), "GNSS auto start pose %s (request '%s')", core_->enabled() ? "ENABLED" : "DISABLED", m->data.c_str());
        });
    sub_loc_ = create_subscription<std_msgs::msg::String>("/localization/state", 1,
        [this](std_msgs::msg::String::ConstSharedPtr m) {
          const auto kv = buggy_core::parse_kv(m->data);
          const auto it = kv.find("state");
          loc_state_ = it == kv.end() ? "uninitialized" : it->second;
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() {step();});
  }

private:
  double now_s() {return get_clock()->now().seconds();}

  void step()
  {
    const auto init = core_->step(now_s(), loc_state_);
    if (init) {
      PoseWithCovarianceStamped m;
      m.header.frame_id = "map";
      m.header.stamp = get_clock()->now();
      m.pose.pose.position.x = init->x;
      m.pose.pose.position.y = init->y;
      m.pose.pose.orientation.z = std::sin(init->yaw / 2.0);
      m.pose.pose.orientation.w = std::cos(init->yaw / 2.0);
      m.pose.covariance[0] = m.pose.covariance[7] = std::max(init->std_m * init->std_m, 0.01);
      m.pose.covariance[35] = 0.01;
      init_pub_->publish(m);
      RCLCPP_WARN(get_logger(), "GNSS start pose -> /initialpose: x %.2f y %.2f yaw %.1f deg (%s)", init->x, init->y,
        init->yaw * 180.0 / M_PI, core_->status().c_str());
    }
    if (core_->status() != last_status_) {
      last_status_ = core_->status();
      RCLCPP_INFO(get_logger(), "%s", last_status_.c_str());
    }
    std_msgs::msg::String s;
    std::string why;
    const bool good = core_->fix_good(now_s(), &why);
    const auto & f = core_->fix();
    char b[256];
    std::snprintf(b, sizeof(b), ";gnss_state=%d;std_m=%.3f;heading=%.1f;fix_age=%.1f;fix_ok=%d;usable=%d;enabled=%d",
      f ? f->state : -1, f ? f->std_m : -1.0, f ? f->heading_cw_deg : -1.0, f ? now_s() - f->time : -1.0,
      good ? 1 : 0, core_->usable() ? 1 : 0, core_->enabled() ? 1 : 0);
    s.data = "status=" + core_->status() + ";attempts=" + std::to_string(core_->attempts()) + ";localization=" + loc_state_ + b +
      ";why=" + (good ? std::string("-") : why);
    status_pub_->publish(s);
  }

  std::unique_ptr<buggy_core::GnssInitCore> core_;
  std::string loc_state_ = "uninitialized", last_status_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr init_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<msg_interfaces::msg::Hcinspvatzcb>::SharedPtr sub_fix_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_loc_, sub_req_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GnssPoseInitializer>());
  rclcpp::shutdown();
  return 0;
}
