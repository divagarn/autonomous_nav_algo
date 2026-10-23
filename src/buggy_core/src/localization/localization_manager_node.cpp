// C++ version of buggy_pcd_lanelet_map/localization_manager.py: RViz 2D Pose Estimate ->
// ndt_align_srv -> seed the EKF -> activate EKF and NDT, then supervise the NDT result and publish
// /localization/state. The state machine and the TF watchdog are in localization_core.cpp.
#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/int32_stamped.hpp>
#include <autoware_internal_localization_msgs/srv/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "buggy_core/localization/localization_core.hpp"

using AlignSrv = autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using std_srvs::srv::SetBool;

class LocalizationManager : public rclcpp::Node
{
public:
  LocalizationManager()
  : Node("localization_manager"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_, this, false)
  {
    buggy_core::LocParams p;
    p.map_frame = declare_parameter<std::string>("map_frame", "map");
    p.base_frame = declare_parameter<std::string>("base_frame", "base_link");
    p.score_ok = declare_parameter<double>("score_ok", 1.4);
    p.lost_timeout = declare_parameter<double>("lost_timeout_sec", 3.0);
    align_timeout_ = declare_parameter<double>("align_timeout_sec", 40.0);
    max_retries_ = static_cast<int>(declare_parameter<int>("max_retries", 2));
    ekf_settle_ = declare_parameter<double>("ekf_settle_sec", 1.5);
    p.jump_m = declare_parameter<double>("jump_m", 0.5);
    map_frame_ = p.map_frame; base_frame_ = p.base_frame;
    core_ = std::make_unique<buggy_core::LocalizationCore>(p);

    rclcpp::QoS qos(1);
    qos.transient_local().reliable().keep_last(1);
    seed_pub_ = create_publisher<PoseWithCovarianceStamped>("/initialpose3d", qos);
    state_pub_ = create_publisher<std_msgs::msg::String>("/localization/state", 1);
    align_ = create_client<AlignSrv>("/ndt_align_srv");
    ekf_trigger_ = create_client<SetBool>("/ekf_localizer/trigger_node");
    ndt_trigger_ = create_client<SetBool>("/ndt_scan_matcher/trigger_node");

    sub_click_ = create_subscription<PoseWithCovarianceStamped>("/initialpose", 10,
        [this](PoseWithCovarianceStamped::ConstSharedPtr m) { on_click(*m); });
    sub_ndt_ = create_subscription<PoseWithCovarianceStamped>("/ndt_pose_with_covariance", 10,
        [this](PoseWithCovarianceStamped::ConstSharedPtr) { std::lock_guard<std::mutex> l(mu_); core_->on_ndt_pose(t_now()); });
    sub_score_ = create_subscription<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "/nearest_voxel_transformation_likelihood", 10,
        [this](autoware_internal_debug_msgs::msg::Float32Stamped::ConstSharedPtr m) { std::lock_guard<std::mutex> l(mu_); core_->set_score(m->data); });
    sub_iter_ = create_subscription<autoware_internal_debug_msgs::msg::Int32Stamped>("/iteration_num", 10,
        [this](autoware_internal_debug_msgs::msg::Int32Stamped::ConstSharedPtr m) { std::lock_guard<std::mutex> l(mu_); core_->set_iterations(m->data); });
    sub_exe_ = create_subscription<autoware_internal_debug_msgs::msg::Float32Stamped>("/exe_time_ms", 10,
        [this](autoware_internal_debug_msgs::msg::Float32Stamped::ConstSharedPtr m) { std::lock_guard<std::mutex> l(mu_); core_->set_exe_ms(m->data); });
    sub_tf_ = create_subscription<tf2_msgs::msg::TFMessage>("/tf", 200,
        [this](tf2_msgs::msg::TFMessage::ConstSharedPtr m) { on_tf(*m); });
    timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(200), [this]() { publish_state(); });
  }

  ~LocalizationManager() override
  {
    stopping_ = true;
    if (thread_.joinable()) {thread_.join();}
  }

private:
  double t_now() { return get_clock()->now().nanoseconds() * 1e-9; }

  void on_tf(const tf2_msgs::msg::TFMessage & msg)
  {
    std::vector<buggy_core::TfEdge> edges;
    for (const auto & t : msg.transforms) {
      edges.push_back({t.child_frame_id, t.header.frame_id, t.transform.translation.x, t.transform.translation.y});
    }
    std::lock_guard<std::mutex> l(mu_);
    core_->on_tf(edges, t_now());
  }

  void on_click(const PoseWithCovarianceStamped & msg)
  {
    std::lock_guard<std::mutex> l(mu_);
    if (busy_) {
      RCLCPP_WARN(get_logger(), "alignment already running, click ignored");
      return;
    }
    busy_ = true;
    core_->set_state("aligning", "NDT aligning...");
    if (thread_.joinable()) {thread_.join();}
    thread_ = std::thread([this, msg]() { run_align(msg); });
  }

  template<typename Future>
  bool wait(Future & f, double timeout)
  {
    double t = 0.0;
    while (t < timeout && !stopping_) {
      if (f.wait_for(std::chrono::milliseconds(50)) == std::future_status::ready) {return true;}
      t += 0.05;
    }
    return f.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
  }

  bool trigger(const rclcpp::Client<SetBool>::SharedPtr & client, const std::string & name)
  {
    if (!client->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "%s trigger service not available", name.c_str());
      return false;
    }
    auto req = std::make_shared<SetBool::Request>();
    req->data = true;
    auto fut = client->async_send_request(req);
    bool ok = false;
    if (wait(fut.future, 5.0)) {ok = fut.future.get()->success;}
    if (ok) {RCLCPP_INFO(get_logger(), "%s activated", name.c_str());} else {RCLCPP_INFO(get_logger(), "%s trigger failed", name.c_str());}
    return ok;
  }

  void run_align(PoseWithCovarianceStamped guess)
  {
    struct Done { LocalizationManager * s; ~Done() { std::lock_guard<std::mutex> l(s->mu_); s->busy_ = false; } } done{this};
    if (guess.header.frame_id.empty()) {guess.header.frame_id = map_frame_;}
    // generous search uncertainty (2 m, ~17 deg) around the click
    guess.pose.covariance[0] = guess.pose.covariance[7] = 4.0;
    guess.pose.covariance[35] = 0.09;
    std::optional<PoseWithCovarianceStamped> aligned;
    for (int attempt = 1; attempt <= max_retries_ && !stopping_; ++attempt) {
      if (!align_->wait_for_service(std::chrono::seconds(2))) {
        set_msg("ndt_align_srv not available (is ndt_scan_matcher up?)");
        continue;
      }
      auto req = std::make_shared<AlignSrv::Request>();
      req->pose_with_covariance = guess;
      req->pose_with_covariance.header.stamp = now();
      auto fut = align_->async_send_request(req);
      if (wait(fut.future, align_timeout_)) {
        const auto r = fut.future.get();
        if (r->success) {
          aligned = r->pose_with_covariance;
          RCLCPP_INFO(get_logger(), "aligned (attempt %d, reliable=%s)", attempt, r->reliable ? "True" : "False");
          break;
        }
      }
      char buf[96];
      std::snprintf(buf, sizeof(buf), "align attempt %d/%d failed", attempt, max_retries_);
      set_msg(buf);
      RCLCPP_WARN(get_logger(), "%s", buf);
      sleep_s(1.0);
    }
    if (!aligned) {
      std::lock_guard<std::mutex> l(mu_);
      core_->set_state("align_failed", "Alignment failed: click closer to the real position, check LiDAR");
      return;
    }
    aligned->header.stamp = now();
    if (aligned->header.frame_id.empty()) {aligned->header.frame_id = map_frame_;}
    seed_pub_->publish(*aligned);
    bool activated;
    {
      std::lock_guard<std::mutex> l(mu_);
      core_->reset_ndt();
      core_->set_state("initialized", "Seed sent, waiting for NDT");
      activated = core_->activated();
    }
    if (!activated) {
      if (trigger(ekf_trigger_, "ekf_localizer")) {
        sleep_s(ekf_settle_);
        activated = trigger(ndt_trigger_, "ndt_scan_matcher");
        std::lock_guard<std::mutex> l(mu_);
        core_->set_activated(activated);
      }
      if (!activated) {
        std::lock_guard<std::mutex> l(mu_);
        core_->set_state("align_failed", "Could not activate EKF/NDT");
      }
    }
  }

  void set_msg(const std::string & m) { std::lock_guard<std::mutex> l(mu_); core_->set_msg(m); }
  void sleep_s(double s)
  {
    for (double t = 0; t < s && !stopping_; t += 0.05) {std::this_thread::sleep_for(std::chrono::milliseconds(50));}
  }

  void publish_state()
  {
    std::optional<buggy_core::MapPose> pose;
    try {
      const auto t = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      const auto & q = t.transform.rotation;
      pose = buggy_core::MapPose{t.transform.translation.x, t.transform.translation.y,
          std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) * 180.0 / M_PI};
    } catch (const tf2::TransformException &) {
    }
    std_msgs::msg::String m;
    {
      std::lock_guard<std::mutex> l(mu_);
      const double now_s = t_now();
      m.data = core_->state_string(now_s, pose);
      const std::string conflict = core_->tf_conflict(now_s);
      if (core_->conflict_changed(conflict)) {
        RCLCPP_ERROR(get_logger(), "TF CONFLICT: %s - another node publishes the same frame (old stack still running on this ROS_DOMAIN_ID?)", conflict.c_str());
      }
    }
    state_pub_->publish(m);
  }

  std::string map_frame_, base_frame_;
  double align_timeout_ = 40.0, ekf_settle_ = 1.5;
  int max_retries_ = 2;
  std::mutex mu_;
  bool busy_ = false;
  std::atomic<bool> stopping_{false};
  std::thread thread_;
  std::unique_ptr<buggy_core::LocalizationCore> core_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr seed_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Client<AlignSrv>::SharedPtr align_;
  rclcpp::Client<SetBool>::SharedPtr ekf_trigger_, ndt_trigger_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_click_, sub_ndt_;
  rclcpp::Subscription<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr sub_score_, sub_exe_;
  rclcpp::Subscription<autoware_internal_debug_msgs::msg::Int32Stamped>::SharedPtr sub_iter_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_tf_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationManager>());
  rclcpp::shutdown();
  return 0;
}
