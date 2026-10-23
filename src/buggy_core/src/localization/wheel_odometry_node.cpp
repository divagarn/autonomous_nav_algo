// C++ version of buggy_perception/wheel_odometry.py: /wheel_uart -> /odom (+ odom->base_link TF
// when publish_tf is true; the live launch sets it false, the EKF gets /odom via odom_to_twist).
// dt is the time between received messages on the node clock, exactly like the Python node.
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <ground_segmentation_msgs/msg/steer_speed.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "buggy_core/localization/wheel_odometry.hpp"

class WheelOdometry : public rclcpp::Node
{
public:
  WheelOdometry()
  : Node("wheel_odometry")
  {
    const double wheelbase = declare_parameter<double>("wheelbase", 1.6);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    integ_ = std::make_unique<buggy_core::WheelIntegrator>(wheelbase);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    if (publish_tf_) {tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);}
    sub_ = create_subscription<ground_segmentation_msgs::msg::SteerSpeed>("/wheel_uart", 10,
        [this](ground_segmentation_msgs::msg::SteerSpeed::ConstSharedPtr m) { on_feedback(*m); });
    RCLCPP_INFO(get_logger(), "Wheel odometry integrator initialized (wheelbase=%.1fm, dead-reckoning from /wheel_uart -> /odom + %s)",
      wheelbase, publish_tf_ ? (odom_frame_ + "->" + base_frame_ + " TF").c_str() : "no TF (publish_tf=false)");
  }

private:
  void on_feedback(const ground_segmentation_msgs::msg::SteerSpeed & msg)
  {
    const rclcpp::Time t = get_clock()->now();
    if (!last_) {last_ = t; return;}
    const double dt = (t - *last_).seconds();
    last_ = t;
    if (dt <= 0.0) {return;}
    const auto & s = integ_->update(dt, msg.steering, msg.speed);
    const double qz = std::sin(s.yaw / 2.0), qw = std::cos(s.yaw / 2.0);

    nav_msgs::msg::Odometry o;
    o.header.stamp = t;
    o.header.frame_id = odom_frame_;
    o.child_frame_id = base_frame_;
    o.pose.pose.position.x = s.x;
    o.pose.pose.position.y = s.y;
    o.pose.pose.orientation.z = qz;
    o.pose.pose.orientation.w = qw;
    o.twist.twist.linear.x = s.speed_ms;
    o.twist.twist.angular.z = s.angular_z;
    odom_pub_->publish(o);

    if (tf_) {
      geometry_msgs::msg::TransformStamped m;
      m.header.stamp = t;
      m.header.frame_id = odom_frame_;
      m.child_frame_id = base_frame_;
      m.transform.translation.x = s.x;
      m.transform.translation.y = s.y;
      m.transform.rotation.z = qz;
      m.transform.rotation.w = qw;
      tf_->sendTransform(m);
    }
  }

  std::string odom_frame_, base_frame_;
  bool publish_tf_ = true;
  std::unique_ptr<buggy_core::WheelIntegrator> integ_;
  std::optional<rclcpp::Time> last_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Subscription<ground_segmentation_msgs::msg::SteerSpeed>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelOdometry>());
  rclcpp::shutdown();
  return 0;
}
