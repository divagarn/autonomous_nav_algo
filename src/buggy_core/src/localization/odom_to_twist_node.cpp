// C++ version of buggy_pcd_lanelet_map/odom_to_twist.py: wheel_odometry's nav_msgs/Odometry ->
// TwistWithCovarianceStamped for the EKF, with honest covariance (small on the two measured axes
// linear.x and angular.z, large on the four never-measured ones, zero elsewhere like the Python).
#include <array>
#include <string>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

class OdomToTwist : public rclcpp::Node
{
public:
  OdomToTwist()
  : Node("odom_to_twist")
  {
    const auto in = declare_parameter<std::string>("input_topic", "/odom");
    const auto out = declare_parameter<std::string>("output_topic", "/twist_with_covariance");
    cov_.fill(0.0);
    cov_[0] = kSmall; cov_[7] = kLarge; cov_[14] = kLarge; cov_[21] = kLarge; cov_[28] = kLarge; cov_[35] = kSmall;
    pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(out, 10);
    sub_ = create_subscription<nav_msgs::msg::Odometry>(in, 10,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { on_odom(*m); });
    RCLCPP_INFO(get_logger(), "odom_to_twist: %s -> %s", in.c_str(), out.c_str());
  }

private:
  static constexpr double kSmall = 0.02, kLarge = 100.0;

  void on_odom(const nav_msgs::msg::Odometry & msg)
  {
    geometry_msgs::msg::TwistWithCovarianceStamped o;
    o.header = msg.header;
    o.header.frame_id = msg.child_frame_id.empty() ? msg.header.frame_id : msg.child_frame_id;
    o.twist.twist = msg.twist.twist;
    o.twist.covariance = cov_;
    pub_->publish(o);
  }

  std::array<double, 36> cov_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomToTwist>());
  rclcpp::shutdown();
  return 0;
}
