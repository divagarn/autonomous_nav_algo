// C++ version of buggy_perception/velodyne_static_tf.py: static transform base_link -> velodyne
// (rear axle -> VLP-16: 1.73 m ahead, 1.00 m up, no rotation). Broadcast once on /tf_static.
#include <cmath>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

class VelodyneStaticTf : public rclcpp::Node
{
public:
  VelodyneStaticTf()
  : Node("velodyne_static_tf_broadcaster"), broadcaster_(this)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = declare_parameter<std::string>("parent_frame", "base_link");
    t.child_frame_id = declare_parameter<std::string>("child_frame", "velodyne");
    t.transform.translation.x = declare_parameter<double>("x", 1.73);     // rear axle -> LiDAR
    t.transform.translation.y = declare_parameter<double>("y", 0.0);
    t.transform.translation.z = declare_parameter<double>("z", 1.00);     // height above ground
    const double roll = declare_parameter<double>("roll", 0.0);           // rad
    const double pitch = declare_parameter<double>("pitch", 0.0);
    const double yaw = declare_parameter<double>("yaw", 0.0);
    // same as tf_transformations.quaternion_from_euler(roll, pitch, yaw)
    const double cr = std::cos(roll / 2), sr = std::sin(roll / 2), cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
    const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
    t.transform.rotation.x = sr * cp * cy - cr * sp * sy;
    t.transform.rotation.y = cr * sp * cy + sr * cp * sy;
    t.transform.rotation.z = cr * cp * sy - sr * sp * cy;
    t.transform.rotation.w = cr * cp * cy + sr * sp * sy;
    broadcaster_.sendTransform(t);
    RCLCPP_INFO(get_logger(), "Broadcasted static transform from %s to %s: x=%.2fm, z=%.2fm",
      t.header.frame_id.c_str(), t.child_frame_id.c_str(), t.transform.translation.x, t.transform.translation.z);
  }

private:
  tf2_ros::StaticTransformBroadcaster broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelodyneStaticTf>());
  rclcpp::shutdown();
  return 0;
}
