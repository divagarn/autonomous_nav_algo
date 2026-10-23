// C++ version of buggy_perception/self_hit_filter.py: drops the vehicle's own returns from
// /non_ground_points -> /non_ground_points_filtered, and publishes the exclusion rectangle
// (/robot_radius_marker, 1 Hz).
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "buggy_core/perception/ground_filter.hpp"

using sensor_msgs::msg::PointCloud2;

class SelfHitFilter : public rclcpp::Node
{
public:
  SelfHitFilter()
  : Node("self_hit_filter")
  {
    // configurable (config/buggy_params.yaml, section self_hit_filter)
    p_.min_range = static_cast<float>(declare_parameter<double>("min_range", 1.0));
    p_.x_min = static_cast<float>(declare_parameter<double>("box_x_min", -2.39));
    p_.x_max = static_cast<float>(declare_parameter<double>("box_x_max", 0.53));
    p_.y_min = static_cast<float>(declare_parameter<double>("box_y_min", -0.72));
    p_.y_max = static_cast<float>(declare_parameter<double>("box_y_max", 0.72));
    p_.z_min = static_cast<float>(declare_parameter<double>("box_z_min", -1.05));
    p_.z_max = static_cast<float>(declare_parameter<double>("box_z_max", -0.10));
    p_.sensor_offset_x = static_cast<float>(declare_parameter<double>("sensor_offset_x", 1.73));
    p_.sensor_offset_y = static_cast<float>(declare_parameter<double>("sensor_offset_y", 0.0));
    p_.half_length = static_cast<float>(declare_parameter<double>("robot_length", 4.9) / 2.0);
    p_.half_width = static_cast<float>(declare_parameter<double>("robot_width", 2.9) / 2.0);
    pub_ = create_publisher<PointCloud2>("/non_ground_points_filtered", 2);
    sub_ = create_subscription<PointCloud2>("/non_ground_points", 2,
        [this](PointCloud2::ConstSharedPtr m) { on_cloud(*m); });
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/robot_radius_marker", 1);
    timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::seconds(1), [this]() { publish_marker(); });
  }

private:
  void publish_marker()
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "base_link";
    m.header.stamp = now();
    m.ns = "robot_radius"; m.id = 0;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.05;
    m.color.r = 1.0; m.color.g = 0.5; m.color.b = 0.0; m.color.a = 0.9;
    const double hl = p_.half_length, hw = p_.half_width;
    const double c[5][2] = {{hl, hw}, {hl, -hw}, {-hl, -hw}, {-hl, hw}, {hl, hw}};
    for (const auto & q : c) {
      geometry_msgs::msg::Point pt; pt.x = q[0]; pt.y = q[1]; pt.z = 0.0;
      m.points.push_back(pt);
    }
    marker_pub_->publish(m);
  }

  void on_cloud(const PointCloud2 & msg)
  {
    if (msg.width == 0) {return;}
    const size_t step = msg.point_step;                // 16: x y z rgb, like the Python reshape(-1, 4)
    const size_t n = msg.data.size() / step;
    buggy_core::self_hit_keep(msg.data.data(), n, step, p_, keep_);
    PointCloud2 out;
    out.header = msg.header;
    out.height = 1;
    out.fields = msg.fields;
    out.is_bigendian = msg.is_bigendian;
    out.point_step = msg.point_step;
    out.is_dense = true;
    out.data.reserve(msg.data.size());
    uint32_t kept = 0;
    for (size_t i = 0; i < n; ++i) {
      if (keep_[i]) {
        out.data.insert(out.data.end(), msg.data.begin() + i * step, msg.data.begin() + (i + 1) * step);
        ++kept;
      }
    }
    out.width = kept;
    out.row_step = msg.point_step * kept;
    pub_->publish(out);
  }

  buggy_core::SelfHitParams p_;
  std::vector<uint8_t> keep_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SelfHitFilter>());
  rclcpp::shutdown();
  return 0;
}
