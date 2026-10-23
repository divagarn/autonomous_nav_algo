// C++ version of buggy_pcd_lanelet_map/lidar_preprocess.py: raw /velodyne_points -> downsampled
// cloud for the NDT scan matcher (rate limited). The filtering is in localization/lidar_preprocess.cpp.
#include <cmath>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "buggy_core/localization/lidar_preprocess.hpp"

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

class LidarPreprocess : public rclcpp::Node
{
public:
  LidarPreprocess()
  : Node("lidar_preprocess")
  {
    p_.voxel = static_cast<float>(declare_parameter<double>("voxel_size", 0.3));
    p_.max_range = static_cast<float>(declare_parameter<double>("max_range", 40.0));
    p_.min_range = static_cast<float>(declare_parameter<double>("min_range", 0.5));
    p_.x_min = static_cast<float>(declare_parameter<double>("self_x_min", -2.5));
    p_.x_max = static_cast<float>(declare_parameter<double>("self_x_max", 0.6));
    p_.y_min = static_cast<float>(declare_parameter<double>("self_y_min", -0.9));
    p_.y_max = static_cast<float>(declare_parameter<double>("self_y_max", 0.9));
    p_.z_min = static_cast<float>(declare_parameter<double>("self_z_min", -1.2));
    p_.z_max = static_cast<float>(declare_parameter<double>("self_z_max", 0.9));
    min_period_ = 1.0 / declare_parameter<double>("max_rate_hz", 10.0);
    const auto in_topic = declare_parameter<std::string>("input_topic", "/velodyne_points");
    const auto out_topic = declare_parameter<std::string>("output_topic", "/localization/pointcloud_downsampled");
    pub_ = create_publisher<PointCloud2>(out_topic, 5);
    sub_ = create_subscription<PointCloud2>(in_topic, 5, [this](PointCloud2::ConstSharedPtr m) { on_cloud(*m); });
    RCLCPP_INFO(get_logger(), "%s -> %s  voxel=%g m range=[%g,%g] m", in_topic.c_str(), out_topic.c_str(),
      static_cast<double>(p_.voxel), static_cast<double>(p_.min_range), static_cast<double>(p_.max_range));
  }

private:
  void on_cloud(const PointCloud2 & msg)
  {
    const double now_s = get_clock()->now().nanoseconds() * 1e-9;
    if (last_ && now_s - *last_ < min_period_ * 0.9) {return;}
    last_ = now_s;
    const size_t n = static_cast<size_t>(msg.width) * msg.height;
    int ox = -1, oy = -1, oz = -1;
    for (const auto & f : msg.fields) {
      if (f.name == "x") {ox = static_cast<int>(f.offset);}
      if (f.name == "y") {oy = static_cast<int>(f.offset);}
      if (f.name == "z") {oz = static_cast<int>(f.offset);}
    }
    if (n == 0 || ox < 0 || oy < 0 || oz < 0 || msg.data.size() < n * msg.point_step) {return;}
    xyz_.resize(3 * n);
    for (size_t i = 0; i < n; ++i) {
      std::memcpy(&xyz_[3 * i], &msg.data[i * msg.point_step + ox], 4);
      std::memcpy(&xyz_[3 * i + 1], &msg.data[i * msg.point_step + oy], 4);
      std::memcpy(&xyz_[3 * i + 2], &msg.data[i * msg.point_step + oz], 4);
    }
    if (!buggy_core::preprocess_cloud(xyz_.data(), n, p_, out_)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "too few points after filtering");
      return;
    }
    PointCloud2 o;
    o.header = msg.header;
    o.height = 1;
    o.width = static_cast<uint32_t>(out_.size() / 3);
    const char * names[3] = {"x", "y", "z"};
    for (int i = 0; i < 3; ++i) {
      PointField f; f.name = names[i]; f.offset = 4 * i; f.datatype = PointField::FLOAT32; f.count = 1;
      o.fields.push_back(f);
    }
    o.is_bigendian = false;
    o.point_step = 12;
    o.row_step = 12 * o.width;
    o.is_dense = true;
    o.data.resize(out_.size() * sizeof(float));
    std::memcpy(o.data.data(), out_.data(), o.data.size());
    pub_->publish(o);
    ++count_;
    if (count_ % 100 == 1) {RCLCPP_INFO(get_logger(), "scan %d: %zu -> %u points", count_, n, o.width);}
  }

  buggy_core::PreprocessParams p_;
  double min_period_ = 0.1;
  std::optional<double> last_;
  int count_ = 0;
  std::vector<float> xyz_, out_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarPreprocess>());
  rclcpp::shutdown();
  return 0;
}
