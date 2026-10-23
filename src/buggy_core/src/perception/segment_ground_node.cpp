// C++ version of buggy_perception/segment_ground.py, 'segment' backend (what the live launch uses:
// SEGMENT_GROUND_METHOD=segment). /velodyne_points -> /ground_points + /non_ground_points.
// The gpu / cpu RANSAC backends stay Python-only (randomised, not used on the vehicle).
#include <cmath>
#include <cstring>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "buggy_core/perception/ground_filter.hpp"

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

class SegmentGround : public rclcpp::Node
{
public:
  SegmentGround()
  : Node("segment_ground")
  {
    // configurable (config/buggy_params.yaml, section segment_ground)
    p_.max_distance = static_cast<float>(declare_parameter<double>("max_distance", 25.0));
    p_.min_height = static_cast<float>(declare_parameter<double>("min_height", -1.60));
    p_.max_height = static_cast<float>(declare_parameter<double>("max_height", 2.00));
    p_.seed_z_min = declare_parameter<double>("seed_z_min", -1.45);
    p_.seed_z_max = declare_parameter<double>("seed_z_max", -0.45);
    p_.num_segments = static_cast<int>(declare_parameter<int>("num_segments", 180));
    p_.bin_size = declare_parameter<double>("bin_size", 0.5);
    p_.max_slope_deg = declare_parameter<double>("max_slope_deg", 10.0);
    p_.max_height_jump = declare_parameter<double>("max_height_jump", 0.20);
    p_.base_thresh = static_cast<float>(declare_parameter<double>("base_thresh", 0.15));
    p_.distance_scale = static_cast<float>(declare_parameter<double>("distance_scale", 0.012));
    p_.max_adaptive = static_cast<float>(declare_parameter<double>("max_adaptive", 0.25));
    p_.ground_max_above = declare_parameter<double>("ground_max_above", 0.15);
    p_.ground_max_below = declare_parameter<double>("ground_max_below", 0.20);
    p_.min_points = static_cast<size_t>(declare_parameter<int>("min_points", 50));
    gpub_ = create_publisher<PointCloud2>("/ground_points", 2);
    ngpub_ = create_publisher<PointCloud2>("/non_ground_points", 2);
    sub_ = create_subscription<PointCloud2>("/velodyne_points", 1,
        [this](PointCloud2::ConstSharedPtr m) { on_cloud(*m); });
    RCLCPP_INFO(get_logger(), "Ground segmentation -- backend: CPU (C++, Himmelsbach-style segment line-fit)");
    RCLCPP_INFO(get_logger(), "segments=%d bin=%.1fm max_slope<%.0fdeg max_height_jump=%.2fm",
      p_.num_segments, p_.bin_size, p_.max_slope_deg, p_.max_height_jump);
  }

private:
  void on_cloud(const PointCloud2 & msg)
  {
    const size_t n = static_cast<size_t>(msg.width) * msg.height;
    if (n < p_.min_points) {return;}
    int ox = -1, oy = -1, oz = -1;
    for (const auto & f : msg.fields) {
      if (f.name == "x") {ox = static_cast<int>(f.offset);}
      if (f.name == "y") {oy = static_cast<int>(f.offset);}
      if (f.name == "z") {oz = static_cast<int>(f.offset);}
    }
    if (ox < 0 || oy < 0 || oz < 0) {return;}
    const size_t step = msg.point_step;
    if (msg.data.size() < n * step) {return;}
    xyz_.clear();
    xyz_.reserve(n * 3);
    for (size_t i = 0; i < n; ++i) {
      float v[3];
      std::memcpy(&v[0], &msg.data[i * step + ox], 4);
      std::memcpy(&v[1], &msg.data[i * step + oy], 4);
      std::memcpy(&v[2], &msg.data[i * step + oz], 4);
      if (std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2])) {
        xyz_.insert(xyz_.end(), v, v + 3);
      }
    }
    const size_t nt = xyz_.size() / 3;
    if (nt < p_.min_points) {return;}
    if (!buggy_core::segment_ground_mask(xyz_.data(), nt, p_, mask_)) {return;}
    std::vector<size_t> g, ng;
    for (size_t i = 0; i < nt; ++i) {(mask_[i] ? g : ng).push_back(i);}
    publish(msg.header, g, true, *gpub_);
    publish(msg.header, ng, false, *ngpub_);
  }

  void publish(const std_msgs::msg::Header & header, const std::vector<size_t> & idx, bool ground,
    rclcpp::Publisher<PointCloud2> & pub)
  {
    if (idx.empty()) {return;}                      // like fast_pub: nothing published when empty
    if (ground) {buggy_core::colour_ground(xyz_.data(), idx, rgb_);} else {buggy_core::colour_non_ground(xyz_.data(), idx, rgb_);}
    PointCloud2 m;
    m.header = header;
    m.height = 1;
    m.width = static_cast<uint32_t>(idx.size());
    const char * names[4] = {"x", "y", "z", "rgb"};
    for (int k = 0; k < 4; ++k) {
      PointField f;
      f.name = names[k]; f.offset = 4 * k;
      f.datatype = k < 3 ? PointField::FLOAT32 : PointField::UINT32;
      f.count = 1;
      m.fields.push_back(f);
    }
    m.is_bigendian = false;
    m.point_step = 16;
    m.row_step = 16 * m.width;
    m.is_dense = true;
    m.data.resize(16 * idx.size());
    for (size_t k = 0; k < idx.size(); ++k) {
      std::memcpy(&m.data[16 * k], &xyz_[3 * idx[k]], 12);
      std::memcpy(&m.data[16 * k + 12], &rgb_[k], 4);
    }
    pub.publish(m);
  }

  buggy_core::GroundParams p_;
  std::vector<float> xyz_;
  std::vector<uint8_t> mask_;
  std::vector<uint32_t> rgb_;
  rclcpp::Publisher<PointCloud2>::SharedPtr gpub_, ngpub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SegmentGround>());
  rclcpp::shutdown();
  return 0;
}
