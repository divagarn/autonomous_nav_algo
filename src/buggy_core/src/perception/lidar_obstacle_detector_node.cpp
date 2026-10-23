// Obstacles in the vehicle's path from the VLP-16 (ground already removed) - C++ version of
// buggy_pcd_lanelet_map/lidar_obstacle_detector.py (same topics, parameters and output).
//
// Input : /non_ground_points_filtered (segment_ground -> self_hit_filter), /planning/route_path,
//         TF map -> base_link.
// Output: objects_topic  (default /perception/lidar_objects)  autoware_perception_msgs/PredictedObjects,
//         published for EVERY processed scan even when empty - it is the heartbeat operation_manager
//         uses to know the detector is alive; marker_topic (/perception/lidar_objects_marker).
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "buggy_core/perception/detector_core.hpp"
#include "buggy_core/perception/obstacle_clustering.hpp"

using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::Shape;

class LidarObstacleDetector : public rclcpp::Node
{
public:
  LidarObstacleDetector()
  : Node("lidar_obstacle_detector"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    prm_.sensor_x = declare_parameter<double>("sensor_x", 1.73);
    prm_.sensor_z = declare_parameter<double>("sensor_z", 1.00);
    prm_.min_height = declare_parameter<double>("min_height", 0.20);
    prm_.max_height = declare_parameter<double>("max_height", 2.00);
    prm_.half_width = declare_parameter<double>("corridor_half_width", 1.25);
    prm_.length = declare_parameter<double>("corridor_length", 25.0);
    prm_.cell_size = declare_parameter<double>("cell_size", 0.30);
    prm_.min_points = declare_parameter<int>("min_points", 5);
    max_hz_ = declare_parameter<double>("max_rate_hz", 10.0);
    // ground aware: height above the LOCAL ground (ground points of the same scan), thin / low
    // clusters (one beam ring on the road) dropped - see detector_core.hpp
    buggy_core::GroundAwareParams gp;
    gp.aware = declare_parameter<bool>("ground_aware", true);
    gp.min_height = declare_parameter<double>("ground_min_height", 0.30);
    gp.reject_thin_low = declare_parameter<bool>("reject_thin_low", true);
    gp.thin_extent = declare_parameter<double>("thin_extent", 0.08);
    gp.low_height = declare_parameter<double>("low_height", 0.50);
    gp.max_age = declare_parameter<double>("ground_max_age", 0.3);
    core_ = std::make_unique<buggy_core::DetectorCore>(prm_, gp);
    const auto objects_topic =
      declare_parameter<std::string>("objects_topic", "/perception/lidar_objects");
    const auto marker_topic =
      declare_parameter<std::string>("marker_topic", "/perception/lidar_objects_marker");

    obj_pub_ = create_publisher<PredictedObjects>(objects_topic, 1);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, 1);
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/planning/route_path", 1, [this](nav_msgs::msg::Path::ConstSharedPtr m) {on_path(*m);});
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/non_ground_points_filtered", 1,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {on_cloud(*m);});
    if (gp.aware) {
      ground_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/ground_points", 1,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {on_ground(*m);});
    }
    RCLCPP_INFO(get_logger(), "lidar_obstacle_detector (C++) -> %s (ground aware: %s)",
      objects_topic.c_str(), gp.aware ? "True" : "False");
  }

private:
  void on_path(const nav_msgs::msg::Path & msg)
  {
    route_.clear();
    for (const auto & p : msg.poses) {
      route_.push_back({p.pose.position.x, p.pose.position.y});
    }
  }

  bool vehicle_pose(double & x, double & y, double & yaw)
  {
    try {
      const auto t = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
      const auto & q = t.transform.rotation;
      x = t.transform.translation.x;
      y = t.transform.translation.y;
      yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  static bool find_offsets(const sensor_msgs::msg::PointCloud2 & m, int & ox, int & oy, int & oz)
  {
    ox = oy = oz = -1;
    for (const auto & f : m.fields) {
      if (f.datatype != sensor_msgs::msg::PointField::FLOAT32) {
        continue;
      }
      if (f.name == "x") {ox = static_cast<int>(f.offset);}
      if (f.name == "y") {oy = static_cast<int>(f.offset);}
      if (f.name == "z") {oz = static_cast<int>(f.offset);}
    }
    return ox >= 0 && oy >= 0 && oz >= 0;
  }

  bool read_points(const sensor_msgs::msg::PointCloud2 & msg, std::vector<buggy_core::Point3> & out)
  {
    int ox, oy, oz;
    if (!find_offsets(msg, ox, oy, oz) || msg.point_step == 0) {
      return false;
    }
    const size_t n = static_cast<size_t>(msg.width) * msg.height;
    out.clear();
    out.reserve(n);
    for (size_t i = 0; i < n && (i + 1) * msg.point_step <= msg.data.size(); ++i) {
      const uint8_t * p = msg.data.data() + i * msg.point_step;
      float x, y, z;
      std::memcpy(&x, p + ox, 4);
      std::memcpy(&y, p + oy, 4);
      std::memcpy(&z, p + oz, 4);
      out.push_back({x, y, z});
    }
    return true;
  }

  void on_ground(const sensor_msgs::msg::PointCloud2 & msg)
  {
    if (msg.width == 0) {
      return;
    }
    if (read_points(msg, ground_)) {
      ground_stamp_ = rclcpp::Time(msg.header.stamp).seconds();
      have_ground_ = true;
    }
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2 & msg)
  {
    const double now = get_clock()->now().seconds();
    if (now - last_ < 1.0 / max_hz_) {
      return;
    }
    double rx, ry, yaw;
    if (!vehicle_pose(rx, ry, yaw)) {
      return;
    }
    last_ = now;
    std::vector<buggy_core::Point3> sensor;
    if (!read_points(msg, sensor)) {
      return;
    }
    const auto corridor = buggy_core::corridor_ahead(route_, rx, ry, yaw, prm_.length);
    const bool fresh_ground =
      have_ground_ && std::abs(rclcpp::Time(msg.header.stamp).seconds() - ground_stamp_) <= core_->ground_params().max_age;
    const auto res = core_->process(sensor, fresh_ground ? &ground_ : nullptr, corridor);
    const auto & confirmed = res.confirmed;
    if (res.used_ground_model && now - last_log_ > 5.0) {
      last_log_ = now;
      RCLCPP_INFO(get_logger(), "ground model: tilt %.1f deg, %d thin/low clusters dropped so far",
        res.plane_tilt_deg, core_->rejected());
    }
    publish(msg.header.stamp, rx, ry, yaw, confirmed);
  }

  void publish(
    const builtin_interfaces::msg::Time & stamp, double rx, double ry, double yaw,
    const std::vector<buggy_core::Cluster> & clusters)
  {
    const double cy_ = std::cos(yaw), sy_ = std::sin(yaw);
    PredictedObjects objs;
    objs.header.stamp = stamp;
    objs.header.frame_id = "map";
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    int id = 0;
    for (const auto & c : clusters) {
      PredictedObject o;
      o.existence_probability = 1.0;
      ObjectClassification cls;
      cls.label = ObjectClassification::UNKNOWN;
      cls.probability = 1.0;
      o.classification.push_back(cls);
      auto & pp = o.kinematics.initial_pose_with_covariance.pose;
      pp.position.x = rx + c.cx * cy_ - c.cy * sy_;
      pp.position.y = ry + c.cx * sy_ + c.cy * cy_;
      pp.position.z = c.h / 2;
      pp.orientation.z = std::sin(yaw / 2);
      pp.orientation.w = std::cos(yaw / 2);
      o.shape.type = Shape::BOUNDING_BOX;
      o.shape.dimensions.x = c.lx;
      o.shape.dimensions.y = c.ly;
      o.shape.dimensions.z = c.h;
      objs.objects.push_back(o);

      visualization_msgs::msg::Marker m;
      m.header = objs.header;
      m.ns = "lidar_obstacle";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose = pp;
      m.scale.x = c.lx;
      m.scale.y = c.ly;
      m.scale.z = c.h;
      m.color.r = 1.0F;
      m.color.a = 0.6F;
      markers.markers.push_back(m);
    }
    obj_pub_->publish(objs);
    marker_pub_->publish(markers);
  }

  buggy_core::DetectorParams prm_;
  std::unique_ptr<buggy_core::DetectorCore> core_;
  std::vector<buggy_core::Point3> ground_;
  double ground_stamp_{0.0}, last_log_{0.0};
  bool have_ground_{false};
  double max_hz_{10.0};
  double last_{0.0};
  std::vector<std::array<double, 2>> route_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<PredictedObjects>::SharedPtr obj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_, ground_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarObstacleDetector>());
  rclcpp::shutdown();
  return 0;
}
