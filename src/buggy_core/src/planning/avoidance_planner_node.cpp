// C++ version of buggy_pcd_lanelet_map/avoidance_planner.py: answers /planning/avoid_request with a
// detour (lane geometry + optional LiDAR free space). The planning itself is planning/avoidance.cpp.
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "buggy_core/planning/avoidance.hpp"

using buggy_core::AvoidParams;
using sensor_msgs::msg::PointCloud2;

class AvoidancePlanner : public rclcpp::Node
{
public:
  AvoidancePlanner()
  : Node("avoidance_planner"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_, this, false)
  {
    frame_ = declare_parameter<std::string>("map_frame", "map");
    const std::string osm = declare_parameter<std::string>("lanelet2_map_path", "");
    use_sensors_ = declare_parameter<bool>("use_sensors", false);
    sensor_z_ = declare_parameter<double>("sensor_z", 1.0);
    keep_sec_ = declare_parameter<double>("scan_memory_sec", 1.5);
    res_ = declare_parameter<double>("grid_res", 0.2);
    prm_.wheelbase = declare_parameter<double>("wheelbase", 1.6);
    prm_.max_steer_deg = declare_parameter<double>("max_steering_deg", 20.0);
    prm_.veh_half_width = declare_parameter<double>("vehicle_half_width", 0.72);
    prm_.side_margin = declare_parameter<double>("side_margin", 0.35);
    prm_.avoid_half_width = declare_parameter<double>("avoid_corridor_half_width", 1.05);
    prm_.offlane_allow = declare_parameter<double>("offlane_allow_m", 0.8);
    prm_.offlane_allow_sensed = declare_parameter<double>("offlane_allow_sensed_m", 8.0);
    prm_.max_offset = declare_parameter<double>("max_offset_m", 6.5);
    prm_.steer_frac = declare_parameter<double>("steer_frac", 0.75);
    prm_.check_steer_frac = declare_parameter<double>("check_steer_frac", 0.95);
    prm_.rear = declare_parameter<double>("rear", 0.7);
    prm_.front = declare_parameter<double>("front", 2.2);
    prm_.footprint_margin = declare_parameter<double>("footprint_margin", 0.15);
    prm_.lead = declare_parameter<double>("lead", 1.0);
    prm_.tail = declare_parameter<double>("tail", 0.6);
    prm_.ground_fraction = declare_parameter<double>("ground_fraction", 0.65);
    prm_.look_ahead = declare_parameter<double>("look_ahead", 40.0);
    prm_.merge_gap = declare_parameter<double>("merge_gap", 8.0);
    prm_.offset_step = declare_parameter<double>("offset_step", 0.5);
    if (!osm.empty()) {lane_ = std::make_unique<buggy_core::LaneModel>(buggy_core::load_lanelets(osm));}
    pub_ = create_publisher<std_msgs::msg::String>("/planning/avoid_result", 5);
    free_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/planning/free_space_marker", 1);
    sub_req_ = create_subscription<std_msgs::msg::String>("/planning/avoid_request", 5,
        [this](std_msgs::msg::String::ConstSharedPtr m) { on_request(m->data); });
    if (use_sensors_) {
      sub_ng_ = create_subscription<PointCloud2>("/non_ground_points_filtered", 1,
          [this](PointCloud2::ConstSharedPtr m) { on_cloud(*m, nonground_, true); });
      sub_g_ = create_subscription<PointCloud2>("/ground_points", 1,
          [this](PointCloud2::ConstSharedPtr m) { on_cloud(*m, ground_, false); });
    }
    RCLCPP_INFO(get_logger(), "avoidance planner up (LiDAR free space: %s)", use_sensors_ ? "True" : "False");
  }

private:
  using Scan = std::pair<double, std::vector<std::array<double, 2>>>;

  double t_now() { return get_clock()->now().nanoseconds() * 1e-9; }

  void on_cloud(const PointCloud2 & msg, std::deque<Scan> & store, bool solid)
  {
    if (msg.width == 0 || msg.point_step < 12) {return;}
    const size_t n = static_cast<size_t>(msg.width) * msg.height;
    if (msg.data.size() < n * msg.point_step) {return;}
    geometry_msgs::msg::TransformStamped t;
    // Python filters first and looks up the TF afterwards; the outcome (nothing stored) is the same
    try {
      t = tf_buffer_.lookupTransform(frame_, msg.header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return;
    }
    const auto & q = t.transform.rotation;
    const double yaw = 2.0 * std::atan2(q.z, q.w);      // planar rotation is enough for x, y
    const double c = std::cos(yaw), s = std::sin(yaw);
    Scan scan;
    scan.second.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      float v[3];
      std::memcpy(v, &msg.data[i * msg.point_step], 12);
      if (!std::isfinite(v[0]) || !std::isfinite(v[1]) || !std::isfinite(v[2])) {continue;}
      if (solid) {
        const double h = static_cast<double>(v[2]) + sensor_z_;      // 0.2 .. 1.9 m above ground
        if (!(h > 0.2 && h < 1.9)) {continue;}
      }
      scan.second.push_back({t.transform.translation.x + c * v[0] - s * v[1],
          t.transform.translation.y + s * v[0] + c * v[1]});
    }
    scan.first = t_now();
    store.push_back(std::move(scan));
    while (!store.empty() && t_now() - store.front().first > keep_sec_) {store.pop_front();}
  }

  std::unique_ptr<buggy_core::FreeSpace> free_space(const buggy_core::Poly & pts)
  {
    if (!use_sensors_ || nonground_.empty() || ground_.empty()) {return nullptr;}
    double lox = 1e300, loy = 1e300, hix = -1e300, hiy = -1e300;
    for (const auto & p : pts) {lox = std::min(lox, p[0]); loy = std::min(loy, p[1]); hix = std::max(hix, p[0]); hiy = std::max(hiy, p[1]);}
    lox -= 10.0; loy -= 10.0; hix += 10.0; hiy += 10.0;
    auto f = std::make_unique<buggy_core::FreeSpace>();
    f->ox = lox; f->oy = loy; f->res = res_;
    f->nx = static_cast<int>(std::ceil((hix - lox) / res_)) + 1;
    f->ny = static_cast<int>(std::ceil((hiy - loy) / res_)) + 1;
    const auto grid = [&](const std::deque<Scan> & store, int grow) {
        std::vector<uint8_t> g(static_cast<size_t>(f->nx) * f->ny, 0);
        for (const auto & sc : store) {
          for (const auto & xy : sc.second) {
            const int i = static_cast<int>(std::floor((xy[0] - lox) / res_));
            const int j = static_cast<int>(std::floor((xy[1] - loy) / res_));
            if (i >= 0 && j >= 0 && i < f->nx && j < f->ny) {g[static_cast<size_t>(i) * f->ny + j] = 1;}
          }
        }
        return grow ? buggy_core::dilate(g, f->nx, f->ny, grow) : g;
      };
    // ground is dilated more: the VLP-16 rings are ~0.7 m apart at 10 m, the gaps are not holes
    f->occ = grid(nonground_, 1);
    f->gnd = grid(ground_, 3);
    return f;
  }

  // The lane is blocked completely (no detour, or the detour has to leave the lane): show the free
  // ground the LiDAR has seen outside the lane in green (the detour outside the lane is magenta).
  void show_free_space(const buggy_core::Poly & pts, const std::optional<buggy_core::Zone> & zone,
    const buggy_core::FreeSpace * free, const nlohmann::json & res)
  {
    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = frame_;
    mk.header.stamp = now();
    mk.ns = "free_space"; mk.id = 0;
    mk.type = visualization_msgs::msg::Marker::CUBE_LIST;
    mk.action = visualization_msgs::msg::Marker::ADD;
    const bool left_lane = !res.value("ok", false) || res.value("outside_lane", false);
    if (!zone || !free || !left_lane) {
      mk.action = visualization_msgs::msg::Marker::DELETEALL;
    } else {
      const auto cells = buggy_core::free_space_cells(pts, *zone, free, lane_.get(), prm_);
      mk.pose.orientation.w = 1.0;
      mk.scale.x = mk.scale.y = 2 * res_;
      mk.scale.z = 0.05;
      mk.color.r = 0.2f; mk.color.g = 1.0f; mk.color.b = 0.3f; mk.color.a = 0.45f;
      mk.lifetime.sec = 20;
      const double z0 = pts.empty() ? 0.0 : pts[0][2];
      for (const auto & c : cells) {
        geometry_msgs::msg::Point p; p.x = c[0]; p.y = c[1]; p.z = z0 + 0.03;
        mk.points.push_back(p);
      }
    }
    visualization_msgs::msg::MarkerArray a;
    a.markers.push_back(mk);
    free_pub_->publish(a);
  }

  void on_request(const std::string & data)
  {
    const auto t0 = std::chrono::steady_clock::now();
    nlohmann::json req;
    buggy_core::Poly pts;
    std::vector<buggy_core::Box> obstacles;
    try {
      req = nlohmann::json::parse(data);
      for (const auto & p : req.at("points")) {pts.push_back({p[0].get<double>(), p[1].get<double>(), p[2].get<double>()});}
      for (const auto & o : req.at("obstacles")) {
        obstacles.push_back({o[0].get<double>(), o[1].get<double>(), o[2].get<double>(), o[3].get<double>(), o[4].get<double>()});
      }
    } catch (const nlohmann::json::exception &) {
      return;
    }
    if (pts.size() < 2) {return;}
    const auto free = free_space(pts);
    nlohmann::json res = buggy_core::plan_detour(pts, obstacles, lane_.get(), free.get(), prm_);
    std::optional<buggy_core::Zone> zone;
    if (res.contains("zone")) {
      zone = buggy_core::Zone{res["zone"]["s0"], res["zone"]["s1"], res["zone"]["lo"], res["zone"]["hi"]};
      res.erase("zone");
    }
    show_free_space(pts, zone, free.get(), res);
    res["id"] = req.contains("id") ? req["id"] : nlohmann::json(nullptr);
    res["probe"] = req.value("probe", false);
    res["sensed"] = free != nullptr;
    std_msgs::msg::String out;
    out.data = res.dump();
    pub_->publish(out);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    char buf[512];
    if (res["ok"].get<bool>()) {
      std::snprintf(buf, sizeof(buf), "OK %s %+.2f m, %.0f m long", res["side"].get<std::string>().c_str(),
        res["offset"].get<double>(), res["length"].get<double>());
    } else {
      std::snprintf(buf, sizeof(buf), "NO - %s", res["reason"].get<std::string>().c_str());
    }
    RCLCPP_INFO(get_logger(), "detour request %s: %s (LiDAR free space: %s, %.0f ms)",
      req.contains("id") ? req["id"].dump().c_str() : "None", buf, free ? "True" : "False", ms);
  }

  std::string frame_;
  bool use_sensors_ = false;
  double sensor_z_ = 1.0, keep_sec_ = 1.5, res_ = 0.2;
  AvoidParams prm_;
  std::unique_ptr<buggy_core::LaneModel> lane_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::deque<Scan> nonground_, ground_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr free_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_req_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_ng_, sub_g_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AvoidancePlanner>());
  rclcpp::shutdown();
  return 0;
}
