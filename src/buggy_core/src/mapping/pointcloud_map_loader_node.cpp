// Own replacement for autoware_pointcloud_map_loader (whole + partial / differential loading).
// Loads the PCD file(s) of the map, publishes the whole map once (transient local) on
// output/pointcloud_map and serves the cells on service/get_partial_pcd_map and
// service/get_differential_pcd_map - the interface ndt_scan_matcher uses. Same parameter and
// topic / service names as the Autoware node, so only the package of the launch node changes.
// Cells: one per PCD file; rectangle from pcd_metadata_path (x_resolution, y_resolution and the
// "<file>.pcd: [min_x, min_y]" entries); cell id = the PCD file's full path, like Autoware.
// Not implemented: enable_downsampled_whole_load, enable_selected_load (not used by the buggy).
#include <algorithm>
#include <cstring>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <autoware_map_msgs/msg/point_cloud_map_cell_with_id.hpp>
#include <autoware_map_msgs/srv/get_differential_point_cloud_map.hpp>
#include <autoware_map_msgs/srv/get_partial_point_cloud_map.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <yaml-cpp/yaml.h>

#include "buggy_core/mapping/pcd_cells.hpp"

namespace fs = std::filesystem;
using autoware_map_msgs::msg::PointCloudMapCellWithID;
using autoware_map_msgs::srv::GetDifferentialPointCloudMap;
using autoware_map_msgs::srv::GetPartialPointCloudMap;
using sensor_msgs::msg::PointCloud2;

// x y z float32, 12 bytes per point (what Autoware publishes; pcl::toROSMsg would pad to 16)
static PointCloud2 to_msg(const pcl::PointCloud<pcl::PointXYZ> & c)
{
  PointCloud2 m;
  m.header.frame_id = "map";
  m.height = 1;
  m.width = static_cast<uint32_t>(c.size());
  const char * names[3] = {"x", "y", "z"};
  for (int i = 0; i < 3; ++i) {
    sensor_msgs::msg::PointField f;
    f.name = names[i]; f.offset = 4 * i; f.datatype = sensor_msgs::msg::PointField::FLOAT32; f.count = 1;
    m.fields.push_back(f);
  }
  m.is_bigendian = false;
  m.point_step = 12;
  m.row_step = 12 * m.width;
  m.is_dense = true;
  m.data.resize(static_cast<size_t>(m.row_step));
  for (size_t i = 0; i < c.size(); ++i) {
    const float v[3] = {c[i].x, c[i].y, c[i].z};
    std::memcpy(&m.data[12 * i], v, 12);
  }
  return m;
}

class PointCloudMapLoader : public rclcpp::Node
{
public:
  PointCloudMapLoader()
  : Node("pointcloud_map_loader")
  {
    const auto paths = declare_parameter<std::vector<std::string>>("pcd_paths_or_directory", std::vector<std::string>{});
    const auto meta_path = declare_parameter<std::string>("pcd_metadata_path", "");
    const bool whole = declare_parameter<bool>("enable_whole_load", true);
    const bool partial = declare_parameter<bool>("enable_partial_load", false);
    declare_parameter<bool>("enable_downsampled_whole_load", false);
    declare_parameter<bool>("enable_selected_load", false);
    declare_parameter<double>("leaf_size", 3.0);

    std::vector<std::string> files;
    for (const auto & p : paths) {
      if (fs::is_directory(p)) {
        for (const auto & e : fs::directory_iterator(p)) {
          if (e.path().extension() == ".pcd") {files.push_back(e.path().string());}
        }
        std::sort(files.begin(), files.end());
      } else {
        files.push_back(p);
      }
    }
    if (files.empty()) {
      RCLCPP_FATAL(get_logger(), "no PCD file given (pcd_paths_or_directory)");
      throw std::runtime_error("no pcd");
    }
    YAML::Node meta;
    if (!meta_path.empty() && fs::exists(meta_path)) {meta = YAML::LoadFile(meta_path);}
    const double xr = meta["x_resolution"] ? meta["x_resolution"].as<double>() : 0.0;
    const double yr = meta["y_resolution"] ? meta["y_resolution"].as<double>() : 0.0;

    pcl::PointCloud<pcl::PointXYZ> all;
    for (const auto & f : files) {
      pcl::PointCloud<pcl::PointXYZ> cloud;
      if (pcl::io::loadPCDFile(f, cloud) < 0) {
        RCLCPP_FATAL(get_logger(), "cannot read PCD %s", f.c_str());
        throw std::runtime_error("bad pcd");
      }
      cloud.is_dense = true;
      const std::string base = fs::path(f).filename().string();
      buggy_core::CellRect r{0, 0, 0, 0};
      bool have_rect = false;
      if (meta[base] && xr > 0 && yr > 0) {
        r = {meta[base][0].as<double>(), meta[base][1].as<double>(), 0, 0};
        r.max_x = r.min_x + xr; r.max_y = r.min_y + yr;
        have_rect = true;
      }
      if (partial && have_rect) {
        PointCloudMapCellWithID cell;
        cell.cell_id = f;
        cell.pointcloud = to_msg(cloud);
        cell.metadata.min_x = static_cast<float>(r.min_x); cell.metadata.min_y = static_cast<float>(r.min_y);
        cell.metadata.max_x = static_cast<float>(r.max_x); cell.metadata.max_y = static_cast<float>(r.max_y);
        cells_.push_back(std::move(cell));
        rects_.push_back(r);
        ids_.push_back(f);
      }
      if (whole) {all += cloud;}
      RCLCPP_INFO(get_logger(), "loaded %s: %zu points", base.c_str(), cloud.size());
    }
    if (partial && cells_.empty()) {
      RCLCPP_WARN(get_logger(), "enable_partial_load but no cell rectangle in '%s': partial / differential services are off", meta_path.c_str());
    }
    if (whole) {
      all.is_dense = true;
      whole_ = to_msg(all);
      rclcpp::QoS qos(1);
      qos.transient_local().reliable();
      pub_ = create_publisher<PointCloud2>("output/pointcloud_map", qos);
      whole_.header.stamp = now();
      pub_->publish(whole_);
      RCLCPP_INFO(get_logger(), "published the whole map: %zu points", all.size());
    }
    if (!cells_.empty()) {
      partial_srv_ = create_service<GetPartialPointCloudMap>("service/get_partial_pcd_map",
          [this](const std::shared_ptr<GetPartialPointCloudMap::Request> rq, std::shared_ptr<GetPartialPointCloudMap::Response> rs) {
            const auto sel = buggy_core::select_cells(rects_, ids_, rq->area.center_x, rq->area.center_y, rq->area.radius, {});
            for (size_t i : sel.add) {rs->new_pointcloud_with_ids.push_back(cells_[i]);}
            rs->header.frame_id = "map"; rs->header.stamp = now();
          });
      diff_srv_ = create_service<GetDifferentialPointCloudMap>("service/get_differential_pcd_map",
          [this](const std::shared_ptr<GetDifferentialPointCloudMap::Request> rq, std::shared_ptr<GetDifferentialPointCloudMap::Response> rs) {
            const auto sel = buggy_core::select_cells(rects_, ids_, rq->area.center_x, rq->area.center_y, rq->area.radius, rq->cached_ids);
            for (size_t i : sel.add) {rs->new_pointcloud_with_ids.push_back(cells_[i]);}
            rs->ids_to_remove = sel.remove;
            rs->header.frame_id = "map"; rs->header.stamp = now();
          });
    }
  }

private:
  PointCloud2 whole_;
  std::vector<PointCloudMapCellWithID> cells_;
  std::vector<buggy_core::CellRect> rects_;
  std::vector<std::string> ids_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_;
  rclcpp::Service<GetPartialPointCloudMap>::SharedPtr partial_srv_;
  rclcpp::Service<GetDifferentialPointCloudMap>::SharedPtr diff_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PointCloudMapLoader>());
  } catch (const std::exception &) {
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
