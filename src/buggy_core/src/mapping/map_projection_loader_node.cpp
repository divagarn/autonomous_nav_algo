// Own replacement for autoware_map_projection_loader: reads map_projector_info.yaml of the map folder
// and publishes it once, transient local, on /map/map_projector_info (autoware_map_msgs/MapProjectorInfo).
// Same topic, QoS and parameter names as the Autoware node, so the launch only changes the package.
// Supported: Local (what the buggy maps use), LocalCartesian, LocalCartesianUTM, MGRS, TransverseMercator.
#include <stdexcept>
#include <string>

#include <autoware_map_msgs/msg/map_projector_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

using autoware_map_msgs::msg::MapProjectorInfo;

class MapProjectionLoader : public rclcpp::Node
{
public:
  MapProjectionLoader()
  : Node("map_projection_loader")
  {
    const auto path = declare_parameter<std::string>("map_projector_info_path", "");
    declare_parameter<std::string>("lanelet2_map_path", "");      // accepted, not needed here
    YAML::Node y;
    try {
      y = YAML::LoadFile(path);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(get_logger(), "cannot read map_projector_info '%s': %s", path.c_str(), e.what());
      throw;
    }
    MapProjectorInfo m;
    m.projector_type = y["projector_type"] ? y["projector_type"].as<std::string>() : "";
    if (m.projector_type != MapProjectorInfo::LOCAL && m.projector_type != MapProjectorInfo::LOCAL_CARTESIAN &&
      m.projector_type != MapProjectorInfo::LOCAL_CARTESIAN_UTM && m.projector_type != MapProjectorInfo::MGRS &&
      m.projector_type != MapProjectorInfo::TRANSVERSE_MERCATOR)
    {
      RCLCPP_FATAL(get_logger(), "unknown projector_type '%s' in %s", m.projector_type.c_str(), path.c_str());
      throw std::runtime_error("bad projector_type");
    }
    m.vertical_datum = y["vertical_datum"] ? y["vertical_datum"].as<std::string>() : "";
    if (y["mgrs_grid"]) {m.mgrs_grid = y["mgrs_grid"].as<std::string>();}
    if (y["map_origin"]) {
      m.map_origin.latitude = y["map_origin"]["latitude"].as<double>();
      m.map_origin.longitude = y["map_origin"]["longitude"].as<double>();
      m.map_origin.altitude = 0.0;                                  // altitude is not used
    }
    // scale factor: only TransverseMercator takes the file's value (default 0.9996); UTM and MGRS use 0.9996,
    // Local and LocalCartesian use 1.0 (as documented in the message)
    if (m.projector_type == MapProjectorInfo::TRANSVERSE_MERCATOR) {
      m.scale_factor = y["scale_factor"] ? y["scale_factor"].as<float>() : 0.9996f;
    } else if (m.projector_type == MapProjectorInfo::LOCAL_CARTESIAN_UTM || m.projector_type == MapProjectorInfo::MGRS) {
      m.scale_factor = 0.9996f;
    } else {
      m.scale_factor = 1.0f;
    }
    rclcpp::QoS qos(1);
    qos.transient_local().reliable();
    pub_ = create_publisher<MapProjectorInfo>("/map/map_projector_info", qos);
    pub_->publish(m);
    RCLCPP_INFO(get_logger(), "map projector: %s (%s)", m.projector_type.c_str(), path.c_str());
  }

private:
  rclcpp::Publisher<MapProjectorInfo>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MapProjectionLoader>());
  } catch (const std::exception &) {
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
