#include "buggy_core/localization/projection.hpp"

#include <array>
#include <cmath>
#include <filesystem>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace buggy_core
{
namespace
{
constexpr double kA = 6378137.0;                    // WGS84
constexpr double kF = 1.0 / 298.257223563;
constexpr double kE2 = kF * (2.0 - kF);

std::array<double, 3> ecef(double lat_deg, double lon_deg)
{
  const double la = lat_deg * M_PI / 180.0, lo = lon_deg * M_PI / 180.0;
  const double n = kA / std::sqrt(1.0 - kE2 * std::sin(la) * std::sin(la));
  return {n * std::cos(la) * std::cos(lo), n * std::cos(la) * std::sin(lo), n * (1.0 - kE2) * std::sin(la)};
}
}  // namespace

std::pair<double, double> Projector::to_xy(double lat, double lon) const
{
  if (type == Type::Local) {return {0.0, 0.0};}
  const auto p = ecef(lat, lon), o = ecef(lat0, lon0);
  const double dx = p[0] - o[0], dy = p[1] - o[1], dz = p[2] - o[2];
  const double la = lat0 * M_PI / 180.0, lo = lon0 * M_PI / 180.0;
  const double east = -std::sin(lo) * dx + std::cos(lo) * dy;
  const double north = -std::sin(la) * std::cos(lo) * dx - std::sin(la) * std::sin(lo) * dy + std::cos(la) * dz;
  return {east, north};
}

Projector Projector::load(const std::string & yaml_path)
{
  Projector p;
  if (!std::filesystem::exists(yaml_path)) {return p;}
  const YAML::Node y = YAML::LoadFile(yaml_path);
  const std::string type = y["projector_type"] ? y["projector_type"].as<std::string>() : "Local";
  if (type == "Local") {return p;}
  if (type != "LocalCartesian") {
    throw std::runtime_error("map_projector_info.yaml: projector_type '" + type + "' is not supported by the buggy "
      "stack (use Local or LocalCartesian): " + yaml_path);
  }
  if (!y["map_origin"] || !y["map_origin"]["latitude"] || !y["map_origin"]["longitude"]) {
    throw std::runtime_error("map_projector_info.yaml: LocalCartesian needs map_origin latitude and longitude: " + yaml_path);
  }
  p.type = Type::LocalCartesian;
  p.lat0 = y["map_origin"]["latitude"].as<double>();
  p.lon0 = y["map_origin"]["longitude"].as<double>();
  return p;
}

Projector Projector::for_osm(const std::string & osm_path)
{
  const std::filesystem::path p(osm_path);
  if (p.extension() == ".yaml") {return load(osm_path);}
  return load((p.parent_path() / "map_projector_info.yaml").string());
}
}  // namespace buggy_core
