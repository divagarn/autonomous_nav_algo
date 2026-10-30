// Map projection of the lanelet2 map: latitude / longitude -> metres in the map frame.
//
// The map folder's map_projector_info.yaml says which one the map uses (same file Autoware's
// map_projection_loader reads):
//   Local           x, y come straight from the local_x / local_y tags (no lat/lon) - the buggy maps so far
//   LocalCartesian  x = East, y = North of map_origin (WGS84 lat/lon), up = 0 (GeographicLib LocalCartesian
//                   with a zero origin height; altitude is not used)  - georeferenced maps, GNSS can be used
// MGRS, LocalCartesianUTM and TransverseMercator are real Autoware options, but not supported here: the loader
// throws a clear error instead of silently placing the lanes wrong.
#pragma once

#include <string>
#include <utility>

namespace buggy_core
{
struct Projector
{
  enum class Type { Local, LocalCartesian };
  Type type = Type::Local;
  double lat0 = 0.0, lon0 = 0.0;           // map origin [deg] (LocalCartesian)

  bool georeferenced() const {return type == Type::LocalCartesian;}
  // lat/lon [deg] -> (x East, y North) [m] in the map frame; (0, 0) for Local
  std::pair<double, double> to_xy(double lat, double lon) const;

  // The map_projector_info.yaml in the folder of the given .osm (or the yaml itself); a missing file means Local.
  static Projector for_osm(const std::string & osm_path);
  static Projector load(const std::string & yaml_path);
};
}  // namespace buggy_core
