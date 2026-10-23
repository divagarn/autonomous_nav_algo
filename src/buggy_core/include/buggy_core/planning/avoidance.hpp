// Detour planning around an obstacle that blocks the route, without ROS types.
// Port of buggy_pcd_lanelet_map/avoidance.py (geometry: see the header comment of that file):
// Frenet S-curve offset profile, arc radius from the steering limit, swept-footprint check against
// obstacles, LiDAR occupancy / ground grids and the drivable lane area.
#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "buggy_core/planning/lanelet_map.hpp"
#include "buggy_core/planning/path_utils.hpp"

namespace buggy_core
{
struct AvoidParams
{
  double wheelbase = 1.6, max_steer_deg = 20.0;
  double steer_frac = 0.75, check_steer_frac = 0.95;
  double veh_half_width = 0.72, side_margin = 0.35;
  double rear = 0.7, front = 2.2, footprint_margin = 0.15;
  double avoid_half_width = 1.05, lead = 1.0, tail = 0.6;
  double offlane_allow = 0.8, offlane_allow_sensed = 8.0;
  double max_offset = 6.5, offset_step = 0.5, ground_fraction = 0.65;
  double look_ahead = 40.0, merge_gap = 8.0;
};

struct Box { double x, y, yaw, length, width; };

struct Zone { double s0, s1, lo, hi; };

// Lane area: union of the lanelet polygons; distance = to the nearest lane boundary vertex.
class LaneModel
{
public:
  explicit LaneModel(const std::vector<Lanelet> & lanelets);
  bool inside(double x, double y) const;
  double dist(double x, double y) const;
  bool within(double x, double y, double r) const;     // a boundary vertex closer than r (early exit)
private:
  struct Poly2 { std::vector<std::array<double, 2>> v; double x0, x1, y0, y1; };
  std::vector<Poly2> polys_;
  std::vector<std::array<double, 2>> bounds_;
};

// LiDAR-derived grids in the map frame (row-major nx x ny, index i * ny + j).
struct FreeSpace
{
  double ox = 0, oy = 0, res = 0.2;
  int nx = 0, ny = 0;
  std::vector<uint8_t> occ, gnd;
  bool has_ground = true;
  // lookup: value (false outside the grid) and whether the cell is inside the grid
  bool lookup(const std::vector<uint8_t> & g, double x, double y, bool * known = nullptr) const;
};

// 4-connected binary dilation, `iterations` times (scipy.ndimage.binary_dilation default)
std::vector<uint8_t> dilate(const std::vector<uint8_t> & g, int nx, int ny, int iterations);

// Free ground outside the lane around a blocked zone (cell centres), for the RViz display: seen as
// ground by the LiDAR, not solid, not inside a lane, within max_offset of the route and 6 m before /
// after the zone. Every `stride`-th cell in raster order, at most max_cells (evenly thinned).
std::vector<std::array<double, 2>> free_space_cells(const Poly & points, const Zone & zone,
  const FreeSpace * free, const LaneModel * lane, const AvoidParams & prm, int stride = 2,
  size_t max_cells = 4000);

// S-curve helpers shared with operation_manager (rejoin the original route after a detour)
double shift_length(double d, double R);
double arc_shift_one(double u, double total, double d, double R);

// points: route ahead (>= 2 points), starting at the vehicle rear axle.
nlohmann::json plan_detour(const Poly & points, const std::vector<Box> & obstacles,
  const LaneModel * lane, const FreeSpace * free, const AvoidParams & prm);
}  // namespace buggy_core
