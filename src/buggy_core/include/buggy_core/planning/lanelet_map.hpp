// Lanelet2 OSM reader (local_x / local_y tags) and the lane geometry used by the route planner.
// Port of Lanelet / load_lanelets / project in route_planner.py.
#pragma once

#include <string>
#include <utility>
#include <vector>

#include "buggy_core/planning/path_utils.hpp"

namespace buggy_core
{
struct Lanelet
{
  std::string id;
  std::pair<std::string, std::string> first, last;    // (left node, right node) ids
  Poly left, right, center;
  double length = 0.0;

  Lanelet(std::string lid, const Poly & l, const Poly & r,
    std::pair<std::string, std::string> f, std::pair<std::string, std::string> la);
};

// Throws std::runtime_error when the file cannot be read.
std::vector<Lanelet> load_lanelets(const std::string & path);

// Index of the centreline point nearest p (x, y) and its xy distance.
std::pair<size_t, double> project(const Lanelet & l, double px, double py);
}  // namespace buggy_core
