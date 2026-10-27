// Point-cloud map split into cells (one cell per PCD file, rectangle from pointcloud_map_metadata.yaml),
// and the cell selection used by the differential / partial map services. No ROS types.
#pragma once

#include <set>
#include <string>
#include <vector>

namespace buggy_core
{
struct CellRect { double min_x, min_y, max_x, max_y; };

struct CellSelection
{
  std::vector<size_t> add;            // cells inside the area that the client does not have yet
  std::vector<std::string> remove;    // cached ids that are no longer inside the area (or unknown)
};

// does the circle (cx, cy, r) touch the rectangle?
bool circle_touches(const CellRect & r, double cx, double cy, double radius);

// area = circle. `ids[i]` is the id of cell i; `cached` is what the client already holds.
CellSelection select_cells(const std::vector<CellRect> & rects, const std::vector<std::string> & ids,
  double cx, double cy, double radius, const std::vector<std::string> & cached);
}  // namespace buggy_core
