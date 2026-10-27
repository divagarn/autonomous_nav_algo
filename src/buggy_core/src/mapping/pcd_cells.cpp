#include "buggy_core/mapping/pcd_cells.hpp"

#include <algorithm>

namespace buggy_core
{
bool circle_touches(const CellRect & r, double cx, double cy, double radius)
{
  const double dx = std::max({r.min_x - cx, 0.0, cx - r.max_x});
  const double dy = std::max({r.min_y - cy, 0.0, cy - r.max_y});
  return dx * dx + dy * dy <= radius * radius;
}

CellSelection select_cells(const std::vector<CellRect> & rects, const std::vector<std::string> & ids,
  double cx, double cy, double radius, const std::vector<std::string> & cached)
{
  CellSelection s;
  std::set<std::string> have(cached.begin(), cached.end()), inside;
  for (size_t i = 0; i < rects.size(); ++i) {
    if (!circle_touches(rects[i], cx, cy, radius)) {continue;}
    inside.insert(ids[i]);
    if (!have.count(ids[i])) {s.add.push_back(i);}
  }
  for (const auto & c : cached) {if (!inside.count(c)) {s.remove.push_back(c);}}
  return s;
}
}  // namespace buggy_core
