// LiDAR obstacle clustering in the route corridor (pure C++, no ROS). Same algorithm as
// buggy_pcd_lanelet_map/lidar_obstacle_detector.py:
//   1. sensor frame -> base_link (translation only), height above ground = z + sensor_z
//   2. keep points  min_height < h < max_height,  0 < x < corridor_length + 3
//   3. keep points within half_width of the route centre line (the corridor ahead)
//   4. 8-connected clusters on a cell_size grid, at least min_points points
// Frame-to-frame confirmation (a cluster must also be there in the previous scan) is in the node.
#ifndef BUGGY_CORE__PERCEPTION__OBSTACLE_CLUSTERING_HPP_
#define BUGGY_CORE__PERCEPTION__OBSTACLE_CLUSTERING_HPP_

#include <array>
#include <vector>

namespace buggy_core
{

struct Point3
{
  double x, y, z;
};

struct Cluster
{
  double cx, cy;      // centre of the bounding box (base_link frame)
  double lx, ly;      // box size (>= 0.4 m)
  double h;           // tallest point above ground
  int n;              // points
  double near_x;      // nearest x
  double hmin{0.0};   // lowest point above ground (vertical extent = h - hmin)
};

struct DetectorParams
{
  double sensor_x{1.73}, sensor_z{1.0};
  double min_height{0.20}, max_height{2.00};
  double half_width{1.25}, length{25.0};
  double cell_size{0.30};
  int min_points{5};
};

using Polyline = std::vector<std::array<double, 2>>;

// Route ahead of the vehicle in the vehicle (base_link) frame, at most `length` long, starting at
// the route point nearest the vehicle. Falls back to a straight line ahead when the route is empty
// or (nearly) ended.
Polyline corridor_ahead(
  const std::vector<std::array<double, 2>> & route_map, double rx, double ry, double yaw,
  double length);

// Steps 1-2: points given in the sensor frame (non-finite points are dropped).
std::vector<Point3> to_base_frame(
  const std::vector<Point3> & sensor_points, const DetectorParams & prm);

// Steps 3-4 on points already in base_link.
std::vector<Cluster> detect_clusters(
  const std::vector<Point3> & base_points, const Polyline & corridor, const DetectorParams & prm);

// Raw grid clustering (step 4) - exposed for tests.
std::vector<Cluster> cluster_grid(const std::vector<Point3> & pts, double cell, int min_points);

}  // namespace buggy_core

#endif  // BUGGY_CORE__PERCEPTION__OBSTACLE_CLUSTERING_HPP_
