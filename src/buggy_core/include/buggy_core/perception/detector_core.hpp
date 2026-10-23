// One LiDAR obstacle detection step without ROS types: sensor frame -> base_link, height above the
// LOCAL ground (when the ground points of the same scan are given), corridor, clustering, thin / low
// rejection and the two-scan confirmation. Port of LidarObstacleDetector.on_cloud (Python reference).
#pragma once

#include <array>
#include <memory>
#include <vector>

#include "buggy_core/perception/ground_model.hpp"
#include "buggy_core/perception/obstacle_clustering.hpp"

namespace buggy_core
{
struct GroundAwareParams
{
  bool aware = true;
  double min_height = 0.30;           // above the LOCAL ground (the ground estimate is a few cm noisy)
  bool reject_thin_low = true;
  double thin_extent = 0.08;          // vertical extent below this ...
  double low_height = 0.50;           // ... and lower than this above ground = one beam ring on the road
  double max_age = 0.3;               // s, ground cloud vs obstacle cloud stamp
};

class DetectorCore
{
public:
  DetectorCore(DetectorParams p, GroundAwareParams g) : p_(p), g_(g) {}

  struct Result
  {
    std::vector<Cluster> clusters;    // this scan (after thin / low rejection)
    std::vector<Cluster> confirmed;   // also present in the previous scan
    bool used_ground_model = false;
    double plane_tilt_deg = 0.0;
  };

  // sensor-frame points of the non-ground cloud; `ground` (sensor frame) or null = flat assumption
  Result process(const std::vector<Point3> & nonground, const std::vector<Point3> * ground,
    const Polyline & corridor);

  int rejected() const { return rejected_; }
  const GroundAwareParams & ground_params() const { return g_; }

private:
  DetectorParams p_;
  GroundAwareParams g_;
  std::vector<std::array<double, 2>> prev_;
  int rejected_ = 0;
};
}  // namespace buggy_core
