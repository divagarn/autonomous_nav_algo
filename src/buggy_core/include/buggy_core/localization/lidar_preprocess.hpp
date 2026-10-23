// LiDAR pre-processing for NDT scan matching, without ROS types: drop non-finite points, range
// limits, self-body crop, voxel-grid centroid downsample. Port of lidar_preprocess.py (the Python
// file is the reference; float32 points, float64 centroid sums, voxels in ascending key order).
#pragma once

#include <cstddef>
#include <vector>

namespace buggy_core
{
struct PreprocessParams
{
  float voxel = 0.3f, max_range = 40.0f, min_range = 0.5f;
  float x_min = -2.5f, x_max = 0.6f, y_min = -0.9f, y_max = 0.9f, z_min = -1.2f, z_max = 0.9f;
  size_t min_points = 100;      // fewer after filtering -> no output
};

// xyz: n x 3 floats. Returns false when too few points remain; otherwise `out` holds the voxel
// centroids as x y z triples.
bool preprocess_cloud(const float * xyz, size_t n, const PreprocessParams & p, std::vector<float> & out);

// voxel-grid centroids of m points (x y z triples), voxel size `leaf`
std::vector<float> voxel_centroids(const std::vector<float> & pts, float leaf);
}  // namespace buggy_core
