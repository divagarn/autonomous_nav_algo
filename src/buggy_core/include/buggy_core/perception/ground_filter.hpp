// LiDAR ground removal and vehicle self-hit filtering, without ROS types.
// Port of buggy_perception/segment_ground.py (the 'segment' backend: Himmelsbach-style angular
// segment line-fit, no GPU / torch) and buggy_perception/self_hit_filter.py. The Python files
// are the reference; the arithmetic is done in the same precision (float32 points, float64 cells).
//   Himmelsbach, Hummel, Franke, Gross: "Fast Segmentation of 3D Point Clouds for Ground
//   Vehicles", IEEE Intelligent Vehicles Symposium 2010.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace buggy_core
{
struct GroundParams
{
  float max_distance = 25.0f, min_height = -1.60f, max_height = 2.00f;
  double seed_z_min = -1.45, seed_z_max = -0.45;
  int num_segments = 180;
  double bin_size = 0.5;
  double max_slope_deg = 10.0;
  double max_height_jump = 0.20;
  float base_thresh = 0.15f, distance_scale = 0.012f, max_adaptive = 0.25f;
  double ground_max_above = 0.15, ground_max_below = 0.20;
  size_t min_points = 50;
};

// xyz: n x 3 floats (finite). mask[i] = 1 where the point is ground. Returns false (no output,
// like the Python node) when there are too few points after the range / height pre-filter.
bool segment_ground_mask(const float * xyz, size_t n, const GroundParams & p, std::vector<uint8_t> & mask);

struct SelfHitParams
{
  float min_range = 1.0f;
  float x_min = -2.39f, x_max = 0.53f, y_min = -0.72f, y_max = 0.72f, z_min = -1.05f, z_max = -0.10f;
  float sensor_offset_x = 1.73f, sensor_offset_y = 0.0f;
  float half_length = 4.9f / 2.0f, half_width = 2.9f / 2.0f;
};

// points are `stride` bytes apart, x y z as the first three floats. keep[i] = 1 to keep.
void self_hit_keep(const uint8_t * data, size_t n, size_t stride, const SelfHitParams & p,
  std::vector<uint8_t> & keep);

// RGB packed like the Python publisher: ground by range, non-ground by height.
void colour_ground(const float * xyz, const std::vector<size_t> & idx, std::vector<uint32_t> & rgb);
void colour_non_ground(const float * xyz, const std::vector<size_t> & idx, std::vector<uint32_t> & rgb);
}  // namespace buggy_core
