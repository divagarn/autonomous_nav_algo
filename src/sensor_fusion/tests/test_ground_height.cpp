#include <gtest/gtest.h>

#include <algorithm>
#include <deque>
#include <vector>

#include "sensor_fusion/types.hpp"

using namespace sensor_fusion;

namespace
{
std::deque<double> history(const std::vector<double> & heights, int window = 15)
{
  std::deque<double> h;
  for (double z : heights) {pushGroundHeight(h, z, window);}
  return h;
}
}  // namespace

// Detection heights above ground of the speed breaker on bag_record_3 (205-207 s, EKF tracker track
// 24037): single returns up to 0.9 m kept the old height EMA above 0.30 m; the median is ground.
TEST(GroundHeight, SpeedBreakerIsGround)
{
  const std::vector<double> bump = {0.81, -0.23, 0.92, 0.09, -0.12, 0.74, 0.71, -0.40, -0.27, 0.06};
  EXPECT_TRUE(groundByHeight(history(bump), 0.30, 8, 0.5));    // median (default)
  EXPECT_TRUE(groundByHeight(history(bump), 0.30, 8, 0.35));
  // the EMA of the EKF path was not: alpha = min(1 / (hits + 1), 0.3), seeded with the first height
  // when the confirmed EKF track (already 41 hits) reached the fusion
  double ema = bump[0];
  for (std::size_t i = 1; i < bump.size(); ++i) {ema += std::min(1.0 / (42.0 + i), 0.3) * (bump[i] - ema);}
  EXPECT_GT(ema, 0.30);
}

// The pedestrian in front of the buggy on the same bag (209-219 s): 0.2 - 1.0 m, not ground.
TEST(GroundHeight, PedestrianIsNotGround)
{
  const std::vector<double> person = {0.36, 0.65, 0.58, 0.38, 0.84, 0.57, 1.02, 0.58, 0.93, 0.83, 0.67, 1.04, 0.91,
    0.69, 0.68, 0.60, 1.02, 0.67, 0.42, 0.48, -0.27, 0.35, 0.36, 0.52, 0.74, 0.57, 0.42, 0.23, 0.34, 0.64};
  EXPECT_FALSE(groundByHeight(history(person), 0.30, 5, 0.35));
  EXPECT_FALSE(groundByHeight(history(person), 0.30, 5, 0.5));
}

// Until min_samples heights are known a (stationary) track counts as ground; the window keeps the last ones.
TEST(GroundHeight, NeedsMinSamplesAndKeepsWindow)
{
  EXPECT_TRUE(groundByHeight(history({1.0, 1.0, 1.0, 1.0}), 0.30, 5, 0.35));
  EXPECT_FALSE(groundByHeight(history({1.0, 1.0, 1.0, 1.0, 1.0}), 0.30, 5, 0.35));
  // a window of 5: only the last five (low) heights count
  const auto h = history({1.0, 1.0, 1.0, 1.0, 1.0, 0.1, 0.1, 0.1, 0.1, 0.1}, 5);
  EXPECT_EQ(h.size(), 5u);
  EXPECT_TRUE(groundByHeight(h, 0.30, 5, 0.35));
}
