#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "sensor_fusion/path_tracker.hpp"

using namespace sensor_fusion;

namespace
{
/// Road edge y = c0 + c1 (x - x0) + c2 (x - x0)^2 from x0 to x1 (as a fitted LaneBoundary).
LaneBoundary edge(double c0, double x0 = 3.0, double x1 = 20.0, double c1 = 0.0, double c2 = 0.0)
{
  LaneBoundary b;
  b.valid = true;
  b.origin = {x0, 0.0};
  b.dir = Eigen::Vector2d::UnitX();
  b.coef = Eigen::Vector3d(c0, c1, c2);
  b.s_min = 0.0;
  b.s_max = x1 - x0;
  for (double s = 0; s <= b.s_max + 1e-9; s += 1.0) {b.points.push_back(b.curveAt(s));}
  return b;
}

LaneEstimate road(double left, double right)
{
  LaneEstimate e;
  e.road_left = edge(left);
  e.road_right = edge(right);
  e.road_width = left - right;
  return e;
}

const Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
const Eigen::Vector2d kStart(-2.2, 0.0);   // base_link 2.2 m behind the radar (working frame = radar)
}  // namespace

// Straight road, vehicle 1 m left of its middle: 21 points 1 m apart from base_link, on the vehicle's
// line (parallel to the edges, not pulled to the middle), lane = +- (1.35 / 2 + 0.5).
TEST(PathTracker, StraightRoadFromTheVehicle)
{
  PathTracker tracker;
  TrackedPath p;
  for (int i = 0; i < 20; ++i) {p = tracker.update(road(2.0, -4.0), kStart, 0.0, I, Eigen::Vector2d::Zero(), 0.0);}
  ASSERT_TRUE(p.valid);
  ASSERT_EQ(p.centre.size(), 21u);
  EXPECT_NEAR((p.centre.front() - kStart).norm(), 0.0, 1e-9);
  for (std::size_t i = 0; i < p.centre.size(); ++i) {
    EXPECT_NEAR(p.centre[i].x(), kStart.x() + i, 0.05) << i;
    EXPECT_NEAR(p.centre[i].y(), 0.0, 0.05) << i;
    EXPECT_NEAR(p.left[i].y() - p.centre[i].y(), 1.175, 1e-6) << i;
    EXPECT_NEAR(p.right[i].y() - p.centre[i].y(), -1.175, 1e-6) << i;
  }
  EXPECT_EQ(p.reference, 0);   // left edge is nearer (2 m)
  EXPECT_GT(p.measured, 10);
}

// Edges jittering +-0.3 m every update: the tracked path moves far less.
TEST(PathTracker, NoisyEdgesGiveASteadyPath)
{
  PathTracker tracker;
  std::mt19937 rng(1);
  std::uniform_real_distribution<double> noise(-0.3, 0.3);
  std::vector<double> y15;
  for (int i = 0; i < 100; ++i) {
    const double n = noise(rng);
    const auto & p = tracker.update(road(3.0 + n, -3.0 + n), kStart, 0.0, I, Eigen::Vector2d::Zero(), 0.0);
    if (i >= 30) {y15.push_back(p.centre[17].y());}   // 15 m ahead of the radar
  }
  double mean = 0, var = 0;
  for (double y : y15) {mean += y;}
  mean /= y15.size();
  for (double y : y15) {var += (y - mean) * (y - mean);}
  const double sd = std::sqrt(var / y15.size());
  EXPECT_NEAR(mean, 0.0, 0.1);
  EXPECT_LT(sd, 0.065);   // edges: 0.17 m standard deviation
}

// A single 2 m jump of the edges (wrong mask) is ignored; a jump that stays is followed: here the road
// moves 2 m left, the path keeps the vehicle's distance to the left edge but stays a lane half width
// (1.175 m) inside the right edge, now at -1 m.
TEST(PathTracker, GateIgnoresOneOffJumps)
{
  PathTracker tracker;
  for (int i = 0; i < 20; ++i) {tracker.update(road(3.0, -3.0), kStart, 0.0, I, Eigen::Vector2d::Zero(), 0.0);}
  const double before = tracker.path().centre[15].y();
  tracker.update(road(5.0, -1.0), kStart, 0.0, I, Eigen::Vector2d::Zero(), 0.0);
  EXPECT_NEAR(tracker.path().centre[15].y(), before, 0.02);
  for (int i = 0; i < 60; ++i) {tracker.update(road(5.0, -1.0), kStart, 0.0, I, Eigen::Vector2d::Zero(), 0.0);}
  EXPECT_NEAR(tracker.path().centre[15].y(), 0.175, 0.05);
}

// Driving forward without edges: the points stay on the ground (move back with the ego motion) and the
// path is re-extended at the far end; after hold_distance it restarts from the ego curvature.
TEST(PathTracker, TracksWithoutEdgesThenRestarts)
{
  PathTracker tracker;
  for (int i = 0; i < 20; ++i) {tracker.update(road(2.0, -4.0), kStart, 0.0, I, Eigen::Vector2d::Zero(), 0.0);}
  const LaneEstimate none;
  // 1 m forward per update: p_now = p_prev - (1, 0)
  for (int i = 0; i < 10; ++i) {tracker.update(none, kStart, 0.0, I, Eigen::Vector2d(-1.0, 0.0), 0.0);}
  EXPECT_NEAR(tracker.path().unmeasured_m, 10.0, 1e-6);
  for (const auto & q : tracker.path().centre) {EXPECT_NEAR(q.y(), 0.0, 0.05);}
  // turning without edges past hold_distance: a new path from the ego curvature (limited to 1 / 4.4 m),
  // fading to straight, never turning more than 60 deg from the vehicle heading
  for (int i = 0; i < 6; ++i) {tracker.update(none, kStart, 0.0, I, Eigen::Vector2d(-1.0, 0.0), 1.0);}
  const auto & c = tracker.path().centre;
  const double kmax = std::tan(20.0 * M_PI / 180.0) / 1.6;
  const double h1 = std::atan2(c[1].y() - c[0].y(), c[1].x() - c[0].x());
  EXPECT_NEAR(h1, 0.5 * kmax, 0.02);   // first step: tangent to the vehicle heading, at the limit
  double prev = h1;
  for (std::size_t i = 2; i < c.size(); ++i) {
    const double h = std::atan2(c[i].y() - c[i - 1].y(), c[i].x() - c[i - 1].x());
    EXPECT_GE(h, prev - 1e-6) << i;                    // turns left ...
    EXPECT_LE(h, 60.0 * M_PI / 180.0 + 1e-6) << i;     // ... at most 60 deg
    prev = h;
  }
  EXPECT_LT(prev - std::atan2(c[19].y() - c[18].y(), c[19].x() - c[18].x()), 0.05);   // nearly straight at the end
}

// A road edge bending faster than the buggy can turn (radius 3 m) gives a path at the curvature limit.
TEST(PathTracker, CurvatureLimitedByTheSteering)
{
  PathTracker tracker;
  LaneEstimate e;
  e.road_left = edge(3.0, 3.0, 20.0, 0.0, 1.0 / 6.0);   // y'' = 1/3 -> radius ~3 m near the start
  for (int i = 0; i < 60; ++i) {tracker.update(e, kStart, 0.0, I, Eigen::Vector2d::Zero(), 0.0);}
  const auto & c = tracker.path().centre;
  const double kmax = std::tan(20.0 * M_PI / 180.0) / 1.6;
  double prev = 0.0;
  for (std::size_t i = 1; i < c.size(); ++i) {
    const double h = std::atan2(c[i].y() - c[i - 1].y(), c[i].x() - c[i - 1].x());
    EXPECT_LE(std::abs(std::remainder(h - prev, 2 * M_PI)), (i == 1 ? 0.5 : 1.0) * kmax + 1e-6) << i;
    prev = h;
  }
}
