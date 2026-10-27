#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "sensor_fusion/lane_estimator.hpp"

using namespace sensor_fusion;

namespace
{
/// Pinhole 960 x 540 camera at the radar origin looking forward (radar 0.5 m above the ground).
fusion_common::CameraModel testCamera()
{
  fusion_common::CameraCalibration c;
  c.width = 960; c.height = 540;
  c.fx = c.fy = 500; c.cx = 480; c.cy = 270;
  c.lens = fusion_common::LensModel::PINHOLE;
  c.R << 0, -1, 0,
    0, 0, -1,
    1, 0, 0;
  return fusion_common::CameraModel(c);
}

/**
 * @brief 768 x 512 mask of a ground scene: road where road(x, y), a lane marking where
 *        marking(x, y), background elsewhere (and above the horizon); ground at ground_z (radar frame).
 */
template<class Road, class Marking>
std::vector<uint8_t> renderMask(const fusion_common::CameraModel & cam, Road road, Marking marking,
  double ground_z = -0.5)
{
  const int w = 768, h = 512;
  std::vector<uint8_t> m(w * h, LaneEstimator::kBackground);
  const Eigen::Vector3d o = cam.cameraPositionInRadar();
  for (int v = 0; v < h; ++v) {
    for (int u = 0; u < w; ++u) {
      const Eigen::Vector3d d = cam.pixelToRadarRay({(u + 0.5) * 960.0 / w - 0.5, (v + 0.5) * 540.0 / h - 0.5});
      if (d.z() > -1e-6) {continue;}
      const Eigen::Vector3d g = o + (ground_z - o.z()) / d.z() * d;
      if (marking(g.x(), g.y())) {m[v * w + u] = LaneEstimator::kLane;} else if (road(g.x(), g.y())) {
        m[v * w + u] = LaneEstimator::kRoad;
      }
    }
  }
  return m;
}
}  // namespace

// Straight road from 3 m right to 2.5 m left of the vehicle, one marking 1.5 m left.
TEST(LaneEstimator, StraightRoadEdgesAndMarking)
{
  const auto cam = testCamera();
  const auto mask = renderMask(cam, [](double, double y) {return y > -3.0 && y < 2.5;},
      [](double, double y) {return std::abs(y - 1.5) < 0.08;});
  LaneEstimator est;
  const auto & e = est.update(0.0, mask.data(), 768, 512, 960, 540, cam, -0.5,
    Eigen::Matrix2d::Identity(), Eigen::Vector2d::Zero());
  ASSERT_TRUE(e.road_left.valid);
  ASSERT_TRUE(e.road_right.valid);
  ASSERT_TRUE(e.lane_left.valid);
  EXPECT_FALSE(e.lane_right.valid);   // no marking right of the vehicle
  for (double x : {5.0, 10.0, 15.0}) {
    EXPECT_NEAR(e.road_left.at(x), 2.5, 0.15) << x;
    EXPECT_NEAR(e.road_right.at(x), -3.0, 0.15) << x;
    EXPECT_NEAR(e.lane_left.at(x), 1.5, 0.15) << x;
  }
  EXPECT_LT(e.road_left.points.front().x(), 4.0);   // from the vehicle outwards
  EXPECT_GT(e.road_left.points.back().x(), 15.0);
  EXPECT_NEAR(e.road_width, 5.5, 0.2);
}

// Road curving left (radius 30 m): the quadratic follows it.
TEST(LaneEstimator, CurvedRoad)
{
  const auto cam = testCamera();
  // centre line y = x^2 / (2 R), road +-2.5 m around it
  const double R = 30.0;
  const auto mask = renderMask(cam, [&](double x, double y) {return std::abs(y - x * x / (2 * R)) < 2.5;},
      [](double, double) {return false;});
  LaneEstimator est;
  const auto & e = est.update(0.0, mask.data(), 768, 512, 960, 540, cam, -0.5,
    Eigen::Matrix2d::Identity(), Eigen::Vector2d::Zero());
  ASSERT_TRUE(e.road_left.valid);
  ASSERT_TRUE(e.road_right.valid);
  for (double x : {5.0, 10.0}) {
    EXPECT_NEAR(e.road_left.at(x), x * x / (2 * R) + 2.5, 0.3) << x;
    EXPECT_NEAR(e.road_right.at(x), x * x / (2 * R) - 2.5, 0.3) << x;
  }
}

// Without road in the image there is no boundary; old points move with the ego motion and expire.
TEST(LaneEstimator, EgoMotionAndExpiry)
{
  const auto cam = testCamera();
  const auto road = renderMask(cam, [](double, double y) {return std::abs(y) < 2.0;}, [](double, double) {return false;});
  const std::vector<uint8_t> empty(768 * 512, LaneEstimator::kBackground);
  LaneEstimator est;
  est.update(0.0, road.data(), 768, 512, 960, 540, cam, -0.5, Eigen::Matrix2d::Identity(), Eigen::Vector2d::Zero());
  // 0.1 s later the vehicle moved 1 m forward and 0.5 m left: points shift back and right
  const auto & e = est.update(0.1, empty.data(), 768, 512, 960, 540, cam, -0.5,
    Eigen::Matrix2d::Identity(), Eigen::Vector2d(-1.0, -0.5));
  ASSERT_TRUE(e.road_left.valid);
  EXPECT_NEAR(e.road_left.at(8.0), 1.5, 0.15);
  EXPECT_NEAR(e.road_right.at(8.0), -2.5, 0.15);
  const auto & later = est.update(1.0, empty.data(), 768, 512, 960, 540, cam, -0.5,
    Eigen::Matrix2d::Identity(), Eigen::Vector2d::Zero());
  EXPECT_FALSE(later.road_left.valid);   // older than keep_s
}

// A person standing on the road (non-road blob left of the vehicle) does not become the road edge;
// a planter strip with nothing behind it does.
TEST(LaneEstimator, ObjectOnRoadIsNoEdge)
{
  const auto cam = testCamera();
  const auto mask = renderMask(cam,
      [](double x, double y) {
        const bool person = x > 5.0 && x < 7.0 && y > 0.8 && y < 1.4;   // 0.6 m wide, on the road
        return y > -3.0 && y < 3.0 && !person;
      },
      [](double, double) {return false;});
  LaneEstimator est;
  const auto & e = est.update(0.0, mask.data(), 768, 512, 960, 540, cam, -0.5,
    Eigen::Matrix2d::Identity(), Eigen::Vector2d::Zero());
  ASSERT_TRUE(e.road_left.valid);
  for (double x : {6.0, 10.0}) {EXPECT_NEAR(e.road_left.at(x), 3.0, 0.2) << x;}
  EXPECT_NEAR(e.road_right.at(6.0), -3.0, 0.2);
}

// Sharp left curve (radius 15 m): the search follows the road where the straight-ahead line has left
// it (beyond 9 m), the edges stay on their circles.
TEST(LaneEstimator, SharpCurveIsFollowed)
{
  const auto cam = testCamera();
  // centre line: circle of radius R around (0, R); road +-2.5 m around it
  const double R = 15.0;
  auto radius = [&](double x, double y) {return std::hypot(x, y - R);};
  const auto mask = renderMask(cam, [&](double x, double y) {return x > 0 && std::abs(radius(x, y) - R) < 2.5;},
      [](double, double) {return false;});
  LaneEstimator est;
  const auto & e = est.update(0.0, mask.data(), 768, 512, 960, 540, cam, -0.5,
    Eigen::Matrix2d::Identity(), Eigen::Vector2d::Zero());
  ASSERT_TRUE(e.road_left.valid);
  ASSERT_TRUE(e.road_right.valid);
  for (const auto & p : e.road_left.points) {EXPECT_NEAR(radius(p.x(), p.y()), R - 2.5, 0.3) << p.transpose();}
  for (const auto & p : e.road_right.points) {EXPECT_NEAR(radius(p.x(), p.y()), R + 2.5, 0.3) << p.transpose();}
  // beyond x = 9 m the road centre is more than 2.5 m left of the straight-ahead line
  EXPECT_GT(e.road_right.points.back().y(), 2.0);
}

// An object almost as wide as the road beside the vehicle (too little road behind it to be bridged) ends
// the rows at the object from the mask alone; with its detector box those rows have a hidden edge and
// the fitted edge stays at the road border.
TEST(LaneEstimator, OccluderBoxHidesEdge)
{
  const auto cam = testCamera();
  auto object = [](double x, double y) {return x > 6.0 && x < 7.5 && y > 0.5 && y < 2.9;};
  // camera 1.5 m above the ground (ground_z = -1.5), object 0.5 m tall: hides the edge from 7.5 to ~11 m
  const double gz = -1.5;
  const auto mask = renderMask(cam, [&](double x, double y) {return std::abs(y) < 3.0 && !object(x, y);},
      [](double, double) {return false;}, gz);
  // detector box: image bounds of the object
  Eigen::Vector4d box(1e9, 1e9, -1e9, -1e9);
  for (double x : {6.0, 7.5}) {
    for (double y : {0.5, 2.9}) {
      for (double z : {gz, gz + 0.5}) {
        const auto uv = cam.projectRadarPoint({x, y, z}, 1e4);
        ASSERT_TRUE(uv);
        box << std::min(box(0), uv->x()), std::min(box(1), uv->y()), std::max(box(2), uv->x()),
          std::max(box(3), uv->y());
      }
    }
  }
  LaneEstimator plain, boxed;
  plain.update(0.0, mask.data(), 768, 512, 960, 540, cam, gz, Eigen::Matrix2d::Identity(), Eigen::Vector2d::Zero());
  const auto & b = boxed.update(0.0, mask.data(), 768, 512, 960, 540, cam, gz,
    Eigen::Matrix2d::Identity(), Eigen::Vector2d::Zero(), {box});
  // rows of the object's footprint (bottom quarter of the box): the left end lies in the box without
  // it (edge at the object), with it there is no left edge (hidden)
  auto ends_in_box = [&](const LaneEstimator & est) {
      int n = 0, rows = 0;
      for (const auto & r : est.rows()) {
        const double v = (r.v + 0.5) * 540.0 / 512;
        if (v < box(3) - 0.25 * (box(3) - box(1)) || v > box(3)) {continue;}
        ++rows;
        n += r.left >= 0 && (r.left + 0.5) * 960.0 / 768 > box(0);
      }
      return std::make_pair(n, rows);
    };
  const auto plain_rows = ends_in_box(plain), boxed_rows = ends_in_box(boxed);
  ASSERT_GT(plain_rows.second, 0);
  EXPECT_EQ(plain_rows.first, plain_rows.second);
  EXPECT_EQ(boxed_rows.first, 0);
  ASSERT_TRUE(b.road_left.valid);
  for (double x : {5.0, 6.5, 10.0}) {EXPECT_NEAR(b.road_left.at(x), 3.0, 0.2) << x;}
}
