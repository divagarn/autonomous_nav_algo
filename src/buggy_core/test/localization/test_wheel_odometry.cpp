// Wheel odometry integrator against the Python node (golden_odom.txt, tools/golden/gen_golden.py).
#include <gtest/gtest.h>

#include <fstream>
#include <sstream>

#include "buggy_core/localization/wheel_odometry.hpp"

using namespace buggy_core;

TEST(WheelOdometry, MatchesPython)
{
  std::ifstream f(std::string(GOLDEN_DIR) + "/golden_odom.txt");
  ASSERT_TRUE(f.good()) << "run tools/golden/gen_golden.py first";
  WheelIntegrator w(1.6);
  std::string line;
  int n = 0;
  double worst = 0;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') {continue;}
    std::istringstream is(line);
    double dt, steer, speed, x, y, qz, qw, v, az;
    is >> dt >> steer >> speed >> x >> y >> qz >> qw >> v >> az;
    const auto & s = w.update(dt, steer, speed);
    EXPECT_NEAR(s.x, x, 1e-12);
    EXPECT_NEAR(s.y, y, 1e-12);
    EXPECT_NEAR(std::sin(s.yaw / 2.0), qz, 1e-12);
    EXPECT_NEAR(std::cos(s.yaw / 2.0), qw, 1e-12);
    EXPECT_NEAR(s.speed_ms, v, 1e-12);
    EXPECT_NEAR(s.angular_z, az, 1e-12);
    worst = std::max({worst, std::abs(s.x - x), std::abs(s.y - y)});
    ++n;
  }
  EXPECT_GT(n, 500);
  std::printf("odometry golden: %d updates, worst position difference %.2e m\n", n, worst);
}

TEST(WheelOdometry, StraightLine)
{
  WheelIntegrator w;
  for (int i = 0; i < 100; ++i) {w.update(0.1, 0.0, 3.6);}      // 1 m/s for 10 s
  EXPECT_NEAR(w.state().x, 10.0, 1e-9);
  EXPECT_NEAR(w.state().y, 0.0, 1e-9);
  EXPECT_DOUBLE_EQ(w.state().angular_z, 0.0);
}

TEST(WheelOdometry, RightSteeringTurnsClockwise)
{
  WheelIntegrator w;
  w.update(0.1, 10.0, 7.2);
  EXPECT_LT(w.state().angular_z, 0.0);                          // +right steering -> negative yaw rate
}
