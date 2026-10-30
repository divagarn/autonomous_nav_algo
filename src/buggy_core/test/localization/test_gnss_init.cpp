// GNSS automatic start pose: map conversion and the gating (when a pose may be sent).
#include <gtest/gtest.h>

#include <cmath>

#include "buggy_core/localization/gnss_init_core.hpp"

using namespace buggy_core;

namespace
{
Projector geo()
{
  Projector p;
  p.type = Projector::Type::LocalCartesian;
  p.lat0 = 28.6139; p.lon0 = 77.2090;
  return p;
}

GnssFix fix_at(double east_m, double north_m, double heading_cw, double t, int state = 4, double std = 0.02)
{
  GnssFix f;
  f.lat = 28.6139 + north_m / 110800.0;
  f.lon = 77.2090 + east_m / (111319.49 * std::cos(28.6139 * M_PI / 180));
  f.heading_cw_deg = heading_cw; f.state = state; f.std_m = std; f.time = t;
  return f;
}

struct Rig
{
  GnssInitParams p;
  GnssInitCore core;
  double t = 100.0;
  std::string loc = "uninitialized";
  explicit Rig(GnssInitParams pp = {}) : p(pp), core(p, geo()) {}
  // one 0.2 s step with a fresh fix
  std::optional<InitPose> tick(int state = 4, double std = 0.02)
  {
    t += 0.2;
    core.on_fix(fix_at(100.0, 50.0, 90.0, t, state, std));
    return core.step(t, loc);
  }
  int count_sent(double sec, int state = 4, double std = 0.02)
  {
    int n = 0;
    for (int i = 0; i < static_cast<int>(sec / 0.2); ++i) {n += tick(state, std).has_value();}
    return n;
  }
};
}  // namespace

TEST(GnssInit, HeadingAndPositionInTheMapFrame)
{
  GnssInitCore c(GnssInitParams{}, geo());
  auto p = c.to_map(fix_at(100.0, 0.0, 90.0, 0.0));            // heading east
  EXPECT_NEAR(p.x, 100.0, 0.5); EXPECT_NEAR(p.y, 0.0, 0.5); EXPECT_NEAR(p.yaw, 0.0, 1e-9);
  p = c.to_map(fix_at(0.0, 30.0, 0.0, 0.0));                  // heading north = +90 deg
  EXPECT_NEAR(p.y, 30.0, 0.5); EXPECT_NEAR(p.yaw, M_PI / 2, 1e-9);
  p = c.to_map(fix_at(0.0, 0.0, 270.0, 0.0));                 // heading west = 180 deg
  EXPECT_NEAR(std::abs(p.yaw), M_PI, 1e-9);
  p = c.to_map(fix_at(0.0, 0.0, 180.0, 0.0));                 // heading south = -90 deg
  EXPECT_NEAR(p.yaw, -M_PI / 2, 1e-9);
}

TEST(GnssInit, LeverArmMovesTheRearAxleBehindTheAntenna)
{
  GnssInitParams pp;
  pp.lever_x = -0.8;                                           // antenna 0.8 m ahead of the rear axle
  GnssInitCore c(pp, geo());
  const auto p = c.to_map(fix_at(100.0, 0.0, 90.0, 0.0));     // driving east
  EXPECT_NEAR(p.x, 99.2, 0.5);
  pp.heading_offset_deg = 10.0;
  GnssInitCore c2(pp, geo());
  EXPECT_NEAR(c2.to_map(fix_at(0.0, 0.0, 90.0, 0.0)).yaw, 10.0 * M_PI / 180, 1e-9);
}

TEST(GnssInit, SendsOnceAfterTheFixWasStable)
{
  Rig r;
  EXPECT_EQ(r.count_sent(2.0), 0) << "not before stable_sec";
  EXPECT_EQ(r.count_sent(2.0), 1) << "exactly one pose";
  EXPECT_EQ(r.core.attempts(), 1);
  r.loc = "aligning";
  EXPECT_EQ(r.count_sent(30.0), 0) << "never while NDT is aligning";
}

TEST(GnssInit, NoPoseForWeakFixes)
{
  Rig r;
  EXPECT_EQ(r.count_sent(10.0, 8), 0) << "RTK fixed WITHOUT heading (state 8) is not enough";
  EXPECT_EQ(r.count_sent(10.0, 1), 0) << "single-point fix";
  EXPECT_EQ(r.count_sent(10.0, 4, 1.5), 0) << "position std 1.5 m";
  EXPECT_NE(r.core.status().find("std"), std::string::npos) << r.core.status();
  Rig f;                                                       // float allowed when configured
  f.p.accepted_states = {4, 5};
  GnssInitCore c(f.p, geo());
  c.on_fix(fix_at(0, 0, 0, 100.0, 5)); c.step(100.0, "uninitialized");
  c.on_fix(fix_at(0, 0, 0, 104.0, 5));
  EXPECT_TRUE(c.step(104.0, "uninitialized").has_value());
}

TEST(GnssInit, ABreakRestartsTheStableTime)
{
  Rig r;
  EXPECT_EQ(r.count_sent(2.4), 0);
  r.count_sent(0.4, 1);                                        // fix degraded for a moment
  EXPECT_EQ(r.count_sent(2.4), 0) << "stable time counts again from zero";
  EXPECT_EQ(r.count_sent(1.0), 1);
}

TEST(GnssInit, OldFixCounts_AsNoFix)
{
  GnssInitCore c(GnssInitParams{}, geo());
  c.on_fix(fix_at(0, 0, 0, 100.0));
  EXPECT_FALSE(c.step(110.0, "uninitialized").has_value());
  EXPECT_NE(c.status().find("old"), std::string::npos) << c.status();
}

TEST(GnssInit, RetriesThenGivesUp)
{
  Rig r;
  EXPECT_EQ(r.count_sent(4.0), 1);
  EXPECT_EQ(r.count_sent(15.0), 0) << "waits retry_sec (20 s) for NDT";
  r.loc = "align_failed";
  EXPECT_EQ(r.count_sent(20.0), 1) << "second attempt";
  EXPECT_EQ(r.count_sent(25.0), 1) << "third attempt";
  EXPECT_EQ(r.count_sent(60.0), 0) << "max_attempts reached: the operator clicks";
  EXPECT_NE(r.core.status().find("giving up"), std::string::npos);
  r.loc = "converged"; r.tick();
  r.loc = "uninitialized";
  EXPECT_GE(r.count_sent(10.0), 1) << "after a convergence the attempts start again";
}

TEST(GnssInit, LostLocalizationIsReinitialisedAfterLostSec)
{
  Rig r;
  r.loc = "converged"; r.count_sent(1.0);
  r.loc = "lost";
  EXPECT_EQ(r.count_sent(4.0), 0) << "lost_sec (5 s) not reached";
  EXPECT_EQ(r.count_sent(4.0), 1);
  Rig off;
  off.p.reinit_when_lost = false;
  GnssInitCore c(off.p, geo());
  c.on_fix(fix_at(0, 0, 0, 100.0)); c.step(100.0, "lost");
  c.on_fix(fix_at(0, 0, 0, 120.0));
  EXPECT_FALSE(c.step(120.0, "lost").has_value());
}

TEST(GnssInit, LocalMapDisablesIt)
{
  GnssInitCore c(GnssInitParams{}, Projector{});
  EXPECT_FALSE(c.usable());
  c.on_fix(fix_at(0, 0, 0, 100.0));
  EXPECT_FALSE(c.step(110.0, "uninitialized").has_value());
  EXPECT_NE(c.status().find("Local"), std::string::npos);
}

TEST(GnssInit, TheOperatorSwitchStopsAndRestartsIt)
{
  Rig r;
  r.core.set_enabled(false);
  EXPECT_EQ(r.count_sent(30.0), 0) << "disabled: never";
  EXPECT_NE(r.core.status().find("disabled"), std::string::npos);
  r.core.set_enabled(true);
  EXPECT_EQ(r.count_sent(2.0), 0) << "after enabling the fix must be stable for stable_sec again";
  EXPECT_EQ(r.count_sent(2.0), 1);
  r.core.set_enabled(false);
  r.core.set_enabled(true);
  EXPECT_EQ(r.core.attempts(), 0) << "switching resets the attempts";
}

TEST(GnssInit, FixGoodReportsWhy)
{
  Rig r;
  std::string why;
  r.core.on_fix(fix_at(0, 0, 0, r.t, 5, 0.02));
  EXPECT_FALSE(r.core.fix_good(r.t, &why));
  EXPECT_NE(why.find("state 5"), std::string::npos) << why;
  r.core.on_fix(fix_at(0, 0, 0, r.t, 4, 0.02));
  EXPECT_TRUE(r.core.fix_good(r.t, &why));
}
