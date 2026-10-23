// Route planning against the Python RoutePlanner on every map in data/maps
// (golden_routes.txt from tools/golden/gen_golden.py): lane geometry, successors incl. ring links,
// shortest lane chains, smoothed paths, one-lap paths. Plus path_utils behaviour tests.
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>

#include "buggy_core/planning/route_core.hpp"

using namespace buggy_core;

TEST(Routes, MatchPython)
{
  std::ifstream f(std::string(GOLDEN_DIR) + "/golden_routes.txt");
  ASSERT_TRUE(f.good()) << "run tools/golden/gen_golden.py first";
  std::unique_ptr<RouteCore> core;
  std::string line, tag;
  int lanes = 0, plans = 0, laps = 0, succs = 0;
  double worst_pt = 0.0, worst_steer = 0.0;
  const auto next = [&](std::string & out) { return static_cast<bool>(std::getline(f, out)); };
  while (next(line)) {
    if (line.empty() || line[0] == '#') {continue;}
    std::istringstream is(line);
    is >> tag;
    if (tag == "MAP") {
      std::string name;
      is >> name;
      const std::string osm = std::string(MAPS_DIR) + "/" + name + "/lanelet2_map.osm";
      ASSERT_TRUE(std::filesystem::exists(osm)) << osm;
      core = std::make_unique<RouteCore>(load_lanelets(osm), RouteParams{});
    } else if (tag == "L") {
      std::string id; size_t n; double len, x0, y0, x1, y1;
      is >> id >> n >> len >> x0 >> y0 >> x1 >> y1;
      const auto & l = core->lanelet(id);
      ASSERT_EQ(l.center.size(), n);
      EXPECT_NEAR(l.length, len, 1e-9);
      EXPECT_NEAR(l.center.front()[0], x0, 1e-9); EXPECT_NEAR(l.center.front()[1], y0, 1e-9);
      EXPECT_NEAR(l.center.back()[0], x1, 1e-9); EXPECT_NEAR(l.center.back()[1], y1, 1e-9);
      ++lanes;
    } else if (tag == "S") {
      std::string id, s;
      is >> id;
      std::vector<std::string> exp;
      while (is >> s) {exp.push_back(s);}
      EXPECT_EQ(core->succ(id), exp) << "successors of " << id;
      ++succs;
    } else if (tag == "P") {
      double sx, sy, gx, gy;
      is >> sx >> sy >> gx >> gy;
      std::string a;
      is >> a;
      const auto res = core->plan(sx, sy, gx, gy);
      if (a == "none") {EXPECT_FALSE(res.has_value()); continue;}
      size_t si = std::stoul(a), gi;
      is >> gi;
      std::vector<std::string> chain;
      std::string s;
      while (is >> s) {chain.push_back(s);}
      ASSERT_TRUE(res.has_value()) << line;
      EXPECT_EQ(res->si, si); EXPECT_EQ(res->gi, gi);
      ASSERT_EQ(res->chain, chain) << line;
      // smoothed path follows
      std::string t;
      ASSERT_TRUE(next(t));
      std::istringstream ts(t);
      std::string tt; size_t n; double before, after;
      ts >> tt >> n >> before >> after;
      const auto sm = core->smooth_pts(core->chain_points(res->chain, res->si, res->gi));
      ASSERT_EQ(sm.pts.size(), n);
      EXPECT_NEAR(sm.before, before, 1e-9);
      worst_steer = std::max(worst_steer, std::abs(sm.after - after));
      for (size_t i = 0; i < n; ++i) {
        std::string pl; next(pl);
        std::istringstream ps(pl);
        double x, y, z; ps >> x >> y >> z;
        worst_pt = std::max({worst_pt, std::abs(sm.pts[i][0] - x), std::abs(sm.pts[i][1] - y), std::abs(sm.pts[i][2] - z)});
      }
      ++plans;
    } else if (tag == "A") {
      double hx, hy; std::string a;
      is >> hx >> hy >> a;
      const auto lap = core->lap_points(hx, hy);
      if (a == "none") {EXPECT_FALSE(lap.has_value()); continue;}
      std::vector<std::string> chain{a};
      std::string s;
      while (is >> s) {chain.push_back(s);}
      ASSERT_TRUE(lap.has_value());
      EXPECT_EQ(lap->chain, chain);
      std::string l2; next(l2);
      std::istringstream ls(l2);
      size_t n; double len;
      ls >> n >> len;
      EXPECT_EQ(lap->pts.size(), n);
      EXPECT_NEAR(length3(lap->pts), len, 1e-8);
      ++laps;
    }
  }
  std::printf("routes golden: %d lanes, %d successor lists, %d plans, %d laps; worst smoothed point diff %.2e m, worst steer diff %.2e deg\n",
    lanes, succs, plans, laps, worst_pt, worst_steer);
  EXPECT_GT(lanes, 10);
  EXPECT_GT(plans, 40);
  EXPECT_GE(laps, 10);
  EXPECT_LT(worst_pt, 1e-6);
}

TEST(PathUtils, SmoothingRespectsSteeringLimit)
{
  Poly pts;
  for (int i = 0; i <= 40; ++i) {pts.push_back({i * 0.5, 0.0, 0.0});}
  for (int i = 1; i <= 20; ++i) {pts.push_back({20.0 + 0.02 * i, i * 0.5, 0.0});}    // ~90 deg corner
  const auto r = smooth(pts, 1.6, 18.0, 0.6);
  EXPECT_GT(r.before, 18.0);
  EXPECT_LT(r.after, r.before);
  EXPECT_EQ(r.pts.front(), resample(pts, 0.5).front());
  EXPECT_EQ(r.pts.back(), resample(pts, 0.5).back());
}

TEST(PathUtils, ResampleKeepsEndpoints)
{
  const auto r = resample({{0, 0, 0}, {10, 0, 0}}, 0.5);
  EXPECT_EQ(r.size(), 21u);
  EXPECT_DOUBLE_EQ(r.back()[0], 10.0);
}
