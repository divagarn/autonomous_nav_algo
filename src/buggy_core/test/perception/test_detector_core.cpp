// Ground-aware obstacle detection against the Python detector (golden_groundaware.jsonl): slopes,
// pitch / roll, ground leaked into the non-ground cloud, boxes, thin single-ring bumps; with and
// without the ground cloud and the thin / low rule.
#include <gtest/gtest.h>

#include <cmath>
#include <fstream>

#include <nlohmann/json.hpp>

#include "buggy_core/perception/detector_core.hpp"

using namespace buggy_core;
using nlohmann::json;

static std::vector<Point3> pts(const json & a)
{
  std::vector<Point3> out;
  for (const auto & p : a) {out.push_back({p[0].get<double>(), p[1].get<double>(), p[2].get<double>()});}
  return out;
}

TEST(DetectorCore, MatchesPythonGroundAware)
{
  std::ifstream f(std::string(GOLDEN_DIR) + "/golden_groundaware.jsonl");
  ASSERT_TRUE(f.good()) << "run tools/golden/gen_golden.py first";
  std::string line;
  int scenes = 0, clusters = 0, mismatched_scenes = 0, dropped = 0;
  while (std::getline(f, line)) {
    const json sc = json::parse(line);
    const int variant = sc["variant"];
    GroundAwareParams gp;
    gp.aware = variant != 2;
    gp.reject_thin_low = variant != 3;
    DetectorCore core(DetectorParams{}, gp);
    const auto ground = pts(sc["ground"]);
    const auto ng = pts(sc["nonground"]);
    const auto corridor = corridor_ahead({}, 0, 0, 0, 25.0);
    DetectorCore::Result r;
    for (int i = 0; i < 3; ++i) {r = core.process(ng, (gp.aware && variant != 1) ? &ground : nullptr, corridor);}
    const auto & exp = sc["clusters"];
    bool same = r.confirmed.size() == exp.size();
    if (same) {
      for (size_t i = 0; i < exp.size(); ++i) {
        same = same && std::abs(r.confirmed[i].cx - exp[i][0].get<double>()) < 1e-3 &&
          std::abs(r.confirmed[i].cy - exp[i][1].get<double>()) < 1e-3 &&
          std::abs(r.confirmed[i].h - exp[i][4].get<double>()) < 1e-3 &&
          std::abs(r.confirmed[i].n - exp[i][5].get<int>()) <= 1;
      }
    }
    if (!same) {
      ++mismatched_scenes;
      ADD_FAILURE() << "scene " << scenes << " variant " << variant << ": C++ " << r.confirmed.size() << " clusters, Python " << exp.size();
    }
    EXPECT_EQ(core.rejected() / 3, sc["rejected"].get<int>()) << "scene " << scenes;
    clusters += static_cast<int>(exp.size());
    dropped += sc["rejected"].get<int>();
    ++scenes;
  }
  std::printf("ground-aware golden: %d scenes, %d confirmed clusters, %d thin/low clusters dropped, %d scenes differ\n",
    scenes, clusters, dropped, mismatched_scenes);
  EXPECT_EQ(scenes, 60);
  EXPECT_GT(clusters, 20);
  EXPECT_GT(dropped, 3);
}

TEST(GroundModel, FollowsASlope)
{
  std::vector<Point3> g;
  for (double x = 2; x < 25; x += 0.5) {
    for (double y = -2; y <= 2; y += 0.4) {g.push_back({x, y, 0.07 * x});}      // 4 deg uphill
  }
  GroundModel m(g, 25.0);
  EXPECT_TRUE(m.has_plane());
  EXPECT_NEAR(m.tilt_deg(), 4.0, 0.2);
  EXPECT_NEAR(m.height(10.0, 0.0), 0.7, 0.03);
  EXPECT_NEAR(m.height(24.0, 0.5), 1.68, 0.05);
}

TEST(GroundModel, FlatFallbackWithoutGround)
{
  GroundModel m({}, 25.0);
  EXPECT_FALSE(m.has_plane());
  EXPECT_EQ(m.height(10.0, 0.0), 0.0);
}

TEST(DetectorCore, SlopeLeakIsNotAnObstacleButABoxIs)
{
  std::vector<Point3> ground, ng;
  for (double x = 2; x < 25; x += 0.5) {
    for (double y = -2; y <= 2; y += 0.4) {ground.push_back({x - 1.73, y, -1.0 + 0.07 * x});}
  }
  for (double x = 14; x < 24; x += 0.2) {               // far ground wrongly in the non-ground cloud
    for (double y = -1.6; y <= 1.6; y += 0.2) {ng.push_back({x - 1.73, y, -1.0 + 0.07 * x});}
  }
  const auto corridor = corridor_ahead({}, 0, 0, 0, 25.0);
  DetectorCore flat(DetectorParams{}, GroundAwareParams{false, 0.3, true, 0.08, 0.5, 0.3});
  DetectorCore aware(DetectorParams{}, GroundAwareParams{});
  for (int i = 0; i < 2; ++i) {flat.process(ng, nullptr, corridor); aware.process(ng, &ground, corridor);}
  EXPECT_FALSE(flat.process(ng, nullptr, corridor).confirmed.empty()) << "the flat reference mistakes the slope for an obstacle";
  EXPECT_TRUE(aware.process(ng, &ground, corridor).confirmed.empty());
  for (double x = 9.8; x < 10.2; x += 0.05) {          // a real box 1 m tall at 10 m
    for (double z = 0.05; z < 1.0; z += 0.1) {ng.push_back({x - 1.73, 0.0 + (x - 9.8), -1.0 + 0.07 * 10 + z});}
  }
  for (int i = 0; i < 2; ++i) {aware.process(ng, &ground, corridor);}
  EXPECT_FALSE(aware.process(ng, &ground, corridor).confirmed.empty());
}
