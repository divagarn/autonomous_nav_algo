// plan_detour against the Python reference on 56 scenes (golden_avoid.jsonl): lane geometry from
// real maps, random obstacles (small, vehicle sized, wide barriers), with and without LiDAR grids
// and the lane model. Compares success, the reason text, side, offset and every detour point.
#include <gtest/gtest.h>

#include <fstream>
#include <map>
#include <memory>

#include "buggy_core/planning/avoidance.hpp"

using namespace buggy_core;
using nlohmann::json;

TEST(Avoidance, MatchesPython)
{
  std::ifstream f(std::string(GOLDEN_DIR) + "/golden_avoid.jsonl");
  ASSERT_TRUE(f.good()) << "run tools/golden/gen_golden.py first";
  std::map<std::string, std::unique_ptr<LaneModel>> lanes;
  std::string line;
  int scenes = 0, detours = 0, refused = 0, cell_scenes = 0;
  double worst = 0.0;
  while (std::getline(f, line)) {
    const json sc = json::parse(line);
    const std::string name = sc["map"];
    if (!lanes.count(name)) {
      lanes[name] = std::make_unique<LaneModel>(load_lanelets(std::string(MAPS_DIR) + "/" + name + "/lanelet2_map.osm"));
    }
    Poly pts;
    for (const auto & p : sc["points"]) {pts.push_back({p[0], p[1], p[2]});}
    std::vector<Box> obs;
    for (const auto & o : sc["obstacles"]) {obs.push_back({o[0], o[1], o[2], o[3], o[4]});}
    std::unique_ptr<FreeSpace> free;
    if (!sc["free"].is_null()) {
      const auto & fj = sc["free"];
      free = std::make_unique<FreeSpace>();
      free->ox = fj["origin"][0]; free->oy = fj["origin"][1]; free->res = fj["res"];
      free->nx = fj["nx"]; free->ny = fj["ny"];
      free->occ.assign(static_cast<size_t>(free->nx) * free->ny, 0);
      free->gnd = free->occ;
      const auto fill = [&](std::vector<uint8_t> & g, const json & rects, uint8_t v) {
          for (const auto & r : rects) {
            const int r0 = r[0], r1 = r[1], r2 = r[2], r3 = r[3];
            for (int i = std::max(r0, 0); i <= std::min(r1, free->nx - 1); ++i) {
              for (int j = std::max(r2, 0); j <= std::min(r3, free->ny - 1); ++j) {g[static_cast<size_t>(i) * free->ny + j] = v;}
            }
          }
        };
      fill(free->occ, fj["occ_rects"], 1);
      fill(free->gnd, fj["gnd_rects"], 1);
      fill(free->gnd, fj["holes"], 0);
    }
    const LaneModel * lane = sc["lane"].get<bool>() ? lanes[name].get() : nullptr;
    json got = plan_detour(pts, obs, lane, free.get(), AvoidParams{});
    if (!sc["zone"].is_null() && free) {                       // free ground shown when the lane is blocked
      ASSERT_TRUE(got.contains("zone"));
      const Zone z{got["zone"]["s0"], got["zone"]["s1"], got["zone"]["lo"], got["zone"]["hi"]};
      EXPECT_NEAR(z.s0, sc["zone"]["s0"].get<double>(), 1e-9);
      const auto cells = free_space_cells(pts, z, free.get(), lane, AvoidParams{});
      EXPECT_EQ(static_cast<int>(cells.size()), sc["cells_n"].get<int>()) << "scene " << scenes + 1;
      double sx = 0, sy = 0;
      for (const auto & c : cells) {sx += c[0]; sy += c[1];}
      EXPECT_NEAR(sx, sc["cells_sum"][0].get<double>(), 1e-6) << "scene " << scenes + 1;
      EXPECT_NEAR(sy, sc["cells_sum"][1].get<double>(), 1e-6);
      ++cell_scenes;
    }
    if (sc["result"].contains("room_needed")) {
      ASSERT_TRUE(got.contains("room_needed"));
      EXPECT_NEAR(got["room_needed"].get<double>(), sc["result"]["room_needed"].get<double>(), 1e-9) << "scene " << scenes + 1;
    }
    got.erase("zone");
    const json & exp = sc["result"];
    ++scenes;
    ASSERT_EQ(got["ok"], exp["ok"]) << "scene " << scenes << ": got '" << got.value("reason", "") << "' expected '" << exp.value("reason", "") << "'";
    if (!exp["ok"].get<bool>()) {
      EXPECT_EQ(got["reason"], exp["reason"]) << "scene " << scenes;
      EXPECT_NEAR(got["room_before"].get<double>(), exp["room_before"].get<double>(), 1e-9);
      ++refused;
      continue;
    }
    ++detours;
    EXPECT_EQ(got["side"], exp["side"]) << "scene " << scenes;
    EXPECT_EQ(got["ib"], exp["ib"]) << "scene " << scenes;
    EXPECT_EQ(got["outside_lane"], exp["outside_lane"]) << "scene " << scenes;
    EXPECT_EQ(got["inside"], exp["inside"]) << "per-point inside-lane flags, scene " << scenes;
    for (const char * k : {"offset", "length", "shift_length", "radius", "room_before"}) {
      EXPECT_NEAR(got[k].get<double>(), exp[k].get<double>(), 1e-9) << k << " scene " << scenes;
    }
    ASSERT_EQ(got["segment"].size(), exp["segment"].size());
    for (size_t i = 0; i < exp["segment"].size(); ++i) {
      for (int c = 0; c < 3; ++c) {
        worst = std::max(worst, std::abs(got["segment"][i][c].get<double>() - exp["segment"][i][c].get<double>()));
      }
    }
  }
  std::printf("avoidance golden: %d scenes (%d detours, %d refusals), worst detour point difference %.2e m, free-space cells checked in %d scenes\n", scenes, detours, refused, worst, cell_scenes);
  EXPECT_EQ(scenes, 56);
  EXPECT_GT(detours, 20);
  EXPECT_LT(worst, 1e-9);
}

TEST(Avoidance, NoObstacleNoDetour)
{
  Poly pts;
  for (int i = 0; i < 100; ++i) {pts.push_back({i * 0.5, 0.0, 0.0});}
  const auto r = plan_detour(pts, {}, nullptr, nullptr, AvoidParams{});
  EXPECT_FALSE(r["ok"].get<bool>());
  EXPECT_EQ(r["reason"], "no obstacle on the route ahead");
}

TEST(Avoidance, PedestrianOnStraightRoadGetsADetour)
{
  Poly pts;
  for (int i = 0; i < 160; ++i) {pts.push_back({i * 0.5, 0.0, 0.0});}
  const auto r = plan_detour(pts, {{20.0, 0.0, 0.0, 0.5, 0.5}}, nullptr, nullptr, AvoidParams{});
  ASSERT_TRUE(r["ok"].get<bool>()) << r["reason"];
  EXPECT_GT(std::abs(r["offset"].get<double>()), 1.0);
}
