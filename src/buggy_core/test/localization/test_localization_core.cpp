// Localization supervisor logic against the Python LocalizationManager (golden_locstate.jsonl):
// replayed event sequences (TF edges, NDT pose, score, state changes) must give the same
// /localization/state text every time.
#include <gtest/gtest.h>

#include <fstream>

#include <nlohmann/json.hpp>

#include "buggy_core/localization/localization_core.hpp"

using namespace buggy_core;
using nlohmann::json;

TEST(Localization, StateTextMatchesPython)
{
  std::ifstream f(std::string(GOLDEN_DIR) + "/golden_locstate.jsonl");
  ASSERT_TRUE(f.good()) << "run tools/golden/gen_golden.py first";
  std::string line;
  int scenes = 0, texts = 0, conflicts = 0, jumps = 0, states[5] = {0, 0, 0, 0, 0};
  while (std::getline(f, line)) {
    const json sc = json::parse(line);
    LocalizationCore core;
    for (const auto & e : sc["events"]) {
      const std::string kind = e[0];
      if (kind == "tf") {
        std::vector<TfEdge> edges;
        for (const auto & t : e[2]) {edges.push_back({t[0], t[1], t[2], t[3]});}
        core.on_tf(edges, e[1].get<double>());
      } else if (kind == "ndt") {core.on_ndt_pose(e[1].get<double>());}
      else if (kind == "score") {core.set_score(e[1].get<double>());}
      else if (kind == "iters") {core.set_iterations(e[1].get<double>());}
      else if (kind == "exe") {core.set_exe_ms(e[1].get<double>());}
      else if (kind == "set") {core.set_state(e[1], e[2]); core.set_activated(e[3].get<bool>());}
      else if (kind == "reset") {core.reset_ndt();}
      else if (kind == "pub") {
        std::optional<MapPose> pose;
        if (!e[2].is_null()) {pose = MapPose{e[2][0], e[2][1], e[2][2].get<double>()};}
        // python gives yaw in degrees from the quaternion; the golden stores the raw yaw (rad)
        if (pose) {const double yaw = pose->yaw_deg; pose->yaw_deg = std::atan2(2 * (std::cos(yaw / 2) * std::sin(yaw / 2)), 1 - 2 * std::sin(yaw / 2) * std::sin(yaw / 2)) * 180.0 / M_PI;}
        const std::string got = core.state_string(e[1].get<double>(), pose);
        ASSERT_EQ(got, e[3].get<std::string>()) << "scene " << scenes;
        ++texts;
        if (got.find("tf_conflict=;") == std::string::npos) {++conflicts;}
        if (got.find("jumps=0;") == std::string::npos) {++jumps;}
        for (int k = 0; k < 5; ++k) {
          static const char * names[5] = {"state=converged", "state=degraded", "state=lost", "state=initialized", "state=uninitialized"};
          if (got.rfind(names[k], 0) == 0) {++states[k];}
        }
      }
    }
    ++scenes;
  }
  std::printf("localization golden: %d scenes, %d state texts identical (%d with a TF conflict, %d with jumps; converged %d degraded %d lost %d initialized %d uninit %d)\n",
    scenes, texts, conflicts, jumps, states[0], states[1], states[2], states[3], states[4]);
  EXPECT_EQ(scenes, 40);
  EXPECT_GT(texts, 200);
  EXPECT_GT(conflicts, 20);
}

TEST(Localization, ConvergedNeedsFreshPoseAndScore)
{
  LocalizationCore c;
  c.set_state("initialized", "");
  c.set_activated(true);
  c.on_ndt_pose(10.0);
  c.set_score(2.0);
  c.update_state(10.5);
  EXPECT_EQ(c.state(), "converged");
  c.update_state(11.5);
  EXPECT_EQ(c.state(), "degraded");           // stale 1-3 s
  c.update_state(14.5);
  EXPECT_EQ(c.state(), "lost");
  c.on_ndt_pose(20.0);
  c.set_score(1.0);
  c.update_state(20.1);
  EXPECT_EQ(c.state(), "degraded");           // low score
}
