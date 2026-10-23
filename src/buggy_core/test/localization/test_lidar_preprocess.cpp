// lidar_preprocess against the Python node (golden_preprocess.jsonl): non-finite points, range,
// self-body crop and voxel centroids must give the same output cloud, point for point.
#include <gtest/gtest.h>

#include <cmath>
#include <fstream>

#include <nlohmann/json.hpp>

#include "buggy_core/localization/lidar_preprocess.hpp"

using namespace buggy_core;
using nlohmann::json;

TEST(LidarPreprocess, MatchesPython)
{
  std::ifstream f(std::string(GOLDEN_DIR) + "/golden_preprocess.jsonl");
  ASSERT_TRUE(f.good()) << "run tools/golden/gen_golden.py first";
  std::string line;
  int scans = 0, empty = 0;
  size_t points = 0;
  double worst = 0.0;
  while (std::getline(f, line)) {
    const json sc = json::parse(line);
    std::vector<float> in;
    for (const auto & p : sc["points"]) {
      for (int c = 0; c < 3; ++c) {if (p[c].is_string()) {in.push_back(p[c].get<std::string>() == "nan" ? std::nanf("") : INFINITY);} else {in.push_back(p[c].get<float>());}}
    }
    PreprocessParams prm;
    prm.voxel = sc["voxel"].get<float>();
    std::vector<float> out;
    const bool ok = preprocess_cloud(in.data(), in.size() / 3, prm, out);
    if (sc["out"].is_null()) {
      EXPECT_FALSE(ok) << "scan " << scans;
      ++empty;
    } else {
      ASSERT_TRUE(ok) << "scan " << scans;
      ASSERT_EQ(out.size() / 3, sc["out"].size()) << "scan " << scans;
      for (size_t i = 0; i < sc["out"].size(); ++i) {
        for (int c = 0; c < 3; ++c) {
          const float e = sc["out"][i][c].get<float>();
          worst = std::max(worst, static_cast<double>(std::abs(out[3 * i + c] - e)));
          ASSERT_EQ(out[3 * i + c], e) << "scan " << scans << " voxel " << i;
        }
      }
      points += sc["out"].size();
    }
    ++scans;
  }
  std::printf("preprocess golden: %d scans (%d refused as too sparse), %zu voxel centroids identical (worst %.1e)\n", scans, empty, points, worst);
  EXPECT_EQ(scans, 12);
  EXPECT_GE(empty, 1);
}

TEST(LidarPreprocess, VoxelAveragesPointsInACell)
{
  const std::vector<float> pts = {0.01f, 0.01f, 0.01f, 0.05f, 0.05f, 0.05f, 5.0f, 5.0f, 5.0f};
  const auto c = voxel_centroids(pts, 0.3f);
  ASSERT_EQ(c.size(), 6u);
  EXPECT_NEAR(c[0], 0.03f, 1e-6);
  EXPECT_NEAR(c[3], 5.0f, 1e-6);
}
