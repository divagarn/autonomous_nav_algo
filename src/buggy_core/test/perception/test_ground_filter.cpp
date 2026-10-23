// Ground segmentation ('segment' backend) and self-hit filter against the Python reference
// (golden_ground.txt, from tools/golden/gen_golden.py).
#include <gtest/gtest.h>

#include <fstream>
#include <sstream>

#include "buggy_core/perception/ground_filter.hpp"

using namespace buggy_core;

TEST(GroundFilter, MatchesPython)
{
  std::ifstream f(std::string(GOLDEN_DIR) + "/golden_ground.txt");
  ASSERT_TRUE(f.good()) << "run tools/golden/gen_golden.py first";
  std::vector<float> xyz, hxyz;
  std::vector<int> gexp, kexp;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') {continue;}
    std::istringstream is(line);
    std::string tag;
    float x, y, z;
    int v;
    is >> tag >> x >> y >> z >> v;
    if (tag == "G") {xyz.insert(xyz.end(), {x, y, z}); gexp.push_back(v);}
    if (tag == "H") {hxyz.insert(hxyz.end(), {x, y, z}); kexp.push_back(v);}
  }
  ASSERT_GT(gexp.size(), 1000u);
  std::vector<uint8_t> mask;
  ASSERT_TRUE(segment_ground_mask(xyz.data(), gexp.size(), GroundParams{}, mask));
  size_t diff = 0, ground = 0;
  for (size_t i = 0; i < gexp.size(); ++i) {
    diff += (mask[i] != gexp[i]);
    ground += mask[i];
  }
  std::printf("ground: %zu points, %zu ground, %zu differences vs Python\n", gexp.size(), ground, diff);
  EXPECT_EQ(diff, 0u);

  // self-hit: rows of 16 bytes (x y z rgb)
  std::vector<float> rows;
  for (size_t i = 0; i < kexp.size(); ++i) {rows.insert(rows.end(), {hxyz[3 * i], hxyz[3 * i + 1], hxyz[3 * i + 2], 0.0f});}
  std::vector<uint8_t> keep;
  self_hit_keep(reinterpret_cast<const uint8_t *>(rows.data()), kexp.size(), 16, SelfHitParams{}, keep);
  size_t kd = 0;
  for (size_t i = 0; i < kexp.size(); ++i) {kd += (keep[i] != kexp[i]);}
  std::printf("self-hit: %zu points, %zu differences vs Python\n", kexp.size(), kd);
  EXPECT_EQ(kd, 0u);
}

TEST(GroundFilter, TooFewPointsGivesNoOutput)
{
  std::vector<float> xyz(3 * 10, 0.5f);
  std::vector<uint8_t> mask;
  EXPECT_FALSE(segment_ground_mask(xyz.data(), 10, GroundParams{}, mask));
}
