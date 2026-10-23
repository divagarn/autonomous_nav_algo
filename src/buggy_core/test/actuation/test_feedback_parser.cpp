// ECU feedback parsing against the Python UartBridge (golden_feedback.jsonl): byte streams cut at
// random places, with garbage and bad end bytes, must give the same packets in the same order.
#include <gtest/gtest.h>

#include <fstream>

#include <nlohmann/json.hpp>

#include "buggy_core/actuation/feedback_parser.hpp"

using namespace buggy_core;
using nlohmann::json;

static std::vector<uint8_t> unhex(const std::string & h)
{
  std::vector<uint8_t> out;
  for (size_t i = 0; i + 1 < h.size(); i += 2) {out.push_back(static_cast<uint8_t>(std::stoi(h.substr(i, 2), nullptr, 16)));}
  return out;
}

TEST(FeedbackParser, MatchesPython)
{
  std::ifstream f(std::string(GOLDEN_DIR) + "/golden_feedback.jsonl");
  ASSERT_TRUE(f.good()) << "run tools/golden/gen_golden.py first";
  std::string line;
  int streams = 0, packets = 0, warned = 0;
  while (std::getline(f, line)) {
    const json sc = json::parse(line);
    FeedbackParser p;
    for (size_t i = 0; i < sc["chunks"].size(); ++i) {
      const auto bytes = unhex(sc["chunks"][i]);
      std::vector<std::string> warn;
      const auto got = p.feed(bytes.data(), bytes.size(), &warn);
      warned += static_cast<int>(warn.size());
      const auto & exp = sc["expected"][i];
      ASSERT_EQ(got.size(), exp.size()) << "stream " << streams << " chunk " << i;
      for (size_t k = 0; k < got.size(); ++k) {
        EXPECT_NEAR(got[k].steering_deg, exp[k][0].get<double>(), 1e-12);
        EXPECT_NEAR(got[k].speed_kmh, exp[k][1].get<double>(), 1e-12);
        ++packets;
      }
    }
    ++streams;
  }
  std::printf("feedback golden: %d streams, %d packets identical (%d bad-marker warnings)\n", streams, packets, warned);
  EXPECT_EQ(streams, 60);
  EXPECT_GT(packets, 400);
  EXPECT_GT(warned, 5);
}

TEST(FeedbackParser, DecodesKnownPacket)
{
  FeedbackParser p;
  // raw 3000 -> 0 deg; raw 2000 -> -10 deg; speed 4 km/h
  const uint8_t pkt[6] = {0x28, 0x0B, 0xB8, 4, 0, 0x29};
  const auto r = p.feed(pkt, 6);
  ASSERT_EQ(r.size(), 1u);
  EXPECT_NEAR(r[0].steering_deg, 0.0, 1e-9);
  EXPECT_EQ(r[0].speed_kmh, 4.0);
}

TEST(FeedbackParser, ClampsSteering)
{
  FeedbackParser p;
  const uint8_t pkt[6] = {0x28, 0x1F, 0x40, 2, 0, 0x29};      // raw 8000 -> +50 deg -> clamp 20
  EXPECT_EQ(p.feed(pkt, 6).at(0).steering_deg, 20.0);
}

TEST(FeedbackParser, WaitsForRestOfPacket)
{
  FeedbackParser p;
  const uint8_t a[3] = {0x28, 0x0B, 0xB8}, b[3] = {4, 0, 0x29};
  EXPECT_TRUE(p.feed(a, 3).empty());
  EXPECT_EQ(p.feed(b, 3).size(), 1u);
}
