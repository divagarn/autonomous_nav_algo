// LocalCartesian projection: the lane loader reads a georeferenced copy of small_loop (lat/lon only, no local_x/y)
// and must give the lanes of the Local map; unsupported projector types are refused.
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>

#include "buggy_core/localization/projection.hpp"
#include "buggy_core/planning/lanelet_map.hpp"

using namespace buggy_core;

TEST(Projection, EastNorthMatchTheEllipsoid)
{
  Projector p;
  p.type = Projector::Type::LocalCartesian;
  p.lat0 = 28.6139; p.lon0 = 77.2090;
  auto en = p.to_xy(28.6139, 77.2090 + 100.0 / (111319.49 * std::cos(28.6139 * M_PI / 180)));
  EXPECT_NEAR(en.first, 100.0, 0.5); EXPECT_NEAR(en.second, 0.0, 0.5);
  en = p.to_xy(28.6139 + 100.0 / 110800.0, 77.2090);
  EXPECT_NEAR(en.first, 0.0, 0.5); EXPECT_NEAR(en.second, 100.0, 0.5);
  en = p.to_xy(28.6139, 77.2090);
  EXPECT_NEAR(en.first, 0.0, 1e-6); EXPECT_NEAR(en.second, 0.0, 1e-6);
}

TEST(Projection, GeoreferencedCopyGivesTheSameLanes)
{
  const std::string local = std::string(MAPS_DIR) + "/small_loop/lanelet2_map.osm";
  const std::string geo = std::string(TEST_DIR) + "/small_loop_geo/lanelet2_map.osm";
  EXPECT_TRUE(Projector::for_osm(geo).georeferenced());
  EXPECT_FALSE(Projector::for_osm(local).georeferenced());
  const auto a = load_lanelets(local), b = load_lanelets(geo);
  ASSERT_EQ(a.size(), b.size());
  ASSERT_GT(a.size(), 0u);
  for (size_t i = 0; i < a.size(); ++i) {
    ASSERT_EQ(a[i].id, b[i].id);
    ASSERT_EQ(a[i].center.size(), b[i].center.size());
    for (size_t k = 0; k < a[i].center.size(); ++k) {
      EXPECT_LT(std::hypot(a[i].center[k][0] - b[i].center[k][0], a[i].center[k][1] - b[i].center[k][1]), 0.03);
    }
  }
}

TEST(Projection, UnsupportedTypeIsRefused)
{
  const auto dir = std::filesystem::temp_directory_path() / "buggy_proj_test";
  std::filesystem::create_directories(dir);
  const auto f = dir / "map_projector_info.yaml";
  { std::ofstream(f) << "projector_type: MGRS\nmgrs_grid: 43RFN\n"; }
  EXPECT_THROW(Projector::load(f.string()), std::runtime_error);
  { std::ofstream(f) << "projector_type: LocalCartesian\n"; }
  EXPECT_THROW(Projector::load(f.string()), std::runtime_error);
  EXPECT_FALSE(Projector::load((dir / "missing.yaml").string()).georeferenced());     // no file = Local
}

TEST(Projection, LocalXyWinOverLatLonOfAnotherOrigin)
{
  // Vector Map Builder exports lat/lon of ITS origin next to local_x / local_y in the point cloud frame
  const auto dir = std::filesystem::temp_directory_path() / "buggy_proj_test2";
  std::filesystem::create_directories(dir);
  { std::ofstream(dir / "map_projector_info.yaml") << "projector_type: LocalCartesian\nmap_origin:\n  latitude: 12.9712936\n  longitude: 80.2439507\n"; }
  {
    std::ofstream(dir / "lanelet2_map.osm") <<
      "<osm><node id='1' lat='12.66254' lon='80.07905'><tag k='local_x' v='-5.0'/><tag k='local_y' v='-6.0'/></node>"
      "<node id='2' lat='12.66273' lon='80.07917'><tag k='local_x' v='8.0'/><tag k='local_y' v='15.0'/></node>"
      "<node id='3' lat='12.66254' lon='80.07906'><tag k='local_x' v='-5.0'/><tag k='local_y' v='-3.0'/></node>"
      "<node id='4' lat='12.66273' lon='80.07918'><tag k='local_x' v='8.0'/><tag k='local_y' v='18.0'/></node>"
      "<way id='10'><nd ref='1'/><nd ref='2'/></way><way id='11'><nd ref='3'/><nd ref='4'/></way>"
      "<relation id='20'><member type='way' ref='10' role='left'/><member type='way' ref='11' role='right'/>"
      "<tag k='type' v='lanelet'/></relation></osm>";
  }
  const auto l = load_lanelets((dir / "lanelet2_map.osm").string());
  ASSERT_EQ(l.size(), 1u);
  EXPECT_NEAR(l[0].left[0][0], -5.0, 1e-9); EXPECT_NEAR(l[0].left[0][1], -6.0, 1e-9);
  EXPECT_NEAR(l[0].left.back()[0], 8.0, 1e-9);  EXPECT_NEAR(l[0].left.back()[1], 15.0, 1e-9);     // lanes are resampled every 0.5 m
}
