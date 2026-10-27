#include <gtest/gtest.h>

#include "buggy_core/mapping/pcd_cells.hpp"

using namespace buggy_core;

TEST(PcdCells, CircleTouchesRect)
{
  const CellRect r{-150, -150, 350, 350};
  EXPECT_TRUE(circle_touches(r, 0, 0, 10));
  EXPECT_TRUE(circle_touches(r, 360, 100, 15));
  EXPECT_FALSE(circle_touches(r, 400, 100, 15));
  EXPECT_FALSE(circle_touches(r, 400, 400, 60));
  EXPECT_TRUE(circle_touches(r, 400, 400, 71));
}

TEST(PcdCells, DifferentialSelection)
{
  const std::vector<CellRect> rects{{0, 0, 100, 100}, {100, 0, 200, 100}, {200, 0, 300, 100}};
  const std::vector<std::string> ids{"a", "b", "c"};
  auto s = select_cells(rects, ids, 50, 50, 30, {});
  EXPECT_EQ(s.add, (std::vector<size_t>{0}));
  EXPECT_TRUE(s.remove.empty());
  s = select_cells(rects, ids, 100, 50, 10, {"a"});
  EXPECT_EQ(s.add, (std::vector<size_t>{1}));
  EXPECT_TRUE(s.remove.empty());
  s = select_cells(rects, ids, 250, 50, 10, {"a", "b", "zzz"});
  EXPECT_EQ(s.add, (std::vector<size_t>{2}));
  EXPECT_EQ(s.remove, (std::vector<std::string>{"a", "b", "zzz"}));
}
