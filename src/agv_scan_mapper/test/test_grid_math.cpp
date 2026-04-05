#include <gtest/gtest.h>
#include "agv_scan_mapper/grid_math.hpp"
#include <cmath>

using namespace agv_scan_mapper;

// ── world_to_grid ──

TEST(GridMath, WorldToGridOrigin)
{
  // Point at origin with origin_x=-10.0, resolution=0.05
  EXPECT_EQ(world_to_grid(-10.0, -10.0, 0.05), 0);
}

TEST(GridMath, WorldToGridPositive)
{
  // Point at 0.0 with origin=-10.0, res=0.05 → cell 200
  EXPECT_EQ(world_to_grid(0.0, -10.0, 0.05), 200);
}

TEST(GridMath, WorldToGridNegative)
{
  // Point before origin → negative cell (out of bounds)
  EXPECT_LT(world_to_grid(-11.0, -10.0, 0.05), 0);
}

// ── in_bounds ──

TEST(GridMath, InBoundsValid)
{
  EXPECT_TRUE(in_bounds(0, 0, 400, 400));
  EXPECT_TRUE(in_bounds(199, 199, 400, 400));
  EXPECT_TRUE(in_bounds(399, 399, 400, 400));
}

TEST(GridMath, InBoundsInvalid)
{
  EXPECT_FALSE(in_bounds(-1, 0, 400, 400));
  EXPECT_FALSE(in_bounds(0, -1, 400, 400));
  EXPECT_FALSE(in_bounds(400, 0, 400, 400));
  EXPECT_FALSE(in_bounds(0, 400, 400, 400));
}

// ── log_odds_to_probability ──

TEST(GridMath, LogOddsToProbZero)
{
  // log_odds = 0 → probability = 0.5
  EXPECT_NEAR(log_odds_to_probability(0.0), 0.5, 1e-6);
}

TEST(GridMath, LogOddsToProbHighPositive)
{
  // Large positive log_odds → probability near 1.0
  double p = log_odds_to_probability(5.0);
  EXPECT_GT(p, 0.99);
  EXPECT_LE(p, 1.0);
}

TEST(GridMath, LogOddsToProbHighNegative)
{
  // Large negative log_odds → probability near 0.0
  double p = log_odds_to_probability(-5.0);
  EXPECT_LT(p, 0.01);
  EXPECT_GE(p, 0.0);
}

TEST(GridMath, LogOddsToProbMonotonic)
{
  // Higher log_odds → higher probability
  EXPECT_LT(log_odds_to_probability(-1.0), log_odds_to_probability(0.0));
  EXPECT_LT(log_odds_to_probability(0.0), log_odds_to_probability(1.0));
}

// ── bresenham_line ──

TEST(GridMath, BresenhamHorizontal)
{
  auto cells = bresenham_line(0, 0, 5, 0);
  EXPECT_EQ(cells.size(), 5u);  // 0..4 (endpoint excluded)
  for (size_t i = 0; i < cells.size(); ++i) {
    EXPECT_EQ(cells[i].first, static_cast<int>(i));
    EXPECT_EQ(cells[i].second, 0);
  }
}

TEST(GridMath, BresenhamVertical)
{
  auto cells = bresenham_line(0, 0, 0, 4);
  EXPECT_EQ(cells.size(), 4u);
  for (size_t i = 0; i < cells.size(); ++i) {
    EXPECT_EQ(cells[i].first, 0);
    EXPECT_EQ(cells[i].second, static_cast<int>(i));
  }
}

TEST(GridMath, BresenhamDiagonal)
{
  auto cells = bresenham_line(0, 0, 3, 3);
  EXPECT_EQ(cells.size(), 3u);
  for (size_t i = 0; i < cells.size(); ++i) {
    EXPECT_EQ(cells[i].first, static_cast<int>(i));
    EXPECT_EQ(cells[i].second, static_cast<int>(i));
  }
}

TEST(GridMath, BresenhamReverse)
{
  auto cells = bresenham_line(5, 0, 0, 0);
  EXPECT_EQ(cells.size(), 5u);
  // Should go from 5 down to 1
  EXPECT_EQ(cells.front().first, 5);
  EXPECT_EQ(cells.back().first, 1);
}

TEST(GridMath, BresenhamSamePoint)
{
  auto cells = bresenham_line(3, 3, 3, 3);
  EXPECT_TRUE(cells.empty());  // No cells when start == end
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
