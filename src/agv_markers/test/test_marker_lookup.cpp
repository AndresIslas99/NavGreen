#include <gtest/gtest.h>
#include <map>
#include <string>
#include <cmath>

struct MarkerPose {
  double x, y, z, yaw;
};

TEST(MarkerLookup, FindKnownMarker) {
  std::map<int, MarkerPose> registry;
  registry[0] = {0.0, 0.0, 0.3, 0.0};
  registry[1] = {5.0, 0.0, 0.3, 0.0};
  registry[2] = {5.0, 3.0, 0.3, M_PI / 2.0};

  auto it = registry.find(1);
  ASSERT_NE(it, registry.end());
  EXPECT_NEAR(it->second.x, 5.0, 1e-6);
  EXPECT_NEAR(it->second.y, 0.0, 1e-6);
}

TEST(MarkerLookup, UnknownMarkerNotFound) {
  std::map<int, MarkerPose> registry;
  registry[0] = {0.0, 0.0, 0.3, 0.0};

  auto it = registry.find(99);
  EXPECT_EQ(it, registry.end());
}

TEST(MarkerLookup, RangeCheck) {
  double px = 2.0, py = 1.5, pz = 0.0;
  double range = std::sqrt(px * px + py * py + pz * pz);
  double max_range = 3.0;
  EXPECT_LT(range, max_range);

  double far_x = 10.0;
  double far_range = std::sqrt(far_x * far_x);
  EXPECT_GT(far_range, max_range);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
