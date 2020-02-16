#include "test.h"
#include <cmath>

#include "baldr/edge_elevation.h"

using namespace std;
using namespace valhalla::baldr;

// Expected size is 48 bytes. Since there are still "spare" bits
// we want to alert if somehow any change grows this structure size
constexpr size_t kEdgeElevationExpectedSize = 4;

namespace {

TEST(EdgeElevation, Sizeof) {
  EXPECT_EQ(sizeof(EdgeElevation), kEdgeElevationExpectedSize);
}

TEST(EdgeElevation, TestMaxSlope) {
  // Test setting max slope and reading back values
  EdgeElevation edge_elev(0.0f, 0.0f, 0.0f);
  edge_elev.set_max_up_slope(5.0f);
  EXPECT_EQ(edge_elev.max_up_slope(), 5);

  edge_elev.set_max_up_slope(15.0f);
  EXPECT_EQ(edge_elev.max_up_slope(), 15);

  edge_elev.set_max_up_slope(-5.0f);
  EXPECT_EQ(edge_elev.max_up_slope(), 0);

  edge_elev.set_max_up_slope(25.0f);
  EXPECT_EQ(edge_elev.max_up_slope(), 28);

  edge_elev.set_max_up_slope(71.5f);
  EXPECT_EQ(edge_elev.max_up_slope(), 72);

  edge_elev.set_max_up_slope(88.0f);
  EXPECT_EQ(edge_elev.max_up_slope(), 76);

  edge_elev.set_max_up_slope(15.7f);
  EXPECT_EQ(edge_elev.max_up_slope(), 16);

  edge_elev.set_max_down_slope(-5.5f);
  EXPECT_EQ(edge_elev.max_down_slope(), -6);

  edge_elev.set_max_down_slope(-15.0f);
  EXPECT_EQ(edge_elev.max_down_slope(), -15);

  edge_elev.set_max_down_slope(5.0f);
  EXPECT_EQ(edge_elev.max_down_slope(), 0);

  edge_elev.set_max_down_slope(-25.0f);
  EXPECT_EQ(edge_elev.max_down_slope(), -28);

  edge_elev.set_max_down_slope(-71.5f);
  EXPECT_EQ(edge_elev.max_down_slope(), -72);

  edge_elev.set_max_down_slope(-88.0f);
  EXPECT_EQ(edge_elev.max_down_slope(), -76);

  edge_elev.set_max_down_slope(-15.7f);
  EXPECT_EQ(edge_elev.max_down_slope(), -16);
}

TEST(EdgeElevation, TestMeanElevation) {
  // Test setting mean elevation and reading back values
  EdgeElevation edge_elev(0.0f, 0.0f, 0.0f);
  edge_elev.set_mean_elevation(kMinElevation - 100.0f);
  EXPECT_EQ(edge_elev.mean_elevation(), kMinElevation);

  edge_elev.set_mean_elevation(kMaxElevation + 100.0f);
  EXPECT_EQ(edge_elev.mean_elevation(), kMaxElevation);

  edge_elev.set_mean_elevation(0.0f);
  EXPECT_NEAR(edge_elev.mean_elevation(), 0, kElevationBinSize);

  edge_elev.set_mean_elevation(100.0f);
  EXPECT_NEAR(edge_elev.mean_elevation(), 100.0f, kElevationBinSize);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
