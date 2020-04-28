
#include "baldr/directededge.h"

#include "test.h"

using namespace std;
using namespace valhalla::baldr;

// Expected size is 48 bytes. Since there are still "spare" bits
// we want to alert if somehow any change grows this structure size
constexpr size_t kDirectedEdgeExpectedSize = 48;

namespace {

TEST(DirectedEdge, test_sizeof) {
  EXPECT_EQ(sizeof(DirectedEdge), kDirectedEdgeExpectedSize);
}

TEST(DirectedEdge, TestWriteRead) {
  // Test building a directed edge and reading back values
  DirectedEdge directededge;
  directededge.set_turntype(0, Turn::Type::kStraight);
  directededge.set_turntype(1, Turn::Type::kLeft);
  directededge.set_turntype(3, Turn::Type::kRight);
  directededge.set_turntype(2, Turn::Type::kSharpRight);
  directededge.set_turntype(5, Turn::Type::kSharpLeft);

  EXPECT_EQ(directededge.turntype(0), Turn::Type::kStraight);
  EXPECT_EQ(directededge.turntype(3), Turn::Type::kRight);
  EXPECT_EQ(directededge.turntype(5), Turn::Type::kSharpLeft);
  EXPECT_EQ(directededge.turntype(1), Turn::Type::kLeft);
  EXPECT_EQ(directededge.turntype(2), Turn::Type::kSharpRight);

  directededge.set_stopimpact(5, 7);
  directededge.set_stopimpact(1, 4);
  directededge.set_stopimpact(3, 0);

  EXPECT_EQ(directededge.stopimpact(3), 0);
  EXPECT_EQ(directededge.stopimpact(5), 7);
  EXPECT_EQ(directededge.stopimpact(1), 4);

  // name consistency should be false by default
  EXPECT_FALSE(directededge.name_consistency(2));

  directededge.set_name_consistency(4, true);
  directededge.set_name_consistency(1, false);
  directededge.set_name_consistency(7, true);
  directededge.set_name_consistency(6, true);

  EXPECT_TRUE(directededge.name_consistency(4));
  EXPECT_FALSE(directededge.name_consistency(1));
  EXPECT_TRUE(directededge.name_consistency(7));
  EXPECT_TRUE(directededge.name_consistency(6));

  // Overwrite idx 6 with false
  directededge.set_name_consistency(6, false);
  EXPECT_FALSE(directededge.name_consistency(6)) << "overwrite test failed";
}

TEST(DirectedEdge, TestMaxSlope) {
  // Test setting max slope and reading back values
  DirectedEdge edge;

  edge.set_max_up_slope(5.0f);
  EXPECT_EQ(edge.max_up_slope(), 5);

  edge.set_max_up_slope(15.0f);
  EXPECT_EQ(edge.max_up_slope(), 15);

  edge.set_max_up_slope(-5.0f);
  EXPECT_EQ(edge.max_up_slope(), 0);

  edge.set_max_up_slope(25.0f);
  EXPECT_EQ(edge.max_up_slope(), 28);

  edge.set_max_up_slope(71.5f);
  EXPECT_EQ(edge.max_up_slope(), 72);

  edge.set_max_up_slope(88.0f);
  EXPECT_EQ(edge.max_up_slope(), 76);

  edge.set_max_up_slope(15.7f);
  EXPECT_EQ(edge.max_up_slope(), 16);

  edge.set_max_down_slope(-5.5f);
  EXPECT_EQ(edge.max_down_slope(), -6);

  edge.set_max_down_slope(-15.0f);
  EXPECT_EQ(edge.max_down_slope(), -15);

  edge.set_max_down_slope(5.0f);
  EXPECT_EQ(edge.max_down_slope(), 0);

  edge.set_max_down_slope(-25.0f);
  EXPECT_EQ(edge.max_down_slope(), -28);

  edge.set_max_down_slope(-71.5f);
  EXPECT_EQ(edge.max_down_slope(), -72);

  edge.set_max_down_slope(-88.0f);
  EXPECT_EQ(edge.max_down_slope(), -76);

  edge.set_max_down_slope(-15.7f);
  EXPECT_EQ(edge.max_down_slope(), -16);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
