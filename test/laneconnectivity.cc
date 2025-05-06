#include "baldr/laneconnectivity.h"
#include "test.h"

#include <string>

using namespace std;
using namespace valhalla::baldr;

namespace {

void TryTestLaneConnectivity(const std::string& lanes) {
  EXPECT_EQ(LaneConnectivityLanes(lanes).to_string(), lanes);
}

TEST(LaneConnectivity, TestLaneConnectivity) {
  TryTestLaneConnectivity("1");
  TryTestLaneConnectivity("1|1");
  TryTestLaneConnectivity("1|2");
  TryTestLaneConnectivity("1|2|3|4|5|6|7|8|9|10|11|12|13|14|15");

  EXPECT_THROW(LaneConnectivityLanes("1|16").to_string(), std::out_of_range);
  EXPECT_THROW(LaneConnectivityLanes("1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1").to_string(),
               std::out_of_range);
  EXPECT_THROW(LaneConnectivityLanes("|1|").to_string(), std::invalid_argument);
  EXPECT_THROW(LaneConnectivityLanes("|").to_string(), std::invalid_argument);
  EXPECT_THROW(LaneConnectivityLanes("||").to_string(), std::invalid_argument);
}

TEST(LaneConnectivity, SizeOf) {
  EXPECT_EQ(sizeof(LaneConnectivity), 24);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
