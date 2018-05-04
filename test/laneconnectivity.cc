#include "test.h"

#include "baldr/laneconnectivity.h"

#include <string>

using namespace std;
using namespace valhalla::baldr;

namespace {

void TryTestLaneConnectivity(const std::string& lanes) {
  std::string ls = LaneConnectivityLanes(lanes).to_string();
  if (ls != lanes) {
    throw std::runtime_error("Incorrect lane mask for " + lanes + " (" + ls + ")");
  }
}

void TryTestLaneConnectivityShouldFail(const std::string& lanes) {
  try {
    LaneConnectivityLanes(lanes).to_string();
    throw std::runtime_error("Expected error for " + lanes);
  } catch (std::invalid_argument&) {
  } catch (std::out_of_range&) {}
}

void TestLaneConnectivity() {
  TryTestLaneConnectivity("1");
  TryTestLaneConnectivity("1|1");
  TryTestLaneConnectivity("1|2");
  TryTestLaneConnectivity("1|2|3|4|5|6|7|8|9|10|11|12|13|14|15");
  TryTestLaneConnectivityShouldFail("1|16");
  TryTestLaneConnectivityShouldFail("1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1");
  TryTestLaneConnectivityShouldFail("|1|");
  TryTestLaneConnectivityShouldFail("|");
  TryTestLaneConnectivityShouldFail("||");
}

void TestSizeOf() {
  if (sizeof(LaneConnectivity) != 24) {
    throw std::runtime_error("Invalid sizeof(LaneConnectivity): " +
                             std::to_string(sizeof(LaneConnectivity)));
  }
}

} // namespace

int main(void) {
  test::suite suite("laneconnectivity");

  suite.test(TEST_CASE(TestLaneConnectivity));
  suite.test(TEST_CASE(TestSizeOf));

  return suite.tear_down();
}
