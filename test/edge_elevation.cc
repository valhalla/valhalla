#include "test.h"
#include <cmath>

#include "baldr/edge_elevation.h"

using namespace std;
using namespace valhalla::baldr;

// Expected size is 48 bytes. Since there are still "spare" bits
// we want to alert if somehow any change grows this structure size
constexpr size_t kEdgeElevationExpectedSize = 4;

namespace {

void test_sizeof() {
  if (sizeof(EdgeElevation) != kEdgeElevationExpectedSize)
    throw std::runtime_error("EdgeElevation size should be " +
                             std::to_string(kEdgeElevationExpectedSize) + " bytes" + " but is " +
                             std::to_string(sizeof(EdgeElevation)));
}

void TestMaxSlope() {
  // Test setting max slope and reading back values
  EdgeElevation edge_elev(0.0f, 0.0f, 0.0f);
  edge_elev.set_max_up_slope(5.0f);
  if (edge_elev.max_up_slope() != 5) {
    throw runtime_error("EdgeElevation max_up_slope test 1 failed");
  }
  edge_elev.set_max_up_slope(15.0f);
  if (edge_elev.max_up_slope() != 15) {
    throw runtime_error("EdgeElevation max_up_slope test 2 failed");
  }
  edge_elev.set_max_up_slope(-5.0f);
  if (edge_elev.max_up_slope() != 0) {
    throw runtime_error("EdgeElevation max_up_slope test 3 failed");
  }
  edge_elev.set_max_up_slope(25.0f);
  if (edge_elev.max_up_slope() != 28) {
    throw runtime_error("EdgeElevation max_up_slope test 4 failed");
  }
  edge_elev.set_max_up_slope(71.5f);
  if (edge_elev.max_up_slope() != 72) {
    throw runtime_error("EdgeElevation max_up_slope test 5 failed");
  }
  edge_elev.set_max_up_slope(88.0f);
  if (edge_elev.max_up_slope() != 76) {
    throw runtime_error("EdgeElevation max_up_slope test 6 failed");
  }
  edge_elev.set_max_up_slope(15.7f);
  if (edge_elev.max_up_slope() != 16) {
    throw runtime_error("EdgeElevation max_up_slope test 7 failed");
  }

  edge_elev.set_max_down_slope(-5.5f);
  if (edge_elev.max_down_slope() != -6) {
    throw runtime_error("EdgeElevation max_down_slope test 1 failed");
  }
  edge_elev.set_max_down_slope(-15.0f);
  if (edge_elev.max_down_slope() != -15) {
    throw runtime_error("EdgeElevation max_down_slope test 2 failed");
  }
  edge_elev.set_max_down_slope(5.0f);
  if (edge_elev.max_down_slope() != 0) {
    throw runtime_error("edge_elev max_down_slope test 3 failed");
  }
  edge_elev.set_max_down_slope(-25.0f);
  if (edge_elev.max_down_slope() != -28) {
    throw runtime_error("EdgeElevation max_down_slope test 4 failed");
  }
  edge_elev.set_max_down_slope(-71.5f);
  if (edge_elev.max_down_slope() != -72) {
    throw runtime_error("EdgeElevation max_down_slope test 5 failed");
  }
  edge_elev.set_max_down_slope(-88.0f);
  if (edge_elev.max_down_slope() != -76) {
    throw runtime_error("EdgeElevation max_down_slope test 6 failed");
  }
  edge_elev.set_max_down_slope(-15.7f);
  if (edge_elev.max_down_slope() != -16) {
    throw runtime_error("EdgeElevation max_down_slope test 7 failed");
  }
}

void TestMeanElevation() {
  // Test setting mean elevation and reading back values
  EdgeElevation edge_elev(0.0f, 0.0f, 0.0f);
  edge_elev.set_mean_elevation(kMinElevation - 100.0f);
  if (edge_elev.mean_elevation() != kMinElevation) {
    throw runtime_error("EdgeElevation mean_elevation test 1 failed");
  }

  edge_elev.set_mean_elevation(kMaxElevation + 100.0f);
  if (edge_elev.mean_elevation() != kMaxElevation) {
    throw runtime_error("EdgeElevation mean_elevation test 2 failed");
  }

  edge_elev.set_mean_elevation(0.0f);
  if (std::abs(edge_elev.mean_elevation()) > kElevationBinSize) {
    throw runtime_error("EdgeElevation mean_elevation test 3 failed");
  }

  edge_elev.set_mean_elevation(100.0f);
  if (std::abs(edge_elev.mean_elevation() - 100.0f) > kElevationBinSize) {
    throw runtime_error("EdgeElevation mean_elevation test 4 failed");
  }
}

} // namespace

int main(void) {
  test::suite suite("edge_elevation");

  suite.test(TEST_CASE(test_sizeof));

  // Test max slope
  suite.test(TEST_CASE(TestMaxSlope));

  // Test mean elevation
  suite.test(TEST_CASE(TestMeanElevation));

  return suite.tear_down();
}
