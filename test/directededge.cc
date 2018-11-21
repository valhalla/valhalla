#include "test.h"

#include "baldr/directededge.h"

using namespace std;
using namespace valhalla::baldr;

// Expected size is 48 bytes. Since there are still "spare" bits
// we want to alert if somehow any change grows this structure size
constexpr size_t kDirectedEdgeExpectedSize = 48;

namespace {

void test_sizeof() {
  if (sizeof(DirectedEdge) != kDirectedEdgeExpectedSize)
    throw std::runtime_error("DirectedEdge size should be " +
                             std::to_string(kDirectedEdgeExpectedSize) + " bytes" + " but is " +
                             std::to_string(sizeof(DirectedEdge)));
}

void TestWriteRead() {
  // Test building a directed edge and reading back values
  DirectedEdge directededge;
  directededge.set_turntype(0, Turn::Type::kStraight);
  directededge.set_turntype(1, Turn::Type::kLeft);
  directededge.set_turntype(3, Turn::Type::kRight);
  directededge.set_turntype(2, Turn::Type::kSharpRight);
  directededge.set_turntype(5, Turn::Type::kSharpLeft);
  if (directededge.turntype(0) != Turn::Type::kStraight) {
    throw runtime_error("DirectedEdge turn type for localidx 0 test failed");
  }
  if (directededge.turntype(3) != Turn::Type::kRight) {
    throw runtime_error("DirectedEdge turn type for localidx 3 test failed");
  }
  if (directededge.turntype(5) != Turn::Type::kSharpLeft) {
    throw runtime_error("DirectedEdge turn type for localidx 5 test failed");
  }
  if (directededge.turntype(1) != Turn::Type::kLeft) {
    throw runtime_error("DirectedEdge turn type for localidx 1 test failed");
  }
  if (directededge.turntype(2) != Turn::Type::kSharpRight) {
    throw runtime_error("DirectedEdge turn type for localidx 2 test failed");
  }

  directededge.set_stopimpact(5, 7);
  directededge.set_stopimpact(1, 4);
  directededge.set_stopimpact(3, 0);
  if (directededge.stopimpact(3) != 0) {
    throw runtime_error("DirectedEdge stopimpact for localidx 3 test failed");
  }
  if (directededge.stopimpact(5) != 7) {
    throw runtime_error("DirectedEdge stopimpact for localidx 5 test failed");
  }
  if (directededge.stopimpact(1) != 4) {
    throw runtime_error("DirectedEdge stopimpact for localidx 1 test failed");
  }

  // name consistency should be false by default
  if (directededge.name_consistency(2) != false) {
    throw runtime_error("DirectedEdge name_consistency for idx 2 test failed");
  }

  directededge.set_name_consistency(4, true);
  directededge.set_name_consistency(1, false);
  directededge.set_name_consistency(7, true);
  directededge.set_name_consistency(6, true);
  if (directededge.name_consistency(4) != true) {
    throw runtime_error("DirectedEdge name_consistency for idx 4 test failed");
  }
  if (directededge.name_consistency(1) != false) {
    throw runtime_error("DirectedEdge name_consistency for idx 1 test failed");
  }
  if (directededge.name_consistency(7) != true) {
    throw runtime_error("DirectedEdge name_consistency for idx 7 test failed");
  }
  if (directededge.name_consistency(6) != true) {
    throw runtime_error("DirectedEdge name_consistency for idx 6 test failed");
  }

  // Overwrite idx 6 with false
  directededge.set_name_consistency(6, false);
  if (directededge.name_consistency(6) != false) {
    throw runtime_error("DirectedEdge name_consistency overwrite idx 6 test failed");
  }
}

void TestMaxSlope() {
  // Test setting max slope and reading back values
  DirectedEdge edge;
  edge.set_max_up_slope(5.0f);
  if (edge.max_up_slope() != 5) {
    throw runtime_error("DirectedEdge max_up_slope test 1 failed");
  }
  edge.set_max_up_slope(15.0f);
  if (edge.max_up_slope() != 15) {
    throw runtime_error("DirectedEdge max_up_slope test 2 failed");
  }
  edge.set_max_up_slope(-5.0f);
  if (edge.max_up_slope() != 0) {
    throw runtime_error("DirectedEdge max_up_slope test 3 failed");
  }
  edge.set_max_up_slope(25.0f);
  if (edge.max_up_slope() != 28) {
    throw runtime_error("DirectedEdge max_up_slope test 4 failed");
  }
  edge.set_max_up_slope(71.5f);
  if (edge.max_up_slope() != 72) {
    throw runtime_error("DirectedEdge max_up_slope test 5 failed");
  }
  edge.set_max_up_slope(88.0f);
  if (edge.max_up_slope() != 76) {
    throw runtime_error("DirectedEdge max_up_slope test 6 failed");
  }
  edge.set_max_up_slope(15.7f);
  if (edge.max_up_slope() != 16) {
    throw runtime_error("DirectedEdge max_up_slope test 7 failed");
  }

  edge.set_max_down_slope(-5.5f);
  if (edge.max_down_slope() != -6) {
    throw runtime_error("DirectedEdge max_down_slope test 1 failed");
  }
  edge.set_max_down_slope(-15.0f);
  if (edge.max_down_slope() != -15) {
    throw runtime_error("DirectedEdge max_down_slope test 2 failed");
  }
  edge.set_max_down_slope(5.0f);
  if (edge.max_down_slope() != 0) {
    throw runtime_error("edge max_down_slope test 3 failed");
  }
  edge.set_max_down_slope(-25.0f);
  if (edge.max_down_slope() != -28) {
    throw runtime_error("DirectedEdge max_down_slope test 4 failed");
  }
  edge.set_max_down_slope(-71.5f);
  if (edge.max_down_slope() != -72) {
    throw runtime_error("DirectedEdge max_down_slope test 5 failed");
  }
  edge.set_max_down_slope(-88.0f);
  if (edge.max_down_slope() != -76) {
    throw runtime_error("DirectedEdge max_down_slope test 6 failed");
  }
  edge.set_max_down_slope(-15.7f);
  if (edge.max_down_slope() != -16) {
    throw runtime_error("DirectedEdge max_down_slope test 7 failed");
  }
}

} // namespace

int main(void) {
  test::suite suite("directededge");

  suite.test(TEST_CASE(test_sizeof));

  // Write to file and read into DirectedEdge
  suite.test(TEST_CASE(TestWriteRead));

  // Test max slopes
  suite.test(TEST_CASE(TestMaxSlope));

  return suite.tear_down();
}
