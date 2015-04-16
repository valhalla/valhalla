#include "test.h"

#include <vector>
#include <iostream>
#include <fstream>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/directededge.h>
#include "mjolnir/directededgebuilder.h"
#include <boost/shared_array.hpp>
#include <memory>

using namespace std;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

void TestWriteRead() {
  // Test building a directed edge and reading back values
  DirectedEdgeBuilder directededgebuilder;
  directededgebuilder.set_turntype(0, Turn::Type::kStraight);
  directededgebuilder.set_turntype(1, Turn::Type::kLeft);
  directededgebuilder.set_turntype(3, Turn::Type::kRight);
  directededgebuilder.set_turntype(2, Turn::Type::kSharpRight);
  directededgebuilder.set_turntype(5, Turn::Type::kSharpLeft);
  if (directededgebuilder.turntype(0) != Turn::Type::kStraight) {
    throw runtime_error("DirectedEdge turn type for localidx 0 test failed");
  }
  if (directededgebuilder.turntype(3) != Turn::Type::kRight) {
    throw runtime_error("DirectedEdge turn type for localidx 3 test failed");
  }
  if (directededgebuilder.turntype(5) != Turn::Type::kSharpLeft) {
    throw runtime_error("DirectedEdge turn type for localidx 5 test failed");
  }
  if (directededgebuilder.turntype(1) != Turn::Type::kLeft) {
    throw runtime_error("DirectedEdge turn type for localidx 1 test failed");
  }
  if (directededgebuilder.turntype(2) != Turn::Type::kSharpRight) {
    throw runtime_error("DirectedEdge turn type for localidx 2 test failed");
  }

  directededgebuilder.set_stopimpact(5, 7);
  directededgebuilder.set_stopimpact(1, 4);
  directededgebuilder.set_stopimpact(3, 0);
  if (directededgebuilder.stopimpact(3) != 0) {
    throw runtime_error("DirectedEdge stopimpact for localidx 3 test failed");
  }
  if (directededgebuilder.stopimpact(5) != 7) {
    throw runtime_error("DirectedEdge stopimpact for localidx 5 test failed");
  }
  if (directededgebuilder.stopimpact(1) != 4) {
    throw runtime_error("DirectedEdge stopimpact for localidx 1 test failed");
  }
}

}

int main() {
  test::suite suite("directededgebuilder");

  // Write to file and read into DirectedEdge
  suite.test(TEST_CASE(TestWriteRead));

  return suite.tear_down();
}
