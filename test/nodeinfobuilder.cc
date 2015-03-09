#include "test.h"

#include <vector>
#include <iostream>
#include <fstream>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/nodeinfo.h>
#include "mjolnir/nodeinfobuilder.h"
#include <boost/shared_array.hpp>
#include <memory>

using namespace std;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

void TestWriteRead() {
  // Test building NodeInfo and reading back values
  NodeInfoBuilder nodebuilder;

  // Headings get rounded off to nearest 2 degrees
  nodebuilder.set_heading(0, 266.5f);
  nodebuilder.set_heading(3, 185.5f);
  nodebuilder.set_heading(2, 32.5f);
  nodebuilder.set_heading(6, 145.5f);
  if (nodebuilder.heading(0) != 266) {
    throw runtime_error("NodeInfo heading for localidx 0 test failed " +
                        std::to_string(nodebuilder.heading(0)));
  }
  if (nodebuilder.heading(2) != 32) {
     throw runtime_error("NodeInfo heading for localidx 2 test failed " +
                         std::to_string(nodebuilder.heading(2)));
  }
  if (nodebuilder.heading(3) != 186) {
    throw runtime_error("NodeInfo heading for localidx 3 test failed " +
                        std::to_string(nodebuilder.heading(5)));
  }
  if (nodebuilder.heading(6) != 146) {
    throw runtime_error("NodeInfo heading for localidx 6 test failed " +
                        std::to_string(nodebuilder.heading(6)));
  }

  nodebuilder.set_name_consistency(0, 4, true);
  nodebuilder.set_name_consistency(3, 1, false);
  nodebuilder.set_name_consistency(2, 7, true);
  nodebuilder.set_name_consistency(6, 6, true);
  if (nodebuilder.name_consistency(0, 4) != true) {
    throw runtime_error("NodeInfo name_consistency for 0,4 test failed");
  }
  if (nodebuilder.name_consistency(4, 0) != true) {
    throw runtime_error("NodeInfo name_consistency for 4,0 test failed");
  }
  if (nodebuilder.name_consistency(1, 3) != false) {
    throw runtime_error("NodeInfo name_consistency for 1,3 test failed");
  }
  if (nodebuilder.name_consistency(7, 2) != true) {
    throw runtime_error("NodeInfo name_consistency for 7,2 test failed");
  }
  if (nodebuilder.name_consistency(6, 6) != true) {
    throw runtime_error("NodeInfo name_consistency for 6,6 test failed");
  }

  nodebuilder.set_local_driveability(3, Driveability::kBoth);
  nodebuilder.set_local_driveability(5, Driveability::kNone);
  nodebuilder.set_local_driveability(7, Driveability::kForward);
  nodebuilder.set_local_driveability(1, Driveability::kBackward);
  if (nodebuilder.local_driveability(3) != Driveability::kBoth) {
    throw runtime_error("NodeInfo local_driveability 3 test failed");
  }
  if (nodebuilder.local_driveability(5) != Driveability::kNone) {
    throw runtime_error("NodeInfo local_driveability 5 test failed");
  }
  if (nodebuilder.local_driveability(7) != Driveability::kForward) {
    throw runtime_error("NodeInfo local_driveability 5 test failed");
  }
  if (nodebuilder.local_driveability(1) != Driveability::kBackward) {
    throw runtime_error("NodeInfo local_driveability 5 test failed");
  }
}

}

int main() {
  test::suite suite("nodeinfobuilder");

  // Write to file and read into NodeInfoEdge
  suite.test(TEST_CASE(TestWriteRead));

  return suite.tear_down();
}
