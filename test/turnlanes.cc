#include <algorithm>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "baldr/turnlanes.h"

#include "test.h"

using namespace std;
using namespace valhalla::baldr;

// Expected size is 8 bytes. We want to alert if somehow any change grows
// this structure size as that indicates incompatible tiles.
constexpr size_t kTurnLanesExpectedSize = 8;

namespace {

void test_sizeof() {
  if (sizeof(valhalla::baldr::TurnLanes) != kTurnLanesExpectedSize)
    throw std::runtime_error("TurnLanes size should be " + std::to_string(kTurnLanesExpectedSize) +
                             " bytes" + " but is " +
                             std::to_string(sizeof(valhalla::baldr::TurnLanes)));
}

void test_access() {
  TurnLanes tl(32, 1234);
  if (tl.edgeindex() != 32) {
    throw std::runtime_error("TurnLanes edgeindex failed");
  }
  if (tl.text_offset() != 1234) {
    throw std::runtime_error("TurnLanes text_offset failed");
  }
}

void test_static_methods() {
  std::string osm_turn_lanes = "left|through;right|";
  std::string val_internal = TurnLanes::GetTurnLaneString(osm_turn_lanes);
  std::vector<uint16_t> masks = TurnLanes::lanemasks(val_internal);
  std::string val_turn_lanes = TurnLanes::turnlane_string(masks);
  if (osm_turn_lanes != val_turn_lanes) {
    throw std::runtime_error("TurnLanes mismatch: " + val_turn_lanes +
                             " expected: " + osm_turn_lanes);
  }

  osm_turn_lanes = "|through;right||none|slight_left|left";
  val_internal = TurnLanes::GetTurnLaneString(osm_turn_lanes);
  masks = TurnLanes::lanemasks(val_internal);
  val_turn_lanes = TurnLanes::turnlane_string(masks);
  if (osm_turn_lanes != val_turn_lanes) {
    throw std::runtime_error("TurnLanes mismatch: " + val_turn_lanes +
                             " expected: " + osm_turn_lanes);
  }

  osm_turn_lanes = "merge_to_left||reverse|merge_to_right";
  val_internal = TurnLanes::GetTurnLaneString(osm_turn_lanes);
  masks = TurnLanes::lanemasks(val_internal);
  val_turn_lanes = TurnLanes::turnlane_string(masks);
  if (osm_turn_lanes != val_turn_lanes) {
    throw std::runtime_error("TurnLanes mismatch: " + val_turn_lanes +
                             " expected: " + osm_turn_lanes);
  }

  osm_turn_lanes = "none||none||none|";
  val_internal = TurnLanes::GetTurnLaneString(osm_turn_lanes);
  masks = TurnLanes::lanemasks(val_internal);
  val_turn_lanes = TurnLanes::turnlane_string(masks);
  if (osm_turn_lanes != val_turn_lanes) {
    throw std::runtime_error("TurnLanes mismatch: " + val_turn_lanes +
                             " expected: " + osm_turn_lanes);
  }

  // Test invalid values
  osm_turn_lanes = "|blah||none||none|";
  val_internal = TurnLanes::GetTurnLaneString(osm_turn_lanes);
  masks = TurnLanes::lanemasks(val_internal);
  val_turn_lanes = TurnLanes::turnlane_string(masks);
  if ("|||none||none|" != val_turn_lanes) {
    throw std::runtime_error("TurnLanes mismatch: " + val_turn_lanes +
                             " expected: " + "|||none||none|");
  }

  osm_turn_lanes = "blah|blah|";
  val_internal = TurnLanes::GetTurnLaneString(osm_turn_lanes);
  masks = TurnLanes::lanemasks(val_internal);
  val_turn_lanes = TurnLanes::turnlane_string(masks);
  if ("||" != val_turn_lanes) {
    throw std::runtime_error("TurnLanes mismatch: " + val_turn_lanes + " expected: " + "||");
  }
}

} // namespace

int main() {
  test::suite suite("turnlanes");

  // Test sizeof the structure
  suite.test(TEST_CASE(test_sizeof));

  // Test access methods
  suite.test(TEST_CASE(test_access));

  // Test static methods
  suite.test(TEST_CASE(test_static_methods));

  return suite.tear_down();
}
