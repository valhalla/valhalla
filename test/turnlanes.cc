#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "baldr/turnlanes.h"
#include "odin/directionsbuilder.h"
#include "odin/enhancedtrippath.h"

#include <valhalla/proto/api.pb.h>
#include <valhalla/proto/directions.pb.h>
#include <valhalla/proto/options.pb.h>
#include <valhalla/proto/trip.pb.h>

#include "test.h"

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

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

void test_turn_lanes(const std::string filename,
                     int expected_routes_size,
                     int expected_legs_size,
                     int expected_maneuvers_size,
                     int maneuver_index,
                     const std::string expected_turn_lanes) {
  // Load pinpoint test
  std::string path_bytes = test::load_binary_file(filename);
  if (path_bytes.size() == 0) {
    throw std::runtime_error("path_bytes is empty");
  }

  // Create the request from the path bytes
  valhalla::Api request;
  request.ParseFromString(path_bytes);

  // Build the directions
  valhalla::odin::DirectionsBuilder().Build(request);

  // Validate routes size
  int found_routes_size = request.directions().routes_size();
  if (found_routes_size != expected_routes_size) {
    throw std::runtime_error("Invalid routes size - found: " + std::to_string(found_routes_size) +
                             " | expected: " + std::to_string(expected_routes_size));
  }

  // Validate legs size
  int found_legs_size = request.directions().routes(0).legs_size();
  if (found_legs_size != expected_legs_size) {
    throw std::runtime_error("Invalid legs size - found: " + std::to_string(found_legs_size) +
                             " | expected: " + std::to_string(expected_legs_size));
  }

  // Validate maneuvers size
  int found_maneuvers_size = request.directions().routes(0).legs(0).maneuver_size();
  if (found_maneuvers_size != expected_maneuvers_size) {
    throw std::runtime_error(
        "Invalid maneuvers size - found: " + std::to_string(found_maneuvers_size) +
        " | expected: " + std::to_string(expected_maneuvers_size));
  }

  // Validate turn lanes - get turn lanes from the prev edge of the specified maneuver index
  valhalla::odin::EnhancedTripLeg etl(*request.mutable_trip()->mutable_routes(0)->mutable_legs(0));
  auto prev_edge = etl.GetPrevEdge(
      request.directions().routes(0).legs(0).maneuver(maneuver_index).begin_path_index());
  std::string found_turn_lanes = prev_edge->TurnLanesToString();
  if (found_turn_lanes != expected_turn_lanes) {
    throw std::runtime_error("Invalid turn lanes - found: " + found_turn_lanes +
                             " | expected: " + expected_turn_lanes);
  }
}

void validate_turn_lanes() {

  int expected_routes_size = 1;
  int expected_legs_size = 1;
  int expected_maneuvers_size = 3;
  int maneuver_index = 1;

  // Test right active
  test_turn_lanes({VALHALLA_SOURCE_DIR "test/pinpoints/turn_lanes/right_active_pinpoint.pbf"},
                  expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                  "[ left | through | through;right ACTIVE ]");

  // Test left active
  test_turn_lanes({VALHALLA_SOURCE_DIR "test/pinpoints/turn_lanes/left_active_pinpoint.pbf"},
                  expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                  "[ left ACTIVE | through | through;right ]");

  // Test right most slight left active
  test_turn_lanes({VALHALLA_SOURCE_DIR
                   "test/pinpoints/turn_lanes/right_most_slight_left_active_pinpoint.pbf"},
                  expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                  "[ slight_left | slight_left ACTIVE | slight_right | right ]");

  // Test slight right active
  test_turn_lanes({VALHALLA_SOURCE_DIR "test/pinpoints/turn_lanes/slight_right_active_pinpoint.pbf"},
                  expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                  "[ slight_left | slight_left | slight_right ACTIVE | right ]");

  // Test left most left u-turn active
  test_turn_lanes({VALHALLA_SOURCE_DIR
                   "test/pinpoints/turn_lanes/left_most_left_uturn_active_pinpoint.pbf"},
                  expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                  "[ left ACTIVE | left | left | through | through;right ]");

  // Test left reverse active
  test_turn_lanes({VALHALLA_SOURCE_DIR "test/pinpoints/turn_lanes/left_reverse_active_pinpoint.pbf"},
                  expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                  "[ reverse ACTIVE | through | through | right ]");

  expected_maneuvers_size = 4;
  // Test right most left active
  test_turn_lanes({VALHALLA_SOURCE_DIR
                   "test/pinpoints/turn_lanes/right_most_left_active_pinpoint.pbf"},
                  expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                  "[ left | left;through ACTIVE | through;right ]");

  // Test both left active
  test_turn_lanes({VALHALLA_SOURCE_DIR "test/pinpoints/turn_lanes/both_left_active_pinpoint.pbf"},
                  expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                  "[ left ACTIVE | left;through ACTIVE | through;right ]");

  // Test left most left active
  test_turn_lanes({VALHALLA_SOURCE_DIR
                   "test/pinpoints/turn_lanes/left_most_left_active_pinpoint.pbf"},
                  expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                  "[ left ACTIVE | left;through | through;right ]");
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

  // Test turn lanes
  suite.test(TEST_CASE(validate_turn_lanes));

  return suite.tear_down();
}
