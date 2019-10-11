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

void test_right_active() {
  // Load pinpoint test
  std::string pbf_filename = {VALHALLA_SOURCE_DIR
                              "test/pinpoints/turn_lanes/right_active_pinpoint.pbf"};
  std::string path_bytes;
  std::ifstream input_pbf(pbf_filename, std::ios::in | std::ios::binary);
  if (input_pbf.is_open()) {
    input_pbf.seekg(0, std::ios::end);
    path_bytes.resize(input_pbf.tellg());
    input_pbf.seekg(0, std::ios::beg);
    input_pbf.read(&path_bytes[0], path_bytes.size());
    input_pbf.close();
  } else {
    throw std::runtime_error("Failed to read " + pbf_filename);
  }

  valhalla::Api request;
  request.ParseFromString(path_bytes);

  if (path_bytes.size() == 0) {
    throw std::runtime_error("path_bytes is empty");
  }

  valhalla::odin::DirectionsBuilder().Build(request);

  // Validate routes size
  if (request.directions().routes_size() != 1) {
    throw std::runtime_error("Invalid routes size");
  }

  // Validate legs size
  if (request.directions().routes(0).legs_size() != 1) {
    throw std::runtime_error("Invalid legs size");
  }

  // Validate maneuvers size
  if (request.directions().routes(0).legs(0).maneuver_size() != 3) {
    throw std::runtime_error("Invalid maneuvers size");
  }

  valhalla::odin::EnhancedTripLeg etl(*request.mutable_trip()->mutable_routes(0)->mutable_legs(0));
  auto prev_edge =
      etl.GetPrevEdge(request.directions().routes(0).legs(0).maneuver(1).begin_path_index());
  std::string found = prev_edge->TurnLanesToString();
  std::string expected = "[ left | through | through;right ACTIVE ]";

  // Validate turn lanes
  if (expected != found) {
    throw std::runtime_error("Invalid turn lanes - found: " + found + " | expected: " + expected);
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

  // Test right active
  suite.test(TEST_CASE(test_right_active));

  return suite.tear_down();
}
