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

#include "proto/api.pb.h"
#include "proto/directions.pb.h"
#include "proto/options.pb.h"
#include "proto/trip.pb.h"

#include "test.h"

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla::baldr;

// Expected size is 8 bytes. We want to alert if somehow any change grows
// this structure size as that indicates incompatible tiles.
constexpr size_t kTurnLanesExpectedSize = 8;

namespace {

TEST(Turnlanes, test_sizeof) {
  EXPECT_EQ(sizeof(valhalla::baldr::TurnLanes), kTurnLanesExpectedSize);
}

TEST(Turnlanes, test_access) {
  TurnLanes tl(32, 1234);
  EXPECT_EQ(tl.edgeindex(), 32);
  EXPECT_EQ(tl.text_offset(), 1234);
}

TEST(Turnlanes, test_static_methods) {
  std::string osm_turn_lanes = "left|through;right|";
  std::string val_internal = TurnLanes::GetTurnLaneString(osm_turn_lanes);
  std::vector<uint16_t> masks = TurnLanes::lanemasks(val_internal);
  std::string val_turn_lanes = TurnLanes::turnlane_string(masks);
  EXPECT_EQ(val_turn_lanes, osm_turn_lanes);

  osm_turn_lanes = "|through;right||none|slight_left|left";
  val_internal = TurnLanes::GetTurnLaneString(osm_turn_lanes);
  masks = TurnLanes::lanemasks(val_internal);
  val_turn_lanes = TurnLanes::turnlane_string(masks);
  EXPECT_EQ(val_turn_lanes, osm_turn_lanes);

  osm_turn_lanes = "merge_to_left||reverse|merge_to_right";
  val_internal = TurnLanes::GetTurnLaneString(osm_turn_lanes);
  masks = TurnLanes::lanemasks(val_internal);
  val_turn_lanes = TurnLanes::turnlane_string(masks);
  EXPECT_EQ(val_turn_lanes, osm_turn_lanes);

  osm_turn_lanes = "none||none||none|";
  val_internal = TurnLanes::GetTurnLaneString(osm_turn_lanes);
  masks = TurnLanes::lanemasks(val_internal);
  val_turn_lanes = TurnLanes::turnlane_string(masks);
  EXPECT_EQ(val_turn_lanes, osm_turn_lanes);

  // Test invalid values
  osm_turn_lanes = "|blah||none||none|";
  val_internal = TurnLanes::GetTurnLaneString(osm_turn_lanes);
  masks = TurnLanes::lanemasks(val_internal);
  val_turn_lanes = TurnLanes::turnlane_string(masks);
  EXPECT_EQ(val_turn_lanes, "|||none||none|");

  osm_turn_lanes = "blah|blah|";
  val_internal = TurnLanes::GetTurnLaneString(osm_turn_lanes);
  masks = TurnLanes::lanemasks(val_internal);
  val_turn_lanes = TurnLanes::turnlane_string(masks);
  EXPECT_EQ(val_turn_lanes, "||");
}

void test_turn_lanes(const std::string& filename,
                     int expected_routes_size,
                     int expected_legs_size,
                     int expected_maneuvers_size,
                     int maneuver_index,
                     const std::string& expected_turn_lanes) {
  // Load pinpoint test
  std::string path_bytes = test::load_binary_file(filename);

  ASSERT_NE(path_bytes.size(), 0);

  // Create the request from the path bytes
  valhalla::Api request;
  request.ParseFromString(path_bytes);

  // Build the directions
  valhalla::odin::DirectionsBuilder().Build(request);

  // Validate routes size
  int found_routes_size = request.directions().routes_size();
  EXPECT_EQ(found_routes_size, expected_routes_size);

  // Validate legs size
  int found_legs_size = request.directions().routes(0).legs_size();
  EXPECT_EQ(found_legs_size, expected_legs_size);

  // Validate maneuvers size
  int found_maneuvers_size = request.directions().routes(0).legs(0).maneuver_size();
  EXPECT_EQ(found_maneuvers_size, expected_maneuvers_size);

  // Validate turn lanes - get turn lanes from the prev edge of the specified maneuver index
  valhalla::odin::EnhancedTripLeg etl(*request.mutable_trip()->mutable_routes(0)->mutable_legs(0));
  auto prev_edge = etl.GetPrevEdge(
      request.directions().routes(0).legs(0).maneuver(maneuver_index).begin_path_index());
  std::string found_turn_lanes = prev_edge->TurnLanesToString();
  EXPECT_EQ(found_turn_lanes, expected_turn_lanes);
}

TEST(Turnlanes, validate_turn_lanes) {

  int expected_routes_size = 1;
  int expected_legs_size = 1;
  int expected_maneuvers_size = 3;
  int maneuver_index = 1;

  // Test right active
  test_turn_lanes({VALHALLA_SOURCE_DIR "test/pinpoints/turn_lanes/right_active_pinpoint.pbf"},
                  expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                  "[ left | through | through;*right* ACTIVE ]");

  // Test left active
  test_turn_lanes({VALHALLA_SOURCE_DIR "test/pinpoints/turn_lanes/left_active_pinpoint.pbf"},
                  expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                  "[ *left* ACTIVE | through | through;right ]");

  // Test right most slight left active
  test_turn_lanes({VALHALLA_SOURCE_DIR
                   "test/pinpoints/turn_lanes/right_most_slight_left_active_pinpoint.pbf"},
                  expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                  "[ *slight_left* VALID | *slight_left* ACTIVE | slight_right | right ]");

  // Test slight right active
  test_turn_lanes({VALHALLA_SOURCE_DIR "test/pinpoints/turn_lanes/slight_right_active_pinpoint.pbf"},
                  expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                  "[ slight_left | slight_left | *slight_right* ACTIVE | right ]");

  // Test left most left u-turn active
  test_turn_lanes({VALHALLA_SOURCE_DIR
                   "test/pinpoints/turn_lanes/left_most_left_uturn_active_pinpoint.pbf"},
                  expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                  "[ *left* ACTIVE | left | left | through | through;right ]");

  // Test left reverse active
  test_turn_lanes({VALHALLA_SOURCE_DIR "test/pinpoints/turn_lanes/left_reverse_active_pinpoint.pbf"},
                  expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                  "[ *reverse* ACTIVE | through | through | right ]");

  expected_maneuvers_size = 4;
  // Test right most left active
  test_turn_lanes({VALHALLA_SOURCE_DIR
                   "test/pinpoints/turn_lanes/right_most_left_active_pinpoint.pbf"},
                  expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                  "[ *left* VALID | *left*;through ACTIVE | through;right ]");

  // Test both left active
  test_turn_lanes({VALHALLA_SOURCE_DIR "test/pinpoints/turn_lanes/both_left_active_pinpoint.pbf"},
                  expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                  "[ *left* ACTIVE | *left*;through ACTIVE | through;right ]");

  // Test left most left active
  test_turn_lanes({VALHALLA_SOURCE_DIR
                   "test/pinpoints/turn_lanes/left_most_left_active_pinpoint.pbf"},
                  expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                  "[ *left* ACTIVE | *left*;through VALID | through;right ]");
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
