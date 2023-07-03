#include "gurka.h"
#include <gtest/gtest.h>

#include "odin/enhancedtrippath.h"

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

class InstructionsInternalIntersection : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 10;

    const std::string ascii_map = R"(
              E D
              | |
              | |
              | |
              | |
         L----F-C----K
              | |
         I----G-B----J
              | |
              | |
              | |
              | |
              H A
    )";

    const gurka::ways ways =
        {{"AB",
          {{"highway", "primary"},
           {"oneway", "yes"},
           {"name", "Broken Land Parkway"},
           {"lanes", "3"},
           {"turn:lanes", "left|through|through"}}},
         {"BC", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Broken Land Parkway"}}},
         {"CD", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Broken Land Parkway"}}},
         {"EF", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Broken Land Parkway"}}},
         {"FG", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Broken Land Parkway"}}},
         {"GH", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Broken Land Parkway"}}},
         {"IG", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Patuxent Woods Drive"}}},
         {"GB", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Snowden River Parkway"}}},
         {"BJ", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Snowden River Parkway"}}},
         {"KC", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Snowden River Parkway"}}},
         {"CF", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Snowden River Parkway"}}},
         {"FL", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Patuxent Woods Drive"}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

    map =
        gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_instructions_internal_intersection",
                          {{"mjolnir.admin",
                            {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map InstructionsInternalIntersection::map = {};

///////////////////////////////////////////////////////////////////////////////
// Drive north and turn left
TEST_F(InstructionsInternalIntersection, DriveNorth_BrokenLandParkway_TurnLeft_PatuxentWoodsDrive) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "L"}, "auto");

  // Verify path
  gurka::assert::raw::expect_path(result, {"Broken Land Parkway", "Broken Land Parkway",
                                           "Snowden River Parkway", "Patuxent Woods Drive"});

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});

  // Verify maneuver begin path indexes
  gurka::assert::raw::expect_maneuver_begin_path_indexes(result, {0, 2, 4});

  int maneuver_index = 0;

  // Verify start distance includes internal edge
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Drive north on Broken Land Parkway.",
      "Drive north. Then Turn left onto Patuxent Woods Drive.", "",
      "Drive north on Broken Land Parkway. Then Turn left onto Patuxent Woods Drive.",
      "Continue for 70 meters.");

  // Verify the turn left onto Patuxent Woods Drive instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, ++maneuver_index, "Turn left onto Patuxent Woods Drive.",
      "Turn left. Then You will arrive at your destination.", "Turn left onto Patuxent Woods Drive.",
      "Turn left onto Patuxent Woods Drive. Then You will arrive at your destination.",
      "Continue for 40 meters.");

  // Verify that the turn lane info is copied to the straight internal edge
  auto maneuver = result.directions().routes(0).legs(0).maneuver(maneuver_index);
  EXPECT_EQ(maneuver.type(), DirectionsLeg_Maneuver_Type_kLeft);
  odin::EnhancedTripLeg etl(*result.mutable_trip()->mutable_routes(0)->mutable_legs(0));
  auto prev_edge = etl.GetPrevEdge(maneuver.begin_path_index());
  ASSERT_TRUE(prev_edge);
  EXPECT_EQ(prev_edge->turn_lanes_size(), 3);
  EXPECT_EQ(prev_edge->TurnLanesToString(), "[ *left* ACTIVE | through | through ]");
}

///////////////////////////////////////////////////////////////////////////////
// Start on internal edge - Drive north and turn left
TEST_F(InstructionsInternalIntersection,
       StartInternal_DriveNorth_BrokenLandParkway_TurnLeft_PatuxentWoodsDrive) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"B", "L"}, "auto");

  // Verify path
  gurka::assert::raw::expect_path(result, {"Broken Land Parkway", "Snowden River Parkway",
                                           "Patuxent Woods Drive"});

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});

  // Verify maneuver begin path indexes
  gurka::assert::raw::expect_maneuver_begin_path_indexes(result, {0, 1, 3});

  int maneuver_index = 0;

  // Verify start distance includes internal edge
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Drive north on Broken Land Parkway.",
      "Drive north. Then Turn left onto Patuxent Woods Drive.", "",
      "Drive north on Broken Land Parkway. Then Turn left onto Patuxent Woods Drive.",
      "Continue for 20 meters.");

  // Verify the turn left onto Patuxent Woods Drive instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, ++maneuver_index, "Turn left onto Patuxent Woods Drive.",
      "Turn left. Then You will arrive at your destination.", "Turn left onto Patuxent Woods Drive.",
      "Turn left onto Patuxent Woods Drive. Then You will arrive at your destination.",
      "Continue for 40 meters.");
}

///////////////////////////////////////////////////////////////////////////////
// Drive north and turn left - end on internal
TEST_F(InstructionsInternalIntersection,
       DriveNorth_BrokenLandParkway_TurnLeft_SnowdenRiverParkway_EndInternal) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto");

  // Verify path
  gurka::assert::raw::expect_path(result, {"Broken Land Parkway", "Broken Land Parkway",
                                           "Snowden River Parkway"});

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});

  // Verify maneuver begin path indexes
  gurka::assert::raw::expect_maneuver_begin_path_indexes(result, {0, 2, 3});

  int maneuver_index = 0;

  // Verify start distance includes internal edge
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Drive north on Broken Land Parkway.",
      "Drive north. Then Turn left onto Snowden River Parkway.", "",
      "Drive north on Broken Land Parkway. Then Turn left onto Snowden River Parkway.",
      "Continue for 70 meters.");

  // Verify the turn left onto Snowden River Parkway instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, ++maneuver_index, "Turn left onto Snowden River Parkway.",
      "Turn left. Then You will arrive at your destination.", "Turn left onto Snowden River Parkway.",
      "Turn left onto Snowden River Parkway. Then You will arrive at your destination.",
      "Continue for 10 meters.");
}

///////////////////////////////////////////////////////////////////////////////
// Drive north and turn left - Start and end on internal
TEST_F(InstructionsInternalIntersection,
       StartInternal_DriveNorth_BrokenLandParkway_TurnLeft_SnowdenRiverParkway_EndInternal) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"B", "F"}, "auto");

  // Verify path
  gurka::assert::raw::expect_path(result, {"Broken Land Parkway", "Snowden River Parkway"});

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});

  // Verify maneuver begin path indexes
  gurka::assert::raw::expect_maneuver_begin_path_indexes(result, {0, 1, 2});

  int maneuver_index = 0;

  // Verify start distance includes internal edge
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Drive north on Broken Land Parkway.",
      "Drive north. Then Turn left onto Snowden River Parkway.", "",
      "Drive north on Broken Land Parkway. Then Turn left onto Snowden River Parkway.",
      "Continue for 20 meters.");

  // Verify the turn left onto Snowden River Parkway instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, ++maneuver_index, "Turn left onto Snowden River Parkway.",
      "Turn left. Then You will arrive at your destination.", "Turn left onto Snowden River Parkway.",
      "Turn left onto Snowden River Parkway. Then You will arrive at your destination.",
      "Continue for 10 meters.");
}

///////////////////////////////////////////////////////////////////////////////
// Drive south and turn left
TEST_F(InstructionsInternalIntersection, DriveSouth_BrokenLandParkway_TurnLeft_SnowdenRiverParkway) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"E", "J"}, "auto");

  // Verify path
  gurka::assert::raw::expect_path(result, {"Broken Land Parkway", "Broken Land Parkway",
                                           "Snowden River Parkway", "Snowden River Parkway"});

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});

  // Verify maneuver begin path indexes
  gurka::assert::raw::expect_maneuver_begin_path_indexes(result, {0, 2, 4});

  int maneuver_index = 0;

  // Verify start distance includes internal edge
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Drive south on Broken Land Parkway.",
      "Drive south. Then Turn left onto Snowden River Parkway.", "",
      "Drive south on Broken Land Parkway. Then Turn left onto Snowden River Parkway.",
      "Continue for 70 meters.");

  // Verify the turn left onto Snowden River Parkway instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, ++maneuver_index, "Turn left onto Snowden River Parkway.",
      "Turn left. Then You will arrive at your destination.", "Turn left onto Snowden River Parkway.",
      "Turn left onto Snowden River Parkway. Then You will arrive at your destination.",
      "Continue for 40 meters.");
}

///////////////////////////////////////////////////////////////////////////////
// Drive east and turn left
TEST_F(InstructionsInternalIntersection, DriveEast_PatuxentWoodsDrive_TurnLeft_BrokenLandParkway) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"I", "D"}, "auto");

  // Verify path
  gurka::assert::raw::expect_path(result, {"Patuxent Woods Drive", "Snowden River Parkway",
                                           "Broken Land Parkway", "Broken Land Parkway"});

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});

  // Verify maneuver begin path indexes
  gurka::assert::raw::expect_maneuver_begin_path_indexes(result, {0, 2, 4});

  int maneuver_index = 0;

  // Verify start distance includes internal edge
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Drive east on Patuxent Woods Drive.",
      "Drive east. Then Turn left onto Broken Land Parkway.", "",
      "Drive east on Patuxent Woods Drive. Then Turn left onto Broken Land Parkway.",
      "Continue for 40 meters.");

  // Verify the turn left onto Broken Land Parkway instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, ++maneuver_index, "Turn left onto Broken Land Parkway.",
      "Turn left. Then You will arrive at your destination.", "Turn left onto Broken Land Parkway.",
      "Turn left onto Broken Land Parkway. Then You will arrive at your destination.",
      "Continue for 70 meters.");
}

///////////////////////////////////////////////////////////////////////////////
// Drive west and turn left
TEST_F(InstructionsInternalIntersection, DriveWest_SnowdenRiverParkway_TurnLeft_BrokenLandParkway) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"K", "H"}, "auto");

  // Verify path
  gurka::assert::raw::expect_path(result, {"Snowden River Parkway", "Snowden River Parkway",
                                           "Broken Land Parkway", "Broken Land Parkway"});

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});

  // Verify maneuver begin path indexes
  gurka::assert::raw::expect_maneuver_begin_path_indexes(result, {0, 2, 4});

  int maneuver_index = 0;

  // Verify start distance includes internal edge
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Drive west on Snowden River Parkway.",
      "Drive west. Then Turn left onto Broken Land Parkway.", "",
      "Drive west on Snowden River Parkway. Then Turn left onto Broken Land Parkway.",
      "Continue for 40 meters.");

  // Verify the turn left onto Broken Land Parkway instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, ++maneuver_index, "Turn left onto Broken Land Parkway.",
      "Turn left. Then You will arrive at your destination.", "Turn left onto Broken Land Parkway.",
      "Turn left onto Broken Land Parkway. Then You will arrive at your destination.",
      "Continue for 70 meters.");
}

///////////////////////////////////////////////////////////////////////////////
// Drive north and make a left u-turn
TEST_F(InstructionsInternalIntersection, DriveNorth_BrokenLandParkway_UturnLeft_BrokenLandParkway) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "H"}, "auto");

  // Verify path
  gurka::assert::raw::expect_path(result, {"Broken Land Parkway", "Broken Land Parkway",
                                           "Snowden River Parkway", "Broken Land Parkway",
                                           "Broken Land Parkway"});

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kUturnLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});

  // Verify maneuver begin path indexes
  gurka::assert::raw::expect_maneuver_begin_path_indexes(result, {0, 2, 5});

  int maneuver_index = 0;

  // Verify start distance includes internal edge
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Drive north on Broken Land Parkway.",
      "Drive north. Then Make a left U-turn at Snowden River Parkway.", "",
      "Drive north on Broken Land Parkway. Then Make a left U-turn at Snowden River Parkway.",
      "Continue for 70 meters.");

  // Verify the left u-turn at Snowden River Parkway onto Broken Land Parkway instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, ++maneuver_index,
      "Make a left U-turn at Snowden River Parkway to stay on Broken Land Parkway.",
      "Make a left U-turn. Then You will arrive at your destination.",
      "Make a left U-turn at Snowden River Parkway.",
      "Make a left U-turn at Snowden River Parkway to stay on Broken Land Parkway. Then You will arrive at your destination.",
      "Continue for 80 meters.");
}
