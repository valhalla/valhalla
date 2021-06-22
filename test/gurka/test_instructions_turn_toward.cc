#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

class InstructionsTurnToward : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 25;

    const std::string ascii_map = R"(
              GH
              ||
              ||
              ||
              ||
         I----JK----L
         M----NO----P----A----B----C
              ||             /|\
              ||       D----- | -----F
              ||              |
              ||              |
              QR              E
    )";

    const gurka::ways ways =
        {{"ABC", {{"highway", "primary"}, {"name", "Main Street"}}},
         {"BE",
          {{"highway", "primary"}, {"name", "1st Avenue"}, {"destination", "Baltimore;Washington"}}},
         {"BD",
          {{"highway", "primary"},
           {"name", "Lincoln Avenue"},
           {"destination", "Pittsburgh;Columbus"}}},
         {"BF",
          {{"highway", "primary"},
           {"name", "Jefferson Avenue"},
           {"destination", "Philadelphia;New York"}}},
         {"MNOPA", {{"highway", "primary"}, {"oneway", "yes"}, {"ref", "A2"}}},
         {"GJNQ", {{"highway", "primary"}, {"oneway", "yes"}, {"ref", "B2"}}},
         {"RO", {{"highway", "primary"}, {"oneway", "yes"}, {"ref", "A1"}}},
         {"OK",
          {{"highway", "primary"}, {"oneway", "yes"}, {"ref", "A1"}, {"destination", "Internal"}}},
         {"KH",
          {{"highway", "primary"},
           {"oneway", "yes"},
           {"ref", "B1"},
           {"destination", "Little Italy"},
           {"destination:ref", "B1;C1"}}},
         {"LK", {{"highway", "primary"}, {"oneway", "yes"}, {"ref", "N1"}}},
         {"KJ", {{"highway", "primary"}, {"oneway", "yes"}, {"ref", "A1"}}},
         {"JI",
          {{"highway", "primary"}, {"oneway", "yes"}, {"ref", "A1"}, {"destination", "Lancaster"}}}};

    const gurka::nodes nodes = {
        {"J", {{"highway", "traffic_signals"}, {"name", "Mannenbashi East"}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

    map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/gurka_instructions_turn_toward",
                            {{"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map InstructionsTurnToward::map = {};

///////////////////////////////////////////////////////////////////////////////
// Turn right toward
// "5": "Turn/Bear/Make a sharp <RELATIVE_DIRECTION> toward <TOWARD_SIGN>."
TEST_F(InstructionsTurnToward, TurnRightToward) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the turn right toward instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Turn right toward Baltimore/Washington.",
      "Turn right toward Baltimore, Washington. Then You will arrive at your destination.",
      "Turn right toward Baltimore.",
      "Turn right toward Baltimore, Washington. Then You will arrive at your destination.",
      "Continue for 100 meters.");
}

///////////////////////////////////////////////////////////////////////////////
// Make a sharp right toward
// "5": "Turn/Bear/Make a sharp <RELATIVE_DIRECTION> toward <TOWARD_SIGN>."
TEST_F(InstructionsTurnToward, SharpRightToward) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kSharpRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the sharp right toward instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Make a sharp right toward Pittsburgh/Columbus.",
      "Make a sharp right toward Pittsburgh, Columbus. Then You will arrive at your destination.",
      "Make a sharp right toward Pittsburgh.",
      "Make a sharp right toward Pittsburgh, Columbus. Then You will arrive at your destination.",
      "Continue for 100 meters.");
}

///////////////////////////////////////////////////////////////////////////////
// Bear right toward
// "5": "Turn/Bear/Make a sharp <RELATIVE_DIRECTION> toward <TOWARD_SIGN>."
TEST_F(InstructionsTurnToward, BearRightToward) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kSlightRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the slight right toward instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Bear right toward Philadelphia/New York.",
      "Bear right toward Philadelphia, New York. Then You will arrive at your destination.",
      "Bear right toward Philadelphia.",
      "Bear right toward Philadelphia, New York. Then You will arrive at your destination.",
      "Continue for 100 meters.");
}

///////////////////////////////////////////////////////////////////////////////
// Turn right toward ignore junction name
// "5": "Turn/Bear/Make a sharp <RELATIVE_DIRECTION> toward <TOWARD_SIGN>."
TEST_F(InstructionsTurnToward, TurnRightTowardIgnoreJunctionName) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"G", "I"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the turn right toward instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Turn right toward Lancaster.",
      "Turn right toward Lancaster. Then You will arrive at your destination.",
      "Turn right toward Lancaster.",
      "Turn right toward Lancaster. Then You will arrive at your destination.",
      "Continue for 80 meters.");
}

///////////////////////////////////////////////////////////////////////////////
// Turn left toward
// "5": "Turn/Bear/Make a sharp <RELATIVE_DIRECTION> toward <TOWARD_SIGN>."
TEST_F(InstructionsTurnToward, TurnLeftToward) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"C", "E"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the turn left toward instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Turn left toward Baltimore/Washington.",
      "Turn left toward Baltimore, Washington. Then You will arrive at your destination.",
      "Turn left toward Baltimore.",
      "Turn left toward Baltimore, Washington. Then You will arrive at your destination.",
      "Continue for 100 meters.");
}

///////////////////////////////////////////////////////////////////////////////
// Make a sharp left toward
// "5": "Turn/Bear/Make a sharp <RELATIVE_DIRECTION> toward <TOWARD_SIGN>."
TEST_F(InstructionsTurnToward, SharpLeftToward) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"C", "F"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kSharpLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the sharp left toward instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Make a sharp left toward Philadelphia/New York.",
      "Make a sharp left toward Philadelphia, New York. Then You will arrive at your destination.",
      "Make a sharp left toward Philadelphia.",
      "Make a sharp left toward Philadelphia, New York. Then You will arrive at your destination.",
      "Continue for 100 meters.");
}

///////////////////////////////////////////////////////////////////////////////
// Bear left toward
// "5": "Turn/Bear/Make a sharp <RELATIVE_DIRECTION> toward <TOWARD_SIGN>."
TEST_F(InstructionsTurnToward, BearLeftToward) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"C", "D"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kSlightLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the slight left toward instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Bear left toward Pittsburgh/Columbus.",
      "Bear left toward Pittsburgh, Columbus. Then You will arrive at your destination.",
      "Bear left toward Pittsburgh.",
      "Bear left toward Pittsburgh, Columbus. Then You will arrive at your destination.",
      "Continue for 100 meters.");
}

///////////////////////////////////////////////////////////////////////////////
// Turn left toward using internal edge
// "5": "Turn/Bear/Make a sharp <RELATIVE_DIRECTION> toward <TOWARD_SIGN>."
TEST_F(InstructionsTurnToward, TurnLeftTowardUsingInternalEdge) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"M", "H"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the turn left toward instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Turn left toward B1/C1/Little Italy.",
      "Turn left toward B1, Little Italy. Then You will arrive at your destination.",
      "Turn left toward B1.",
      "Turn left toward B1, Little Italy. Then You will arrive at your destination.",
      "Continue for 200 meters.");
}

///////////////////////////////////////////////////////////////////////////////
// Make a left U-turn toward using internal edges
// "7": "Make a <RELATIVE_DIRECTION> U-turn toward <TOWARD_SIGN>."
TEST_F(InstructionsTurnToward, LeftUturnTowardUsingInternalEdges) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"M", "I"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kUturnLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the left u-turn toward instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Make a left U-turn toward Lancaster.",
      "Make a left U-turn toward Lancaster. Then You will arrive at your destination.",
      "Make a left U-turn toward Lancaster.",
      "Make a left U-turn toward Lancaster. Then You will arrive at your destination.",
      "Continue for 100 meters.");
}

///////////////////////////////////////////////////////////////////////////////
// Continue toward using internal edge
// "3": "Continue toward <TOWARD_SIGN>."
// "3": "Continue toward <TOWARD_SIGN>."
// "6": "Continue toward <TOWARD_SIGN>."
// TODO: expand map for obvious maneuver
// TEST_F(InstructionsTurnToward, ContinueTowardUsingInternalEdge) {
//  auto result = gurka::do_action(valhalla::Options::route, map, {"R", "H"}, "auto");
//
//  // Verify maneuver types
//  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
//                                                DirectionsLeg_Maneuver_Type_kContinue,
//                                                DirectionsLeg_Maneuver_Type_kDestination});
//  int maneuver_index = 1;
//
//  // Verify the turn left toward instructions
//  gurka::assert::raw::expect_instructions_at_maneuver_index(
//      result, maneuver_index, "Continue toward B1/C1/Little Italy.", "Continue.", "Continue toward
//      B1.", "Continue toward B1, Little Italy. Then You will arrive at your destination.", "Continue
//      for 200 meters.");
//}
