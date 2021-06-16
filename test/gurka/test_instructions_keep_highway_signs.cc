#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

//#############################################################################
class InstructionsKeepHighwaySigns : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 1000;

    const std::string ascii_map = R"(
                C--D      J--K
          A--B<--G--H--I<--N--O
                E--F      L--M
    )";

    const gurka::ways ways = {{"AB", {{"highway", "motorway"}, {"name", ""}, {"ref", "A1"}}},
                              {"BC",
                               {{"highway", "motorway"},
                                {"name", ""},
                                {"ref", "B1"},
                                {"destination", "Harrisburg;Lancaster"},
                                {"destination:ref", "B1"}}},
                              {"CD", {{"highway", "motorway"}, {"name", ""}, {"ref", "B1"}}},
                              {"BE",
                               {{"highway", "motorway"},
                                {"name", ""},
                                {"ref", "B2"},
                                {"destination", "Baltimore;Washington"},
                                {"destination:ref", "B2"}}},
                              {"EF", {{"highway", "motorway"}, {"name", ""}, {"ref", "B2"}}},
                              {"BG",
                               {{"highway", "motorway"},
                                {"name", ""},
                                {"ref", "B3"},
                                {"destination", "New York;Philadelphia"},
                                {"destination:ref", "B3"}}},
                              {"GH", {{"highway", "motorway"}, {"name", ""}, {"ref", "B3"}}},
                              {"HI", {{"highway", "motorway"}, {"name", ""}, {"ref", "B3"}}},
                              {"IJ",
                               {{"highway", "motorway"},
                                {"name", ""},
                                {"ref", "C1"},
                                {"junction:ref", "22A"},
                                {"destination", "Harrisburg;Lancaster"},
                                {"destination:ref", "C1"}}},
                              {"JK", {{"highway", "motorway"}, {"name", ""}, {"ref", "C1"}}},
                              {"IL",
                               {{"highway", "motorway"},
                                {"name", ""},
                                {"ref", "C2"},
                                {"junction:ref", "22C"},
                                {"destination", "Baltimore;Washington"},
                                {"destination:ref", "C2"}}},
                              {"LM", {{"highway", "motorway"}, {"name", ""}, {"ref", "C2"}}},
                              {"IN",
                               {{"highway", "motorway"},
                                {"name", ""},
                                {"ref", "C3"},
                                {"junction:ref", "22B"},
                                {"destination", "New York;Philadelphia"},
                                {"destination:ref", "C3"}}},
                              {"NO", {{"highway", "motorway"}, {"name", ""}, {"ref", "C3"}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_instructions_keep_highway_signs",
                            {{"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map InstructionsKeepHighwaySigns::map = {};

///////////////////////////////////////////////////////////////////////////////
// Keep right
TEST_F(InstructionsKeepHighwaySigns, KeepRightBranchToward) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStayRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the keep right toward instructions
  gurka::assert::raw::
      expect_instructions_at_maneuver_index(result, maneuver_index,
                                            "Keep right to take B2 toward Baltimore/Washington.", "",
                                            "Keep right to take B2.",
                                            "Keep right to take B2 toward Baltimore, Washington.",
                                            "Continue for 4 kilometers.");
}

///////////////////////////////////////////////////////////////////////////////
// Keep left
TEST_F(InstructionsKeepHighwaySigns, KeepLeftBranchToward) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStayLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the keep left toward instructions
  gurka::assert::raw::
      expect_instructions_at_maneuver_index(result, maneuver_index,
                                            "Keep left to take B1 toward Harrisburg/Lancaster.", "",
                                            "Keep left to take B1.",
                                            "Keep left to take B1 toward Harrisburg, Lancaster.",
                                            "Continue for 4 kilometers.");
}

///////////////////////////////////////////////////////////////////////////////
// Keep straight
TEST_F(InstructionsKeepHighwaySigns, KeepStraightBranchToward) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "H"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStayStraight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the keep straight toward instructions
  gurka::assert::raw::
      expect_instructions_at_maneuver_index(result, maneuver_index,
                                            "Keep straight to take B3 toward New York/Philadelphia.",
                                            "", "Keep straight to take B3.",
                                            "Keep straight to take B3 toward New York, Philadelphia.",
                                            "Continue for 4 kilometers.");
}

///////////////////////////////////////////////////////////////////////////////
// Keep right to take exit number
// "5": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> toward <TOWARD_SIGN>.",
TEST_F(InstructionsKeepHighwaySigns, KeepRightExitNumberBranchToward) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"H", "M"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStayRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the keep right to take exit number toward instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Keep right to take exit 22C onto C2 toward Baltimore/Washington.", "",
      "Keep right to take exit 22C.",
      "Keep right to take exit 22C onto C2 toward Baltimore, Washington.",
      "Continue for 4 kilometers.");
}

///////////////////////////////////////////////////////////////////////////////
// Keep left to take exit number
TEST_F(InstructionsKeepHighwaySigns, KeepLeftExitNumberBranchToward) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"H", "K"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStayLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the keep left to take exit number toward instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Keep left to take exit 22A onto C1 toward Harrisburg/Lancaster.", "",
      "Keep left to take exit 22A.",
      "Keep left to take exit 22A onto C1 toward Harrisburg, Lancaster.",
      "Continue for 4 kilometers.");
}

///////////////////////////////////////////////////////////////////////////////
// Keep straight to take exit number
TEST_F(InstructionsKeepHighwaySigns, KeepStraightExitNumberBranchToward) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"H", "O"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStayStraight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the keep straight to take exit number toward instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Keep straight to take exit 22B onto C3 toward New York/Philadelphia.",
      "", "Keep straight to take exit 22B.",
      "Keep straight to take exit 22B onto C3 toward New York, Philadelphia.",
      "Continue for 4 kilometers.");
}

//#############################################################################
class InstructionsKeepHighway4LanesTo2And2 : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 1000;

    //  Location: 42.4909166, -83.4193516
    const std::string ascii_map = R"(
                ---C--D--E
          A--B<         F---------\G--H
                ---I--J
    )";

    const gurka::ways ways =
        {{"AB",
          {{"highway", "motorway"},
           {"name", "Walter P Reuther Freeway"},
           {"ref", "I 696 West"},
           {"oneway", "yes"},
           {"lanes", "4"},
           {"turn:lanes", "slight_left|slight_left|slight_right|slight_right"}}},
         {"BC",
          {{"highway", "motorway"},
           {"name", "Walter P Reuther Freeway"},
           {"ref", "I 696 West"},
           {"oneway", "yes"},
           {"lanes", "2"},
           {"destination", "Lansing"},
           {"destination:ref", "I 96 West"}}},
         {"CD",
          {{"highway", "motorway"},
           {"name", "Walter P Reuther Freeway"},
           {"ref", "I 696 West"},
           {"oneway", "yes"},
           {"lanes", "2"}}},
         {"DE",
          {{"highway", "motorway"},
           {"name", "Walter P Reuther Freeway"},
           {"ref", "I 696 West"},
           {"oneway", "yes"},
           {"lanes", "2"}}},
         {"EG",
          {{"highway", "motorway"},
           {"name", "Walter P Reuther Freeway"},
           {"ref", "I 696 West"},
           {"oneway", "yes"},
           {"lanes", "2"}}},
         {"FG", {{"highway", "motorway"}, {"name", ""}, {"ref", "I 96 West"}, {"oneway", "yes"}}},
         {"GH", {{"highway", "motorway"}, {"name", ""}, {"ref", "I 96 West"}, {"oneway", "yes"}}},
         {"BI",
          {{"highway", "motorway_link"},
           {"name", ""},
           {"oneway", "yes"},
           {"lanes", "2"},
           {"destination", "Toledo"},
           {"destination:ref", "I 96 East;I 275"},
           {"junction:ref", "1"}}},
         {"IJ", {{"highway", "motorway_link"}, {"name", ""}, {"oneway", "yes"}, {"lanes", "2"}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

    map = gurka::buildtiles(layout, ways, {}, {},
                            "test/data/gurka_instructions_keep_highway_signs_4lanes_to_2and2",
                            {{"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map InstructionsKeepHighway4LanesTo2And2::map = {};

///////////////////////////////////////////////////////////////////////////////
// Keep left
TEST_F(InstructionsKeepHighway4LanesTo2And2, KeepLeftBranchToward) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "H"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStayLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the keep left toward instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index,
      "Keep left to stay on I 696 West/Walter P Reuther Freeway toward Lansing.", "",
      "Keep left to stay on I 696 West.",
      "Keep left to stay on I 696 West, Walter P Reuther Freeway toward Lansing.",
      "Continue for 16 kilometers.");
}

///////////////////////////////////////////////////////////////////////////////
// Keep right
TEST_F(InstructionsKeepHighway4LanesTo2And2, KeepRightExitBranchToward) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "J"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStayRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the right right toward instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Keep right to take exit 1 onto I 96 East/I 275 toward Toledo.", "",
      "Keep right to take exit 1.", "Keep right to take exit 1 onto I 96 East, I 275 toward Toledo.",
      "Continue for 6 kilometers.");
}

//#############################################################################
class InstructionsKeepHighway3LanesTo2And2 : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 1000;

    // Location: 41.6915626, -83.5111575
    const std::string ascii_map = R"(
                ---C--D
          A--B<
                ---E--F
    )";

    const gurka::ways ways =
        {{"AB",
          {{"highway", "motorway"},
           {"name", ""},
           {"ref", "I 75 South"},
           {"oneway", "yes"},
           {"lanes", "3"},
           {"turn:lanes", "slight_left|slight_left;through|through"}}},
         {"BC",
          {{"highway", "motorway"},
           {"name", ""},
           {"ref", "I 75 South"},
           {"oneway", "yes"},
           {"lanes", "2"},
           {"destination", "Dayton"},
           {"destination:ref", "I 75 South"}}},
         {"CD",
          {{"highway", "motorway"},
           {"name", ""},
           {"ref", "I 75 South"},
           {"oneway", "yes"},
           {"lanes", "2"}}},
         {"BE",
          {{"highway", "motorway_link"},
           {"name", ""},
           {"lanes", "2"},
           {"oneway", "yes"},
           {"destination", "Cleveland"},
           {"destination:ref", "I 280 South"},
           {"junction:ref", "208"}}},
         {"EF", {{"highway", "motorway_link"}, {"name", ""}, {"oneway", "yes"}, {"lanes", "2"}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

    map = gurka::buildtiles(layout, ways, {}, {},
                            "test/data/gurka_instructions_keep_highway_signs_3lanes_to_2and2",
                            {{"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map InstructionsKeepHighway3LanesTo2And2::map = {};

///////////////////////////////////////////////////////////////////////////////
// Keep left
TEST_F(InstructionsKeepHighway3LanesTo2And2, KeepLeftBranchToward) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStayLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the keep left toward instructions
  gurka::assert::raw::
      expect_instructions_at_maneuver_index(result, maneuver_index,
                                            "Keep left to stay on I 75 South toward Dayton.", "",
                                            "Keep left to stay on I 75 South.",
                                            "Keep left to stay on I 75 South toward Dayton.",
                                            "Continue for 6 kilometers.");
}

///////////////////////////////////////////////////////////////////////////////
// Keep right
TEST_F(InstructionsKeepHighway3LanesTo2And2, KeepRightExitBranchToward) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStayRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the right right toward instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Keep right to take exit 208 onto I 280 South toward Cleveland.", "",
      "Keep right to take exit 208.",
      "Keep right to take exit 208 onto I 280 South toward Cleveland.", "Continue for 6 kilometers.");
}

//#############################################################################
class InstructionsKeepHighway2LanesTo1And1 : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 1000;

    //  Location: 37.7831899, -121.1946650
    const std::string ascii_map = R"(
                ---C--D
          A--B<
                ---E--F
    )";

    const gurka::ways ways =
        {{"AB",
          {{"highway", "motorway"},
           {"name", ""},
           {"ref", "CA 120"},
           {"oneway", "yes"},
           {"lanes", "2"},
           {"turn:lanes", "through|slight_right"}}},
         {"BC",
          {{"highway", "motorway"},
           {"name", ""},
           {"ref", "CA 120"},
           {"oneway", "yes"},
           {"lanes", "1"},
           {"destination", "Sacramento;Yosemite;Sonora"},
           {"destination:ref", "CA 99 North;CA 120"}}},
         {"CD",
          {{"highway", "motorway"},
           {"name", ""},
           {"ref", "CA 120"},
           {"oneway", "yes"},
           {"lanes", "1"}}},
         {"BE",
          {{"highway", "motorway_link"},
           {"name", ""},
           {"lanes", "1"},
           {"oneway", "yes"},
           {"destination", "Modesto;Fresno"},
           {"destination:ref", "CA 99 South"},
           {"junction:ref", "6"}}},
         {"EF", {{"highway", "motorway_link"}, {"name", ""}, {"oneway", "yes"}, {"lanes", "1"}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

    map = gurka::buildtiles(layout, ways, {}, {},
                            "test/data/gurka_instructions_keep_highway_signs_2lanes_to_1and1",
                            {{"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map InstructionsKeepHighway2LanesTo1And1::map = {};

///////////////////////////////////////////////////////////////////////////////
// Keep left
TEST_F(InstructionsKeepHighway2LanesTo1And1, KeepLeftBranchToward) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStayLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the keep left toward instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Keep left to stay on CA 120 toward Sacramento/Yosemite/Sonora.", "",
      "Keep left to stay on CA 120.", "Keep left to stay on CA 120 toward Sacramento, Yosemite.",
      "Continue for 6 kilometers.");
}

///////////////////////////////////////////////////////////////////////////////
// Keep right
TEST_F(InstructionsKeepHighway2LanesTo1And1, KeepRightExitBranchToward) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStayRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the right right toward instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Keep right to take exit 6 onto CA 99 South toward Modesto/Fresno.", "",
      "Keep right to take exit 6.",
      "Keep right to take exit 6 onto CA 99 South toward Modesto, Fresno.",
      "Continue for 6 kilometers.");
}
