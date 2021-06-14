#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

//#############################################################################
class InstructionsRightLinkThenTurnExample1 : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 20;

    const std::string ascii_map = R"(

                            M L
                            | |
                            | |
                            | |
                            | |
                            | |
                            | |
                            | |
                            | |
                          H | |
    A-----B--------------------------------C
              D--E--------G-J-K
                 |        | | | 
                 |        I | |
                 F          N O
    )";

    const gurka::ways ways =
        {{"AB",
          {{"highway", "primary"},
           {"name", "Den Boschsingel"},
           {"ref", "R23"},
           {"oneway", "yes"},
           {"lanes", "2"},
           {"maxspeed", "70"}}},
         {"BC",
          {{"highway", "primary"},
           {"name", "Den Boschsingel"},
           {"ref", "R23"},
           {"oneway", "yes"},
           {"lanes", "2"},
           {"maxspeed", "70"}}},
         {"BD",
          {{"highway", "primary_link"},
           {"name", "Den Boschsingel"},
           {"destination", "Mechelen"},
           {"destination:ref", "N26"},
           {"oneway", "yes"},
           {"maxspeed", "30"}}},
         {"DE",
          {{"highway", "primary_link"},
           {"name", "Den Boschsingel"},
           {"oneway", "yes"},
           {"maxspeed", "30"}}},
         {"EF", {{"highway", "residential"}, {"name", "Mechelsevest"}, {"maxspeed", "30"}}},
         {"EG",
          {{"highway", "primary_link"},
           {"name", "Den Boschsingel"},
           {"oneway", "yes"},
           {"maxspeed", "30"}}},
         {"GJ",
          {{"highway", "primary_link"},
           {"name", "Den Boschsingel"},
           {"oneway", "yes"},
           {"maxspeed", "30"}}},
         {"HGI", {{"highway", "cycleway"}, {"oneway", "yes"}}},
         {"JK", {{"highway", "primary_link"}, {"oneway", "yes"}, {"maxspeed", "30"}}},
         {"KL",
          {{"highway", "primary"},
           {"name", "Nieuwe Mechelsesteenweg"},
           {"ref", "N26"},
           {"oneway", "yes"},
           {"maxspeed", "50"}}},
         {"MJ",
          {{"highway", "primary"},
           {"name", "Nieuwe Mechelsesteenweg"},
           {"ref", "N26"},
           {"oneway", "yes"},
           {"maxspeed", "50"}}},
         {"JN",
          {{"highway", "tertiary"}, {"name", "Donkerstraat"}, {"oneway", "yes"}, {"maxspeed", "50"}}},
         {"OK",
          {{"highway", "tertiary"},
           {"name", "Donkerstraat"},
           {"oneway", "yes"},
           {"maxspeed", "50"}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

    map = gurka::buildtiles(layout, ways, {}, {},
                            "test/data/gurka_instructions_right_link_then_turn_example1",
                            {{"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map InstructionsRightLinkThenTurnExample1::map = {};

///////////////////////////////////////////////////////////////////////////////
TEST_F(InstructionsRightLinkThenTurnExample1, RightLinkTurnLeft) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "L"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kSlightRight,
                                                DirectionsLeg_Maneuver_Type_kLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the bear right instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                            "Bear right toward N26/Mechelen.",
                                                            "Bear right toward N26, Mechelen.",
                                                            "Bear right toward N26.",
                                                            "Bear right toward N26, Mechelen.",
                                                            "Continue for 300 meters.");
  // Verify the turn left instructions
  gurka::assert::raw::
      expect_instructions_at_maneuver_index(result, ++maneuver_index,
                                            "Turn left onto Nieuwe Mechelsesteenweg/N26.",
                                            "Turn left.", "Turn left onto Nieuwe Mechelsesteenweg.",
                                            "Turn left onto Nieuwe Mechelsesteenweg, N26.",
                                            "Continue for 200 meters.");
}

///////////////////////////////////////////////////////////////////////////////
TEST_F(InstructionsRightLinkThenTurnExample1, RightLinkTurnRightInMiddleOfLink) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kSlightRight,
                                                DirectionsLeg_Maneuver_Type_kRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the bear right instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Bear right toward N26/Mechelen.",
      "Bear right toward N26, Mechelen. Then Turn right onto Mechelsevest.", "Bear right toward N26.",
      "Bear right toward N26, Mechelen. Then Turn right onto Mechelsevest.",
      "Continue for 90 meters.");
  // Verify the turn right instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, ++maneuver_index, "Turn right onto Mechelsevest.",
      "Turn right. Then You will arrive at your destination.", "Turn right onto Mechelsevest.",
      "Turn right onto Mechelsevest. Then You will arrive at your destination.",
      "Continue for 60 meters.");
}

//#############################################################################
class InstructionsRightLinkThenTurnExample2 : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 20;

    const std::string ascii_map = R"(
             DP
             ||
        R----|O---------------Q
             ||
        S----|M---------------N
             ||
             |K----L
             ||
             ||
             |I----J
             ||
             ||
         G---CF----H
             ||
             ||
             |E
             B
             |
             |
             A
    )";

    const gurka::ways ways =
        {{"AB",
          {{"highway", "primary"}, {"name", "North Capitol Street Northeast"}, {"oneway", "yes"}}},
         {"BC",
          {{"highway", "primary"}, {"name", "North Capitol Street Northeast"}, {"oneway", "yes"}}},
         {"CD",
          {{"highway", "primary"}, {"name", "North Capitol Street Northeast"}, {"oneway", "yes"}}},
         {"BE",
          {{"highway", "primary_link"},
           {"name", ""},
           {"destination:street:to", "New York Avenue"},
           {"oneway", "yes"}}},
         {"EF",
          {{"highway", "primary_link"},
           {"name", ""},
           {"destination:street:to", "New York Avenue"},
           {"oneway", "yes"}}},
         {"FI",
          {{"highway", "primary_link"},
           {"name", ""},
           {"destination:street:to", "New York Avenue"},
           {"oneway", "yes"}}},
         {"GC", {{"highway", "secondary"}, {"name", "M Street Northwest"}, {"oneway", "yes"}}},
         {"CF", {{"highway", "secondary"}, {"name", "M Street Northwest"}, {"oneway", "yes"}}},
         {"FH", {{"highway", "tertiary"}, {"name", "M Street Northeast"}, {"oneway", "yes"}}},
         {"IJ", {{"highway", "residential"}, {"name", "Patterson Street Northeast"}}},
         {"IK", {{"highway", "primary_link"}, {"name", ""}, {"oneway", "yes"}}},
         {"KL", {{"highway", "residential"}, {"name", "N Street Northeast"}, {"oneway", "yes"}}},
         {"KM", {{"highway", "primary_link"}, {"name", ""}, {"oneway", "yes"}}},
         {"SMN",
          {{"highway", "trunk"},
           {"name", "New York Avenue Northeast"},
           {"ref", "US 50;US 1 Alternate"},
           {"oneway", "yes"}}},
         {"MO", {{"highway", "primary_link"}, {"name", ""}, {"oneway", "yes"}}},
         {"QOR",
          {{"highway", "trunk"},
           {"name", "New York Avenue Northeast"},
           {"ref", "US 50;US 1 Alternate"},
           {"oneway", "yes"}}},
         {"OP", {{"highway", "primary_link"}, {"name", ""}, {"oneway", "yes"}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

    map = gurka::buildtiles(layout, ways, {}, {},
                            "test/data/gurka_instructions_right_link_then_turn_example2",
                            {{"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map InstructionsRightLinkThenTurnExample2::map = {};

///////////////////////////////////////////////////////////////////////////////
TEST_F(InstructionsRightLinkThenTurnExample2, RightLinkTurn1stRight) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "H"}, "auto");

  // Verify maneuver types (turn_degree=302)
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kSlightRight,
                                                DirectionsLeg_Maneuver_Type_kRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the bear right instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Bear right toward New York Avenue.",
      "Bear right toward New York Avenue. Then Turn right onto M Street Northeast.",
      "Bear right toward New York Avenue.",
      "Bear right toward New York Avenue. Then Turn right onto M Street Northeast.",
      "Continue for 80 meters.");

  // Verify the turn right instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, ++maneuver_index, "Turn right onto M Street Northeast.",
      "Turn right. Then You will arrive at your destination.", "Turn right onto M Street Northeast.",
      "Turn right onto M Street Northeast. Then You will arrive at your destination.",
      "Continue for 60 meters.");
}

///////////////////////////////////////////////////////////////////////////////
TEST_F(InstructionsRightLinkThenTurnExample2, RightLinkTurn2ndRight) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "J"}, "auto");

  // Verify maneuver types (turn_degree=302)
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kSlightRight,
                                                DirectionsLeg_Maneuver_Type_kRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the bear right instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Bear right toward New York Avenue.",
      "Bear right toward New York Avenue. Then Turn right onto Patterson Street Northeast.",
      "Bear right toward New York Avenue.",
      "Bear right toward New York Avenue. Then Turn right onto Patterson Street Northeast.",
      "Continue for 100 meters.");

  // Verify the turn right instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, ++maneuver_index, "Turn right onto Patterson Street Northeast.",
      "Turn right. Then You will arrive at your destination.",
      "Turn right onto Patterson Street Northeast.",
      "Turn right onto Patterson Street Northeast. Then You will arrive at your destination.",
      "Continue for 60 meters.");
}

///////////////////////////////////////////////////////////////////////////////
TEST_F(InstructionsRightLinkThenTurnExample2, RightLinkTurn3rdRight) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "L"}, "auto");

  // Verify maneuver types (turn_degree=302)
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kSlightRight,
                                                DirectionsLeg_Maneuver_Type_kRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the bear right instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Bear right toward New York Avenue.",
      "Bear right toward New York Avenue. Then Turn right onto N Street Northeast.",
      "Bear right toward New York Avenue.",
      "Bear right toward New York Avenue. Then Turn right onto N Street Northeast.",
      "Continue for 200 meters.");

  // Verify the turn right instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, ++maneuver_index, "Turn right onto N Street Northeast.",
      "Turn right. Then You will arrive at your destination.", "Turn right onto N Street Northeast.",
      "Turn right onto N Street Northeast. Then You will arrive at your destination.",
      "Continue for 60 meters.");
}

///////////////////////////////////////////////////////////////////////////////
TEST_F(InstructionsRightLinkThenTurnExample2, RightLinkTurn4thRight) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "N"}, "auto");

  // Verify maneuver types (turn_degree=302)
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kSlightRight,
                                                DirectionsLeg_Maneuver_Type_kRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the bear right instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Bear right toward New York Avenue.",
      "Bear right toward New York Avenue. Then Turn right onto US 50.",
      "Bear right toward New York Avenue.",
      "Bear right toward New York Avenue. Then Turn right onto US 50.", "Continue for 200 meters.");

  // Verify the turn right instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, ++maneuver_index, "Turn right onto US 50/US 1 Alternate/New York Avenue Northeast.",
      "Turn right. Then You will arrive at your destination.", "Turn right onto US 50.",
      "Turn right onto US 50, US 1 Alternate. Then You will arrive at your destination.",
      "Continue for 200 meters.");
}

//#############################################################################
class InstructionsRightLinkThenTurnExample3 : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 20;

    const std::string ascii_map = R"(
             C
             |
     H-------|-F-----------G
             | |
             | |
             | E
             | |\
             | D \
             |/   \
             B     I
             |
             |
             |
             |
             A
    )";

    const gurka::ways ways = {{"AB",
                               {{"highway", "primary"},
                                {"name", "Den Boschsingel"},
                                {"ref", "R23"},
                                {"oneway", "yes"},
                                {"lanes", "2"}}},
                              {"BC",
                               {{"highway", "primary"},
                                {"name", "Den Boschsingel"},
                                {"ref", "R23"},
                                {"oneway", "yes"},
                                {"lanes", "2"}}},
                              {"BD",
                               {{"highway", "primary_link"},
                                {"name", "Den Boschsingel"},
                                {"oneway", "yes"},
                                {"destination", "Centrum;Brussel"},
                                {"destination:ref", "N2"},
                                {"lanes", "1"}}},
                              {"DE",
                               {{"highway", "primary_link"},
                                {"name", "'s-Hertogenlaan"},
                                {"oneway", "yes"},
                                {"lanes", "1"}}},
                              {"EF",
                               {{"highway", "primary_link"},
                                {"name", "'s-Hertogenlaan"},
                                {"oneway", "yes"},
                                {"lanes", "1"}}},
                              {"FG",
                               {{"highway", "primary"},
                                {"name", "Brusselsesteenweg"},
                                {"ref", "N2"},
                                {"oneway", "no"},
                                {"lanes", "1"}}},
                              {"FH",
                               {{"highway", "primary"},
                                {"name", "Brusselsesteenweg"},
                                {"ref", "N2"},
                                {"oneway", "no"},
                                {"lanes", "1"}}},
                              {"IE",
                               {{"highway", "residential"},
                                {"name", "'s-Hertogenlaan"},
                                {"oneway", "yes"},
                                {"lanes", "1"}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

    map = gurka::buildtiles(layout, ways, {}, {},
                            "test/data/gurka_instructions_right_link_then_turn_example3",
                            {{"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map InstructionsRightLinkThenTurnExample3::map = {};

///////////////////////////////////////////////////////////////////////////////
TEST_F(InstructionsRightLinkThenTurnExample3, RightLinkTurnRight) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "auto");

  // Verify maneuver types (turn_degree=302)
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kSlightRight,
                                                DirectionsLeg_Maneuver_Type_kRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the bear right instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Bear right toward N2/Centrum/Brussel.",
      "Bear right toward N2, Centrum. Then Turn right onto Brusselsesteenweg.",
      "Bear right toward N2.",
      "Bear right toward N2, Centrum. Then Turn right onto Brusselsesteenweg.",
      "Continue for 100 meters.");

  // Verify the turn right instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, ++maneuver_index, "Turn right onto Brusselsesteenweg/N2.",
      "Turn right. Then You will arrive at your destination.", "Turn right onto Brusselsesteenweg.",
      "Turn right onto Brusselsesteenweg, N2. Then You will arrive at your destination.",
      "Continue for 100 meters.");
}

///////////////////////////////////////////////////////////////////////////////
TEST_F(InstructionsRightLinkThenTurnExample3, RightLinkTurnLeft) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "H"}, "auto");

  // Verify maneuver types (turn_degree=302)
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kSlightRight,
                                                DirectionsLeg_Maneuver_Type_kLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the bear right instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Bear right toward N2/Centrum/Brussel.",
      "Bear right toward N2, Centrum. Then Turn left onto Brusselsesteenweg.",
      "Bear right toward N2.",
      "Bear right toward N2, Centrum. Then Turn left onto Brusselsesteenweg.",
      "Continue for 100 meters.");

  // Verify the turn left instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, ++maneuver_index, "Turn left onto Brusselsesteenweg/N2.",
      "Turn left. Then You will arrive at your destination.", "Turn left onto Brusselsesteenweg.",
      "Turn left onto Brusselsesteenweg, N2. Then You will arrive at your destination.",
      "Continue for 100 meters.");
}
