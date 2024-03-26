#include "gurka.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

class InstructionsRoundabout : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 100;

    const std::string ascii_map = R"(
            I
            |
            H
           / \
       A--B   E--F-G
           \ /
            C
            |
            D
    )";

    const gurka::ways ways =
        {{"AB",
          {{"highway", "primary"},
           {"name", "East Governor Road"},
           {"ref", "A 1"},
           {"direction", "East"},
           {"destination", "München;Kürten;Dhünn"},
           {"destination:ref", "A 95;B 2;B 300"}}},
         {"BCEHB",
          {{"highway", "primary"}, {"oneway", "yes"}, {"junction", "roundabout"}, {"name", ""}}},
         {"CD", {{"highway", "primary"}, {"name", "Homestead Lane"}}},
         {"EF",
          {{"highway", "primary"},
           {"name", "East Governor Road"},
           {"ref", "A 1"},
           {"direction", "East"}}},
         {"FG", {{"highway", "primary"}, {"ref", "A 1"}, {"direction", "East"}}},
         {"HI", {{"highway", "primary"}, {"name", ""}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_roundabouts",
                            {{"mjolnir.data_processing.use_direction_on_ways", "true"},
                             {"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map InstructionsRoundabout::map = {};

/*************************************************************/

// enter_roundabout_verbal
// "0": "Enter the roundabout."
TEST_F(InstructionsRoundabout, RoundaboutEnterOnly) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "auto");
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRoundaboutEnter,
                                                DirectionsLeg_Maneuver_Type_kDestination});

  // TODO: known issue - future update to end on a roundabout
  //  Verify the enter_roundabout instructions
#if 0
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                            "Enter the roundabout.",
                                                            "Enter the roundabout.",
                                                            "Enter the roundabout.",
                                                            "Enter the roundabout.", "");
#endif
}

// enter_roundabout_verbal
// "1": "Enter the roundabout and take the <ORDINAL_VALUE> exit."
TEST_F(InstructionsRoundabout, RoundaboutOrdinalOnly) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "I"}, "auto");
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRoundaboutEnter,
                                                DirectionsLeg_Maneuver_Type_kRoundaboutExit,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the enter_roundabout instructions
  gurka::assert::raw::
      expect_instructions_at_maneuver_index(result, maneuver_index,
                                            "Enter the roundabout and take the 3rd exit.",
                                            "Enter the roundabout and take the 3rd exit.",
                                            "Enter the roundabout and take the 3rd exit.",
                                            "Enter the roundabout and take the 3rd exit.", "");

  // Verify the exit_roundabout instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, ++maneuver_index, "Exit the roundabout.",
      "Exit the roundabout. Then You will arrive at your destination.", "",
      "Exit the roundabout. Then You will arrive at your destination.", "Continue for 200 meters.");
}

// enter_roundabout_verbal
// "2": "Enter the roundabout and take the <ORDINAL_VALUE> exit onto <ROUNDABOUT_EXIT_STREET_NAMES>."
TEST_F(InstructionsRoundabout, RoundaboutOntoStreetName) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto");
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRoundaboutEnter,
                                                DirectionsLeg_Maneuver_Type_kRoundaboutExit,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the enter_roundabout instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Enter the roundabout and take the 1st exit onto Homestead Lane.",
      "Enter the roundabout and take the 1st exit.",
      "Enter the roundabout and take the 1st exit onto Homestead Lane.",
      "Enter the roundabout and take the 1st exit onto Homestead Lane.", "");

  // Verify the exit_roundabout instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, ++maneuver_index, "Exit the roundabout onto Homestead Lane.",
      "Exit the roundabout. Then You will arrive at your destination.", "",
      "Exit the roundabout onto Homestead Lane. Then You will arrive at your destination.",
      "Continue for 200 meters.");
}

// enter_roundabout_verbal
// "3": "Enter the roundabout and take the <ORDINAL_VALUE> exit onto
// <ROUNDABOUT_EXIT_BEGIN_STREET_NAMES>."
TEST_F(InstructionsRoundabout, RoundaboutOntoBeginStreetName) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "auto");
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRoundaboutEnter,
                                                DirectionsLeg_Maneuver_Type_kRoundaboutExit,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the enter_roundabout instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index,
      "Enter the roundabout and take the 2nd exit onto East Governor Road/A 1 East.",
      "Enter the roundabout and take the 2nd exit.",
      "Enter the roundabout and take the 2nd exit onto East Governor Road.",
      "Enter the roundabout and take the 2nd exit onto East Governor Road, A 1 East.", "");

  // Verify the exit_roundabout instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, ++maneuver_index,
      "Exit the roundabout onto East Governor Road/A 1 East. Continue on A 1 East.",
      "Exit the roundabout.", "", "Exit the roundabout onto East Governor Road, A 1 East.",
      "Continue on A 1 East for 300 meters.");
}

// enter_roundabout_verbal
// "4": "Enter the roundabout and take the <ORDINAL_VALUE> exit toward <TOWARD_SIGN>."
TEST_F(InstructionsRoundabout, RoundaboutToward) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"I", "A"}, "auto");
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRoundaboutEnter,
                                                DirectionsLeg_Maneuver_Type_kRoundaboutExit,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the enter_roundabout instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index,
      "Enter the roundabout and take the 1st exit toward A 95/B 2/München/Kürten.",
      "Enter the roundabout and take the 1st exit toward A 95, München.",
      "Enter the roundabout and take the 1st exit toward A 95.",
      "Enter the roundabout and take the 1st exit toward A 95, München.", "");

  // Verify the exit_roundabout instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, ++maneuver_index, "Exit the roundabout toward A 95/B 2/München/Kürten.",
      "Exit the roundabout toward A 95, München. Then You will arrive at your destination.", "",
      "Exit the roundabout toward A 95, München. Then You will arrive at your destination.",
      "Continue for 200 meters.");
}

TEST_F(InstructionsRoundabout, RoundaboutExitSuppressed) {
  auto from = "A";
  auto to = "I";
  const std::string& request =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],"costing":"auto","roundabout_exits":false})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();
  auto result = gurka::do_action(valhalla::Options::route, map, request);

  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRoundaboutEnter,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the enter_roundabout instructions
  gurka::assert::raw::
      expect_instructions_at_maneuver_index(result, maneuver_index,
                                            "Enter the roundabout and take the 3rd exit.",
                                            "Enter the roundabout and take the 3rd exit.",
                                            "Enter the roundabout and take the 3rd exit.",
                                            "Enter the roundabout and take the 3rd exit.",
                                            "Continue for 200 meters.");

  // Verify the exit_roundabout is suppressed
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, ++maneuver_index,
                                                            "You have arrived at your destination.",
                                                            "",
                                                            "You will arrive at your destination.",
                                                            "You have arrived at your destination.",
                                                            "");
}

/*************************************************************/
TEST(InstructionsRoundaboutRegression, TurnChannelRoundaboutExitRegression) {
  // Regression test for roundabout exit that is also a turn channel
  // https://www.openstreetmap.org/query?lat=48.62346&lon=2.46777#map=18/48.62310/2.46813
  const std::string ascii_map = R"(
    K---L--------O---P
    1   \       /
         M     N
         \    /2
    A     E--F
    |    /    \
    B---D      G---J
    |    \    /
    C     I--H

  )";

  const gurka::ways ways = {
      {"ABC", {{"highway", "service"}}},
      {"BD", {{"highway", "tertiary"}}},
      {"DIHGFED", {{"highway", "secondary"}, {"junction", "roundabout"}, {"name", ""}}},
      {"GJ", {{"highway", "secondary"}}},
      {"KLOP", {{"highway", "trunk"}, {"oneway", "yes"}, {"name", "La Francilienne"}}},
      {"LME",
       {{"highway", "trunk_link"},
        {"oneway", "yes"},
        {"destination", "Corbeil-Esses-Centre; Z.I. de l'Apport Paris;Ports"},
        {"junction:ref", "30"}}},
      {"FNO", {{"highway", "trunk_link"}, {"oneway", "yes"}, {"name", ""}}},
  };

  const gurka::nodes nodes = {
      {"L",
       {{"highway", "motorway_junction"}, {"ref", "30"}, {"exit_to", "Corbeil-Essonnes centre"}}}};

  const double gridsize = 10;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {5.1079374, 52.0887174});

  auto map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/gurka_roundabouts_regression",
                               {{"mjolnir.admin",
                                 {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});

  auto from = "1";
  auto to = "2";
  const std::string& request =
      (boost::format(R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();

  auto result = gurka::do_action(valhalla::Options::route, map, request);
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kExitRight,
                                                DirectionsLeg_Maneuver_Type_kRoundaboutEnter,
                                                DirectionsLeg_Maneuver_Type_kRoundaboutExit,
                                                DirectionsLeg_Maneuver_Type_kDestinationRight});
  int maneuver_index = 2;

  // Verify the enter_roundabout instructions
  gurka::assert::raw::
      expect_instructions_at_maneuver_index(result, maneuver_index,
                                            "Enter the roundabout and take the 3rd exit.",
                                            "Enter the roundabout and take the 3rd exit.",
                                            "Enter the roundabout and take the 3rd exit.",
                                            "Enter the roundabout and take the 3rd exit.", "");

  maneuver_index = 4;
  // Verify the exit_roundabout is suppressed
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                            "Your destination is on the right.", "",
                                                            "Your destination will be on the right.",
                                                            "Your destination is on the right.", "");
}
