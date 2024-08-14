#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

class InstructionsRoundaboutMulticue : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 100;

    const std::string ascii_map = R"(
            I      O
            |      |
            H      N    P
           / \    / \   |
        A-B   E--F   L--M
           \ /    \ /
            C      J
            |      |
            D      K
    )";

    const gurka::ways ways =
        {{"AB",
          {{"highway", "primary"},
           {"name", "Main Street"},
           {"ref", "A 1"},
           {"maxspeed", "50 mph"},
           {"destination", "München;Kürten;Dhünn"},
           {"destination:ref", "A 95;B 2;B 300"}}},
         {"BCEHB",
          {{"highway", "primary"},
           {"oneway", "yes"},
           {"junction", "roundabout"},
           {"maxspeed", "50 mph"},
           {"name", ""}}},
         {"CD", {{"highway", "primary"}, {"maxspeed", "50 mph"}, {"name", "1st Avenue"}}},
         {"EF",
          {{"highway", "primary"}, {"maxspeed", "50 mph"}, {"name", "Main Street"}, {"ref", "A 1"}}},
         {"HI", {{"highway", "primary"}, {"maxspeed", "50 mph"}, {"name", ""}}},
         {"FJLNF",
          {{"highway", "primary"},
           {"oneway", "yes"},
           {"junction", "roundabout"},
           {"maxspeed", "50 mph"},
           {"name", ""}}},
         {"JK", {{"highway", "primary"}, {"maxspeed", "50 mph"}, {"name", "2nd Avenue"}}},
         {"LM",
          {{"highway", "primary"}, {"maxspeed", "50 mph"}, {"name", "Main Street"}, {"ref", "A 1"}}},
         {"NO", {{"highway", "primary"}, {"maxspeed", "50 mph"}, {"name", ""}}},
         {"PM", {{"highway", "primary"}, {"maxspeed", "50 mph"}, {"name", "3rd Avenue"}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_roundabout_multicue",
                            {{"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map InstructionsRoundaboutMulticue::map = {};

/*************************************************************/

TEST_F(InstructionsRoundaboutMulticue, StartThenEnterRoundaboutMulticueOrdinalOnly) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "I"}, "auto");
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRoundaboutEnter,
                                                DirectionsLeg_Maneuver_Type_kRoundaboutExit,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 0;

  // Verify the start then enter roundabout multicue
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Drive east on Main Street/A 1.",
      "Drive east. Then Enter the roundabout and take the 3rd exit.", "",
      "Drive east on Main Street, A 1. Then Enter the roundabout and take the 3rd exit.",
      "Continue for 100 meters.");

  // Verify the enter_roundabout instructions
  gurka::assert::raw::
      expect_instructions_at_maneuver_index(result, ++maneuver_index,
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

TEST_F(InstructionsRoundaboutMulticue, StartThenEnterRoundaboutMulticueOntoStreetName) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto");
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRoundaboutEnter,
                                                DirectionsLeg_Maneuver_Type_kRoundaboutExit,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 0;

  // Verify the start then enter roundabout multicue
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Drive east on Main Street/A 1.",
      "Drive east. Then Enter the roundabout and take the 1st exit onto 1st Avenue.", "",
      "Drive east on Main Street, A 1. Then Enter the roundabout and take the 1st exit onto 1st Avenue.",
      "Continue for 100 meters.");

  // Verify the enter_roundabout instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, ++maneuver_index, "Enter the roundabout and take the 1st exit onto 1st Avenue.",
      "Enter the roundabout and take the 1st exit.",
      "Enter the roundabout and take the 1st exit onto 1st Avenue.",
      "Enter the roundabout and take the 1st exit onto 1st Avenue.", "");

  // Verify the exit_roundabout instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, ++maneuver_index, "Exit the roundabout onto 1st Avenue.",
      "Exit the roundabout. Then You will arrive at your destination.", "",
      "Exit the roundabout onto 1st Avenue. Then You will arrive at your destination.",
      "Continue for 200 meters.");
}

TEST_F(InstructionsRoundaboutMulticue, TurnThenEnterRoundaboutMulticueOrdinalOnly) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"P", "O"}, "auto");
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRight,
                                                DirectionsLeg_Maneuver_Type_kRoundaboutEnter,
                                                DirectionsLeg_Maneuver_Type_kRoundaboutExit,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the turn then enter roundabout multicue
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Turn right onto Main Street/A 1.",
      "Turn right. Then Enter the roundabout and take the 1st exit.", "Turn right onto Main Street.",
      "Turn right onto Main Street, A 1. Then Enter the roundabout and take the 1st exit.",
      "Continue for 200 meters.");

  // Verify the enter_roundabout instructions
  gurka::assert::raw::
      expect_instructions_at_maneuver_index(result, ++maneuver_index,
                                            "Enter the roundabout and take the 1st exit.",
                                            "Enter the roundabout and take the 1st exit.",
                                            "Enter the roundabout and take the 1st exit.",
                                            "Enter the roundabout and take the 1st exit.", "");

  // Verify the exit_roundabout instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, ++maneuver_index, "Exit the roundabout.",
      "Exit the roundabout. Then You will arrive at your destination.", "",
      "Exit the roundabout. Then You will arrive at your destination.", "Continue for 200 meters.");
}

// Verify that no multicue for exit/enter roundabout
TEST_F(InstructionsRoundaboutMulticue, ExitThenEnterRoundaboutNoMulticue) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"P", "A"}, "auto");
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRight,
                                                DirectionsLeg_Maneuver_Type_kRoundaboutEnter,
                                                DirectionsLeg_Maneuver_Type_kRoundaboutExit,
                                                DirectionsLeg_Maneuver_Type_kRoundaboutEnter,
                                                DirectionsLeg_Maneuver_Type_kRoundaboutExit,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the turn then enter roundabout multicue
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Turn right onto Main Street/A 1.",
      "Turn right. Then Enter the roundabout and take the 2nd exit onto Main Street.",
      "Turn right onto Main Street.",
      "Turn right onto Main Street, A 1. Then Enter the roundabout and take the 2nd exit onto Main Street.",
      "Continue for 200 meters.");

  // Verify the 1st enter_roundabout instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, ++maneuver_index, "Enter the roundabout and take the 2nd exit onto Main Street/A 1.",
      "Enter the roundabout and take the 2nd exit.",
      "Enter the roundabout and take the 2nd exit onto Main Street.",
      "Enter the roundabout and take the 2nd exit onto Main Street, A 1.", "");

  // Verify the 1st exit_roundabout instructions
  // NOTE: Should not be a multicue
  gurka::assert::raw::
      expect_instructions_at_maneuver_index(result, ++maneuver_index,
                                            "Exit the roundabout onto Main Street/A 1.",
                                            "Exit the roundabout.", "",
                                            "Exit the roundabout onto Main Street, A 1.",
                                            "Continue for 200 meters.");

  // Verify the 2nd enter_roundabout instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, ++maneuver_index,
      "Enter the roundabout and take the 2nd exit toward A 95/B 2/München/Kürten.",
      "Enter the roundabout and take the 2nd exit toward A 95, München.",
      "Enter the roundabout and take the 2nd exit toward A 95.",
      "Enter the roundabout and take the 2nd exit toward A 95, München.", "");

  // Verify the 2nd exit_roundabout instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, ++maneuver_index, "Exit the roundabout toward A 95/B 2/München/Kürten.",
      "Exit the roundabout toward A 95, München. Then You will arrive at your destination.", "",
      "Exit the roundabout toward A 95, München. Then You will arrive at your destination.",
      "Continue for 100 meters.");
}
