#include "gurka.h"
#include <gtest/gtest.h>

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
        {{"AB", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Broken Land Parkway"}}},
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
// Left turn
TEST_F(InstructionsInternalIntersection, BrokenLandParkway_TurnLeft_PatuxentWoodsDrive) {
  auto result = gurka::route(map, "A", "L", "auto");

  // Verify path
  gurka::assert::raw::expect_path(result, {"Broken Land Parkway", "Broken Land Parkway",
                                           "Snowden River Parkway", "Patuxent Woods Drive"});

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 0;

  // Verify start distance includes internal edge
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Drive north on Broken Land Parkway.", "",
      "Drive north on Broken Land Parkway. Then Turn left onto Patuxent Woods Drive.",
      "Continue for 70 meters.");

  // Verify the turn left onto Patuxent Woods Drive instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, ++maneuver_index, "Turn left onto Patuxent Woods Drive.",
      "Turn left onto Patuxent Woods Drive.",
      "Turn left onto Patuxent Woods Drive. Then You will arrive at your destination.",
      "Continue for 40 meters.");
}

///////////////////////////////////////////////////////////////////////////////
// Left u-turn
TEST_F(InstructionsInternalIntersection, BrokenLandParkway_UturnLeft_BrokenLandParkway) {
  auto result = gurka::route(map, "A", "H", "auto");

  // Verify path
  gurka::assert::raw::expect_path(result, {"Broken Land Parkway", "Broken Land Parkway",
                                           "Snowden River Parkway", "Broken Land Parkway",
                                           "Broken Land Parkway"});

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kUturnLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 0;

  // Verify start distance includes internal edge
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Drive north on Broken Land Parkway.", "",
      "Drive north on Broken Land Parkway. Then Make a left U-turn at Snowden River Parkway.",
      "Continue for 70 meters.");

  // Verify the left u-turn at Snowden River Parkway onto Broken Land Parkway instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, ++maneuver_index,
      "Make a left U-turn at Snowden River Parkway to stay on Broken Land Parkway.",
      "Make a left U-turn at Snowden River Parkway.",
      "Make a left U-turn at Snowden River Parkway to stay on Broken Land Parkway. Then You will arrive at your destination.",
      "Continue for 80 meters.");
}
