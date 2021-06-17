#include "gurka.h"
#include <gtest/gtest.h>

#include "odin/enhancedtrippath.h"

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

class InstructionsUturn : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {

    const std::string ascii_map = R"(
                            M N
                            | |
           D C              | |
           | |              | |
           | |              | |
           | |         R----L-O----S
           | |              | |
      G----E-B----H----I----K-P----T
           | |              | |
           | |              | |
           | |              | |
           | |              | |
           F A              J Q
	)";
    const gurka::ways ways =
        {{"AB", {{"highway", "primary"}}},
         {"BC", {{"highway", "primary"}}},
         {"DE", {{"highway", "primary"}}},
         {"EF", {{"highway", "primary"}}},
         {"HB", {{"highway", "secondary"}}},
         {"BE", {{"highway", "secondary"}, {"internal_intersection", "true"}}},
         {"EG", {{"highway", "secondary"}}},
         {"QP", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Broken Land Parkway"}}},
         {"PO", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Broken Land Parkway"}}},
         {"ON", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Broken Land Parkway"}}},
         {"ML", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Broken Land Parkway"}}},
         {"LK", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Broken Land Parkway"}}},
         {"KJ", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Broken Land Parkway"}}},
         {"IK", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Patuxent Woods Drive"}}},
         {"KP",
          {{"highway", "primary"},
           {"oneway", "yes"},
           {"name", "Snowden River Parkway"},
           {"internal_intersection", "false"}}},
         {"OL",
          {{"highway", "primary"},
           {"oneway", "yes"},
           {"name", "Snowden River Parkway"},
           {"internal_intersection", "false"}}},
         {"PT", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Snowden River Parkway"}}},
         {"OS", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Snowden River Parkway"}}},
         {"LR", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Patuxent Woods Drive"}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10, {5.1079374, 52.0887174});
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_instructions_uturn",
                            {{"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}},
                             {"mjolnir.data_processing.infer_internal_intersections", "false"}});
  }
};
gurka::map InstructionsUturn::map = {};
///////////////////////////////////////////////////////////////////////////////

// Drive north and turn left at internal intersection
TEST_F(InstructionsUturn, LUturn) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto");
  // Verify steps
  gurka::assert::osrm::expect_steps(result, {"AB", "EF"});

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kUturnLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});

  // Verify maneuver begin path indexes
  gurka::assert::raw::expect_maneuver_begin_path_indexes(result, {0, 1, 3});

  int maneuver_index = 1;

  // Verify the L U-turn instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Make a left U-turn at BE onto EF.",
      "Make a left U-turn. Then You will arrive at your destination.", "Make a left U-turn at BE.",
      "Make a left U-turn at BE onto EF. Then You will arrive at your destination.",
      "Continue for 60 meters.");
}

///////////////////////////////////////////////////////////////////////////////
// Drive north and make a left u-turn
TEST_F(InstructionsUturn, LUturn2) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"Q", "J"}, "auto");

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
      "Make a left U-turn. Then, in 80 meters, You will arrive at your destination.",
      "Make a left U-turn at Snowden River Parkway.",
      "Make a left U-turn at Snowden River Parkway to stay on Broken Land Parkway. Then, in 80 meters, You will arrive at your destination.",
      "Continue for 80 meters.");
}
