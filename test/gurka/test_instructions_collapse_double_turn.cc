#include "gurka.h"
#include <gtest/gtest.h>

#include "odin/enhancedtrippath.h"

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

class InstructionsCollapseDoubleTurn : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
                            E
                            |
                       A    D  
                       |    |
                       |    |
                       B----C----F
                       |    |
                       G----H----I  
			    )";
    const gurka::ways ways =
        {{"AG", {{"highway", "motorway_link"}}},
         {"BG", {{"highway", "motorway_link"}}},
         {"GH", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "South River Road"}}},
         {"HI", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "South River Road"}}},
         {"HC", {{"highway", "tertiary"}, {"internal_intersection", "false"}}},
         {"CD", {{"highway", "motorway_link"}}},
         {"DE", {{"highway", "trunk"}, {"name", "US 322"}}},
         {"CB", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "South River Road"}}},
         {"FC", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "South River Road"}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_instructions_collapse_double_turn",
                            {{"mjolnir.data_processing.infer_internal_intersections", "false"}});
  }
};
gurka::map InstructionsCollapseDoubleTurn::map = {};

///////////////////////////////////////////////////////////////////////////////

// Drive north and turn left

TEST_F(InstructionsCollapseDoubleTurn, CollapseToOneLeft) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "auto");
  // Verify steps
  gurka::assert::osrm::expect_steps(result, {"AG", "South River Road", "CD"});

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kLeft,
                                                DirectionsLeg_Maneuver_Type_kRampLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});

  // Verify maneuver begin path indexes
  gurka::assert::raw::expect_maneuver_begin_path_indexes(result, {0, 1, 2, 5});

  int maneuver_index = 1;

  // Verify the L turn instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Turn left onto South River Road.",
      "Turn left. Then Turn left to take the ramp.", "Turn left onto South River Road.",
      "Turn left onto South River Road. Then Turn left to take the ramp.", "Continue for 50 meters.");

  maneuver_index = 2;

  // Verify the L turn instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Turn left to take the ramp.", "", "Turn left to take the ramp.",
      "Turn left to take the ramp. Then, in 70 meters, You will arrive at your destination.",
      "Continue for 70 meters.");
}

///////////////////////////////////////////////////////////////////////////////
