#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

//#############################################################################
class InstructionsIsStraightest : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 1000;

    const std::string ascii_map = R"(
                                    C--------------------D
         A-------B------------------E
                                    \---------------F
    )";

    const gurka::ways ways =
        {{"AB",
          {{"highway", "primary"}, {"name", "Belair Road"}, {"ref", "US 1"}, {"oneway", "yes"}}},
         {"BC",
          {{"highway", "primary"}, {"name", "Belair Road"}, {"ref", "US 1"}, {"oneway", "yes"}}},
         {"CD",
          {{"highway", "primary"}, {"name", "Belair Road"}, {"ref", "US 1"}, {"oneway", "yes"}}},
         {"BE",
          {{"highway", "motorway_link"},
           {"name", ""},
           {"destination", "Essex"},
           {"destination:ref", "I 695 South"}}},
         {"EF", {{"highway", "motorway_link"}, {"name", ""}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_instructions_is_straightest",
                            {{"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map InstructionsIsStraightest::map = {};

///////////////////////////////////////////////////////////////////////////////
TEST_F(InstructionsIsStraightest, SingleStraight) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 0;

  // Verify single straight instructions
  // prev2curr_turn_degree=355
  // straightest_xedge_turn_degree=0
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                            "Drive east on Belair Road/US 1.",
                                                            "Drive east.", "",
                                                            "Drive east on Belair Road, US 1.",
                                                            "Continue for 30 kilometers.");
}
