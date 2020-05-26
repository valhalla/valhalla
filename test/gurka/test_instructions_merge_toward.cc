#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

class InstructionsMergeToward : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 1000;

    const std::string ascii_map = R"(
          A--B--C--D
            /
           E     
    )";

    const gurka::ways ways = {{"AB", {{"highway", "motorway"}, {"name", ""}, {"oneway", "yes"}}},
                              {"BC",
                               {{"highway", "motorway"},
                                {"name", ""},
                                {"oneway", "yes"},
                                {"destination", "Harrisburg;Lancaster"},
                                {"destination:ref", "A6"}}},
                              {"CD", {{"highway", "motorway"}, {"name", ""}, {"oneway", "yes"}}},
                              {"EB",
                               {{"highway", "motorway_link"}, {"name", ""}, {"oneway", "yes"}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_instructions_merge_toward",
                            {{"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map InstructionsMergeToward::map = {};

///////////////////////////////////////////////////////////////////////////////
// Merge left toward
// "5" : "Merge <RELATIVE_DIRECTION> toward <TOWARD_SIGN>."
TEST_F(InstructionsMergeToward, MergeLeftToward) {
  auto result = gurka::route(map, "E", "D", "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kMergeLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the merge left toward instructions
  gurka::assert::raw::
      expect_instructions_at_maneuver_index(result, maneuver_index,
                                            "Merge left toward A6/Harrisburg/Lancaster.",
                                            "Merge left toward A6.",
                                            "Merge left toward A6, Harrisburg.",
                                            "Continue for 4 kilometers.");
}
