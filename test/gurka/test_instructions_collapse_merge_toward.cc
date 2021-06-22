#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

class InstructionsCollapseMergeToward : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 1000;

    const std::string ascii_map = R"(
           G
           |
         A------B--C--D
           |  < I---J
           | H
           |/
           E
           |
           |
           F     
    )";

    const gurka::ways ways = {{"AB", {{"highway", "motorway"}, {"name", ""}, {"oneway", "yes"}}},
                              {"BC",
                               {{"highway", "motorway"},
                                {"name", ""},
                                {"oneway", "yes"},
                                {"destination", "Harrisburg;Lancaster"},
                                {"destination:ref", "A6"}}},
                              {"CD", {{"highway", "motorway"}, {"name", ""}, {"oneway", "yes"}}},
                              {"EH", {{"highway", "motorway_link"}, {"name", ""}, {"oneway", "yes"}}},
                              {"HB", {{"highway", "motorway_link"}, {"name", ""}, {"oneway", "yes"}}},
                              {"HI", {{"highway", "motorway_link"}, {"name", ""}, {"oneway", "yes"}}},
                              {"FE", {{"highway", "primary"}, {"name", "Main Street"}}},
                              {"EG", {{"highway", "primary"}, {"name", "Main Street"}}},
                              {"IJ",
                               {{"highway", "primary"}, {"name", "1st Avenue"}, {"oneway", "yes"}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_instructions_merge_toward",
                            {{"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map InstructionsCollapseMergeToward::map = {};

///////////////////////////////////////////////////////////////////////////////
// Merge left toward
// "5" : "Merge <RELATIVE_DIRECTION> toward <TOWARD_SIGN>."
TEST_F(InstructionsCollapseMergeToward, MergeLeftToward) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"F", "D"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRampRight,
                                                DirectionsLeg_Maneuver_Type_kStayLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 2;

  // Verify left fork and collapsed merge
  gurka::assert::raw::
      expect_instructions_at_maneuver_index(result, maneuver_index,
                                            "Keep left to take A6 toward Harrisburg/Lancaster.", "",
                                            "Keep left to take A6.",
                                            "Keep left to take A6 toward Harrisburg, Lancaster.",
                                            "Continue for 6 kilometers.");

  // Collapsed
  //  ++maneuver_index;
  //
  //  // Verify the merge left toward instructions
  //  gurka::assert::raw::
  //      expect_instructions_at_maneuver_index(result, maneuver_index,
  //                                            "Merge left toward A6/Harrisburg/Lancaster.",
  //                                            "Merge left.",
  //                                            "Merge left toward A6.",
  //                                            "Merge left toward A6, Harrisburg.",
  //                                            "Continue for 4 kilometers.");
}
