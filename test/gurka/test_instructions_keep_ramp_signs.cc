#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

class InstructionsKeepRampSigns : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 1000;

    const std::string ascii_map = R"(
                C--D 
          A--B<--G--H
                E--F
    )";

    const gurka::ways ways = {{"AB", {{"highway", "motorway_link"}, {"name", ""}, {"ref", "A 4"}}},
                              {"BC",
                               {{"highway", "motorway_link"},
                                {"name", ""},
                                {"ref", "A 4"},
                                {"destination", "Köln"},
                                {"destination:ref", "A 4"}}},
                              {"CD", {{"highway", "motorway_link"}, {"name", ""}, {"ref", "A 4"}}},
                              {"BE",
                               {{"highway", "motorway_link"},
                                {"name", ""},
                                {"ref", "A 61"},
                                {"destination", "Koblenz"},
                                {"destination:ref", "A 61"}}},
                              {"EF", {{"highway", "motorway_link"}, {"name", ""}, {"ref", "A 61"}}},
                              {"BG",
                               {{"highway", "motorway_link"},
                                {"name", ""},
                                {"ref", "A 4"},
                                {"destination", "Venlo"},
                                {"destination:ref", "A 61"}}},
                              {"GH", {{"highway", "motorway_link"}, {"name", ""}, {"ref", "A 4"}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_instructions_keep_ramp_signs",
                            {{"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map InstructionsKeepRampSigns::map = {};

///////////////////////////////////////////////////////////////////////////////
// Keep right branch toward
TEST_F(InstructionsKeepRampSigns, KeepRightBranchToward) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStayRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the keep right toward instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                            "Keep right to take A 61 toward Koblenz.",
                                                            "", "Keep right to take A 61.",
                                                            "Keep right to take A 61 toward Koblenz.",
                                                            "Continue for 4 kilometers.");
}

///////////////////////////////////////////////////////////////////////////////
// Keep left branch toward
TEST_F(InstructionsKeepRampSigns, KeepLeftBranchToward) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStayLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the keep left toward instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                            "Keep left to take A 4 toward Köln.", "",
                                                            "Keep left to take A 4.",
                                                            "Keep left to take A 4 toward Köln.",
                                                            "Continue for 4 kilometers.");
}

///////////////////////////////////////////////////////////////////////////////
// Keep straight branch toward
TEST_F(InstructionsKeepRampSigns, KeepStraightBranchToward) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "H"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStayStraight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the keep straight toward instructions
  gurka::assert::raw::
      expect_instructions_at_maneuver_index(result, maneuver_index,
                                            "Keep straight to take A 61 toward Venlo.", "",
                                            "Keep straight to take A 61.",
                                            "Keep straight to take A 61 toward Venlo.",
                                            "Continue for 4 kilometers.");
}
