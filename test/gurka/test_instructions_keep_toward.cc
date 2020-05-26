#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

class InstructionsKeepToward : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 1000;

    const std::string ascii_map = R"(
                C--D
          A--B<--G--H
                E--F
    )";

    const gurka::ways ways = {{"AB", {{"highway", "motorway"}, {"name", ""}, {"ref", "A1"}}},
                              {"BC",
                               {{"highway", "motorway"},
                                {"name", ""},
                                {"ref", "B1"},
                                {"destination", "Harrisburg;Lancaster"},
                                {"destination:ref", "B1"}}},
                              {"CD", {{"highway", "motorway"}, {"name", ""}, {"ref", "B1"}}},
                              {"BE",
                               {{"highway", "motorway"},
                                {"name", ""},
                                {"ref", "B2"},
                                {"destination", "Baltimore;Washington"},
                                {"destination:ref", "B2"}}},
                              {"EF", {{"highway", "motorway"}, {"name", ""}, {"ref", "B2"}}},
                              {"BG",
                               {{"highway", "motorway"},
                                {"name", ""},
                                {"ref", "B3"},
                                {"destination", "New York;Philadelphia"},
                                {"destination:ref", "B3"}}},
                              {"GH", {{"highway", "motorway"}, {"name", ""}, {"ref", "B3"}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_instructions_keep_toward",
                            {{"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map InstructionsKeepToward::map = {};

///////////////////////////////////////////////////////////////////////////////
// Keep right toward
// "4": "Keep <RELATIVE_DIRECTION> toward <TOWARD_SIGN>."
TEST_F(InstructionsKeepToward, KeepRightToward) {
  auto result = gurka::route(map, "A", "F", "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStayRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the keep right toward instructions
  gurka::assert::raw::
      expect_instructions_at_maneuver_index(result, maneuver_index,
                                            "Keep right toward B2/Baltimore/Washington.",
                                            "Keep right toward B2.",
                                            "Keep right toward B2, Baltimore.",
                                            "Continue for 4 kilometers.");
}

///////////////////////////////////////////////////////////////////////////////
// Keep left toward
// "4": "Keep <RELATIVE_DIRECTION> toward <TOWARD_SIGN>."
TEST_F(InstructionsKeepToward, KeepLeftToward) {
  auto result = gurka::route(map, "A", "D", "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStayLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the keep left toward instructions
  gurka::assert::raw::
      expect_instructions_at_maneuver_index(result, maneuver_index,
                                            "Keep left toward B1/Harrisburg/Lancaster.",
                                            "Keep left toward B1.",
                                            "Keep left toward B1, Harrisburg.",
                                            "Continue for 4 kilometers.");
}

///////////////////////////////////////////////////////////////////////////////
// Keep straight toward
// "4": "Keep <RELATIVE_DIRECTION> toward <TOWARD_SIGN>."
TEST_F(InstructionsKeepToward, KeepStraightToward) {
  auto result = gurka::route(map, "A", "H", "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStayStraight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the keep straight toward instructions
  gurka::assert::raw::
      expect_instructions_at_maneuver_index(result, maneuver_index,
                                            "Keep straight toward B3/New York/Philadelphia.",
                                            "Keep straight toward B3.",
                                            "Keep straight toward B3, New York.",
                                            "Continue for 4 kilometers.");
}
