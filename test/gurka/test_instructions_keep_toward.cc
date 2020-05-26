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
                C--D      J--K
          A--B<--G--H--I<--N--O
                E--F      L--M
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
                              {"GH", {{"highway", "motorway"}, {"name", ""}, {"ref", "B3"}}},
                              {"HI", {{"highway", "motorway"}, {"name", ""}, {"ref", "B3"}}},
                              {"IJ",
                               {{"highway", "motorway"},
                                {"name", ""},
                                {"ref", "C1"},
                                {"junction:ref", "22A"},
                                {"destination", "Harrisburg;Lancaster"},
                                {"destination:ref", "C1"}}},
                              {"JK", {{"highway", "motorway"}, {"name", ""}, {"ref", "C1"}}},
                              {"IL",
                               {{"highway", "motorway"},
                                {"name", ""},
                                {"ref", "C2"},
                                {"junction:ref", "22C"},
                                {"destination", "Baltimore;Washington"},
                                {"destination:ref", "C2"}}},
                              {"LM", {{"highway", "motorway"}, {"name", ""}, {"ref", "C2"}}},
                              {"IN",
                               {{"highway", "motorway"},
                                {"name", ""},
                                {"ref", "C3"},
                                {"junction:ref", "22B"},
                                {"destination", "New York;Philadelphia"},
                                {"destination:ref", "C3"}}},
                              {"NO", {{"highway", "motorway"}, {"name", ""}, {"ref", "C3"}}}};

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

///////////////////////////////////////////////////////////////////////////////
// Keep right to take exit number toward
// "5": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> toward <TOWARD_SIGN>.",
TEST_F(InstructionsKeepToward, KeepRightExitNumberToward) {
  auto result = gurka::route(map, "H", "M", "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStayRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the keep right to take exit number toward instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Keep right to take exit 22C toward C2/Baltimore/Washington.",
      "Keep right to take exit 22C.", "Keep right to take exit 22C toward C2, Baltimore.",
      "Continue for 4 kilometers.");
}

///////////////////////////////////////////////////////////////////////////////
// Keep left to take exit number toward
// "5": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> toward <TOWARD_SIGN>.",
TEST_F(InstructionsKeepToward, KeepLeftExitNumberToward) {
  auto result = gurka::route(map, "H", "K", "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStayLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the keep left to take exit number toward instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Keep left to take exit 22A toward C1/Harrisburg/Lancaster.",
      "Keep left to take exit 22A.", "Keep left to take exit 22A toward C1, Harrisburg.",
      "Continue for 4 kilometers.");
}

///////////////////////////////////////////////////////////////////////////////
// Keep straight to take exit number toward
// "5": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> toward <TOWARD_SIGN>.",
TEST_F(InstructionsKeepToward, KeepStraightExitNumberToward) {
  auto result = gurka::route(map, "H", "O", "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStayStraight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the keep straight to take exit number toward instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Keep straight to take exit 22B toward C3/New York/Philadelphia.",
      "Keep straight to take exit 22B.", "Keep straight to take exit 22B toward C3, New York.",
      "Continue for 4 kilometers.");
}
