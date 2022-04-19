#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

class InstructionsKeepNonHighwaySigns : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 1000;

    const std::string ascii_map = R"(
                  C--D      J--K
          A--B<--G--H--I<--N--O
                  E--F      L--M
    )";

    const gurka::ways ways = {{"AB", {{"highway", "primary"}, {"name", ""}, {"ref", "A1"}}},
                              {"BC",
                               {{"highway", "primary"},
                                {"name", ""},
                                {"ref", "B1"},
                                {"destination", "Harrisburg;Lancaster"},
                                {"destination:ref", "B1"}}},
                              {"CD", {{"highway", "primary"}, {"name", ""}, {"ref", "B1"}}},
                              {"BE",
                               {{"highway", "primary"},
                                {"name", ""},
                                {"ref", "B2"},
                                {"destination", "Baltimore;Washington"},
                                {"destination:ref", "B2"}}},
                              {"EF", {{"highway", "primary"}, {"name", ""}, {"ref", "B2"}}},
                              {"BG",
                               {{"highway", "primary"},
                                {"name", ""},
                                {"ref", "B3"},
                                {"destination", "New York;Philadelphia"},
                                {"destination:ref", "B3"}}},
                              {"GH", {{"highway", "primary"}, {"name", ""}, {"ref", "B3"}}},
                              {"HI", {{"highway", "primary"}, {"name", ""}, {"ref", "B3"}}},
                              {"IJ",
                               {{"highway", "primary"},
                                {"name", ""},
                                {"ref", "C1"},
                                {"junction:ref", "22A"},
                                {"destination", "Harrisburg;Lancaster"},
                                {"destination:ref", "C1"}}},
                              {"JK", {{"highway", "primary"}, {"name", ""}, {"ref", "C1"}}},
                              {"IL",
                               {{"highway", "primary"},
                                {"name", ""},
                                {"ref", "C2"},
                                {"junction:ref", "22C"},
                                {"destination", "Baltimore;Washington"},
                                {"destination:ref", "C2"}}},
                              {"LM", {{"highway", "primary"}, {"name", ""}, {"ref", "C2"}}},
                              {"IN",
                               {{"highway", "primary"},
                                {"name", ""},
                                {"ref", "C3"},
                                {"junction:ref", "22B"},
                                {"destination", "New York;Philadelphia"},
                                {"destination:ref", "C3"}}},
                              {"NO", {{"highway", "primary"}, {"name", ""}, {"ref", "C3"}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

    map =
        gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_instructions_keep_nonhighway_signs",
                          {{"mjolnir.admin",
                            {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map InstructionsKeepNonHighwaySigns::map = {};

///////////////////////////////////////////////////////////////////////////////
// Keep right
TEST_F(InstructionsKeepNonHighwaySigns, KeepRightBranchToward) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStayRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the keep right toward instructions
  gurka::assert::raw::
      expect_instructions_at_maneuver_index(result, maneuver_index,
                                            "Keep right to take B2 toward Baltimore/Washington.", "",
                                            "Keep right to take B2.",
                                            "Keep right to take B2 toward Baltimore, Washington.",
                                            "Continue for 5 kilometers.");
}

///////////////////////////////////////////////////////////////////////////////
// Keep left
TEST_F(InstructionsKeepNonHighwaySigns, KeepLeftBranchToward) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStayLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the keep left toward instructions
  gurka::assert::raw::
      expect_instructions_at_maneuver_index(result, maneuver_index,
                                            "Keep left to take B1 toward Harrisburg/Lancaster.", "",
                                            "Keep left to take B1.",
                                            "Keep left to take B1 toward Harrisburg, Lancaster.",
                                            "Continue for 5 kilometers.");
}

///////////////////////////////////////////////////////////////////////////////
// Keep straight
TEST_F(InstructionsKeepNonHighwaySigns, KeepStraightBranchToward) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "H"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStayStraight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the keep straight toward instructions
  gurka::assert::raw::
      expect_instructions_at_maneuver_index(result, maneuver_index,
                                            "Keep straight to take B3 toward New York/Philadelphia.",
                                            "", "Keep straight to take B3.",
                                            "Keep straight to take B3 toward New York, Philadelphia.",
                                            "Continue for 4 kilometers.");
}

///////////////////////////////////////////////////////////////////////////////
// Keep right to take exit number
TEST_F(InstructionsKeepNonHighwaySigns, KeepRightExitNumberBranchToward) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"H", "M"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStayRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the keep right to take exit number toward instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Keep right to take exit 22C onto C2 toward Baltimore/Washington.", "",
      "Keep right to take exit 22C.",
      "Keep right to take exit 22C onto C2 toward Baltimore, Washington.",
      "Continue for 5 kilometers.");
}

///////////////////////////////////////////////////////////////////////////////
// Keep left to take exit number
TEST_F(InstructionsKeepNonHighwaySigns, KeepLeftExitNumberBranchToward) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"H", "K"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStayLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the keep left to take exit number toward instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Keep left to take exit 22A onto C1 toward Harrisburg/Lancaster.", "",
      "Keep left to take exit 22A.",
      "Keep left to take exit 22A onto C1 toward Harrisburg, Lancaster.",
      "Continue for 5 kilometers.");
}

///////////////////////////////////////////////////////////////////////////////
// Keep straight to take exit number
TEST_F(InstructionsKeepNonHighwaySigns, KeepStraightExitNumberBranchToward) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"H", "O"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStayStraight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the keep straight to take exit number toward instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Keep straight to take exit 22B onto C3 toward New York/Philadelphia.",
      "", "Keep straight to take exit 22B.",
      "Keep straight to take exit 22B onto C3 toward New York, Philadelphia.",
      "Continue for 4 kilometers.");
}
