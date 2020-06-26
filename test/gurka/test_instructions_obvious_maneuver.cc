#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

class InstructionsObviousManeuver : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 1000;

    const std::string ascii_map = R"(
                                ---I
       C-------                /
       A-------B----D----E----G----H
                         |         |
                         |         |
                         |         |
                         F         J
    )";

    const gurka::ways ways = {{"AB",
                               {{"highway", "primary"}, {"name", "Vine Street"}, {"oneway", "yes"}}},
                              {"BC",
                               {{"highway", "primary"}, {"name", "Vine Street"}, {"oneway", "yes"}}},
                              {"BD", {{"highway", "primary"}, {"name", "Middletown Road"}}},
                              {"DE", {{"highway", "primary"}, {"name", "Hanover Street"}}},
                              {"EF", {{"highway", "primary"}, {"name", "Main Street"}}},
                              {"EG", {{"highway", "primary"}, {"name", "Hanover Street"}}},
                              {"GI", {{"highway", "primary"}, {"name", "Hanover Street"}}},
                              {"GH", {{"highway", "primary"}, {"name", "Maple Street"}}},
                              {"HJ", {{"highway", "primary"}, {"name", "1st Avenue"}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_instructions_obvious_maneuver",
                            {{"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map InstructionsObviousManeuver::map = {};

///////////////////////////////////////////////////////////////////////////////
TEST_F(InstructionsObviousManeuver, IgnoreSimpleNameChange) {
  auto result = gurka::route(map, "B", "F", "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 0;

  // Verify single maneuver prior to the right turn
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                            "Drive east on Middletown Road.", "",
                                                            "Drive east on Middletown Road.",
                                                            "Continue for 6 kilometers.");

  // Verify right turn
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, ++maneuver_index,
                                                            "Turn right onto Main Street.",
                                                            "Turn right onto Main Street.",
                                                            "Turn right onto Main Street.",
                                                            "Continue for 4 kilometers.");
}

///////////////////////////////////////////////////////////////////////////////
TEST_F(InstructionsObviousManeuver, IgnoreOpposingSameNameIntersectingEdge) {
  auto result = gurka::route(map, "A", "F", "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 0;

  // Verify single maneuver prior to the right turn
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                            "Drive east on Vine Street.", "",
                                                            "Drive east on Vine Street.",
                                                            "Continue for 11 kilometers.");

  // Verify right turn
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, ++maneuver_index,
                                                            "Turn right onto Main Street.",
                                                            "Turn right onto Main Street.",
                                                            "Turn right onto Main Street.",
                                                            "Continue for 4 kilometers.");
}

///////////////////////////////////////////////////////////////////////////////
TEST_F(InstructionsObviousManeuver, NotObviousBecauseSameNameIntersectingEdge) {
  auto result = gurka::route(map, "D", "J", "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kContinue,
                                                DirectionsLeg_Maneuver_Type_kRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 0;

  // Verify start maneuver prior to the continune maneuver
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                            "Drive east on Hanover Street.", "",
                                                            "Drive east on Hanover Street.",
                                                            "Continue for 6 kilometers.");

  // Verify continue because of same name intersecting edge
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, ++maneuver_index,
                                                            "Continue on Maple Street.",
                                                            "Continue on Maple Street.",
                                                            "Continue on Maple Street.",
                                                            "Continue for 3 kilometers.");

  // Verify right turn
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, ++maneuver_index,
                                                            "Turn right onto 1st Avenue.",
                                                            "Turn right onto 1st Avenue.",
                                                            "Turn right onto 1st Avenue.",
                                                            "Continue for 4 kilometers.");
}
