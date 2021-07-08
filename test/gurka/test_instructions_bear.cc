#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

class InstructionsBear : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 1000;

    const std::string ascii_map = R"(
             G 
             |    C--D
          A--B<
             |    E--F
             H
    )";

    const gurka::ways ways =
        {{"AB", {{"highway", "primary"}, {"name", "9th Street"}}},
         {"BC", {{"highway", "primary"}, {"name", "Hayes Street"}}},
         {"CD", {{"highway", "primary"}, {"name", "Hayes Street"}}},
         {"BE", {{"highway", "primary"}, {"name", "Larkin Street"}}},
         {"EF", {{"highway", "primary"}, {"name", "Larkin Street"}}},
         {"HBG", {{"highway", "primary"}, {"name", "Market Street"}, {"oneway", "yes"}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_instructions_bear",
                            {{"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map InstructionsBear::map = {};

///////////////////////////////////////////////////////////////////////////////
// Keep right
TEST_F(InstructionsBear, KeepRightBranchToward) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kSlightRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the keep right toward instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                            "Bear right onto Larkin Street.",
                                                            "Bear right.",
                                                            "Bear right onto Larkin Street.",
                                                            "Bear right onto Larkin Street.",
                                                            "Continue for 5 kilometers.");
}

///////////////////////////////////////////////////////////////////////////////
// Keep left
TEST_F(InstructionsBear, KeepLeftBranchToward) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kSlightLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the keep left toward instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                            "Bear left onto Hayes Street.",
                                                            "Bear left.",
                                                            "Bear left onto Hayes Street.",
                                                            "Bear left onto Hayes Street.",
                                                            "Continue for 5 kilometers.");
}
