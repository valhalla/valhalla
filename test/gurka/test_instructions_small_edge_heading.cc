#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

//#############################################################################
class InstructionsSmallEdgeHeading : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 1;

    const std::string ascii_map = R"(
            C------------------------------B------------------------------A
            F------------------------------E------------------------------D
                                          /
                                         |
                                         |
                                         |
                                         |
                                         G
    )";

    const gurka::ways ways =
        {{"AB", {{"highway", "residential"}, {"name", "Perczel Miklós utca"}, {"oneway", "yes"}}},
         {"BC", {{"highway", "residential"}, {"name", "Perczel Miklós utca"}, {"oneway", "yes"}}},
         {"DE", {{"highway", "footway"}, {"name", ""}}},
         {"EF", {{"highway", "footway"}, {"name", ""}}},
         {"BE", {{"highway", "residential"}, {"name", "Boltív köz"}}},
         {"EG", {{"highway", "residential"}, {"name", "Boltív köz"}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_instructions_small_edge_heading",
                            {{"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map InstructionsSmallEdgeHeading::map = {};

///////////////////////////////////////////////////////////////////////////////
TEST_F(InstructionsSmallEdgeHeading, LeftTurn) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify left turn
  // NOTE: old code had "Keep straight to take Boltív köz."
  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, maneuver_index, "Turn left onto Boltív köz.",
      "Turn left. Then You will arrive at your destination.", "Turn left onto Boltív köz.",
      "Turn left onto Boltív köz. Then You will arrive at your destination.",
      "Continue for less than 10 meters.");
}
