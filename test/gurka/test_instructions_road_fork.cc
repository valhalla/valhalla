#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

//#############################################################################
class InstructionsRoadFork1 : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 1000;

    //  Location: 50.883388, 4.712684
    const std::string ascii_map = R"(
                ---C
          A--B<
                ---D
    )";

    const gurka::ways ways =
        {{"AB",
          {{"highway", "secondary"},
           {"name", "Diestsevest"},
           {"ref", "R23"},
           {"oneway", "yes"},
           {"lanes", "2"},
           {"turn:lanes", "through|slight_right"}}},
         {"BC",
          {{"highway", "secondary"},
           {"name", "Oostertunnel"},
           {"ref", "R23"},
           {"oneway", "yes"},
           {"lanes", "1"}}},
         {"BD",
          {{"highway", "residential"}, {"name", "Diestsevest"}, {"oneway", "yes"}, {"lanes", "1"}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_instructions_road_fork_1",
                            {{"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map InstructionsRoadFork1::map = {};

///////////////////////////////////////////////////////////////////////////////
// Keep left
TEST_F(InstructionsRoadFork1, KeepLeft) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "C"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStayLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the keep left toward instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                            "Keep left to take Oostertunnel/R23.", "",
                                                            "Keep left to take Oostertunnel.",
                                                            "Keep left to take Oostertunnel, R23.",
                                                            "Continue for 4 kilometers.");
}

///////////////////////////////////////////////////////////////////////////////
// Keep right
TEST_F(InstructionsRoadFork1, KeepRight) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStayRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the right right toward instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                            "Keep right to stay on Diestsevest.", "",
                                                            "Keep right to stay on Diestsevest.",
                                                            "Keep right to stay on Diestsevest.",
                                                            "Continue for 4 kilometers.");
}
